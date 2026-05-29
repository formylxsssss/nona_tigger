#include "rs485_control.h"
#include <string.h>

typedef struct
{
    RS485_ControlMode_t mode;
    uint8_t auto_running;
    uint8_t pulse_active;
    uint32_t pulse_width_ms;
    uint32_t interval_ms;
    uint32_t pulse_started_tick;
    uint32_t next_auto_tick;
} RS485_ControlState_t;

static UART_HandleTypeDef *s_huart = NULL;
static RS485_ControlState_t s_state;

static volatile uint8_t s_rx_byte = 0U;
static volatile uint8_t s_rx_buf[RS485_CTRL_MAX_FRAME_LEN];
static volatile uint8_t s_rx_len = 0U;
static volatile uint8_t s_rx_expect_len = 0U;
static volatile uint8_t s_frame_ready = 0U;
static volatile uint32_t s_last_rx_tick = 0U;
static uint8_t s_pending_frame[RS485_CTRL_MAX_FRAME_LEN];
static uint8_t s_pending_len = 0U;

static void RS485_Control_SetTxMode(void);
static void RS485_Control_SetRxMode(void);
static void RS485_Control_ResetParser(void);
static void RS485_Control_TryCompleteFrame(void);
static void RS485_Control_ProcessFrame(const uint8_t *frame, uint8_t frame_len);
static void RS485_Control_SendResponse(uint8_t cmd, uint8_t result, const uint8_t *data, uint8_t data_len);
static void RS485_Control_TriggerOutput(void);
static void RS485_Control_StopAuto(void);
static void RS485_Control_StartAuto(void);
static uint16_t RS485_Control_Crc16(const uint8_t *data, uint16_t len);
static uint16_t RS485_Control_ReadU16LE(const uint8_t *p);
static void RS485_Control_WriteU16LE(uint8_t *p, uint16_t v);
static void RS485_Control_WriteU32LE(uint8_t *p, uint32_t v);

HAL_StatusTypeDef RS485_Control_Init(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    s_huart = huart;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = RS485_CTRL_DE_PIN | TRIGGER_OUT_PIN | TRIGGER_OE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(RS485_CTRL_DE_GPIO_PORT, RS485_CTRL_DE_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRIGGER_OUT_GPIO_PORT, TRIGGER_OUT_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRIGGER_OE_GPIO_PORT, TRIGGER_OE_PIN, TRIGGER_OE_ACTIVE_LEVEL);

    s_state.mode = RS485_CTRL_MODE_TRIGGER;
    s_state.auto_running = 0U;
    s_state.pulse_active = 0U;
    s_state.pulse_width_ms = RS485_CTRL_DEFAULT_PULSE_MS;
    s_state.interval_ms = RS485_CTRL_DEFAULT_INTERVAL_SEC * 1000U;
    s_state.pulse_started_tick = 0U;
    s_state.next_auto_tick = 0U;

    s_frame_ready = 0U;
    s_pending_len = 0U;
    RS485_Control_ResetParser();
    RS485_Control_SetRxMode();

    return HAL_OK;
}

HAL_StatusTypeDef RS485_Control_StartReceiveIT(void)
{
    if (s_huart == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Receive_IT(s_huart, (uint8_t *)&s_rx_byte, 1U);
}

void RS485_Control_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t b;

    if ((s_huart == NULL) || (huart == NULL) || (huart->Instance != s_huart->Instance))
    {
        return;
    }

    b = s_rx_byte;

    if (s_frame_ready == 0U)
    {
        if ((s_rx_len == 0U) && (b != RS485_CTRL_HEADER_0))
        {
            (void)HAL_UART_Receive_IT(s_huart, (uint8_t *)&s_rx_byte, 1U);
            return;
        }

        if ((s_rx_len == 1U) && (b != RS485_CTRL_HEADER_1))
        {
            s_rx_len = (b == RS485_CTRL_HEADER_0) ? 1U : 0U;
            s_rx_buf[0] = b;
            (void)HAL_UART_Receive_IT(s_huart, (uint8_t *)&s_rx_byte, 1U);
            return;
        }

        if (s_rx_len < RS485_CTRL_MAX_FRAME_LEN)
        {
            s_rx_buf[s_rx_len++] = b;
            s_last_rx_tick = HAL_GetTick();

            if (s_rx_len == 3U)
            {
                uint8_t len = s_rx_buf[2];

                if ((len == 0U) || (len > (RS485_CTRL_MAX_DATA_LEN + 1U)))
                {
                    RS485_Control_ResetParser();
                }
                else
                {
                    s_rx_expect_len = (uint8_t)(len + RS485_CTRL_FRAME_OVERHEAD);
                }
            }

            if ((s_rx_expect_len > 0U) && (s_rx_len == s_rx_expect_len))
            {
                RS485_Control_TryCompleteFrame();
            }
        }
        else
        {
            RS485_Control_ResetParser();
        }
    }

    (void)HAL_UART_Receive_IT(s_huart, (uint8_t *)&s_rx_byte, 1U);
}

void RS485_Control_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((s_huart == NULL) || (huart == NULL) || (huart->Instance != s_huart->Instance))
    {
        return;
    }

    RS485_Control_ResetParser();
    __HAL_UART_CLEAR_PEFLAG(s_huart);
    (void)HAL_UART_Receive_IT(s_huart, (uint8_t *)&s_rx_byte, 1U);
}

void RS485_Control_Poll(void)
{
    uint8_t frame[RS485_CTRL_MAX_FRAME_LEN];
    uint8_t frame_len;
    uint32_t now = HAL_GetTick();

    if ((s_rx_len > 0U) && ((uint32_t)(now - s_last_rx_tick) > 50U))
    {
        RS485_Control_ResetParser();
    }

    if ((s_state.pulse_active != 0U) &&
        ((uint32_t)(now - s_state.pulse_started_tick) >= s_state.pulse_width_ms))
    {
        HAL_GPIO_WritePin(TRIGGER_OUT_GPIO_PORT, TRIGGER_OUT_PIN, GPIO_PIN_RESET);
        s_state.pulse_active = 0U;
    }

    if ((s_state.mode == RS485_CTRL_MODE_AUTO) &&
        (s_state.auto_running != 0U) &&
        (s_state.pulse_active == 0U) &&
        ((int32_t)(now - s_state.next_auto_tick) >= 0))
    {
        RS485_Control_TriggerOutput();
        s_state.next_auto_tick = now + s_state.interval_ms;
    }

    if (s_frame_ready == 0U)
    {
        return;
    }

    __disable_irq();
    frame_len = s_pending_len;
    if (frame_len > RS485_CTRL_MAX_FRAME_LEN)
    {
        frame_len = RS485_CTRL_MAX_FRAME_LEN;
    }
    memcpy(frame, s_pending_frame, frame_len);
    s_frame_ready = 0U;
    s_pending_len = 0U;
    __enable_irq();

    RS485_Control_ProcessFrame(frame, frame_len);
}

static void RS485_Control_ProcessFrame(const uint8_t *frame, uint8_t frame_len)
{
    uint8_t len;
    uint8_t cmd;
    const uint8_t *data;
    uint8_t data_len;
    uint8_t tx_data[16];
    uint8_t tx_len = 0U;
    uint8_t result = RS485_CTRL_RES_OK;

    if ((frame == NULL) || (frame_len < RS485_CTRL_FRAME_OVERHEAD + 1U))
    {
        return;
    }

    len = frame[2];
    cmd = frame[3];
    data = &frame[4];
    data_len = (uint8_t)(len - 1U);

    switch (cmd)
    {
        case RS485_CTRL_CMD_SET_MODE:
        {
            if (data_len != 1U)
            {
                result = RS485_CTRL_RES_ERR_LEN;
            }
            else if (data[0] == RS485_CTRL_MODE_VALUE_AUTO)
            {
                s_state.mode = RS485_CTRL_MODE_AUTO;
                result = RS485_CTRL_RES_OK;
            }
            else if (data[0] == RS485_CTRL_MODE_VALUE_TRIGGER)
            {
                RS485_Control_StopAuto();
                s_state.mode = RS485_CTRL_MODE_TRIGGER;
                result = RS485_CTRL_RES_OK;
            }
            else
            {
                result = RS485_CTRL_RES_ERR_PARAM;
            }
        } break;

        case RS485_CTRL_CMD_SET_INTERVAL:
        {
            uint16_t sec;

            if (data_len != 2U)
            {
                result = RS485_CTRL_RES_ERR_LEN;
            }
            else
            {
                sec = RS485_Control_ReadU16LE(data);
                if (sec == 0U)
                {
                    result = RS485_CTRL_RES_ERR_PARAM;
                }
                else
                {
                    s_state.interval_ms = (uint32_t)sec * 1000U;
                    result = RS485_CTRL_RES_OK;
                }
            }
        } break;

        case RS485_CTRL_CMD_SET_PULSE:
        {
            uint16_t ms;

            if (data_len != 2U)
            {
                result = RS485_CTRL_RES_ERR_LEN;
            }
            else
            {
                ms = RS485_Control_ReadU16LE(data);
                if ((ms == 0U) || (ms > 60000U))
                {
                    result = RS485_CTRL_RES_ERR_PARAM;
                }
                else
                {
                    s_state.pulse_width_ms = ms;
                    result = RS485_CTRL_RES_OK;
                }
            }
        } break;

        case RS485_CTRL_CMD_START:
        {
            if (data_len != 0U)
            {
                result = RS485_CTRL_RES_ERR_LEN;
            }
            else if (s_state.mode != RS485_CTRL_MODE_AUTO)
            {
                result = RS485_CTRL_RES_ERR_MODE;
            }
            else
            {
                RS485_Control_StartAuto();
                result = RS485_CTRL_RES_OK;
            }
        } break;

        case RS485_CTRL_CMD_STOP:
        {
            if (data_len != 0U)
            {
                result = RS485_CTRL_RES_ERR_LEN;
            }
            else
            {
                RS485_Control_StopAuto();
                result = RS485_CTRL_RES_OK;
            }
        } break;

        case RS485_CTRL_CMD_TRIGGER:
        {
            if (data_len != 0U)
            {
                result = RS485_CTRL_RES_ERR_LEN;
            }
            else if (s_state.mode != RS485_CTRL_MODE_TRIGGER)
            {
                result = RS485_CTRL_RES_ERR_MODE;
            }
            else if (s_state.pulse_active != 0U)
            {
                result = RS485_CTRL_RES_ERR_BUSY;
            }
            else
            {
                RS485_Control_TriggerOutput();
                result = RS485_CTRL_RES_OK;
            }
        } break;

        case RS485_CTRL_CMD_GET_STATUS:
        {
            if (data_len != 0U)
            {
                result = RS485_CTRL_RES_ERR_LEN;
            }
            else
            {
                tx_data[0] = (s_state.mode == RS485_CTRL_MODE_AUTO) ? RS485_CTRL_MODE_VALUE_AUTO : RS485_CTRL_MODE_VALUE_TRIGGER;
                tx_data[1] = s_state.auto_running;
                tx_data[2] = s_state.pulse_active;
                RS485_Control_WriteU32LE(&tx_data[3], s_state.interval_ms / 1000U);
                RS485_Control_WriteU32LE(&tx_data[7], s_state.pulse_width_ms);
                tx_len = 11U;
                result = RS485_CTRL_RES_OK;
            }
        } break;

        default:
        {
            result = RS485_CTRL_RES_ERR_CMD;
        } break;
    }

    RS485_Control_SendResponse(cmd, result, tx_data, tx_len);
}

static void RS485_Control_TriggerOutput(void)
{
    s_state.pulse_started_tick = HAL_GetTick();
    s_state.pulse_active = 1U;
    HAL_GPIO_WritePin(TRIGGER_OE_GPIO_PORT, TRIGGER_OE_PIN, TRIGGER_OE_ACTIVE_LEVEL);
    HAL_GPIO_WritePin(TRIGGER_OUT_GPIO_PORT, TRIGGER_OUT_PIN, GPIO_PIN_SET);
}

static void RS485_Control_StartAuto(void)
{
    s_state.auto_running = 1U;
    s_state.next_auto_tick = HAL_GetTick();
}

static void RS485_Control_StopAuto(void)
{
    s_state.auto_running = 0U;
}

static void RS485_Control_ResetParser(void)
{
    s_rx_len = 0U;
    s_rx_expect_len = 0U;
    s_last_rx_tick = HAL_GetTick();
}

static void RS485_Control_TryCompleteFrame(void)
{
    uint16_t crc_calc;
    uint16_t crc_recv;
    uint8_t total_len = s_rx_expect_len;

    crc_calc = RS485_Control_Crc16((const uint8_t *)s_rx_buf, (uint16_t)(total_len - 2U));
    crc_recv = RS485_Control_ReadU16LE((const uint8_t *)&s_rx_buf[total_len - 2U]);

    if ((crc_calc == crc_recv) && (s_frame_ready == 0U))
    {
        memcpy(s_pending_frame, (const void *)s_rx_buf, total_len);
        s_pending_len = total_len;
        s_frame_ready = 1U;
    }

    RS485_Control_ResetParser();
}

static void RS485_Control_SendResponse(uint8_t cmd, uint8_t result, const uint8_t *data, uint8_t data_len)
{
    uint8_t tx_buf[RS485_CTRL_MAX_FRAME_LEN];
    uint8_t payload_len;
    uint8_t total_len;
    uint16_t crc;

    if ((s_huart == NULL) || (data_len > RS485_CTRL_MAX_DATA_LEN))
    {
        return;
    }

    payload_len = (uint8_t)(2U + data_len);
    total_len = (uint8_t)(payload_len + RS485_CTRL_FRAME_OVERHEAD);

    tx_buf[0] = RS485_CTRL_HEADER_0;
    tx_buf[1] = RS485_CTRL_HEADER_1;
    tx_buf[2] = payload_len;
    tx_buf[3] = (uint8_t)(cmd | 0x80U);
    tx_buf[4] = result;

    if ((data != NULL) && (data_len > 0U))
    {
        memcpy(&tx_buf[5], data, data_len);
    }

    crc = RS485_Control_Crc16(tx_buf, (uint16_t)(total_len - 2U));
    RS485_Control_WriteU16LE(&tx_buf[total_len - 2U], crc);

    RS485_Control_SetTxMode();
    (void)HAL_UART_Transmit(s_huart, tx_buf, total_len, 1000U);

    while (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_TC) == RESET)
    {
    }

    RS485_Control_SetRxMode();
}

static void RS485_Control_SetTxMode(void)
{
    HAL_GPIO_WritePin(RS485_CTRL_DE_GPIO_PORT, RS485_CTRL_DE_PIN, GPIO_PIN_SET);
}

static void RS485_Control_SetRxMode(void)
{
    HAL_GPIO_WritePin(RS485_CTRL_DE_GPIO_PORT, RS485_CTRL_DE_PIN, GPIO_PIN_RESET);
}

static uint16_t RS485_Control_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t j;

    if (data == NULL)
    {
        return 0xFFFFU;
    }

    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0U; j < 8U; j++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc >>= 1U;
                crc ^= 0xA001U;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static uint16_t RS485_Control_ReadU16LE(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void RS485_Control_WriteU16LE(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static void RS485_Control_WriteU32LE(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}
