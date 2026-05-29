#include "rs485_modbus.h"
#include "display_logic.h"
#include "pulse_out.h"
#include <string.h>

/*==================== 静态变量 ====================*/
static UART_HandleTypeDef *s_huart = NULL;

static volatile uint8_t  s_rx_byte = 0U;
static volatile uint8_t  s_rx_buf[RS485_MB_FRAME_MAX_LEN];
static volatile uint16_t s_rx_len = 0U;
static volatile uint16_t s_rx_expect_len = 0U;
static volatile uint32_t s_last_rx_tick = 0U;

static volatile uint8_t  s_pending_buf[RS485_MB_FRAME_MAX_LEN];
static volatile uint16_t s_pending_len = 0U;
static volatile uint8_t  s_pending_ready = 0U;

/*==================== 静态函数声明 ====================*/
static void     RS485_MB_DE_Init(void);
static void     RS485_MB_SetTxMode(void);
static void     RS485_MB_SetRxMode(void);
static void     RS485_MB_ResetParser(void);

static uint16_t RS485_MB_ReadU16LE(const uint8_t *p);
static uint32_t RS485_MB_ReadU32LE(const uint8_t *p);
static void     RS485_MB_WriteU16LE(uint8_t *p, uint16_t v);
static void     RS485_MB_WriteU32LE(uint8_t *p, uint32_t v);

static void     RS485_MB_TryCompleteFrame(void);
static void     RS485_MB_ProcessFrame(const uint8_t *frame, uint16_t frame_len);
static void     RS485_MB_SendResponse(uint8_t addr, uint8_t cmd, const uint8_t *payload, uint16_t payload_len);

/*==================== 初始化 ====================*/
HAL_StatusTypeDef RS485_MB_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    s_huart = huart;

    RS485_MB_DE_Init();
    RS485_MB_SetRxMode();
    RS485_MB_ResetParser();

    s_pending_len = 0U;
    s_pending_ready = 0U;

    return HAL_OK;
}

/*==================== 初始化 DE/RE 方向控制脚 PA4 ====================*/
static void RS485_MB_DE_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin   = RS485_MB_DE_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RS485_MB_DE_GPIO_PORT, &GPIO_InitStruct);

    /* 默认接收模式 */
    HAL_GPIO_WritePin(RS485_MB_DE_GPIO_PORT, RS485_MB_DE_PIN, GPIO_PIN_RESET);
}

/*==================== 切换发送 ====================*/
static void RS485_MB_SetTxMode(void)
{
    HAL_GPIO_WritePin(RS485_MB_DE_GPIO_PORT, RS485_MB_DE_PIN, GPIO_PIN_SET);
}

/*==================== 切换接收 ====================*/
static void RS485_MB_SetRxMode(void)
{
    HAL_GPIO_WritePin(RS485_MB_DE_GPIO_PORT, RS485_MB_DE_PIN, GPIO_PIN_RESET);
}

/*==================== 开始 1 字节中断接收 ====================*/
HAL_StatusTypeDef RS485_MB_StartReceiveIT(void)
{
    if (s_huart == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Receive_IT(s_huart, (uint8_t *)&s_rx_byte, 1U);
}

/*==================== Modbus CRC16 ====================*/
uint16_t RS485_MB_CRC16(const uint8_t *data, uint16_t len)
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
        crc ^= (uint16_t)data[i];
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

/*==================== 读取 little-endian ====================*/
static uint16_t RS485_MB_ReadU16LE(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t RS485_MB_ReadU32LE(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void RS485_MB_WriteU16LE(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static void RS485_MB_WriteU32LE(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

/*==================== 复位接收状态机 ====================*/
static void RS485_MB_ResetParser(void)
{
    s_rx_len = 0U;
    s_rx_expect_len = 0U;
    s_last_rx_tick = HAL_GetTick();
}

/*==================== 接收回调 ====================*/
void RS485_MB_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((s_huart == NULL) || (huart == NULL))
    {
        return;
    }

    if (huart->Instance != s_huart->Instance)
    {
        return;
    }

    /* 若上一帧还没处理完，新来的完整帧直接丢弃，避免覆盖 */
    if (s_pending_ready == 0U)
    {
        if (s_rx_len < RS485_MB_FRAME_MAX_LEN)
        {
            s_rx_buf[s_rx_len++] = s_rx_byte;
            s_last_rx_tick = HAL_GetTick();

            /* 前 4 字节收够后，确定整帧长度 */
            if (s_rx_len == 4U)
            {
                uint16_t payload_len = RS485_MB_ReadU16LE((const uint8_t *)&s_rx_buf[2]);

                if (payload_len > RS485_MB_RX_MAX_PAYLOAD)
                {
                    RS485_MB_ResetParser();
                }
                else
                {
                    s_rx_expect_len = (uint16_t)(payload_len + RS485_MB_FRAME_OVERHEAD);
                }
            }

            if ((s_rx_expect_len >= RS485_MB_FRAME_OVERHEAD) &&
                (s_rx_len == s_rx_expect_len))
            {
                RS485_MB_TryCompleteFrame();
            }
            else if (s_rx_len > RS485_MB_FRAME_MAX_LEN)
            {
                RS485_MB_ResetParser();
            }
        }
        else
        {
            RS485_MB_ResetParser();
        }
    }

    (void)HAL_UART_Receive_IT(s_huart, (uint8_t *)&s_rx_byte, 1U);
}

/*==================== 错误回调 ====================*/
void RS485_MB_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((s_huart == NULL) || (huart == NULL))
    {
        return;
    }

    if (huart->Instance != s_huart->Instance)
    {
        return;
    }

    RS485_MB_ResetParser();

    /* F1 上清错后重新挂接收即可 */
    __HAL_UART_CLEAR_PEFLAG(s_huart);

    (void)HAL_UART_Receive_IT(s_huart, (uint8_t *)&s_rx_byte, 1U);
}

/*==================== 尝试完成一帧 ====================*/
static void RS485_MB_TryCompleteFrame(void)
{
    uint16_t crc_calc;
    uint16_t crc_recv;
    uint16_t total_len;

    total_len = s_rx_expect_len;
    if (total_len < RS485_MB_FRAME_OVERHEAD)
    {
        RS485_MB_ResetParser();
        return;
    }

    crc_calc = RS485_MB_CRC16((const uint8_t *)s_rx_buf, (uint16_t)(total_len - 2U));
    crc_recv = RS485_MB_ReadU16LE((const uint8_t *)&s_rx_buf[total_len - 2U]);

    if (crc_calc == crc_recv)
    {
        /* 复制给主循环处理 */
        memcpy((void *)s_pending_buf, (const void *)s_rx_buf, total_len);
        s_pending_len = total_len;
        s_pending_ready = 1U;
    }

    /* 无论是否 CRC 正确，都复位解析器，等下一帧 */
    RS485_MB_ResetParser();
}

/*==================== 主循环轮询 ====================*/
void RS485_MB_Poll(void)
{
    uint8_t local_frame[RS485_MB_FRAME_MAX_LEN];
    uint16_t local_len;

    /* 帧间超时，防止半帧卡死 */
    if ((s_rx_len > 0U) &&
        ((HAL_GetTick() - s_last_rx_tick) > RS485_MB_INTERBYTE_TIMEOUT_MS))
    {
        RS485_MB_ResetParser();
    }

    if (s_pending_ready == 0U)
    {
        return;
    }

    __disable_irq();
    local_len = s_pending_len;
    if (local_len > RS485_MB_FRAME_MAX_LEN)
    {
        local_len = RS485_MB_FRAME_MAX_LEN;
    }
    memcpy(local_frame, (const void *)s_pending_buf, local_len);
    s_pending_ready = 0U;
    s_pending_len = 0U;
    __enable_irq();

    RS485_MB_ProcessFrame(local_frame, local_len);
}

/*==================== 处理一帧 ====================*/
static void RS485_MB_ProcessFrame(const uint8_t *frame, uint16_t frame_len)
{
    uint8_t addr;
    uint8_t cmd;
    uint16_t payload_len;
    const uint8_t *payload;
    uint8_t is_broadcast;
    uint8_t tx_payload[RS485_MB_TX_MAX_PAYLOAD];
    uint16_t tx_len = 0U;

    if ((frame == NULL) || (frame_len < RS485_MB_FRAME_OVERHEAD))
    {
        return;
    }

    addr        = frame[0];
    cmd         = frame[1];
    payload_len = RS485_MB_ReadU16LE(&frame[2]);
    payload     = &frame[4];

    if (frame_len != (uint16_t)(payload_len + RS485_MB_FRAME_OVERHEAD))
    {
        return;
    }

    is_broadcast = (addr == RS485_MB_BROADCAST_ADDR) ? 1U : 0U;

    /* 地址不匹配，直接丢弃 */
    if ((addr != RS485_MB_DEVICE_ADDR) && (is_broadcast == 0U))
    {
        return;
    }

    switch (cmd)
    {
        case RS485_MB_CMD_START:
        {
            if (payload_len != 0U)
            {
                tx_payload[0] = RS485_MB_RES_ERR_LEN;
                tx_len = 1U;
            }
            else
            {
                if (PulseOut_Start() == HAL_OK)
                {
                    tx_payload[0] = RS485_MB_RES_OK;
                    DisplayLogic_SetOn();
                }
                else
                {
                    tx_payload[0] = RS485_MB_RES_ERR_EXEC;
                }
                tx_len = 1U;
            }
        } break;

        case RS485_MB_CMD_STOP:
        {
            if (payload_len != 0U)
            {
                tx_payload[0] = RS485_MB_RES_ERR_LEN;
                tx_len = 1U;
            }
            else
            {
                if (PulseOut_Stop() == HAL_OK)
                {
                    tx_payload[0] = RS485_MB_RES_OK;
                    DisplayLogic_SetOff();
                }
                else
                {
                    tx_payload[0] = RS485_MB_RES_ERR_EXEC;
                }
                tx_len = 1U;
            }
        } break;

        case RS485_MB_CMD_SET_US:
        {
            uint32_t pulse_us;
            uint32_t period_us;

            if (payload_len != 8U)
            {
                tx_payload[0] = RS485_MB_RES_ERR_LEN;
                tx_len = 1U;
            }
            else
            {
                pulse_us  = RS485_MB_ReadU32LE(&payload[0]);
                period_us = RS485_MB_ReadU32LE(&payload[4]);

                if ((pulse_us == 0U) || (period_us == 0U) || (pulse_us > period_us))
                {
                    tx_payload[0] = RS485_MB_RES_ERR_PARAM;
                    tx_len = 1U;
                }
                else
                {
                    if (PulseOut_SetUs(pulse_us, period_us) == HAL_OK)
                    {
                        tx_payload[0] = RS485_MB_RES_OK;
                        DisplayLogic_SetConfig(DISP_UNIT_US, pulse_us, period_us);
                    }
                    else
                    {
                        tx_payload[0] = RS485_MB_RES_ERR_EXEC;
                    }
                    tx_len = 1U;
                }
            }
        } break;

        case RS485_MB_CMD_SET_MS:
        {
            uint32_t pulse_ms;
            uint32_t period_ms;

            if (payload_len != 8U)
            {
                tx_payload[0] = RS485_MB_RES_ERR_LEN;
                tx_len = 1U;
            }
            else
            {
                pulse_ms  = RS485_MB_ReadU32LE(&payload[0]);
                period_ms = RS485_MB_ReadU32LE(&payload[4]);

                if ((pulse_ms == 0U) || (period_ms == 0U) || (pulse_ms > period_ms))
                {
                    tx_payload[0] = RS485_MB_RES_ERR_PARAM;
                    tx_len = 1U;
                }
                else
                {
                    if (PulseOut_SetMs(pulse_ms, period_ms) == HAL_OK)
                    {
                        tx_payload[0] = RS485_MB_RES_OK;
                        DisplayLogic_SetConfig(DISP_UNIT_MS, pulse_ms, period_ms);
                    }
                    else
                    {
                        tx_payload[0] = RS485_MB_RES_ERR_EXEC;
                    }
                    tx_len = 1U;
                }
            }
        } break;

        case RS485_MB_CMD_GET_STATUS:
{
    if (payload_len != 0U)
    {
        tx_payload[0] = RS485_MB_RES_ERR_LEN;
        tx_len = 1U;
    }
    else
    {
        PulseOut_Status_t st;

        PulseOut_GetStatus(&st);

        /*
         * 返回格式：
         * [result][running][pulse_us(4)][period_us(4)]
         * 共 10 字节
         */
        tx_payload[0] = RS485_MB_RES_OK;
        tx_payload[1] = st.running;
        RS485_MB_WriteU32LE(&tx_payload[2], st.pulse_us);
        RS485_MB_WriteU32LE(&tx_payload[6], st.period_us);
        tx_len = 10U;
    }
} break;

        default:
        {
            tx_payload[0] = RS485_MB_RES_ERR_CMD;
            tx_len = 1U;
        } break;
    }

    /*
     * 广播帧只执行，不回包
     * 建议：
     *   广播只用于 START / STOP / SET_US / SET_MS
     *   GET_STATUS 不要用广播
     */
    if (is_broadcast != 0U)
    {
        return;
    }

    RS485_MB_SendResponse(addr, (uint8_t)(cmd | 0x80U), tx_payload, tx_len);
}

/*==================== 发送应答帧 ====================*/
static void RS485_MB_SendResponse(uint8_t addr, uint8_t cmd, const uint8_t *payload, uint16_t payload_len)
{
    uint8_t tx_buf[RS485_MB_TX_MAX_PAYLOAD + RS485_MB_FRAME_OVERHEAD];
    uint16_t total_len;
    uint16_t crc;

    if (s_huart == NULL)
    {
        return;
    }

    if (payload_len > RS485_MB_TX_MAX_PAYLOAD)
    {
        return;
    }

    tx_buf[0] = addr;
    tx_buf[1] = cmd;
    RS485_MB_WriteU16LE(&tx_buf[2], payload_len);

    if ((payload != NULL) && (payload_len > 0U))
    {
        memcpy(&tx_buf[4], payload, payload_len);
    }

    total_len = (uint16_t)(payload_len + RS485_MB_FRAME_OVERHEAD);

    crc = RS485_MB_CRC16(tx_buf, (uint16_t)(total_len - 2U));
    RS485_MB_WriteU16LE(&tx_buf[total_len - 2U], crc);

    RS485_MB_SetTxMode();

    (void)HAL_UART_Transmit(s_huart, tx_buf, total_len, 1000U);

    while (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_TC) == RESET)
    {
    }

    RS485_MB_SetRxMode();
}