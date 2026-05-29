#include "rs485_control.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
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
static volatile char s_rx_line[RS485_CTRL_RX_LINE_MAX_LEN];
static volatile uint16_t s_rx_len = 0U;
static volatile uint8_t s_line_ready = 0U;
static char s_pending_line[RS485_CTRL_RX_LINE_MAX_LEN];

static void RS485_Control_SetTxMode(void);
static void RS485_Control_SetRxMode(void);
static void RS485_Control_SendText(const char *text);
static void RS485_Control_SendLine(const char *prefix, const char *text);
static void RS485_Control_ProcessLine(char *line);
static void RS485_Control_TriggerOutput(void);
static void RS485_Control_StopAuto(void);
static void RS485_Control_StartAuto(void);
static void RS485_Control_FormatStatus(char *buf, uint16_t len);
static char *RS485_Control_Trim(char *s);
static int RS485_Control_TokenEquals(const char *a, const char *b);

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

    s_rx_len = 0U;
    s_line_ready = 0U;
    memset(s_pending_line, 0, sizeof(s_pending_line));

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

    if (s_line_ready == 0U)
    {
        if ((b == '\r') || (b == '\n'))
        {
            if (s_rx_len > 0U)
            {
                s_rx_line[s_rx_len] = '\0';
                memcpy(s_pending_line, (const void *)s_rx_line, s_rx_len + 1U);
                s_line_ready = 1U;
                s_rx_len = 0U;
            }
        }
        else if ((b >= 0x20U) && (b <= 0x7EU))
        {
            if (s_rx_len < (RS485_CTRL_RX_LINE_MAX_LEN - 1U))
            {
                s_rx_line[s_rx_len++] = (char)b;
            }
            else
            {
                s_rx_len = 0U;
            }
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

    s_rx_len = 0U;
    __HAL_UART_CLEAR_PEFLAG(s_huart);
    (void)HAL_UART_Receive_IT(s_huart, (uint8_t *)&s_rx_byte, 1U);
}

void RS485_Control_Poll(void)
{
    char line[RS485_CTRL_RX_LINE_MAX_LEN];
    uint32_t now = HAL_GetTick();

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

    if (s_line_ready == 0U)
    {
        return;
    }

    __disable_irq();
    strncpy(line, s_pending_line, sizeof(line) - 1U);
    line[sizeof(line) - 1U] = '\0';
    s_pending_line[0] = '\0';
    s_line_ready = 0U;
    __enable_irq();

    RS485_Control_ProcessLine(line);
}

static void RS485_Control_ProcessLine(char *line)
{
    char *cmd;
    char *arg;

    line = RS485_Control_Trim(line);
    if (*line == '\0')
    {
        return;
    }

    cmd = strtok(line, " \t");
    arg = strtok(NULL, " \t");

    if (cmd == NULL)
    {
        return;
    }

    if (RS485_Control_TokenEquals(cmd, "MODE") != 0)
    {
        if (arg == NULL)
        {
            RS485_Control_SendLine("ERR ", "MODE PARAM\r\n");
        }
        else if (RS485_Control_TokenEquals(arg, "AUTO") != 0)
        {
            s_state.mode = RS485_CTRL_MODE_AUTO;
            RS485_Control_SendLine("OK ", "MODE AUTO\r\n");
        }
        else if ((RS485_Control_TokenEquals(arg, "TRIGGER") != 0) ||
                 (RS485_Control_TokenEquals(arg, "TRIG") != 0))
        {
            RS485_Control_StopAuto();
            s_state.mode = RS485_CTRL_MODE_TRIGGER;
            RS485_Control_SendLine("OK ", "MODE TRIGGER\r\n");
        }
        else
        {
            RS485_Control_SendLine("ERR ", "MODE PARAM\r\n");
        }
    }
    else if (RS485_Control_TokenEquals(cmd, "INTERVAL") != 0)
    {
        uint32_t sec;

        if (arg == NULL)
        {
            RS485_Control_SendLine("ERR ", "INTERVAL PARAM\r\n");
            return;
        }

        sec = (uint32_t)strtoul(arg, NULL, 10);
        if ((sec == 0U) || (sec > 86400U))
        {
            RS485_Control_SendLine("ERR ", "INTERVAL RANGE\r\n");
            return;
        }

        s_state.interval_ms = sec * 1000U;
        RS485_Control_SendLine("OK ", "INTERVAL\r\n");
    }
    else if (RS485_Control_TokenEquals(cmd, "PULSE") != 0)
    {
        uint32_t ms;

        if (arg == NULL)
        {
            RS485_Control_SendLine("ERR ", "PULSE PARAM\r\n");
            return;
        }

        ms = (uint32_t)strtoul(arg, NULL, 10);
        if ((ms == 0U) || (ms > 60000U))
        {
            RS485_Control_SendLine("ERR ", "PULSE RANGE\r\n");
            return;
        }

        s_state.pulse_width_ms = ms;
        RS485_Control_SendLine("OK ", "PULSE\r\n");
    }
    else if (RS485_Control_TokenEquals(cmd, "START") != 0)
    {
        if (s_state.mode != RS485_CTRL_MODE_AUTO)
        {
            RS485_Control_SendLine("ERR ", "NOT AUTO\r\n");
        }
        else
        {
            RS485_Control_StartAuto();
            RS485_Control_SendLine("OK ", "START\r\n");
        }
    }
    else if (RS485_Control_TokenEquals(cmd, "STOP") != 0)
    {
        RS485_Control_StopAuto();
        RS485_Control_SendLine("OK ", "STOP\r\n");
    }
    else if ((RS485_Control_TokenEquals(cmd, "TRIG") != 0) ||
             (RS485_Control_TokenEquals(cmd, "TRIGGER") != 0))
    {
        if (s_state.mode != RS485_CTRL_MODE_TRIGGER)
        {
            RS485_Control_SendLine("ERR ", "NOT TRIGGER MODE\r\n");
        }
        else if (s_state.pulse_active != 0U)
        {
            RS485_Control_SendLine("ERR ", "BUSY\r\n");
        }
        else
        {
            RS485_Control_TriggerOutput();
            RS485_Control_SendLine("OK ", "TRIG\r\n");
        }
    }
    else if (RS485_Control_TokenEquals(cmd, "STATUS") != 0)
    {
        char status[96];

        RS485_Control_FormatStatus(status, sizeof(status));
        RS485_Control_SendText(status);
    }
    else
    {
        RS485_Control_SendLine("ERR ", "CMD\r\n");
    }
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

static void RS485_Control_FormatStatus(char *buf, uint16_t len)
{
    const char *mode = (s_state.mode == RS485_CTRL_MODE_AUTO) ? "AUTO" : "TRIGGER";
    uint32_t interval_sec = s_state.interval_ms / 1000U;

    (void)snprintf(buf, len,
                   "STATUS MODE=%s RUN=%u INTERVAL=%lus PULSE=%lums BUSY=%u\r\n",
                   mode,
                   (unsigned int)s_state.auto_running,
                   (unsigned long)interval_sec,
                   (unsigned long)s_state.pulse_width_ms,
                   (unsigned int)s_state.pulse_active);
}

static void RS485_Control_SetTxMode(void)
{
    HAL_GPIO_WritePin(RS485_CTRL_DE_GPIO_PORT, RS485_CTRL_DE_PIN, GPIO_PIN_SET);
}

static void RS485_Control_SetRxMode(void)
{
    HAL_GPIO_WritePin(RS485_CTRL_DE_GPIO_PORT, RS485_CTRL_DE_PIN, GPIO_PIN_RESET);
}

static void RS485_Control_SendText(const char *text)
{
    uint16_t len;

    if ((s_huart == NULL) || (text == NULL))
    {
        return;
    }

    len = (uint16_t)strlen(text);

    RS485_Control_SetTxMode();
    (void)HAL_UART_Transmit(s_huart, (uint8_t *)text, len, 1000U);

    while (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_TC) == RESET)
    {
    }

    RS485_Control_SetRxMode();
}

static void RS485_Control_SendLine(const char *prefix, const char *text)
{
    char buf[RS485_CTRL_RX_LINE_MAX_LEN + 8U];

    if ((prefix == NULL) || (text == NULL))
    {
        return;
    }

    (void)snprintf(buf, sizeof(buf), "%s%s", prefix, text);
    RS485_Control_SendText(buf);
}

static char *RS485_Control_Trim(char *s)
{
    char *end;

    while ((*s != '\0') && isspace((unsigned char)*s))
    {
        s++;
    }

    end = s + strlen(s);
    while ((end > s) && isspace((unsigned char)*(end - 1)))
    {
        end--;
    }
    *end = '\0';

    return s;
}

static int RS485_Control_TokenEquals(const char *a, const char *b)
{
    while ((*a != '\0') && (*b != '\0'))
    {
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b))
        {
            return 0;
        }

        a++;
        b++;
    }

    return ((*a == '\0') && (*b == '\0')) ? 1 : 0;
}
