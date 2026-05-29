#ifndef RS485_CONTROL_H
#define RS485_CONTROL_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RS485_CTRL_DE_GPIO_PORT          GPIOA
#define RS485_CTRL_DE_PIN                GPIO_PIN_4

#define TRIGGER_OUT_GPIO_PORT            GPIOA
#define TRIGGER_OUT_PIN                  GPIO_PIN_5

#define TRIGGER_OE_GPIO_PORT             GPIOA
#define TRIGGER_OE_PIN                   GPIO_PIN_6
#define TRIGGER_OE_ACTIVE_LEVEL          GPIO_PIN_RESET

#define RS485_CTRL_RX_LINE_MAX_LEN       64U
#define RS485_CTRL_DEFAULT_PULSE_MS      100U
#define RS485_CTRL_DEFAULT_INTERVAL_SEC  5U

typedef enum
{
    RS485_CTRL_MODE_AUTO = 0,
    RS485_CTRL_MODE_TRIGGER = 1
} RS485_ControlMode_t;

HAL_StatusTypeDef RS485_Control_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef RS485_Control_StartReceiveIT(void);

void RS485_Control_Poll(void);
void RS485_Control_RxCpltCallback(UART_HandleTypeDef *huart);
void RS485_Control_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* RS485_CONTROL_H */
