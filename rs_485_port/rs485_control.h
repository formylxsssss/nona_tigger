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

#define RS485_CTRL_HEADER_0              0xAAU
#define RS485_CTRL_HEADER_1              0x55U
#define RS485_CTRL_MAX_DATA_LEN          16U
#define RS485_CTRL_FRAME_OVERHEAD        5U
#define RS485_CTRL_MAX_FRAME_LEN         (RS485_CTRL_MAX_DATA_LEN + 2U + RS485_CTRL_FRAME_OVERHEAD)
#define RS485_CTRL_DEFAULT_PULSE_MS      100U
#define RS485_CTRL_DEFAULT_INTERVAL_SEC  5U

#define RS485_CTRL_CMD_SET_MODE          0x01U
#define RS485_CTRL_CMD_SET_INTERVAL      0x02U
#define RS485_CTRL_CMD_SET_PULSE         0x03U
#define RS485_CTRL_CMD_START             0x04U
#define RS485_CTRL_CMD_STOP              0x05U
#define RS485_CTRL_CMD_TRIGGER           0x06U
#define RS485_CTRL_CMD_GET_STATUS        0x07U

#define RS485_CTRL_MODE_VALUE_AUTO       0x00U
#define RS485_CTRL_MODE_VALUE_TRIGGER    0x01U

#define RS485_CTRL_RES_OK                0x00U
#define RS485_CTRL_RES_ERR_CMD           0x01U
#define RS485_CTRL_RES_ERR_LEN           0x02U
#define RS485_CTRL_RES_ERR_PARAM         0x03U
#define RS485_CTRL_RES_ERR_BUSY          0x04U
#define RS485_CTRL_RES_ERR_MODE          0x05U

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
