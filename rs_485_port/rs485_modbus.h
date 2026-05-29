#ifndef RS485_MODBUS_H
#define RS485_MODBUS_H

#include "stm32f1xx_hal.h"
#include "pulse_out.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==================== 用户可修改参数 ====================*/
#define RS485_MB_DEVICE_ADDR             0x01U
#define RS485_MB_BROADCAST_ADDR          0x00U

#define RS485_MB_DE_GPIO_PORT            GPIOA
#define RS485_MB_DE_PIN                  GPIO_PIN_4

#define RS485_MB_RX_MAX_PAYLOAD          32U
#define RS485_MB_TX_MAX_PAYLOAD          32U
#define RS485_MB_FRAME_OVERHEAD          6U   /* ADDR+CMD+LEN(2)+CRC(2) */
#define RS485_MB_FRAME_MAX_LEN           (RS485_MB_RX_MAX_PAYLOAD + RS485_MB_FRAME_OVERHEAD)

#define RS485_MB_INTERBYTE_TIMEOUT_MS    20U

/*==================== 命令字 ====================*/
#define RS485_MB_CMD_START               0x01U
#define RS485_MB_CMD_STOP                0x02U
#define RS485_MB_CMD_SET_US              0x03U
#define RS485_MB_CMD_SET_MS              0x04U
#define RS485_MB_CMD_GET_STATUS          0x05U

/*==================== 返回码 ====================*/
#define RS485_MB_RES_OK                  0x00U
#define RS485_MB_RES_ERR_CMD             0x01U
#define RS485_MB_RES_ERR_LEN             0x02U
#define RS485_MB_RES_ERR_PARAM           0x03U
#define RS485_MB_RES_ERR_EXEC            0x04U

/*==================== 对外接口 ====================*/

/*
 * 初始化 485 协议模块
 * 说明：
 * 1. huart 传 USART2 的句柄
 * 2. 该函数内部会初始化 PA4 为 485 DE/RE 方向控制脚
 * 3. UART2 的 PA2/PA3 初始化仍然放在 usart.c 里完成
 */
HAL_StatusTypeDef RS485_MB_Init(UART_HandleTypeDef *huart);

/* 开始 1 字节中断接收 */
HAL_StatusTypeDef RS485_MB_StartReceiveIT(void);

/*
 * 主循环里持续调用
 * 作用：
 * 1. 处理已收完整帧
 * 2. 处理帧间超时复位
 */
void RS485_MB_Poll(void);

/* 在 HAL_UART_RxCpltCallback() 里调用 */
void RS485_MB_RxCpltCallback(UART_HandleTypeDef *huart);

/* 在 HAL_UART_ErrorCallback() 里调用 */
void RS485_MB_ErrorCallback(UART_HandleTypeDef *huart);

/* Modbus CRC16 */
uint16_t RS485_MB_CRC16(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* RS485_MODBUS_H */