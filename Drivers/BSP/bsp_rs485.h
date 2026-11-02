#ifndef __BSP_RS485_H
#define __BSP_RS485_H

#include "main.h"
#include "com_crc.h"

/* NJ70 快捷读指令帧长：01 06 + float(4 字节) + ADD8(1 字节) */
#define QUICK_CMD_SIZE  7

/* RE 引脚：1 = 发送，0 = 接收 */
#define RS485_RE(x) do{ (x) ? \
	(HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET)) : \
	(HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET)); \
}while(0)

void Bsp_rs485_tx(uint8_t *buf, uint8_t len);

#endif
