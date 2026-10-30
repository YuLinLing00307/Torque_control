#ifndef __RS485_H
#define __RS485_H

#include "main.h"

#include "com_crc.h"

#define QUICK_CMD_SIZE 7

#define RS485_RE(x) do{ (x) ? \
(HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET)) : \
(HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET)); \
}while(0)

void Bsp_rs485_tx(uint8_t *buf, uint8_t len);


#endif
