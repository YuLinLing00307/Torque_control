#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"

void Bsp_Can_Init(void);

/* 返回：0 成功 / 1 错误 / 2 忙 / 3 邮箱一直不空（超时） */
uint8_t Bsp_Can_SendData(uint32_t type, uint32_t ID, uint8_t *msgData, uint8_t num);

#endif
