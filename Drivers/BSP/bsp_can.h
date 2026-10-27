#ifndef __CAN_H
#define __CAN_H

#include "main.h"

void Bsp_Can_Init(void);
uint8_t Bsp_Can_SendData(uint32_t type, uint32_t ID,uint8_t *msgData,uint8_t num);

#endif
