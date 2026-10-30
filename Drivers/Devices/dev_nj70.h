#ifndef __DEV_NJ70_H
#define __DEV_NJ70_H

#include "main.h"
#include "bsp_rs485.h"

/* 启动 RS485 的 IDLE+DMA 接收，缓冲区由调用者提供（APP 层持有） */
void Dev_NJ70_Init(uint8_t *rx_buf, uint16_t len);

/* 组快捷读指令（01 06）并发出去，一问一答模式 */
void Dev_NJ70_Send_Quick_Cmd(uint8_t* tx_buffer);

#endif
