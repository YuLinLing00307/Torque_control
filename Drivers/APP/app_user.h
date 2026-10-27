#ifndef __USER_H
#define __USER_H

#include "main.h"

#include "bsp_rs485.h"
#include "bsp_can.h"
#include "joint.h"
#include "com_filter.h"

// 向力矩传感器发送命令的缓冲区
#define MAX_TX_BUFFER_SIZE 50
// 力传感器接收缓冲区
#define MAX_RX_BUFFER_SIZE 50

extern uint8_t tx_buffer[MAX_TX_BUFFER_SIZE];
extern uint8_t rx_buffer[MAX_RX_BUFFER_SIZE];

void tx_proc(void);
void rx_proc(void);
void led_proc(void);
void test_proc(void);

#endif
