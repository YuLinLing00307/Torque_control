#ifndef __APP_NJ70_H
#define __APP_NJ70_H

#include "main.h"

#include "dev_nj70.h"
#include "com_filter.h"

// 向力矩传感器发送命令的缓冲区
#define MAX_TX_BUFFER_SIZE 50
// 力传感器接收缓冲区
#define MAX_RX_BUFFER_SIZE 50

extern uint8_t tx_buffer[MAX_TX_BUFFER_SIZE];
extern uint8_t rx_buffer[MAX_RX_BUFFER_SIZE];
extern _Bool torque_first_flag;
extern float torque;
extern float torque_after_comp;

void App_NJ70_Task(void);

#endif

