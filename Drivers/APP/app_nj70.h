#ifndef __APP_NJ70_H
#define __APP_NJ70_H

#include "main.h"
#include "dev_nj70.h"
#include "com_filter.h"

#define MAX_TX_BUFFER_SIZE  50
#define MAX_RX_BUFFER_SIZE  50

extern uint8_t tx_buffer[MAX_TX_BUFFER_SIZE];
extern uint8_t rx_buffer[MAX_RX_BUFFER_SIZE];
extern _Bool   torque_first_flag;
extern float   torque;              /* 原始力矩 N.m，已一阶低通 */
extern float   torque_after_comp;   /* 扣掉重力矩后的外力矩 N.m */

void App_NJ70_Task(void);

#endif
