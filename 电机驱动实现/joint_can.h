#ifndef __JOINT_CAN_H
#define __JOINT_CAN_H

#include "main.h"

/* 运行模式（CiA402） */
#define MODE_PP      1      /* 轮廓位置 */
#define MODE_VEL     3      /* 速度     */
#define MODE_TORQUE  4      /* 力矩     */
#define MODE_HOMING  6      /* 回零     */
#define MODE_CSP     8      /* 周期同步位置（轨迹/力控用） */
#define MODE_CSV     9      /* 周期同步速度 */
#define MODE_CST    10      /* 周期同步力矩 */

void    Joint_Init(CAN_HandleTypeDef *hcan, uint8_t node_id);

uint8_t Joint_SDO_Write(uint16_t index, uint8_t sub, uint32_t value, uint8_t width);
uint8_t Joint_SDO_Read(uint16_t index, uint8_t sub, uint32_t *out);

uint16_t Joint_GetStatus(void);
uint8_t  Joint_IsEnabled(void);
uint8_t  Joint_IsFaulted(void);
uint8_t  Joint_FaultReset(void);

uint8_t Joint_Enable(void);
void    Joint_Disable(void);
uint8_t Joint_SetMode(uint8_t mode);

int32_t  Joint_GetPosition(void);
int32_t  Joint_GetVelocity(void);

uint8_t Joint_MovePP(int32_t target, uint32_t profile_vel, uint32_t profile_acc);
uint8_t Joint_SetTargetPos(int32_t target);
uint8_t Joint_SetTargetVel(int32_t vel);

void    Joint_Heartbeat(void);

#endif
