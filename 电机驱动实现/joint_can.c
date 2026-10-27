/* ============================================================
 *  MXBS-14 关节模组 · STM32 CANopen(CiA402) 最小驱动
 *  走 PIN3/PIN4 (CANL/CANH)，标准帧，1000kbps，节点 ID 可配
 *
 *  依赖：STM32 HAL (CAN 外设已由 CubeMX 初始化)
 *  用法：
 *      Joint_Init(&hcan1, 1);
 *      Joint_Enable();
 *      Joint_SetMode(MODE_CSP);
 *      Joint_SetTargetPos(0);
 *      主循环 1ms 调一次 Joint_Heartbeat()，负责 SYNC + 状态轮询
 * ============================================================ */
#include "joint_can.h"
#include <string.h>

/* ---------------- 内部状态 ---------------- */
static CAN_HandleTypeDef *s_hcan = NULL;
static uint8_t  s_node = 1;
static uint16_t s_status = 0;
static uint16_t s_last_err = 0;

/* ---------------- 底层：发一帧 ---------------- */
static void can_send(uint16_t id, const uint8_t *d, uint8_t dlc)
{
    CAN_TxHeaderTypeDef tx;
    uint32_t mailbox;
    tx.StdId = id;
    tx.ExtId = 0;
    tx.IDE   = CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = dlc;
    tx.TransmitGlobalTime = DISABLE;
    HAL_CAN_AddTxMessage(s_hcan, &tx, (uint8_t *)d, &mailbox);
}

/* ---------------- 底层：收一帧（超时不阻塞太久） ---------------- */
static uint8_t can_recv(uint16_t id, uint8_t *d, uint8_t *dlc, uint32_t timeout_ms)
{
    CAN_RxHeaderTypeDef rx;
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < timeout_ms) {
        if (HAL_CAN_GetRxFifoFillLevel(s_hcan, CAN_RX_FIFO0) > 0) {
            if (HAL_CAN_GetRxMessage(s_hcan, CAN_RX_FIFO0, &rx, d) == HAL_OK) {
                if (rx.IDE == CAN_ID_STD && rx.StdId == id) {
                    *dlc = rx.DLC;
                    return 1;
                }
                /* 不是自己等的帧，丢弃继续等 */
            }
        }
    }
    return 0;
}

/* ================= SDO ================= */

/* 写 4 字节以内：width = 1 / 2 / 4 */
uint8_t Joint_SDO_Write(uint16_t index, uint8_t sub, uint32_t value, uint8_t width)
{
    uint8_t d[8] = {0};
    uint8_t cmd, rsp[8], dlc = 0;

    switch (width) {
        case 1: cmd = 0x2F; break;
        case 2: cmd = 0x2B; break;
        case 3: cmd = 0x27; break;
        default: cmd = 0x23; width = 4; break;
    }
    d[0] = cmd;
    d[1] = index & 0xFF;
    d[2] = (index >> 8) & 0xFF;
    d[3] = sub;
    memcpy(&d[4], &value, width);

    can_send(0x600 + s_node, d, 8);
    if (!can_recv(0x580 + s_node, rsp, &dlc, 50)) return 0;
    return (rsp[0] == 0x60);          /* 0x60 = 写成功 */
}

uint8_t Joint_SDO_Read(uint16_t index, uint8_t sub, uint32_t *out)
{
    uint8_t d[8] = {0x40, 0, 0, 0, 0, 0, 0, 0};
    uint8_t rsp[8], dlc = 0;

    d[1] = index & 0xFF;
    d[2] = (index >> 8) & 0xFF;
    d[3] = sub;

    can_send(0x600 + s_node, d, 8);
    if (!can_recv(0x580 + s_node, rsp, &dlc, 50)) return 0;

    switch (rsp[0] & 0xE0) {
        case 0x40: *out = (uint32_t)rsp[4]; return 1;                       /* 1 字节 */
        case 0x60: *out = (uint32_t)(rsp[4] | (rsp[5] << 8)); return 1;     /* 2 字节 */
        default:
            if (rsp[0] == 0x80) { *out = 0; return 0; }                     /* SDO 异常 */
            *out = (uint32_t)rsp[4] | ((uint32_t)rsp[5] << 8) |
                   ((uint32_t)rsp[6] << 16) | ((uint32_t)rsp[7] << 24);
            return 1;
    }
}

/* ================= 接口层 ================= */

void Joint_Init(CAN_HandleTypeDef *hcan, uint8_t node_id)
{
    s_hcan = hcan;
    s_node = node_id;
}

uint16_t Joint_GetStatus(void)
{
    uint32_t v = 0;
    if (Joint_SDO_Read(0x6041, 0, &v)) s_status = (uint16_t)v;
    return s_status;
}

/* 状态字判断：用 &0x6F 掩码，和上位机 device_control.py 一致 */
uint8_t Joint_IsEnabled(void)  { return (Joint_GetStatus() & 0x6F) == 0x27; }
uint8_t Joint_IsFaulted(void)  { return (Joint_GetStatus() & 0x4F) == 0x08; }

uint8_t Joint_FaultReset(void)
{
    Joint_SDO_Write(0x6040, 0, 0x0080, 2);   /* bit7 上升沿 */
    HAL_Delay(50);
    return !Joint_IsFaulted();
}

/* 完整使能流程，返回 1 成功 */
uint8_t Joint_Enable(void)
{
    if (Joint_IsFaulted()) {
        if (!Joint_FaultReset()) return 0;
    }
    if (!Joint_SDO_Write(0x6040, 0, 0x0006, 2)) return 0;  /* Shutdown */
    HAL_Delay(20);
    if (!Joint_SDO_Write(0x6040, 0, 0x0007, 2)) return 0;  /* Switch on */
    HAL_Delay(20);
    if (!Joint_SDO_Write(0x6040, 0, 0x000F, 2)) return 0;  /* Enable operation */
    HAL_Delay(20);
    return Joint_IsEnabled();
}

void Joint_Disable(void)
{
    Joint_SDO_Write(0x6040, 0, 0x0006, 2);
}

uint8_t Joint_SetMode(uint8_t mode)
{
    Joint_SDO_Write(0x6060, 0, mode, 1);
    HAL_Delay(10);
    {   uint32_t v = 0;
        if (Joint_SDO_Read(0x6061, 0, &v)) return ((uint8_t)v == mode);
    }
    return 0;
}

int32_t Joint_GetPosition(void)
{
    uint32_t v = 0;
    return Joint_SDO_Read(0x6064, 0, &v) ? (int32_t)v : 0;
}

int32_t Joint_GetVelocity(void)
{
    uint32_t v = 0;
    return Joint_SDO_Read(0x606C, 0, &v) ? (int32_t)v : 0;
}

/* 位置模式(PP)：写目标位置 + bit4 上升沿触发 */
uint8_t Joint_MovePP(int32_t target, uint32_t profile_vel, uint32_t profile_acc)
{
    if (!Joint_SetMode(1)) return 0;
    Joint_SDO_Write(0x6081, 0, profile_vel, 4);   /* profile velocity    */
    Joint_SDO_Write(0x6083, 0, profile_acc, 4);   /* profile acceleration*/
    Joint_SDO_Write(0x607A, 0, (uint32_t)target, 4);
    Joint_SDO_Write(0x6040, 0, 0x000F, 2);        /* bit4 先清零 */
    Joint_SDO_Write(0x6040, 0, 0x001F, 2);        /* bit4 上升沿，开始运动 */
    return 1;
}

/* CSP：力控/轨迹跟随用，每个周期只更新目标位置，不重触发 */
uint8_t Joint_SetTargetPos(int32_t target)
{
    return Joint_SDO_Write(0x607A, 0, (uint32_t)target, 4);
}

/* 速度模式：默认 RPDO 映射里就是 60FF，这里用 SDO 也能写 */
uint8_t Joint_SetTargetVel(int32_t vel)
{
    return Joint_SDO_Write(0x60FF, 0, (uint32_t)vel, 4);
}

/* 1ms 周期调用：发 SYNC（默认 PDO 是同步传输，不发 SYNC 就不动） */
void Joint_Heartbeat(void)
{
    can_send(0x080, NULL, 0);
}
