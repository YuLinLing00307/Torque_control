#ifndef __JOINT_H
#define __JOINT_H

#include "main.h"
#include "joint_od.h"

/* =====================================================================
 *  MXBS-14 关节模组 · CANopen(CiA402) 协议层
 *  依赖：Drivers/BSP/bsp_can.c 的 Bsp_Can_SendData()
 *        HAL_CAN_RxFifo0MsgPendingCallback 里调用 Joint_RxDispatch()
 *        主循环里周期调用 Joint_Task()
 * ===================================================================== */

/* SDO 事务状态 */
typedef enum
{
    JOINT_SDO_IDLE = 0,     /* 空闲，可以发起新事务 */
    JOINT_SDO_WAIT,         /* 已发出，等应答 */
    JOINT_SDO_OK,           /* 成功（读到的值在 Joint_SdoPoll 的 *value） */
    JOINT_SDO_ABORT,        /* 从站返回 SDO abort，错误码见 Joint_SdoAbortCode() */
    JOINT_SDO_TIMEOUT       /* 超时无应答 */
} JointSdoState;

/* 运行模式（CiA402 0x6060） */
#define JOINT_MODE_PP       1
#define JOINT_MODE_PV       3
#define JOINT_MODE_TQ       4
#define JOINT_MODE_IP       7
#define JOINT_MODE_CSP      8
#define JOINT_MODE_CSV      9
#define JOINT_MODE_CST      10

/* 企业文档 0x6061 实测支持的模式（比 0x6502 的位图更准） */
#define JOINT_MODE_FORCE    11      /* 力控模式 */
#define JOINT_MODE_ANALOG_V 22      /* 模拟量控速度 */
#define JOINT_MODE_ANALOG_T 23      /* 模拟量控力矩 */
#define JOINT_MODE_PULSE_P  31      /* 脉冲控位置 */
#define JOINT_MODE_PWM_V    42      /* PWM 控速度 */
#define JOINT_MODE_REPEAT   60      /* 往复位置模式 */
#define JOINT_MODE_MULTI    62      /* 多段运行模式 */

/* ---------- 初始化 ---------- */
void Joint_Init(uint8_t node_id);

/* ---------- 接收：在 CAN 接收中断里调用，只做分派，不打印 ---------- */
void Joint_RxDispatch(uint16_t std_id, const uint8_t *data, uint8_t dlc);

/* ---------- 周期任务：主循环或 1ms 任务里调用，负责 SDO 超时 ---------- */
void Joint_Task(void);

/* ---------- SDO 非阻塞接口 ----------
 * 用法：
 *     if (Joint_SdoState() == JOINT_SDO_IDLE) Joint_SdoRead(0x6064, 0);
 *     JointSdoState st = Joint_SdoPoll(&v);
 *     if (st == JOINT_SDO_OK) { 用 v }
 * 注意：JOINT_SDO_OK / ABORT / TIMEOUT 这三个终态只返回一次，读走后回到 IDLE
 */
uint8_t       Joint_SdoRead(uint16_t index, uint8_t sub);
uint8_t       Joint_SdoWrite(uint16_t index, uint8_t sub, uint32_t value, uint8_t width);
JointSdoState Joint_SdoPoll(uint32_t *value);
JointSdoState Joint_SdoState(void);
uint32_t      Joint_SdoAbortCode(void);
void          Joint_SetSdoTimeout(uint32_t ms);

/* ---------- SDO 阻塞接口：仅上电自检 / 调试用，不要放进控制环 ---------- */
uint8_t Joint_SdoReadBlock(uint16_t index, uint8_t sub, uint32_t *value, uint32_t timeout_ms);
uint8_t Joint_SdoWriteBlock(uint16_t index, uint8_t sub, uint32_t value, uint8_t width);

/* ---------- TPDO1 缓存 ----------
 * 默认映射：6041 状态字(16) + 60FD 数字输入(32) + 6061 当前模式(8)
 * 注意：位置不在默认 TPDO1 里，要单独用 SDO 读 0x6064，或改 TPDO 映射
 */
uint16_t Joint_Status(void);
uint32_t Joint_DigitalInputs(void);
uint8_t  Joint_ModeDisplay(void);
uint16_t Joint_Emcy(void);              /* 最近一次 EMCY 错误码，0 = 无 */

/* ---------- NMT: 链路探针 ----------
 * 用途：让驱动器主动回一帧 boot-up(0x700+节点号)，这一帧不依赖任何对象字典。
 * 只要收到 boot-up，就证明【波特率 / 接线 / CAN1_RX0 中断 / 节点号】全对。
 */
#define NMT_START_NODE           0x01u
#define NMT_STOP_NODE            0x02u
#define NMT_RESET_NODE           0x81u
#define NMT_RESET_COMM           0x82u

uint8_t  Joint_NmtResetNode(uint8_t node);   /* 会触发 boot-up 0x701 */
uint8_t  Joint_NmtStart(uint8_t node);       /* 进入 Operational */
uint32_t Joint_BootCount(void);              /* 收到的 boot-up 帧数 */

/* ---------- SYNC：CSP/CSV/CST 都靠它推动，必须由主站周期发 ---------- */
uint8_t Joint_Sync(void);

/* ---------- 统计（调试用） ---------- */
uint32_t Joint_RxCount(void);
uint32_t Joint_SdoTimeoutCount(void);
uint32_t Joint_TxFailCount(void);

#endif
