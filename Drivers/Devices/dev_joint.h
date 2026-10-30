#ifndef __DEV_JOINT_H
#define __DEV_JOINT_H

#include "main.h"
#include "dev_joint_od.h"
#include "bsp_can.h"

/* =====================================================================
 *  MXBS-14 关节模组 · CANopen(CiA402) 协议层
 *  依赖：Drivers/BSP/bsp_can.c 的 Bsp_Can_SendData()
 *        HAL_CAN_RxFifo0MsgPendingCallback 里调用 Dev_Joint_RxDispatch()
 *        主循环里周期调用 Dev_Joint_Task()
 * ===================================================================== */

/* SDO 事务状态 */
typedef enum
{
    JOINT_SDO_IDLE = 0,     /* 空闲，可以发起新事务 */
    JOINT_SDO_WAIT,         /* 已发出，等应答 */
    JOINT_SDO_OK,           /* 成功（读到的值在 Dev_Joint_SdoPoll 的 *value） */
    JOINT_SDO_ABORT,        /* 从站返回 SDO abort，错误码见 Dev_Joint_SdoAbortCode() */
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
void Dev_Joint_Init(uint8_t node_id);

/* ---------- 接收：在 CAN 接收中断里调用，只做分派，不打印 ---------- */
void Dev_Joint_RxDispatch(uint16_t std_id, const uint8_t *data, uint8_t dlc);

/* ---------- 周期任务：主循环或 1ms 任务里调用，负责 SDO 超时 ---------- */
void Dev_Joint_Task(void);

/* ---------- SDO 非阻塞接口 ----------
 * 用法：
 *     if (Dev_Joint_SdoState() == JOINT_SDO_IDLE) Dev_Joint_SdoRead(0x6064, 0);
 *     JointSdoState st = Dev_Joint_SdoPoll(&v);
 *     if (st == JOINT_SDO_OK) { 用 v }
 * 注意：JOINT_SDO_OK / ABORT / TIMEOUT 这三个终态只返回一次，读走后回到 IDLE
 */
uint8_t       Dev_Joint_SdoRead(uint16_t index, uint8_t sub);
uint8_t       Dev_Joint_SdoWrite(uint16_t index, uint8_t sub, uint32_t value, uint8_t width);
JointSdoState Dev_Joint_SdoPoll(uint32_t *value);
JointSdoState Dev_Joint_SdoState(void);
uint32_t      Dev_Joint_SdoAbortCode(void);
void          Dev_Joint_SdoReset(void);         /* 强制把通道打回 IDLE */
uint32_t      Dev_Joint_SdoGhostCnt(void);      /* 被丢弃的迟到/错序应答数 */
void          Dev_Joint_SetSdoTimeout(uint32_t ms);

/* ---------- SDO 阻塞接口：仅上电自检 / 调试用，不要放进控制环 ---------- */
uint8_t Dev_Joint_SdoReadBlock(uint16_t index, uint8_t sub, uint32_t *value, uint32_t timeout_ms);
uint8_t Dev_Joint_SdoWriteBlock(uint16_t index, uint8_t sub, uint32_t value, uint8_t width);

/* ---------- TPDO1 缓存 ----------
 * 默认映射：6041 状态字(16) + 60FD 数字输入(32) + 6061 当前模式(8)
 * 注意：位置不在默认 TPDO1 里，要单独用 SDO 读 0x6064，或改 TPDO 映射
 */
uint16_t Dev_Joint_Status(void);
uint32_t Dev_Joint_DigitalInputs(void);
uint8_t  Dev_Joint_ModeDisplay(void);
uint16_t Dev_Joint_Emcy(void);              /* 最近一次 EMCY 错误码，0 = 无 */
uint32_t Dev_Joint_EmcyCount(void);         /* EMCY 帧数：给"故障发生时刻"打时间戳 */

/* ---------- NMT: 链路探针 ----------
 * 用途：让驱动器主动回一帧 boot-up(0x700+节点号)，这一帧不依赖任何对象字典。
 * 只要收到 boot-up，就证明【波特率 / 接线 / CAN1_RX0 中断 / 节点号】全对。
 */
#define NMT_PREOP_NODE           0x80u     /* 退回 Pre-operational，改 PDO 映射前用 */
#define NMT_START_NODE           0x01u
#define NMT_STOP_NODE            0x02u
#define NMT_RESET_NODE           0x81u
#define NMT_RESET_COMM           0x82u

uint8_t  Dev_Joint_NmtResetNode(uint8_t node);   /* 会触发 boot-up 0x701 */
uint8_t  Dev_Joint_NmtStart(uint8_t node);       /* 进入 Operational */
uint8_t  Dev_Joint_NmtPreOp(uint8_t node);       /* 退回 Pre-operational（改 PDO 映射前必须）*/
uint32_t Dev_Joint_BootCount(void);              /* 收到的 boot-up 帧数 */
/* 心跳里带回的 NMT 状态：5 = Operational，127 = Pre-operational，0xFF = 还没收到过。
 * PDO 只在 Operational 下生效，这是判断 RPDO 能不能用的唯一直接证据。 */
uint8_t  Dev_Joint_NmtState(void);

/* ---------- SYNC：CSP/CSV/CST 都靠它推动，必须由主站周期发 ---------- */
uint8_t Dev_Joint_Sync(void);

/* TPDO1 改映射成 {0x6041, 0x6064}，让位置走 PDO 而不是 SDO 轮询。
 * 必须在 NMT Start 之前调用（Pre-operational 下才能改映射）。
 * 成功返回 1，之后用 Dev_Joint_TpdoGetPos() 零开销取位置。 */
uint8_t  Dev_Joint_Tpdo1SetupPos(void);
int32_t  Dev_Joint_TpdoGetPos(void);
uint32_t Dev_Joint_TpdoPosCount(void);

/* ---------- 统计（调试用） ---------- */
uint32_t Dev_Joint_RxCount(void);
uint32_t Dev_Joint_SdoTimeoutCount(void);
uint32_t Dev_Joint_TxFailCount(void);
uint32_t Dev_Joint_Tpdo1Count(void);    /* 收到的 TPDO1 帧数 */

/* ---------- 位置读取（语义封装） ----------
 * APP 层不要直接碰 0x6064 / SDO。
 *   Dev_Joint_ReadPos()  发一次 SDO 读位置（阻塞约 1~2 ms），成功则更新缓存并返回 1
 * 位置反馈已经全走 TPDO1（Dev_Joint_TpdoGetPos），这个函数只在调试/自检里用，
 * 100 Hz 导纳环里绝不调用。
 */
uint8_t Dev_Joint_ReadPos(int32_t *pos);

/* 冻结 ReadPos：置 1 后只返回缓存，不再发 SDO。
 * 100 Hz 控制环独占 SDO 通道时必须打开，否则 20 Hz 的位置读会插进来抢通道。 */
void Dev_Joint_SetPosFreeze(uint8_t on);

#endif
