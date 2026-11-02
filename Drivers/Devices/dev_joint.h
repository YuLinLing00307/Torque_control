#ifndef __DEV_JOINT_H
#define __DEV_JOINT_H

#include "main.h"
#include "dev_joint_od.h"
#include "bsp_can.h"

/* =====================================================================
 *  MXBS-14 关节模组 · CANopen(CiA402) 协议层
 *
 *  接线：
 *      发送      Bsp_Can_SendData()            （BSP/bsp_can.c）
 *      接收      Dev_Joint_RxDispatch()        （CAN RX 中断里调用）
 *      周期      Dev_Joint_Task()              （主循环或 1ms 任务）
 * ===================================================================== */

/* ================ 一、运行模式（写 0x6060，读 0x6061） ================ */
#define JOINT_MODE_PP       1       /* Profile Position，本项目唯一的落点 */
#define JOINT_MODE_IP       7       /* Interpolated Position */
#define JOINT_MODE_CSP      8       /* Cyclic Sync Position */
#define JOINT_MODE_CSV      9       /* Cyclic Sync Velocity */
#define JOINT_MODE_CST      10      /* Cyclic Sync Torque */
/* 企业文档另有 11 力控 / 22 模拟量速度 / 23 模拟量力矩 / 31 脉冲位置 /
 * 42 PWM 速度 / 60 往复 / 62 多段，本项目用不到。
 * 实测：CSP / CSV / IP 在这台驱动器上全都跑不了，只有 PP 可用 —— 见《导纳控制调参记录.md》。 */

/* ================ 二、NMT 命令（另有 0x02 停止 / 0x82 复位通信） ================ */
#define NMT_START_NODE      0x01u   /* 进 Operational，PDO 才开始生效 */
#define NMT_PREOP_NODE      0x80u   /* 退回 Pre-operational，改 PDO 映射前必须 */
#define NMT_RESET_NODE      0x81u   /* 复位节点，会回一帧 boot-up */

/* ================ 三、SDO 事务状态 ================ */
typedef enum
{
    JOINT_SDO_IDLE = 0,     /* 空闲，可以发起新事务 */
    JOINT_SDO_WAIT,         /* 已发出，等应答 */
    JOINT_SDO_OK,           /* 成功（读到的值从 Dev_Joint_SdoPoll 的 *value 取） */
    JOINT_SDO_ABORT,        /* 从站回了 SDO abort，错误码见 Dev_Joint_SdoAbortCode() */
    JOINT_SDO_TIMEOUT       /* 超时无应答 */
} JointSdoState;

/* ================ 四、初始化 / 接收 / 周期 ================ */
void Dev_Joint_Init(uint8_t node_id);

/* CAN 接收中断里调用，只做分派，不打印、不阻塞 */
void Dev_Joint_RxDispatch(uint16_t std_id, const uint8_t *data, uint8_t dlc);

/* 周期任务，负责 SDO 超时判定 */
void Dev_Joint_Task(void);

/* ================ 五、SDO 非阻塞接口（控制环用这个） ================
 *     if (Dev_Joint_SdoState() == JOINT_SDO_IDLE) Dev_Joint_SdoRead(0x6064, 0);
 *     if (Dev_Joint_SdoPoll(&v) == JOINT_SDO_OK) { 用 v }
 *
 * 铁律：单通道。必须 IDLE 才能发下一条；OK/ABORT/TIMEOUT 三个终态只返回一次，
 *       读走后自动回 IDLE。写确认(0x60)和读应答都置 OK，但写确认时 value = 0，
 *       所以要靠"我发起过读"标志区分，别把 0 当成位置反馈。
 */
uint8_t       Dev_Joint_SdoRead(uint16_t index, uint8_t sub);
uint8_t       Dev_Joint_SdoWrite(uint16_t index, uint8_t sub, uint32_t value, uint8_t width);
JointSdoState Dev_Joint_SdoPoll(uint32_t *value);
JointSdoState Dev_Joint_SdoState(void);
uint32_t      Dev_Joint_SdoAbortCode(void);
void          Dev_Joint_SdoReset(void);         /* 强制把通道打回 IDLE */
uint32_t      Dev_Joint_SdoGhostCnt(void);      /* 被丢弃的迟到 / 错序应答数 */
void          Dev_Joint_SetSdoTimeout(uint32_t ms);

/* 阻塞版：上电自检、调试用，绝不能放进控制环 */
uint8_t Dev_Joint_SdoReadBlock(uint16_t index, uint8_t sub, uint32_t *value, uint32_t timeout_ms);
uint8_t Dev_Joint_SdoWriteBlock(uint16_t index, uint8_t sub, uint32_t value, uint8_t width);

/* ================ 六、TPDO1 ================
 * 出厂映射：0x6041 状态字(16) + 0x60FD 数字输入(32) + 0x6061 当前模式(8)
 * 位置默认不在里面，下面 Dev_Joint_Tpdo1SetupPos() 会改成 {0x6041, 0x6064}。
 */
uint16_t Dev_Joint_Status(void);
uint32_t Dev_Joint_DigitalInputs(void);
uint8_t  Dev_Joint_ModeDisplay(void);
uint16_t Dev_Joint_Emcy(void);              /* 最近一次 EMCY 错误码，0 = 无 */
uint32_t Dev_Joint_EmcyCount(void);         /* EMCY 帧数，给故障发生时刻打时间戳 */

/* 改映射成 {0x6041, 0x6064}：位置改走 PDO，运行期不用再发 SDO 轮询。
 * 必须在 NMT Start 之前调用（只有 Pre-operational 下能改映射）。成功返回 1。 */
uint8_t  Dev_Joint_Tpdo1SetupPos(void);
int32_t  Dev_Joint_TpdoGetPos(void);        /* 零开销取最新位置快照 */
uint32_t Dev_Joint_TpdoPosCount(void);      /* 已收到的位置帧数，用来判链路死活 */

/* ================ 七、NMT ================
 * 链路探针：让驱动器回一帧 boot-up(0x700+节点号)，这一帧不依赖任何对象字典。
 * 只要收到 boot-up，就证明 波特率 / 接线 / CAN1_RX0 中断 / 节点号 全对。
 */
uint8_t  Dev_Joint_NmtStart(uint8_t node);
uint8_t  Dev_Joint_NmtPreOp(uint8_t node);
uint8_t  Dev_Joint_NmtResetNode(uint8_t node);
uint32_t Dev_Joint_BootCount(void);
/* 心跳里带回的 NMT 状态：5 = Operational，127 = Pre-operational，0xFF = 还没收到。
 * PDO 只在 Operational 下生效，这是判断 RPDO 通没通的唯一直接证据。 */
uint8_t  Dev_Joint_NmtState(void);

/* ================ 八、位置读取（语义封装） ================
 * APP 层不要直接碰 0x6064 / SDO。
 * 位置反馈已经全走 TPDO1（Dev_Joint_TpdoGetPos），这个函数只在调试 / 自检里用。
 */
uint8_t Dev_Joint_ReadPos(int32_t *pos);    /* 发一次 SDO 读，阻塞约 1~2 ms */
void    Dev_Joint_SetPosFreeze(uint8_t on); /* 置 1 后只返回缓存，不再发 SDO */

/* ================ 九、统计 ================ */
uint32_t Dev_Joint_RxCount(void);
uint32_t Dev_Joint_SdoTimeoutCount(void);
uint32_t Dev_Joint_TxFailCount(void);
uint32_t Dev_Joint_Tpdo1Count(void);

#endif
