#include "dev_joint.h"
#include <stdio.h>

/* SDO 应答命令字（请求命令字见 joint_od.h） */
#define SDO_SCS_WRITE_OK    0x60u
#define SDO_SCS_ABORT       0x80u

/* =====================================================================
 *  内部变量
 * ===================================================================== */
static uint8_t       s_node        = 1;
static JointSdoState s_sdo_state   = JOINT_SDO_IDLE;
static uint32_t      s_sdo_value   = 0;
static uint32_t      s_sdo_abort   = 0;
static uint32_t      s_sdo_t0      = 0;
static uint32_t      s_sdo_timeout = 50;
static uint16_t      s_sdo_index   = 0xFFFFu;  /* 当前在等的请求身份 */
static uint8_t       s_sdo_sub     = 0xFFu;
static uint32_t      s_sdo_ghost   = 0;        /* 被丢弃的迟到/错序应答数 */

static uint16_t s_status  = 0;
static uint32_t s_di      = 0;
static uint8_t  s_mode    = 0;
static uint16_t s_emcy    = 0;
/* EMCY 帧计数。驱动器一跳故障立刻发一帧 0x080+node，这是全总线唯一
 * "带时刻的故障通知"。APP 的 1 kHz 中断拿它当时间戳定格黑匣子，
 * 于是能拿到"故障发生前 512 ms"的现场，而不是 1 s 之后的一次抽查。 */
static volatile uint32_t s_emcy_cnt = 0;

static uint32_t s_rx_cnt      = 0;
static uint32_t s_boot_cnt    = 0;
static uint8_t  s_nmt_state   = 0xFFu;   /* 心跳里的 NMT 状态：5=Operational 127=Pre-op */
static uint32_t s_sdo_to_cnt  = 0;
static uint32_t s_tx_fail_cnt = 0;
static uint32_t s_tpdo1_cnt   = 0;   /* TPDO1 帧数：为 0 说明状态字通道没走通 */

/* =====================================================================
 *  SDO 发送
 * ===================================================================== */
static uint8_t sdo_send(uint8_t cmd, uint16_t index, uint8_t sub, uint32_t value)
{
    uint8_t d[8];

    d[0] = cmd;
    d[1] = (uint8_t)(index & 0xFFu);
    d[2] = (uint8_t)(index >> 8);
    d[3] = sub;
    d[4] = (uint8_t)(value);
    d[5] = (uint8_t)(value >> 8);
    d[6] = (uint8_t)(value >> 16);
    d[7] = (uint8_t)(value >> 24);

    if (Bsp_Can_SendData(CAN_ID_STD, COB_SDO_RX(s_node), d, 8) != HAL_OK)
    {
        s_tx_fail_cnt++;
        return 0;
    }

    s_sdo_abort = 0;
    s_sdo_t0    = HAL_GetTick();
    s_sdo_state = JOINT_SDO_WAIT;
    return 1;
}

/* =====================================================================
 *  SDO 应答处理（中断上下文）
 * ===================================================================== */
static void sdo_on_response(const uint8_t *d, uint8_t dlc)
{
    // d[0] 是 SDO 应答的第一个字节，也就是“命令字”，决定了这帧数据是什么结果
    uint8_t  cmd = d[0];
    uint16_t idx = (uint16_t)((uint16_t)d[1] | ((uint16_t)d[2] << 8));
    uint8_t  sub = d[3];

    /* 只认"当前真正在等的那条请求"的应答，两道关卡缺一不可：
     *   1) 没有请求在等（state != WAIT）-> 这帧是某种迟到的残响，丢掉；
     *   2) 索引/子索引对不上 -> 是别的请求的数据，丢掉。
     * 原来读应答分支两个都不查，抓到 0x4X 就无条件写 s_sdo_value。
     * 结果：一次超时之后迟到的应答会顶掉下一次读的位置值，
     * 主循环拿旧位置当新反馈，fdb 看上去"卡住"或"跳变"；
     * 0x6062 反复读出 -5.0e8 / 7.3e7 / 1.7e9 这种垃圾也是同一个原因。 */
    if (s_sdo_state != JOINT_SDO_WAIT || idx != s_sdo_index || sub != s_sdo_sub)
    {
        s_sdo_ghost++;
        return;
    }

    /* ---------- 情况一：写指令成功 ---------- */
    // 刚才我们向驱动器发了个写参数指令，它回个 0x60 表示“写好了”
    if (cmd == SDO_SCS_WRITE_OK)
    {
        /* 只有"这条写确实是我们在等的那条"才清值。
         * 原来是无条件 s_sdo_value = 0 —— 读应答刚把位置存进去，
         * 紧跟其后的写确认就把值冲成 0，主循环拿到 0 当位置反馈，
         * 看门狗立刻误判，位置年龄也永远不会更新（表现为"反馈断流"）。 */
        if (s_sdo_state == JOINT_SDO_WAIT)
        {
            s_sdo_value = 0;
            s_sdo_state = JOINT_SDO_OK;
        }
        return;
    }

    /* ---------- 情况二：读指令的应答 ---------- */
    if ((cmd & 0xE0u) == SDO_READ)
    {
        // 【神级操作】这行代码算出了这次读了几个字节。
        // 比如读了4字节，这2位就是0 (4-0=4)；读了2字节，这2位是2 (4-2=2)
        uint8_t  n = (uint8_t)(4u - ((cmd >> 2) & 0x03u));
        uint32_t v = 0;
        uint8_t  i;

        // 把 d[4] 到 d[4+n-1] 的有效数据，按照小端模式，拼成一个 32 位整数
        for (i = 0; i < n && i < 4u; i++)
            v |= (uint32_t)d[4 + i] << (8u * i);

        s_sdo_value = v; // 把拼好的数据存到全局变量里
        if (s_sdo_state == JOINT_SDO_WAIT) s_sdo_state = JOINT_SDO_OK; // 标记读取成功
        return;
    }

    /* ---------- 情况三：指令被拒绝（Abort） ---------- */
    // 0x80 表示 SDO 异常。比如你发了一个不存在的地址，或者格式不对
    if (cmd == SDO_SCS_ABORT)
    {
        // 把 d[4]~d[7] 拼起来，这是驱动器返回的“错误代码”，方便查手册
        s_sdo_abort = (uint32_t)d[4]
                    | ((uint32_t)d[5] << 8)
                    | ((uint32_t)d[6] << 16)
                    | ((uint32_t)d[7] << 24);
        s_sdo_value = 0;
        if (s_sdo_state == JOINT_SDO_WAIT) s_sdo_state = JOINT_SDO_ABORT; // 标记异常
        return;
    }
}

/* =====================================================================
 *  TPDO1 解析（中断上下文）
 *  默认映射：6041(16) + 60FD(32) + 6061(8)
 * ===================================================================== */
/* ---- TPDO1 的"位置布局" ----
 * 默认布局 {0x6041, 0x60FD, 0x6061} 里没有位置，位置只能 SDO 轮询 0x6064。
 * 把 0x1A00 改成 {0x6041, 0x6064} 以后，位置每帧跟状态字一起上来，
 * 运行期一条 SDO 都不用发 —— 这是排除"20 ms 一条 SDO 打断驱动器插补"
 * 这个变量唯一干净的办法。 */
static uint8_t  s_tpdo_pos_layout = 0u;
static volatile int32_t  s_tpdo_pos     = 0;
static volatile uint32_t s_tpdo_pos_cnt = 0u;

static void tpdo1_on_rx(const uint8_t *d, uint8_t dlc)
{
    s_tpdo1_cnt++;

    if (dlc >= 2u)
        s_status = (uint16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8));

    if (s_tpdo_pos_layout != 0u)
    {
        /* 位置布局：第 2..5 字节 = 0x6064（i32 小端） */
        if (dlc >= 6u)
        {
            s_tpdo_pos = (int32_t)((uint32_t)d[2]
                       | ((uint32_t)d[3] << 8)
                       | ((uint32_t)d[4] << 16)
                       | ((uint32_t)d[5] << 24));
            s_tpdo_pos_cnt++;
        }
        return;
    }

    if (dlc >= 6u)
        s_di = (uint32_t)d[2]
             | ((uint32_t)d[3] << 8)
             | ((uint32_t)d[4] << 16)
             | ((uint32_t)d[5] << 24);

    if (dlc >= 7u)
        s_mode = d[6];
}

/* =====================================================================
 *  对外接口
 * ===================================================================== */
void Dev_Joint_Init(uint8_t node_id)
{
    s_node        = node_id;
    s_sdo_state   = JOINT_SDO_IDLE;
    s_sdo_timeout = 50;
    s_status      = 0;
    s_di          = 0;
    s_mode        = 0;
    s_emcy        = 0;
    s_tpdo1_cnt   = 0;
}

void Dev_Joint_RxDispatch(uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
    s_rx_cnt++;

    if (std_id == COB_SDO_TX(s_node))
    {
        if (dlc >= 8u) sdo_on_response(data, dlc);
        return;
    }

    if (std_id == COB_TPDO1(s_node))
    {
        tpdo1_on_rx(data, dlc);
        return;
    }

    if (std_id == COB_EMCY(s_node))
    {
        if (dlc >= 2u) s_emcy = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
        s_emcy_cnt++;
        return;
    }

    if (std_id == COB_HEARTBEAT(s_node))
    {
        s_boot_cnt++;                 /* boot-up 或心跳，证明链路是通的 */
        /* 心跳第一字节是 NMT 状态，这是判断"PDO 到底能不能用"的唯一直接证据：
         *   0 = Boot-up   4 = Stopped   5 = Operational   127 = Pre-operational
         * Pre-operational 下 PDO 一律不处理（SDO 照常）。
         * 之前 TPDO1 全程 0 帧，就是因为节点一直停在 Pre-op，从没发过 NMT Start。 */
        if (dlc >= 1u) s_nmt_state = data[0];
        return;
    }

    /* 其他帧（NMT / 心跳）暂时忽略 */
}

void Dev_Joint_Task(void)
{
    /* 收帧走 CAN 中断 -> HAL_CAN_RxFifo0MsgPendingCallback -> Dev_Joint_RxDispatch，
       这里只做 SDO 超时检测，不碰 FIFO。 */
    if (s_sdo_state == JOINT_SDO_WAIT)
    {
        if ((HAL_GetTick() - s_sdo_t0) > s_sdo_timeout)
        {
            s_sdo_state = JOINT_SDO_TIMEOUT;
            s_sdo_to_cnt++;
        }
    }
}

uint8_t Dev_Joint_SdoRead(uint16_t index, uint8_t sub)
{
    /* 检查 + 占位必须原子：TIM3 中断(100 Hz) 和主循环都会调这里，
     * 中间被插进来的话两个请求会共用同一套静态缓冲，拼出半帧垃圾值写进对象字典。 */
    __disable_irq();
    /* 必须要求 IDLE 而不是"只要不是 WAIT"。
     * 终态 OK 意味着有一个结果还没被主循环取走；此时放行新请求，
     * 结果就被覆盖了 —— 读位置正是这样丢的。 */
    if (s_sdo_state != JOINT_SDO_IDLE)
    {
        __enable_irq();
        return 0;
    }
    s_sdo_index = index;                            /* 记下身份，应答要对得上 */
    s_sdo_sub   = sub;
    s_sdo_t0    = HAL_GetTick();                    /* 占位同时起表，避免被判超时 */
    s_sdo_state = JOINT_SDO_WAIT;                   /* 先占位，挡住后来者 */
    __enable_irq();

    if (!sdo_send(SDO_READ, index, sub, 0))
    {
        s_sdo_state = JOINT_SDO_IDLE;               /* 发送失败，释放 */
        return 0;
    }
    return 1;
}

uint8_t Dev_Joint_SdoWrite(uint16_t index, uint8_t sub, uint32_t value, uint8_t width)
{
    uint8_t cmd;

    switch (width)
    {
        case 1:  cmd = SDO_WRITE_1B; break;
        case 2:  cmd = SDO_WRITE_2B; break;
        case 3:  cmd = SDO_WRITE_3B; break;
        default: cmd = SDO_WRITE_4B; break;
    }

    /* 同 Dev_Joint_SdoRead：先关中断占位，再在中断开启状态下发送。
     * 占位这几条指令不能包含 Bsp_Can_SendData —— 它里面有等待邮箱的循环，
     * 总线故障时会拖很久，关中断期间等待会让 SysTick 一起停摆。 */
    __disable_irq();
    if (s_sdo_state != JOINT_SDO_IDLE)      /* 同上：终态没被取走时不许覆盖 */
    {
        __enable_irq();
        return 0;
    }
    s_sdo_index = index;
    s_sdo_sub   = sub;
    s_sdo_t0    = HAL_GetTick();
    s_sdo_state = JOINT_SDO_WAIT;
    __enable_irq();

    if (!sdo_send(cmd, index, sub, value))
    {
        s_sdo_state = JOINT_SDO_IDLE;
        return 0;
    }
    return 1;
}

JointSdoState Dev_Joint_SdoPoll(uint32_t *value)
{
    JointSdoState st;

    /* 读终态 + 清回 IDLE 必须原子。
     * TIM3(100 Hz) 的写会在这里插进来：读到的还是 OK，清 IDLE 时把 ISR
     * 刚占的 WAIT 一起覆盖掉，那条请求从此失联，而它的确认帧晚到时
     * 又会冲掉下一次读回的值。 */
    __disable_irq();
    st = s_sdo_state;
    if (st == JOINT_SDO_OK || st == JOINT_SDO_ABORT || st == JOINT_SDO_TIMEOUT)
    {
        if (value != 0) *value = s_sdo_value;
        s_sdo_state = JOINT_SDO_IDLE;       /* 终态只报一次 */
    }
    __enable_irq();

    return st;
}

/* 强制把 SDO 通道打回 IDLE。
 * 用途只有一个：TIM3 刚被关掉时，很可能有一条 ISR 发出的 0x607A 写
 * 还挂在 WAIT 上、永远不会有人应答。而 Dev_Joint_SdoRead 要求 state == IDLE
 * 才肯发请求 —— 于是停机后所有诊断读会在 1 us 内全部返回失败，
 * 日志上写一片“读失败”，看着像驱动器死了，其实是我们自己把通道焊死了。
 * 停机和启动前都要调一次。 */
void Dev_Joint_SdoReset(void)
{
    __disable_irq();
    s_sdo_state = JOINT_SDO_IDLE;
    s_sdo_value = 0;
    s_sdo_index = 0xFFFFu;
    s_sdo_sub   = 0xFFu;
    __enable_irq();
}

JointSdoState Dev_Joint_SdoState(void)      { return s_sdo_state; }
uint32_t      Dev_Joint_SdoAbortCode(void)  { return s_sdo_abort; }
uint32_t      Dev_Joint_SdoGhostCnt(void)   { return s_sdo_ghost; }
void          Dev_Joint_SetSdoTimeout(uint32_t ms) { s_sdo_timeout = ms; }

uint8_t Dev_Joint_SdoReadBlock(uint16_t index, uint8_t sub, uint32_t *value, uint32_t timeout_ms)
{
    uint32_t      t0 = HAL_GetTick();
    uint32_t      v  = 0;
    JointSdoState st;

    if (!Dev_Joint_SdoRead(index, sub)) return 0;

    while ((HAL_GetTick() - t0) < timeout_ms)
    {
        Dev_Joint_Task();
        st = Dev_Joint_SdoPoll(&v);

        if (st == JOINT_SDO_OK)
        {
            if (value != 0) *value = v;
            return 1;
        }
        if (st == JOINT_SDO_ABORT || st == JOINT_SDO_TIMEOUT) return 0;

        HAL_Delay(1);
    }

    /* 超时退出：必须把状态清回 IDLE，否则下一帧读会被 WAIT 直接挡掉 */
    if (s_sdo_state == JOINT_SDO_WAIT) s_sdo_state = JOINT_SDO_IDLE;

    return 0;
}

uint8_t Dev_Joint_SdoWriteBlock(uint16_t index, uint8_t sub, uint32_t value, uint8_t width)
{
    uint32_t      t0 = HAL_GetTick();
    JointSdoState st;

    if (!Dev_Joint_SdoWrite(index, sub, value, width)) return 0;

    while ((HAL_GetTick() - t0) < 100)
    {
        Dev_Joint_Task();
        st = Dev_Joint_SdoPoll(0);

        if (st == JOINT_SDO_OK) return 1;
        if (st == JOINT_SDO_ABORT || st == JOINT_SDO_TIMEOUT) return 0;

        HAL_Delay(1);
    }

    return 0;
}

uint16_t Dev_Joint_Status(void)         { return s_status; }
uint32_t Dev_Joint_DigitalInputs(void)  { return s_di; }
uint8_t  Dev_Joint_ModeDisplay(void)    { return s_mode; }
uint16_t Dev_Joint_Emcy(void)           { return s_emcy; }
uint32_t Dev_Joint_EmcyCount(void)      { return s_emcy_cnt; }

uint8_t Dev_Joint_Sync(void)
{
    static uint8_t dummy[8] = {0};

    if (Bsp_Can_SendData(CAN_ID_STD, COB_SYNC, dummy, 0) != HAL_OK)
    {
        s_tx_fail_cnt++;
        return 0;
    }
    return 1;
}

/* =====================================================================
 *  NMT：链路层探针
 * ===================================================================== */
static uint8_t nmt_send(uint8_t cs, uint8_t node)
{
    uint8_t d[2];

    d[0] = cs;          /* 命令字 */
    d[1] = node;        /* 目标节点，0 = 全部 */

    if (Bsp_Can_SendData(CAN_ID_STD, COB_NMT, d, 2) != HAL_OK)
    {
        s_tx_fail_cnt++;
        return 0;
    }
    return 1;
}

uint8_t Dev_Joint_NmtResetNode(uint8_t node) { return nmt_send(NMT_RESET_NODE, node); }
uint8_t Dev_Joint_NmtStart(uint8_t node)     { return nmt_send(NMT_START_NODE, node); }
uint8_t Dev_Joint_NmtPreOp(uint8_t node)     { return nmt_send(NMT_PREOP_NODE, node); }
uint32_t Dev_Joint_BootCount(void)           { return s_boot_cnt; }
uint8_t  Dev_Joint_NmtState(void)            { return s_nmt_state; }

uint32_t Dev_Joint_RxCount(void)          { return s_rx_cnt; }
uint32_t Dev_Joint_SdoTimeoutCount(void)  { return s_sdo_to_cnt; }
uint32_t Dev_Joint_TxFailCount(void)      { return s_tx_fail_cnt; }
uint32_t Dev_Joint_Tpdo1Count(void)       { return s_tpdo1_cnt; }


/* =====================================================================
 *  TPDO1 改映射成 {0x6041, 0x6064}：状态字 + 实际位置
 *  只在通信参数（0x1800/0x1A00）上动，不碰任何电机参数。
 *  失败时不还原也没关系 —— 原映射只是少了位置，SDO 照样能读。
 * ===================================================================== */
#define TPDO_MAP_STATUSWORD   0x60410010u
#define TPDO_MAP_POSITION     0x60640020u

uint8_t Dev_Joint_Tpdo1SetupPos(void)
{
    uint32_t cob = 0u, typ = 0u, num = 0u, m1 = 0u, m2 = 0u;
    uint32_t want = (uint32_t)(0x180u + s_node);

    /* 改映射必须在 Pre-operational 下做。上一轮跑完节点可能还停在
     * Operational，这里先退回去，否则写 0x1A00 会被 abort。 */
    Dev_Joint_NmtPreOp(s_node);
    HAL_Delay(20u);
    s_nmt_state = 0xFFu;

    if (!Dev_Joint_SdoReadBlock(OD_TPDO1_COB, 1u, &cob, 100u)) goto fail;
    if (!Dev_Joint_SdoReadBlock(OD_TPDO1_COB, 2u, &typ, 100u)) goto fail;
    if (!Dev_Joint_SdoReadBlock(OD_TPDO1_MAP, 0u, &num, 100u)) goto fail;
    if (!Dev_Joint_SdoReadBlock(OD_TPDO1_MAP, 1u, &m1,  100u)) goto fail;
    if (!Dev_Joint_SdoReadBlock(OD_TPDO1_MAP, 2u, &m2,  100u)) goto fail;

    printf("[ADM] TPDO1 原状 COB=0x%08lX 类型=%lu 项数=%lu [0x%08lX, 0x%08lX]\r\n",
           (unsigned long)cob, (unsigned long)typ, (unsigned long)num,
           (unsigned long)m1, (unsigned long)m2);

    if (!(num == 2u && m1 == TPDO_MAP_STATUSWORD && m2 == TPDO_MAP_POSITION))
    {
        if (!Dev_Joint_SdoWriteBlock(OD_TPDO1_COB, 1u, cob | 0x80000000u, 4u)) goto fail;
        if (!Dev_Joint_SdoWriteBlock(OD_TPDO1_MAP, 0u, 0u, 1u)) goto fail;
        if (!Dev_Joint_SdoWriteBlock(OD_TPDO1_MAP, 1u, TPDO_MAP_STATUSWORD, 4u)) goto fail;
        if (!Dev_Joint_SdoWriteBlock(OD_TPDO1_MAP, 2u, TPDO_MAP_POSITION, 4u)) goto fail;
        if (!Dev_Joint_SdoWriteBlock(OD_TPDO1_MAP, 0u, 2u, 1u)) goto fail;
        printf("[ADM] TPDO1 已重映射：%lu 项 -> [0x60410010, 0x60640020]\r\n",
               (unsigned long)num);
    }

    Dev_Joint_SdoWriteBlock(OD_TPDO1_COB, 2u, 255u, 1u);    /* 异步，变化就发 */

    /* ---- PDO 发送时序：抑制时间 + 事件定时器 ----
     * 实测：sub5 写 5 ms、回读也是 5 ms，但 TPDO 实际只有约 11 帧/秒
     * （57 帧 / 5 s）。说明卡住的不是事件定时器，是"抑制时间" sub3
     * （单位 100 us，出厂常见 1000 = 100 ms），两次发送之间被它锁死。
     * 这里 sub3 写 0、sub5 写 2 ms，写完把两个都读回来打印 —— 看得见才算数。 */
    {
        uint32_t inb = 0u, evt = 0u;
        uint8_t  ok3 = 0u, ok5 = 0u;

        Dev_Joint_SdoReadBlock(OD_TPDO1_COB, 3u, &inb, 100u);
        Dev_Joint_SdoReadBlock(OD_TPDO1_COB, 5u, &evt, 100u);
        printf("[ADM] 0x1800 原状：sub3(抑制) = %lu (= %lu ms)，sub5(定时) = %lu ms\r\n",
               (unsigned long)inb, (unsigned long)(inb / 10u), (unsigned long)evt);

        ok3 = Dev_Joint_SdoWriteBlock(OD_TPDO1_COB, 3u, 0u, 2u);
        ok5 = Dev_Joint_SdoWriteBlock(OD_TPDO1_COB, 5u, 2u, 2u);

        Dev_Joint_SdoReadBlock(OD_TPDO1_COB, 3u, &inb, 100u);
        Dev_Joint_SdoReadBlock(OD_TPDO1_COB, 5u, &evt, 100u);
        printf("[ADM] 0x1800 新值：sub3 = %lu (%s)，sub5 = %lu ms (%s)\r\n",
               (unsigned long)inb, ok3 ? "OK" : "写失败",
               (unsigned long)evt, ok5 ? "OK" : "写失败");
        if ((inb > 10u) || (evt > 10u))
            printf("[ADM] 警告：PDO 发送间隔仍被限制在约 %lu ms，反馈会偏慢\r\n",
                   (unsigned long)(((inb / 10u) > evt) ? (inb / 10u) : evt));
    }

    if (!Dev_Joint_SdoWriteBlock(OD_TPDO1_COB, 1u, want, 4u)) goto fail;

    if (!Dev_Joint_SdoReadBlock(OD_TPDO1_MAP, 0u, &num, 100u)) goto fail;
    if (!Dev_Joint_SdoReadBlock(OD_TPDO1_MAP, 2u, &m2,  100u)) goto fail;
    if (!Dev_Joint_SdoReadBlock(OD_TPDO1_COB, 1u, &cob, 100u)) goto fail;

    if (num != 2u || m2 != TPDO_MAP_POSITION || (cob & 0x80000000u) != 0u)
    {
        printf("[ADM] TPDO1 回读不一致：%lu 项 m2=0x%08lX cob=0x%08lX\r\n",
               (unsigned long)num, (unsigned long)m2, (unsigned long)cob);
        goto fail;
    }

    s_tpdo_pos_cnt    = 0u;
    s_tpdo_pos_layout = 1u;
    return 1u;

fail:
    printf("[ADM] TPDO1 配置失败 abort=0x%08lX\r\n",
           (unsigned long)Dev_Joint_SdoAbortCode());
    s_tpdo_pos_layout = 0u;
    return 0u;
}

int32_t Dev_Joint_TpdoGetPos(void)
{
    return s_tpdo_pos;
}

uint32_t Dev_Joint_TpdoPosCount(void)
{
    return s_tpdo_pos_cnt;
}

/* =====================================================================
 *  位置读取（语义封装）
 * ===================================================================== */
#define DEV_POS_PERIOD_MS   50u     /* 0x6064 读取节流周期 */
static int32_t  s_pos_cache  = 0;
static uint32_t s_pos_t0     = 0;
static uint8_t  s_pos_freeze = 0u;

void Dev_Joint_SetPosFreeze(uint8_t on)
{
    s_pos_freeze = on;
}

uint8_t Dev_Joint_ReadPos(int32_t *pos)
{
    uint32_t v   = 0;
    uint32_t now = HAL_GetTick();

    /* 冻结态：只返回缓存，不碰 SDO。百赫兹控制环占用 SDO 时必开。 */
    if (s_pos_freeze != 0u)
    {
        if (pos != 0) *pos = s_pos_cache;
        return 1;
    }

    /* 节流到 20 Hz。原来 app_nj70 每收到一帧传感器数据（100 Hz）就读一次 0x6064，
     * 和导纳环 100 Hz 写 0x607A 完全同频，两个请求挤同一个 SDO 状态机；
     * 读还带 50 ms 阻塞超时，一旦超时就把主循环卡住，目标位置下发跟着断。
     * 20 Hz 对重力补偿显示和跟随偏差看门狗都够用。 */
    if (s_pos_t0 != 0u && (now - s_pos_t0) < DEV_POS_PERIOD_MS)
    {
        if (pos != 0) *pos = s_pos_cache;
        return 1;
    }
    s_pos_t0 = now;

    if (!Dev_Joint_SdoReadBlock(OD_POSITION_ACTUAL, 0, &v, 20))
        return 0;

    s_pos_cache = (int32_t)v;
    if (pos != 0) *pos = s_pos_cache;
    return 1;
}


/* =====================================================================
 *  CAN 接收回调（HAL 弱函数的强定义，全局唯一）
 *  中断上下文：只取帧 + 分派，不打印、不阻塞
 * ===================================================================== */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader = {0};
    uint8_t rxData[8] = {0};

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
        return;
    if (rxHeader.IDE == CAN_ID_STD)
        Dev_Joint_RxDispatch((uint16_t)rxHeader.StdId, rxData, (uint8_t)rxHeader.DLC);
}
