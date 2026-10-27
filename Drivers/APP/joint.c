#include "joint.h"
#include "bsp_can.h"

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

static uint16_t s_status  = 0;
static uint32_t s_di      = 0;
static uint8_t  s_mode    = 0;
static uint16_t s_emcy    = 0;

static uint32_t s_rx_cnt      = 0;
static uint32_t s_boot_cnt    = 0;
static uint32_t s_sdo_to_cnt  = 0;
static uint32_t s_tx_fail_cnt = 0;

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
    uint8_t cmd = d[0];

    /* ---------- 情况一：写指令成功 ---------- */
    // 刚才我们向驱动器发了个写参数指令，它回个 0x60 表示“写好了”
    if (cmd == SDO_SCS_WRITE_OK)
    {
        s_sdo_value = 0; // 写操作没数据返回，清零
        
        // 如果状态机正在 WAIT（等应答），立刻改成 OK，主循环里就能看到成功了
        if (s_sdo_state == JOINT_SDO_WAIT) s_sdo_state = JOINT_SDO_OK;
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
static void tpdo1_on_rx(const uint8_t *d, uint8_t dlc)
{
    if (dlc >= 2u)
        s_status = (uint16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8));

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
void Joint_Init(uint8_t node_id)
{
    s_node        = node_id;
    s_sdo_state   = JOINT_SDO_IDLE;
    s_sdo_timeout = 50;
    s_status      = 0;
    s_di          = 0;
    s_mode        = 0;
    s_emcy        = 0;
}

void Joint_RxDispatch(uint16_t std_id, const uint8_t *data, uint8_t dlc)
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
        return;
    }

    if (std_id == COB_HEARTBEAT(s_node))
    {
        s_boot_cnt++;                 /* boot-up 或心跳，证明链路是通的 */
        return;
    }

    /* 其他帧（NMT / 心跳）暂时忽略 */
}

void Joint_Task(void)
{
    /* 收帧走 CAN 中断 -> HAL_CAN_RxFifo0MsgPendingCallback -> Joint_RxDispatch，
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

uint8_t Joint_SdoRead(uint16_t index, uint8_t sub)
{
    if (s_sdo_state == JOINT_SDO_WAIT) return 0;    /* 上一条还没回 */
    s_sdo_state = JOINT_SDO_IDLE;
    return sdo_send(SDO_READ, index, sub, 0);
}

uint8_t Joint_SdoWrite(uint16_t index, uint8_t sub, uint32_t value, uint8_t width)
{
    uint8_t cmd;

    if (s_sdo_state == JOINT_SDO_WAIT) return 0;

    switch (width)
    {
        case 1:  cmd = SDO_WRITE_1B; break;
        case 2:  cmd = SDO_WRITE_2B; break;
        case 3:  cmd = SDO_WRITE_3B; break;
        default: cmd = SDO_WRITE_4B; break;
    }

    s_sdo_state = JOINT_SDO_IDLE;
    return sdo_send(cmd, index, sub, value);
}

JointSdoState Joint_SdoPoll(uint32_t *value)
{
    JointSdoState st = s_sdo_state;

    if (st == JOINT_SDO_OK || st == JOINT_SDO_ABORT || st == JOINT_SDO_TIMEOUT)
    {
        if (value != 0) *value = s_sdo_value;
        s_sdo_state = JOINT_SDO_IDLE;       /* 终态只报一次 */
    }

    return st;
}

JointSdoState Joint_SdoState(void)      { return s_sdo_state; }
uint32_t      Joint_SdoAbortCode(void)  { return s_sdo_abort; }
void          Joint_SetSdoTimeout(uint32_t ms) { s_sdo_timeout = ms; }

uint8_t Joint_SdoReadBlock(uint16_t index, uint8_t sub, uint32_t *value, uint32_t timeout_ms)
{
    uint32_t      t0 = HAL_GetTick();
    uint32_t      v  = 0;
    JointSdoState st;

    if (!Joint_SdoRead(index, sub)) return 0;

    while ((HAL_GetTick() - t0) < timeout_ms)
    {
        Joint_Task();
        st = Joint_SdoPoll(&v);

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

uint8_t Joint_SdoWriteBlock(uint16_t index, uint8_t sub, uint32_t value, uint8_t width)
{
    uint32_t      t0 = HAL_GetTick();
    JointSdoState st;

    if (!Joint_SdoWrite(index, sub, value, width)) return 0;

    while ((HAL_GetTick() - t0) < 100)
    {
        Joint_Task();
        st = Joint_SdoPoll(0);

        if (st == JOINT_SDO_OK) return 1;
        if (st == JOINT_SDO_ABORT || st == JOINT_SDO_TIMEOUT) return 0;

        HAL_Delay(1);
    }

    return 0;
}

uint16_t Joint_Status(void)         { return s_status; }
uint32_t Joint_DigitalInputs(void)  { return s_di; }
uint8_t  Joint_ModeDisplay(void)    { return s_mode; }
uint16_t Joint_Emcy(void)           { return s_emcy; }

uint8_t Joint_Sync(void)
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

uint8_t Joint_NmtResetNode(uint8_t node) { return nmt_send(NMT_RESET_NODE, node); }
uint8_t Joint_NmtStart(uint8_t node)     { return nmt_send(NMT_START_NODE, node); }
uint32_t Joint_BootCount(void)           { return s_boot_cnt; }

uint32_t Joint_RxCount(void)          { return s_rx_cnt; }
uint32_t Joint_SdoTimeoutCount(void)  { return s_sdo_to_cnt; }
uint32_t Joint_TxFailCount(void)      { return s_tx_fail_cnt; }
