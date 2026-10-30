#include "app_joint.h"
#include "app_nj70.h"
#include <math.h>
#include <stdio.h>

/* htim3 定义在 Core/Src/tim.c，main.h 里没有 extern，这里自己声明 */
extern TIM_HandleTypeDef htim3;

/* =====================================================================
 *  周期任务：目前只做 SDO 超时检测（Dev_Joint_Task）
 * ===================================================================== */
void App_Joint_Task(void)
{
    Dev_Joint_Task();
}

/* =====================================================================
 *  导纳控制 · PP 位置流版（2026-10-30 定版）
 *
 *  为什么是 PP：
 *    CSP 1 kHz 点流实测驱动器恒定超前 350~410 counts，并多次跳 0x3230；
 *    CSV 速度口也压不住，且有嗡鸣。
 *    PP 位置流（每 20 ms 改一次 0x607A）连跑 400 点：0x603F 恒 0、
 *    TEC/REC/LEC 全 0、净漂移 40 counts(0.03 deg)、两跑逐点重合。
 *    所以导纳输出改走 PP，位置环仍然在驱动器内部闭环 —— 仍然是位控。
 *
 *  分工（硬约束，违反任何一条都会重演之前的事故）：
 *    TIM3 ISR 1 kHz —— 每 10 拍做一次浮点递推（100 Hz），只算 pos_cmd。
 *                      不发帧、不读 SDO、不打印、不 HAL_Delay。
 *    主循环 20 ms   —— PP 换点：0x0F -> 写 0x607A -> 0x3F（bit4 上升沿）。
 *                      三次阻塞 SDO 写，每次约 1 ms，50 Hz 下占主循环
 *                      不到 20%，NJ70 是 50 ms 一问一答，完全不受影响。
 *    位置反馈       —— TPDO1 映射 {0x6041, 0x6064}，0x1800.5 = 2 ms、
 *                      0x1800.3 = 0（抑制时间清零），实测约 500 帧/秒。
 *                      SDO 通道整个让给换点，运行期不再轮询 0x6064。
 *
 *  速度上限：PP 每个点都按"到点停"规划，能维持的稳态速度约为
 *            0x6083 * 换点周期 = 1e6 * 0.02 = 20000 counts/s，
 *            高于软件限速 ADM_VEL_MAX(8000)，所以 0x6083 不是瓶颈。
 *            实测代价是每拍约 v^2/(2a) = 32 counts 的规划滞后（0.02 deg）。
 * ===================================================================== */

#define ADM_STEP_IDLE    0u
#define ADM_STEP_RUN     30u

#define ADM_STOP_NONE     0u
#define ADM_STOP_KEY      1u    /* 按键主动停 */
#define ADM_STOP_TAU      2u    /* 外力矩超限 */
#define ADM_STOP_ERR      3u    /* 跟随误差超限 */
#define ADM_STOP_PP       4u    /* PP 换点连续失败（SDO 不通） */
#define ADM_STOP_FDB_LOST 5u    /* 位置反馈断流（TPDO 不来帧） */
#define ADM_STOP_STEP     6u    /* 启动流程失败 */
#define ADM_STOP_DRV_FAULT 7u   /* 驱动器自身进故障态（0x6041 bit3） */

/* 位置读数合理性校验：TPDO 上来的 0x6064 理论上不会有坏值，
 * 但节点重连、映射被改、半帧拼接时仍可能来一个离谱的数，
 * 一旦写进 s_adm_pos_fdb 会同时污染重力补偿和跟随误差判断。 */
#define ADM_FDB_ABS_MAX      (1048576L)  /* +-2 圈 */
#define ADM_FDB_COUNT_PER_MS (542L)      /* 270882 counts/s 的 2 倍 */
#define ADM_FDB_SLACK_CNT    (2000L)

static volatile uint8_t  s_adm_run     = 0u;   /* 1 = ISR 在递推 */
static volatile float    s_adm_pos_cmd = 0.0f; /* 模型算出的位置（只给人看） */
static volatile float    s_adm_pos_tgt = 0.0f; /* 实际写进 0x607A 的位置 = cmd + 超前量 */
static volatile float    s_adm_lead_v  = 0.0f; /* 超前量用的速度，一阶低通，换向不硬跳 */
static volatile float    s_adm_delta_pos = 0.0f;
static volatile float    s_adm_delta_vel = 0.0f;
static volatile float    s_adm_tau_grav  = 0.0f;
/* ---- 运行时增益档位（KEY1 循环切，用于 A/B 对比手感）----
 * 三档只改软硬：M/B/K 同比例缩放，wn = sqrt(K/M) 和 zeta = B/(2*sqrt(K*M))
 * 都保持不变，所以响应快慢一样，切的就是"多硬"。
 * 基准来自 ADM_PRESET 选出的 ADM_M/B/K，改预设这三行会自动跟随。
 *   ADM_GAIN_SCALE 里 1.00 那一档必须与基准完全一致，所以默认停在 idx=1。
 *
 * 为什么不在 CycleGain 里直接写这三个变量：硬浮点写到一半被 TIM3 打断，
 * ISR 会读到半新半旧的 M/B/K（3 条指令的窗口，概率低但不是零）。
 * 所以 APP 侧只写一个 u8 请求位，ISR 在拍边界上一次性生效，
 * 顺便让切档时刻确定，冲击可复现。 */
#define ADM_GAIN_LEVELS     3u
static const float   s_adm_gain_scale[ADM_GAIN_LEVELS] = { 0.75f, 1.00f, 1.3333f };
static volatile uint8_t s_adm_gain_req = 1u;   /* APP 侧请求的档位（0..2） */
static uint8_t          s_adm_gain_cur = 1u;   /* ISR 侧已生效的档位 */
static volatile float s_adm_m     = ADM_M;         /* 当前生效值，递推直接用 */
static volatile float s_adm_b     = ADM_B;
static volatile float s_adm_k     = ADM_K;
static volatile float s_adm_inv_m = 1.0f / ADM_M;
static void Adm_ApplyGain(uint8_t idx);        /* 实现见文件末尾 */

static volatile uint32_t s_adm_beat    = 0u;   /* 递推心跳 */

static volatile int32_t  s_adm_pos_ref = 0;    /* 平衡位置 = 使能后位置，运行中不改 */
static int32_t  s_adm_pos_home  = 0;           /* 软限位基准 */
static uint8_t  s_adm_step      = ADM_STEP_IDLE;
static uint32_t s_adm_step_t    = 0u;
static uint32_t s_adm_mon_t     = 0u;          /* 20 ms 主节拍 */
static uint32_t s_adm_print_t   = 0u;
static uint32_t s_adm_run_t0    = 0u;
static uint32_t s_adm_pos_t     = 0u;          /* 上次收到 TPDO 位置帧的时刻 */
static int32_t  s_adm_pos_fdb   = 0;
static uint32_t s_adm_tpdo_lc   = 0u;          /* 上一帧 TPDO 序号，判新帧 */
static uint8_t  s_adm_fdb_valid = 0u;
static uint16_t s_adm_sw        = 0xFFFFu;     /* TPDO 带来的 0x6041 */
static uint16_t s_adm_sw_last   = 0xFFFFu;
static uint8_t  s_adm_stop_rsn  = ADM_STOP_NONE;
static uint8_t  s_adm_tau_cnt   = 0u;
static uint8_t  s_adm_err_cnt   = 0u;

static volatile int32_t s_adm_tgt_prev = 0;    /* 上一拍指令，量单拍跳变 */
static volatile int32_t s_adm_jump_pk  = 0;
static volatile int32_t s_adm_err_pk   = 0;

/* ---- PP 换点状态 ---- */
static int32_t  s_adm_pp_sent   = 0;   /* 上一次被驱动器采信的 0x607A */
static uint32_t s_adm_pp_ok     = 0u;  /* 成功换点次数 */
static uint32_t s_adm_pp_bad    = 0u;  /* 失败次数 */
static uint8_t  s_adm_pp_fail   = 0u;  /* 连续失败计数 */

/* ---- 静态保持自检 ---- */
static uint8_t  s_adm_hold      = 0u;
static uint32_t s_adm_hold_t    = 0u;

/* ---- 打印用的驱动器自述量，500 ms 刷新一次 ---- */
static int32_t  s_adm_tq_pm     = 0;   /* 0x6077 实际力矩 ‰额定 */
static int32_t  s_adm_vel_fb    = 0;   /* 0x606C 实际速度 counts/s */
static int32_t  s_adm_tau_echo  = 0;   /* 打印用的外力矩 mN.m */
static uint32_t s_adm_drv_err   = 0u;  /* 0x603F */
static uint8_t  s_adm_mode_old  = 8u;

/* ---------------------------------------------------------------------
 *  位置读数是否可信
 * ------------------------------------------------------------------- */
static uint8_t Adm_PosAccept(int32_t now)
{
    int32_t  d;
    int32_t  lim;
    uint32_t ms;

    if (now > ADM_FDB_ABS_MAX || now < -ADM_FDB_ABS_MAX) return 0u;
    if (s_adm_fdb_valid == 0u) return 1u;

    d  = now - s_adm_pos_fdb;
    ms = uwTick - s_adm_pos_t;
    if (ms > 500u) ms = 500u;

    lim = (int32_t)ms * ADM_FDB_COUNT_PER_MS + ADM_FDB_SLACK_CNT;
    if (d > lim || d < -lim) return 0u;

    return 1u;
}

/* ---------------------------------------------------------------------
 *  100 Hz：导纳递推（TIM3 每 1 ms 进来，第 10 次做一次）
 *  只算"这一拍应该在哪"，不碰 CAN、不碰 SDO。
 * ------------------------------------------------------------------- */
static void Adm_Solve100(void)
{
    float   tau_ext;
    float   acc_cmd;
    float   delta_vel;
    float   delta_pos;
    float   pos_cmd;
    float   pos_tgt;
    float   pos_lo;
    float   pos_hi;
    int32_t target;

    s_adm_beat++;

    /* 档位切换只在拍边界生效，见上方 s_adm_gain_req 的注释 */
    if (s_adm_gain_req != s_adm_gain_cur)
    {
        Adm_ApplyGain(s_adm_gain_req);
    }

    /* 外力矩 = 传感器读数 - 该姿态下的重力矩，静止时应≈0 */
    tau_ext = (torque - s_adm_tau_grav) * ADM_TAU_SIGN;

    /* 静态保持自检：把外力矩强行当 0，delta 恒 0，目标完全静止 */
    if (s_adm_hold != 0u)
    {
        s_adm_delta_vel = 0.0f;
        s_adm_delta_pos = 0.0f;
        tau_ext = 0.0f;
    }

    delta_vel = s_adm_delta_vel;
    delta_pos = s_adm_delta_pos;

    /* 导纳方程： M*acc + B*vel + K*delta = tau_ext
     * 实测位置不参与 pos_cmd，只用于重力补偿和看门狗。 */
    acc_cmd = (tau_ext - s_adm_b * delta_vel - s_adm_k * delta_pos) * s_adm_inv_m;

    /* 先限速度再积分，顺序不能反 */
    delta_vel += acc_cmd * ADM_DT;
    if (delta_vel >  ADM_VEL_MAX) delta_vel =  ADM_VEL_MAX;
    if (delta_vel < -ADM_VEL_MAX) delta_vel = -ADM_VEL_MAX;

    delta_pos += delta_vel * ADM_DT;

    pos_cmd = (float)s_adm_pos_ref + delta_pos;

    /* 软限位相对启动位置，纯粹钳位 */
    pos_lo = (float)(s_adm_pos_home - ADM_LIMIT_CNT);
    pos_hi = (float)(s_adm_pos_home + ADM_LIMIT_CNT);
    if (pos_cmd > pos_hi)
    {
        pos_cmd   = pos_hi;
        delta_pos = pos_hi - (float)s_adm_pos_ref;
        if (delta_vel > 0.0f) delta_vel = 0.0f;
    }
    if (pos_cmd < pos_lo)
    {
        pos_cmd   = pos_lo;
        delta_pos = pos_lo - (float)s_adm_pos_ref;
        if (delta_vel < 0.0f) delta_vel = 0.0f;
    }

    s_adm_delta_pos = delta_pos;
    s_adm_delta_vel = delta_vel;
    s_adm_pos_cmd   = pos_cmd;

    /* 超前补偿：把目标按当前速度前移 ADM_LEAD_MS，抵消驱动器追阶梯的滞后。
     * 速度先过一阶低通 —— 换向时 delta_vel 从 -16000 直接跳到 +16000，
     * 不滤的话目标位置会瞬移 500+ counts，那一下就是一次可听见的"哒"。
     * 软限位对超前后的目标再钳一次，免得贴限位时多冲。 */
    s_adm_lead_v += (delta_vel - s_adm_lead_v) * ADM_LEAD_LPF;
    pos_tgt = pos_cmd + s_adm_lead_v * ADM_LEAD_S;
    if (pos_tgt > pos_hi) pos_tgt = pos_hi;
    if (pos_tgt < pos_lo) pos_tgt = pos_lo;
    s_adm_pos_tgt = pos_tgt;

    target = (int32_t)pos_tgt;

    /* 两个峰值，纯测量。单拍跳变上限见 App_Joint_Adm_Stop() 的打印 */
    {
        int32_t j = target - s_adm_tgt_prev;
        int32_t e = target - s_adm_pos_fdb;
        if (j < 0) j = -j;
        if (e < 0) e = -e;
        if (j > s_adm_jump_pk) s_adm_jump_pk = j;
        if (e > s_adm_err_pk)  s_adm_err_pk  = e;
        s_adm_tgt_prev = target;
    }
}

/* ---------------------------------------------------------------------
 *  TIM3 中断入口（1 kHz）
 *  只做递推，一帧 CAN 都不发。换点由主循环负责。
 * ------------------------------------------------------------------- */
static void App_Joint_Adm_Isr(void)
{
    static uint8_t div = 0u;

    if (s_adm_run == 0u) return;

    if (++div < ADM_SUB) return;
    div = 0u;

    Adm_Solve100();
}

/* =====================================================================
 *  TIM3 中断入口（PSC=71 -> 1 MHz，进 RUN 后 ARR=999 -> 1 kHz）
 * ===================================================================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;

    App_Joint_Adm_Isr();
}

/* =====================================================================
 *  重力矩补偿
 *  拟合式 tau = A*cos(theta) + B*sin(theta) + C，theta 由位置 counts 换算。
 * ===================================================================== */

float App_Joint_GetGravityTorque(int32_t pos)
{
    float th = POS_TO_RAD(pos);
    return GRAV_A * cosf(th) + GRAV_B * sinf(th) + GRAV_C;
}

/* =====================================================================
 *  扫角自检（调试专用）：PP 模式从 pos0-35 走 8 点到 pos0+35，
 *  每点等 bit10 -> 稳 1.5 s -> 每 50 ms 采一次力矩取平均。
 *  输出 DATA,idx,target_deg,act_deg,counts,torque_Nm
 *  拟合：tau = a*cos(theta) + b*sin(theta) + c
 * ===================================================================== */
#define TEST_STEP_MS       300u     /* 状态机步进间隔 */
#define MOVE_TIMEOUT_MS   8000u     /* 单点移动超时 */

#define PROFILE_VEL      20000u     /* 0x6081 轮廓速度 counts/s = 13.7 deg/s */
#define PROFILE_ACC      50000u     /* 0x6083/0x6084 加减速度 counts/s^2 */

/* ---- 重力矩扫角采集参数 ---- */
#define SCAN_POINTS        8        /* 采集点数 */
#define SCAN_START_DEG    (-35.0f)  /* 起始角度，相对 pos0 */
#define SCAN_STEP_DEG      10.0f    /* 点间隔 */
#define SCAN_SETTLE_MS    1500u     /* 到位后稳定时间，等静差收敛 */
#define SCAN_SAMPLE_MS      50u     /* 力矩采样间隔 */

static int32_t  s_pos0     = 0;      /* 使能时的起始位置 */
static int32_t  s_target   = 0;      /* 当前目标 */
static uint32_t s_move_t   = 0;      /* 本次移动开始时刻 */
static uint8_t  s_scan_i   = 0;      /* 当前扫角点序号 */
static uint32_t s_settle_t = 0;      /* 稳定期起点 */
static float    s_sum_tau  = 0.0f;   /* 力矩累加 */
static uint16_t s_cnt_tau  = 0;      /* 力矩样本数 */
static uint32_t s_t0       = 0;      /* 首次进入 case 0 的时刻 */

/* ---------------------------------------------------------------------
 *  重力矩扫角采集（PP 模式，0x6060 = 1）
 *
 *  流程：切 PP -> 设轮廓速度/加减速 -> 目标 = 当前位置 -> 06/07/0F 使能
 *        -> 依次走到 pos0-35 到 pos0+35 共 8 个点
 *        -> 每点等 bit10 后再稳 1.5 s，期间每 50 ms 采一次力矩取平均
 *
 *  输出：DATA,idx,target_deg,act_deg,counts,torque_Nm
 *  拟合用实测角度 act_deg，不用 target_deg（位置环有约 1.5 deg 静差）
 * ------------------------------------------------------------------- */
void App_Joint_GravScan_Task(void)
{
    static uint8_t  step   = 0;
    static uint32_t tick   = 0;
    uint32_t v      = 0;
    uint32_t period = 0;
    int32_t  pos    = 0;
    uint16_t sw     = 0;
    float    deg    = 0.0f;

    if (step == 99u) return;

    /* 采样阶段要快，其他阶段按 300 ms 节拍 */
    period = (step == 13u) ? SCAN_SAMPLE_MS : TEST_STEP_MS;
    if (uwTick - tick < period) return;
    tick = uwTick;

    switch (step)
    {
    case 0:     /* 等首帧力矩，再读状态字和起始位置 */
        if (s_t0 == 0u) s_t0 = uwTick;
        if (torque_first_flag == 0u)
        {
            if ((uwTick - s_t0) > 15000u)
            {
                printf("\r\n[FAIL] 传感器 15 s 没收到有效帧，App_NJ70_Task() 在跑吗\r\n");
                step = 99u;
            }
            return;
        }

        printf("\r\n---- 重力矩扫角采集（PP 模式）----\r\n");

        if (!Dev_Joint_SdoReadBlock(OD_STATUSWORD, 0, &v, 100))
        {
            printf("[FAIL] 读状态字超时\r\n");
            step = 99u;
            return;
        }
        printf("状态字 0x%04lX\r\n", (unsigned long)v);

        if (!Dev_Joint_SdoReadBlock(OD_POSITION_ACTUAL, 0, &v, 100))
        {
            printf("[FAIL] 读位置超时\r\n");
            step = 99u;
            return;
        }
        s_pos0 = (int32_t)v;
        printf("起始位置 %ld counts  (%.2f deg)\r\n",
               (long)s_pos0, (double)COUNT_TO_DEG(s_pos0));
        printf("扫角范围 %+.1f ~ %+.1f deg，共 %u 点，每点停 %u ms\r\n",
               (double)SCAN_START_DEG,
               (double)(SCAN_START_DEG + (SCAN_POINTS - 1) * SCAN_STEP_DEG),
               (unsigned)SCAN_POINTS, (unsigned)SCAN_SETTLE_MS);
        printf("当前读数 %.4f N.m（原始值，零偏会进拟合的 c）\r\n",
               (double)torque);
        step = 1u;
        break;

    case 1:     /* 切 PP */
        if (!Dev_Joint_SdoWriteBlock(OD_MODES_OF_OPERATION, 0, JOINT_MODE_PP, 1))
        {
            printf("[FAIL] 写 0x6060=1 失败  abort=0x%08lX\r\n",
                   (unsigned long)Dev_Joint_SdoAbortCode());
            step = 99u;
            return;
        }
        step = 2u;
        break;

    case 2:     /* 确认模式 */
        if (!Dev_Joint_SdoReadBlock(OD_MODE_DISPLAY, 0, &v, 100))
        {
            printf("[FAIL] 读 0x6061 超时\r\n");
            step = 99u;
            return;
        }
        printf("当前模式 %d\r\n", (int)(int8_t)v);
        if ((int8_t)v != JOINT_MODE_PP)
        {
            printf("[FAIL] 模式没有切到 1(PP)\r\n");
            step = 99u;
            return;
        }
        step = 3u;
        break;

    case 3:     /* 轮廓速度 */
        if (!Dev_Joint_SdoWriteBlock(OD_PROFILE_VELOCITY, 0, PROFILE_VEL, 4))
        {
            printf("[FAIL] 写 0x6081 失败\r\n");
            step = 99u;
            return;
        }
        printf("轮廓速度 %u counts/s (%.1f deg/s)\r\n",
               (unsigned int)PROFILE_VEL, (double)COUNT_TO_DEG((int32_t)PROFILE_VEL));
        step = 4u;
        break;

    case 4:     /* 加速度 */
        if (!Dev_Joint_SdoWriteBlock(OD_PROFILE_ACCEL, 0, PROFILE_ACC, 4))
        {
            printf("[FAIL] 写 0x6083 失败\r\n");
            step = 99u;
            return;
        }
        step = 5u;
        break;

    case 5:     /* 减速度 */
        if (!Dev_Joint_SdoWriteBlock(OD_PROFILE_DECEL, 0, PROFILE_ACC, 4))
        {
            printf("[FAIL] 写 0x6084 失败\r\n");
            step = 99u;
            return;
        }
        printf("加减速度 %u counts/s^2\r\n", (unsigned int)PROFILE_ACC);
        step = 6u;
        break;

    case 6:     /* 目标 = 当前位置，避免使能瞬间跳 */
        s_target = s_pos0;
        if (!Dev_Joint_SdoWriteBlock(OD_TARGET_POSITION, 0, (uint32_t)s_target, 4))
        {
            printf("[FAIL] 写 0x607A 失败\r\n");
            step = 99u;
            return;
        }
        step = 7u;
        break;

    case 7:     /* Shutdown */
        Dev_Joint_SdoWriteBlock(OD_CONTROLWORD, 0, CW_SHUTDOWN, 2);
        step = 8u;
        break;

    case 8:     /* Switch On */
        Dev_Joint_SdoWriteBlock(OD_CONTROLWORD, 0, CW_SWITCH_ON, 2);
        step = 9u;
        break;

    case 9:     /* Enable Operation */
        Dev_Joint_SdoWriteBlock(OD_CONTROLWORD, 0, CW_ENABLE_OPERATION, 2);
        step = 10u;
        break;

    case 10:    /* 确认使能 */
        if (!Dev_Joint_SdoReadBlock(OD_STATUSWORD, 0, &v, 100))
        {
            printf("[FAIL] 读状态字超时\r\n");
            step = 99u;
            return;
        }
        printf("状态字 0x%04lX  状态位 0x%02lX\r\n",
               (unsigned long)v, (unsigned long)(v & SW_MASK_STATE));

        if ((v & SW_MASK_STATE) != SW_STATE_ENABLED)
        {
            printf("[FAIL] 没进 Operation enabled(0x27)\r\n");
            printf("       查 STO_IN(PIN1)/STO_COM(PIN2) 是否短接、48V 是否到位\r\n");
            step = 99u;
            return;
        }

        printf("[ OK ] 已使能，开始扫角\r\n");
        printf("DATA,idx,target_deg,act_deg,counts,tau_Nm\r\n");
        s_scan_i = 0u;
        step = 11u;
        break;

    case 11:    /* 发第 s_scan_i 个点 */
        if (s_scan_i >= SCAN_POINTS)
        {
            step = 14u;
            break;
        }
        deg = SCAN_START_DEG + (float)s_scan_i * SCAN_STEP_DEG;
        s_target = s_pos0 + DEG_TO_COUNT(deg);

        Dev_Joint_SdoWriteBlock(OD_TARGET_POSITION, 0, (uint32_t)s_target, 4);
        Dev_Joint_SdoWriteBlock(OD_CONTROLWORD, 0, CW_ENABLE_OPERATION, 2);                    /* bit4 = 0 */
        Dev_Joint_SdoWriteBlock(OD_CONTROLWORD, 0, CW_ENABLE_OPERATION | CW_NEW_SETPOINT, 2);  /* bit4 上升沿 */

        printf("--> 第 %u 点  目标 %+6.1f deg  (%+8ld)\r\n",
               (unsigned)s_scan_i, (double)deg, (long)s_target);
        s_move_t = uwTick;
        step = 12u;
        break;

    case 12:    /* 等到位：先等 600ms 让 bit10 确实清 0 */
        if ((uwTick - s_move_t) < 600u) break;

        if (!Dev_Joint_SdoReadBlock(OD_STATUSWORD, 0, &v, 100))
        {
            printf("[WARN] 读状态字超时，跳过第 %u 点\r\n", (unsigned)s_scan_i);
            s_scan_i++;
            step = 11u;
            break;
        }
        sw = (uint16_t)v;

        if ((sw & 0x0400u) != 0u)           /* bit10 target reached */
        {
            s_settle_t = uwTick;
            s_sum_tau  = 0.0f;
            s_cnt_tau  = 0u;
            step = 13u;
            break;
        }
        if ((uwTick - s_move_t) > MOVE_TIMEOUT_MS)
        {
            printf("[WARN] 第 %u 点超时未到位，跳过\r\n", (unsigned)s_scan_i);
            s_scan_i++;
            step = 11u;
        }
        break;

    case 13:    /* 稳定期采样，50ms 一次 */
        s_sum_tau += torque;
        s_cnt_tau++;

        if ((uwTick - s_settle_t) < SCAN_SETTLE_MS) break;

        if (!Dev_Joint_SdoReadBlock(OD_POSITION_ACTUAL, 0, &v, 100))
        {
            printf("[WARN] 第 %u 点读位置超时\r\n", (unsigned)s_scan_i);
            s_scan_i++;
            step = 11u;
            break;
        }
        pos = (int32_t)v;
        printf("DATA,%u,%.2f,%.2f,%ld,%.4f\r\n",
               (unsigned)s_scan_i,
               (double)(SCAN_START_DEG + (float)s_scan_i * SCAN_STEP_DEG),
               (double)COUNT_TO_DEG(pos),
               (long)pos,
               s_cnt_tau ? (double)(s_sum_tau / (float)s_cnt_tau) : 0.0);

        s_scan_i++;
        step = 11u;
        break;

    case 14:    /* 汇总 */
        printf("---- 采集结束，共 %u 点 ----\r\n", (unsigned)SCAN_POINTS);
        printf("把 DATA 行贴出来，拟合 tau = a*cos(theta) + b*sin(theta) + c\r\n");
        printf("其中 c = 传感器零偏 + 装夹预紧，与角度无关\r\n");
        step = 99u;
        break;

    default:
        step = 99u;
        break;
    }
}

/* =====================================================================
 *  导纳控制 · SDO 薄封装（阻塞，只在主循环里用）
 * ===================================================================== */
static uint8_t Adm_Wr(uint16_t index, uint8_t sub, uint32_t value, uint8_t width)
{
    return Dev_Joint_SdoWriteBlock(index, sub, value, width);
}

static uint8_t Adm_Rd(uint16_t index, uint8_t sub, uint32_t *value)
{
    return Dev_Joint_SdoReadBlock(index, sub, value, 60u);
}

/* ---------------------------------------------------------------------
 *  换点。两种机制由 ADM_USE_IP 选择，逻辑完全不同：
 *
 *  IP（0x6060=7）：只写 0x607A，控制字恒 0x000F。
 *     驱动器按 0x60C2(1 ms) 自己插补过去，**不做"到点停"重规划**，
 *     所以没有 PP 那个 25 Hz 的速度三角，运动是连续的。
 *     只要一条 SDO，换点从约 3 ms 降到约 1 ms。
 *
 *  PP（0x6060=1）：0x0F(bit4=0) -> 写 0x607A -> 0x3F(bit4 上升沿 = 采信新点)。
 *     位置流测试里连跑 400 点零失败，也是现在这版的保底路径。
 * ------------------------------------------------------------------- */
static uint8_t Adm_PpSend(int32_t tgt)
{
#if (ADM_USE_IP != 0u)
    if (!Adm_Wr(OD_TARGET_POSITION, 0u, (uint32_t)tgt, 4u)) return 0u;

    s_adm_pp_sent = tgt;
    return 1u;
#else
    if (!Adm_Wr(OD_CONTROLWORD, 0u, CW_ENABLE_OPERATION, 2u)) return 0u;
    HAL_Delay(1u);                       /* 给驱动器清 bit12 的时间，实测 400 点无失败 */
    if (!Adm_Wr(OD_TARGET_POSITION, 0u, (uint32_t)tgt, 4u))   return 0u;
    HAL_Delay(1u);
    if (!Adm_Wr(OD_CONTROLWORD, 0u, CW_CSP_RUN, 2u))          return 0u;

    s_adm_pp_sent = tgt;
    return 1u;
#endif
}

/* ---------------------------------------------------------------------
 *  从 TPDO 取位置与状态字。没有新帧时什么都不做 ——
 *  一帧都没来过时绝不能把 0 当位置。
 * ------------------------------------------------------------------- */
static void Adm_PollTpdo(void)
{
    uint32_t c = Dev_Joint_TpdoPosCount();

    if (c != s_adm_tpdo_lc)
    {
        s_adm_tpdo_lc = c;

        int32_t p = Dev_Joint_TpdoGetPos();

        if (Adm_PosAccept(p) != 0u)
        {
            s_adm_pos_fdb   = p;
            s_adm_pos_t     = uwTick;
            s_adm_fdb_valid = 1u;
        }
        s_adm_sw = Dev_Joint_Status();
    }
}

/* =====================================================================
 *  导纳控制 · 启动流程
 *  20 ms 一步的非阻塞状态机，任何一步失败都走 Adm_Fail 收尾。
 * ===================================================================== */
static void Adm_Fail(const char *why)
{
    printf("[FAIL] %s\r\n", why);
    s_adm_stop_rsn = ADM_STOP_STEP;
    s_adm_step     = ADM_STEP_IDLE;
    s_adm_run      = 0u;
    Dev_Joint_SetPosFreeze(0u);
}

static void App_Joint_Adm_Step(void)
{
    uint32_t v = 0u;

    switch (s_adm_step)
    {
    case 1u:
        Dev_Joint_SetPosFreeze(1u);      /* 独占 SDO：挡住 app_nj70 的周期性位置读 */
#if (ADM_USE_IP != 0u)
        printf("\r\n[ADM] ===== 导纳控制启动（IP 插补位置 · %u ms 换点）=====\r\n",
               (unsigned)ADM_STR_MS);
#else
        printf("\r\n[ADM] ===== 导纳控制启动（PP 位置流 · %u ms 换点）=====\r\n",
               (unsigned)ADM_STR_MS);
#endif
        printf("[ADM] M=%.2e  B=%.2e  K=%.2e  软限位 +-%.1f deg (预设 %d)\r\n",
               (double)s_adm_m, (double)s_adm_b, (double)s_adm_k,
               (double)ADM_LIMIT_DEG, (int)ADM_PRESET);
        printf("[ADM] 力矩符号 %.0f  等效惯量 J=%.2f kg.m^2  刚度 %.3f N.m/deg  档位 %u/%u\r\n",
               (double)ADM_TAU_SIGN, (double)(s_adm_m * 83443.0f),
               (double)(s_adm_k * 83443.0f / 57.2958f),
               (unsigned)(s_adm_gain_cur + 1u), (unsigned)ADM_GAIN_LEVELS);
        if (torque_first_flag == 0u)
        {
            Adm_Fail("力矩传感器没有数据，先查 App_NJ70_Task / RS485");
            return;
        }
        printf("[ADM] 传感器就绪 tau = %+.4f N.m\r\n", (double)torque);
        s_adm_step = 2u;
        break;

    case 2u:
        if (!Adm_Rd(OD_STATUSWORD, 0u, &v))
        {
            Adm_Fail("读状态字超时");
            return;
        }
        printf("[ADM] 初始状态字 0x%04lX\r\n", (unsigned long)v);
        if ((v & SW_MASK_FAULT) == SW_STATE_FAULT)
        {
            printf("[ADM] 处于故障态，发 0x80 复位\r\n");
            Adm_Wr(OD_CONTROLWORD, 0u, CW_FAULT_RESET, 2u);
            HAL_Delay(200u);
        }
        s_adm_step = 3u;
        break;

    case 3u:
        /* 探一下驱动器支持哪些工作模式。0x6502 是只读位图：
         *   bit0 PP  bit1 VL  bit2 PV  bit5 IP  bit6 CSP  bit7 CSV  bit8 CST
         * 我们关心 IP(0x20)：插补位置模式下驱动器按 0x60C2 自己插补，
         * 不做"到点停"重规划，能消掉 PP 换点的 25 Hz 速度脉冲。 */
        if (Adm_Rd(OD_SUPPORTED_MODES, 0u, &v))
        {
            printf("[ADM] 0x6502 支持模式 = 0x%08lX :", (unsigned long)v);
            if (v & 0x0001u) printf(" PP");
            if (v & 0x0002u) printf(" VL");
            if (v & 0x0004u) printf(" PV");
            if (v & 0x0020u) printf(" IP");
            if (v & 0x0040u) printf(" CSP");
            if (v & 0x0080u) printf(" CSV");
            if (v & 0x0100u) printf(" CST");
            printf("\r\n");
            printf("[ADM]   IP(bit5) %s，CSP(bit6) %s\r\n",
                   (v & 0x0020u) ? "可用" : "不支持",
                   (v & 0x0040u) ? "可用" : "不支持");
        }
        else
        {
            printf("[ADM] 0x6502 读不到（部分驱动器不实现），只能靠试\r\n");
        }
        if (Adm_Rd(OD_MODES_OF_OPERATION, 0u, &v))
        {
            s_adm_mode_old = (uint8_t)(int8_t)v;
            printf("[ADM] 原运行模式 0x6060 = %d（仅记录）\r\n", (int)s_adm_mode_old);
        }
#if (ADM_USE_IP != 0u)
        if (!Adm_Wr(OD_MODES_OF_OPERATION, 0u, JOINT_MODE_IP, 1u))
        {
            Adm_Fail("写 0x6060 = 7 (IP 插补位置模式) 失败");
            return;
        }
#else
        if (!Adm_Wr(OD_MODES_OF_OPERATION, 0u, JOINT_MODE_PP, 1u))
        {
            Adm_Fail("写 0x6060 = 1 (PP 轮廓位置模式) 失败");
            return;
        }
#endif
        s_adm_step = 4u;
        break;

    case 4u:
        if (!Adm_Rd(OD_MODE_DISPLAY, 0u, &v))
        {
            Adm_Fail("读 0x6061 超时");
            return;
        }
#if (ADM_USE_IP != 0u)
        printf("[ADM] 模式确认 %d (%s)\r\n", (int)(int8_t)v,
               ((int8_t)v == JOINT_MODE_IP) ? "IP 插补位置模式" : "不是 IP！");
        if ((int8_t)v != JOINT_MODE_IP)
        {
            Adm_Fail("模式没有切到 7 (IP)");
            return;
        }
#else
        printf("[ADM] 模式确认 %d (%s)\r\n", (int)(int8_t)v,
               ((int8_t)v == JOINT_MODE_PP) ? "PP 轮廓位置模式" : "不是 PP！");
        if ((int8_t)v != JOINT_MODE_PP)
        {
            Adm_Fail("模式没有切到 1 (PP)");
            return;
        }
#endif
        s_adm_step = 5u;
        break;

    case 5u:
        /* 0x6081 轮廓速度：设成 20000 counts/s，远高于软件限速 ADM_VEL_MAX。
         * 只是给 PP 规划器一个上限，真正约束速度的是我们自己的递推。 */
        if (!Adm_Wr(OD_PROFILE_VELOCITY, 0u, (uint32_t)ADM_PROF_VEL, 4u))
        {
            printf("[WARN] 写 0x6081 失败 abort=0x%08lX\r\n",
                   (unsigned long)Dev_Joint_SdoAbortCode());
        }
        else if (Adm_Rd(OD_PROFILE_VELOCITY, 0u, &v))
        {
            printf("[ADM] 0x6081 轮廓速度 = %lu counts/s\r\n", (unsigned long)v);
        }
#if (ADM_USE_IP != 0u)
        /* IP 模式下 0x60C2 决定驱动器多久插补一次。只读确认，不写：
         * 出厂 子1=1、子2=-3 = 1 ms，这个值是对的。
         * 曾改成 10 想"与写点周期对齐"，结果 11.4 deg 稳态误差、
         * 驱动器出力约 0、手感完全消失。别碰。 */
        {
            uint32_t per = 0u, exp = 0u;

            if (Adm_Rd(OD_INTERP_TIME, 1u, &per) & Adm_Rd(OD_INTERP_TIME, 2u, &exp))
            {
                printf("[ADM] 0x60C2 插补周期 = %lu x 10^%ld ms（出厂 1 ms，保持不动）\r\n",
                       (unsigned long)per, (long)(int32_t)(int8_t)(exp & 0xFFu));
            }
            else
            {
                printf("[ADM] 0x60C2 读不到，按出厂 1 ms 处理\r\n");
            }
        }
#endif
        s_adm_step = 6u;
        break;

    case 6u:
        /* 0x6083/0x6084 加减速度：PP 每 20 ms 换一次点，驱动器要能在 20 ms
         * 内从 0 加到 ADM_VEL_MAX，所以 a >= ADM_VEL_MAX/0.02 = 400000。
         * 取 1e6 留 2.5 倍余量，代价是每拍 v^2/(2a) = 32 counts 的规划滞后。 */
        if (!Adm_Wr(OD_PROFILE_ACCEL, 0u, (uint32_t)ADM_PROF_ACC, 4u))
            printf("[WARN] 写 0x6083 失败 abort=0x%08lX\r\n",
                   (unsigned long)Dev_Joint_SdoAbortCode());
        if (!Adm_Wr(OD_PROFILE_DECEL, 0u, (uint32_t)ADM_PROF_ACC, 4u))
            printf("[WARN] 写 0x6084 失败 abort=0x%08lX\r\n",
                   (unsigned long)Dev_Joint_SdoAbortCode());
        if (Adm_Rd(OD_PROFILE_ACCEL, 0u, &v))
            printf("[ADM] 0x6083 轮廓加速度 = %lu counts/s^2（换点周期 %u ms）\r\n",
                   (unsigned long)v, (unsigned)ADM_STR_MS);
        s_adm_step = 7u;
        break;

    case 7u:
        /* TPDO1 映射 {0x6041, 0x6064} + 抑制时间清零 + 2 ms 事件定时。
         * 必须在 Pre-operational 下改映射，函数内部会自己退 PreOp。 */
        if (Dev_Joint_Tpdo1SetupPos() == 0u)
        {
            Adm_Fail("TPDO1 配置失败（没有位置反馈就没法跑导纳环）");
            return;
        }
        s_adm_step = 8u;
        break;

    case 8u:
        /* 使能前先把 0x607A 钉在当前位置并让驱动器采信，
         * 否则它会拿上一轮残留的目标一路冲过去（0x8400 就是这么来的）。 */
        if (!Adm_Rd(OD_POSITION_ACTUAL, 0u, &v))
        {
            Adm_Fail("读 0x6064 超时，拿不到起始位置");
            return;
        }
        s_adm_pos_ref   = (int32_t)v;
        s_adm_pos_fdb   = s_adm_pos_ref;
        s_adm_pos_cmd   = (float)s_adm_pos_ref;
        s_adm_pos_tgt   = (float)s_adm_pos_ref;   /* 静止时超前量 = 0 */
        s_adm_lead_v    = 0.0f;
        s_adm_delta_pos = 0.0f;
        s_adm_delta_vel = 0.0f;
        s_adm_tau_grav  = App_Joint_GetGravityTorque(s_adm_pos_fdb);
        printf("[ADM] 失能态位置 %ld counts (%.2f deg)，预置 0x607A\r\n",
               (long)s_adm_pos_ref, (double)COUNT_TO_DEG(s_adm_pos_ref));

        if (Adm_PpSend(s_adm_pos_ref) == 0u)
        {
            printf("[WARN] 使能前预置 0x607A 失败，继续（使能后会再钉一次）\r\n");
        }
        s_adm_step = 9u;
        break;

    case 9u:
        Adm_Wr(OD_CONTROLWORD, 0u, CW_SHUTDOWN, 2u);
        s_adm_step = 10u;
        break;

    case 10u:
        Adm_Wr(OD_CONTROLWORD, 0u, CW_SWITCH_ON, 2u);
        s_adm_step = 11u;
        break;

    case 11u:
        Adm_Wr(OD_CONTROLWORD, 0u, CW_ENABLE_OPERATION, 2u);
        s_adm_step = 12u;
        break;

    case 12u:
        if (!Adm_Rd(OD_STATUSWORD, 0u, &v))
        {
            Adm_Fail("使能后读状态字超时");
            return;
        }
        if ((v & SW_MASK_STATE) != SW_STATE_ENABLED)
        {
            printf("[FAIL] 没进 Operation enabled，状态字 0x%04lX（状态位 0x%02lX）\r\n",
                   (unsigned long)v, (unsigned long)(v & SW_MASK_STATE));
            printf("       查 STO_IN(PIN1)/STO_COM(PIN2) 是否短接、48V 是否到位\r\n");
            Adm_Fail("使能失败");
            return;
        }
        printf("[ADM] 已使能 0x%04lX\r\n", (unsigned long)v);

        /* 使能这一刻位置被驱动器保持住，读到的才是真值，用它重设基准，
         * 并立刻做一次完整换点把它钉成 PP 的目标。 */
        if (!Adm_Rd(OD_POSITION_ACTUAL, 0u, &v))
        {
            Adm_Fail("使能后读 0x6064 超时");
            return;
        }
        printf("[ADM] 使能后位置 %ld counts (%.2f deg)，与失能态相差 %+ld counts\r\n",
               (long)(int32_t)v, (double)COUNT_TO_DEG((int32_t)v),
               (long)((int32_t)v - s_adm_pos_ref));

        s_adm_pos_ref   = (int32_t)v;
        s_adm_pos_fdb   = s_adm_pos_ref;
        s_adm_pos_cmd   = (float)s_adm_pos_ref;
        s_adm_pos_tgt   = (float)s_adm_pos_ref;   /* 静止时超前量 = 0 */
        s_adm_lead_v    = 0.0f;
        s_adm_delta_pos = 0.0f;
        s_adm_delta_vel = 0.0f;
        s_adm_tau_grav  = App_Joint_GetGravityTorque(s_adm_pos_fdb);

        if (Adm_PpSend(s_adm_pos_ref) == 0u)
        {
            Adm_Fail("使能后钉住 0x607A 失败");
            return;
        }
        printf("[ADM] 锚点 pos_ref = %ld counts (%.2f deg)，tau_grav = %+.4f N.m\r\n",
               (long)s_adm_pos_ref, (double)COUNT_TO_DEG(s_adm_pos_ref), (double)s_adm_tau_grav);
        s_adm_step = 13u;
        break;

    case 13u:
        /* NMT Start：PDO 只有 Operational 下才生效 */
        Dev_Joint_NmtStart(1u);
        HAL_Delay(20u);

        s_adm_beat      = 0u;
        s_adm_pp_ok     = 0u;
        s_adm_pp_bad    = 0u;
        s_adm_pp_fail   = 0u;
        s_adm_tau_cnt   = 0u;
        s_adm_err_cnt   = 0u;
        s_adm_stop_rsn  = ADM_STOP_NONE;
        s_adm_pos_t     = uwTick;
        s_adm_mon_t     = uwTick;
        s_adm_print_t   = uwTick;
        s_adm_run_t0    = uwTick;
        s_adm_tpdo_lc   = Dev_Joint_TpdoPosCount();
        s_adm_fdb_valid = 1u;                 /* 锚点已经可信 */
        s_adm_pos_home  = s_adm_pos_ref;      /* 软限位的绝对基准，之后不再动 */

        s_adm_tgt_prev = (int32_t)s_adm_pos_tgt;
        s_adm_jump_pk  = 0;
        s_adm_err_pk   = 0;

        /* TIM3 提到 1 kHz：PSC 固定 71（1 MHz），ARR 从 10000-1 改成 999 */
        htim3.Instance->ARR = (uint32_t)(ADM_TIM_ARR - 1u);
        htim3.Instance->EGR = TIM_EGR_UG;
        __HAL_TIM_SET_COUNTER(&htim3, 0u);
        __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
        HAL_TIM_Base_Start_IT(&htim3);

#if (ADM_SELFTEST_MS > 0u)
        s_adm_hold   = 1u;
        s_adm_hold_t = uwTick;
        printf("[ADM] 静态保持自检 %u ms：目标钉死在 %ld，看驱动器会不会自己报错\r\n",
               (unsigned)ADM_SELFTEST_MS, (long)s_adm_pos_ref);
#endif

        s_adm_run  = 1u;
        s_adm_step = ADM_STEP_RUN;
        printf("[ADM] [ OK ] 控制环已启动，按键再按一次停机\r\n");
        printf("[ADM]    t_ms     cmd    fdb    err     tgt tau_ext  ok  bad    tq     vel\r\n");
        printf("[ADM]   tgt = 最后一次采信的 0x607A；tq = 0x6077(‰额定)；vel = 0x606C(counts/s)\r\n");
        printf("[ADM]   err = tgt - fdb（驱动器真正感受到的偏差，不是 cmd - fdb）\r\n");
        printf("[ADM]   速度上限 %.0f counts/s (%.1f deg/s)，超前补偿 %.0f ms\r\n",
               (double)ADM_VEL_MAX, (double)COUNT_TO_DEG((int32_t)ADM_VEL_MAX), (double)ADM_LEAD_MS);
#if (ADM_USE_IP != 0u)
        printf("[ADM]   机制：IP 插补（0x6060=7，只写 0x607A，驱动器按 0x60C2 自己插补）\r\n");
        printf("[ADM]   vel 应连续变化；若 vel 仍是 +-24000 脉冲，说明 IP 也按 0x6083 重规划\r\n");
#else
        printf("[ADM]   机制：PP 轮廓（0x0F->0x607A->0x3F 三帧 SDO）\r\n");
#endif
        break;

    default:
        s_adm_step = ADM_STEP_IDLE;
        break;
    }
}

/* =====================================================================
 *  导纳控制 · 对外接口
 * ===================================================================== */
void App_Joint_Adm_Start(void)
{
    if (s_adm_step != ADM_STEP_IDLE) return;

    s_adm_stop_rsn = ADM_STOP_NONE;
    s_adm_step_t   = uwTick;
    s_adm_step     = 1u;

    /* 上一次停机如果没收干净，通道可能还挂在 WAIT 上 */
    Dev_Joint_SdoReset();
    s_adm_sw_last = 0xFFFFu;
    s_adm_hold    = 0u;
    s_adm_pp_sent = 0;
}

/* 把 0x603F 故障码翻成人话 */
static const char *Adm_DrvFaultName(uint32_t code)
{
    switch (code & 0xFFFFu)
    {
    case 0x0000u: return "无故障";
    case 0x2310u: return "过流";
    case 0x2311u: return "过流(2)";
    case 0x3210u: return "过压";
    case 0x3220u: return "欠压";
    case 0x3230u: return "功率级/母线异常";
    case 0x4210u: return "过热";
    case 0x7305u: return "编码器故障";
    case 0x7500u: return "通信/存储故障";
    case 0x8110u: return "CAN 接收溢出";
    case 0x8120u: return "CAN 被动错误";
    case 0x8130u: return "心跳丢失";
    case 0x8400u: return "速度环错误";
    case 0x8611u: return "跟随误差过大";
    case 0x8612u: return "指令超限/参考点";
    case 0xFF00u: return "厂商自定义故障";
    default:      return "其他(查手册)";
    }
}

/* 按键主动停机：单独一个入口，只为了把停机原因写成"按键" */
void App_Joint_Adm_StopKey(void)
{
    s_adm_stop_rsn = ADM_STOP_KEY;
    App_Joint_Adm_Stop();
}

void App_Joint_Adm_Stop(void)
{
    uint32_t v = 0u;
    uint8_t  was_run;

    if (s_adm_step == ADM_STEP_IDLE) return;

    was_run   = (s_adm_step == ADM_STEP_RUN) ? 1u : 0u;
    s_adm_run = 0u;
    HAL_TIM_Base_Stop_IT(&htim3);

    /* 先清 SDO 残留：停定时器那一刻可能还有一帧在飞，
     * 不清掉的话下面每一条诊断读都会立刻失败。 */
    Dev_Joint_SdoReset();
    HAL_Delay(2u);

    if (was_run != 0u)
    {
        /* ---- PP 的"停"就是把目标钉在当前实际位置 ----
         * 不立刻失能：先让位置环在原地刹住，再走 SHUTDOWN，
         * 避免松手瞬间关节靠抱闸硬停。 */
        if (Adm_Rd(OD_POSITION_ACTUAL, 0u, &v))
        {
            s_adm_pos_fdb = (int32_t)v;
            if (Adm_PpSend((int32_t)v) == 0u)
                printf("[ADM] 停机时钉住 0x607A 失败，直接走失能\r\n");
            else
                printf("[ADM] 已把目标钉在 %ld counts，等 150 ms 刹住\r\n", (long)(int32_t)v);
            HAL_Delay(150u);
        }

        /* ---- 停机前回读驱动器（ISR 已停，不占通道）---- */
        if (Adm_Rd(OD_STATUSWORD, 0u, &v))
        {
            s_adm_sw_last = (uint16_t)v;
            printf("[ADM] 0x6041 状态字 = 0x%04lX%s\r\n", (unsigned long)v,
                   ((v & SW_STATE_FAULT) != 0u) ? "   <-- bit3 故障，驱动器已断输出" : "");
            if ((v & SW_STATE_FAULT) != 0u) s_adm_stop_rsn = ADM_STOP_DRV_FAULT;
        }
        else
        {
            printf("[ADM] 0x6041 读失败\r\n");
        }

        if (Adm_Rd(OD_ERROR_CODE, 0u, &v))
        {
            s_adm_drv_err = v;
            printf("[ADM] 0x603F 故障码 = 0x%04lX (%s)\r\n",
                   (unsigned long)(v & 0xFFFFu), Adm_DrvFaultName(v));
        }
        else
        {
            printf("[ADM] 0x603F 读失败\r\n");
        }

        if (Adm_Rd(OD_TARGET_POSITION, 0u, &v))
            printf("[ADM] 0x607A 目标 = %ld  0x6064 实际 = %ld  差 %+ld counts\r\n",
                   (long)(int32_t)v, (long)s_adm_pos_fdb,
                   (long)((int32_t)v - s_adm_pos_fdb));
        else
            printf("[ADM] 0x607A 读失败\r\n");

        if (Adm_Rd(OD_FOLLOWING_ERR, 0u, &v))
            printf("[ADM] 0x6065 跟随误差窗口 = %lu counts (%.1f deg)\r\n",
                   (unsigned long)v, (double)COUNT_TO_DEG((int32_t)v));

        if (Adm_Rd(OD_TORQUE_ACTUAL, 0u, &v))
            printf("[ADM] 0x6077 实际力矩 = %d ‰额定  (0 = 完全没出力)\r\n", (int)(int16_t)v);

        if (Adm_Rd(OD_VELOCITY_ACTUAL, 0u, &v))
            printf("[ADM] 0x606C 实际速度 = %ld counts/s\r\n", (long)(int32_t)v);

        printf("[ADM] TPDO1 累计 %lu 帧\r\n", (unsigned long)Dev_Joint_TpdoPosCount());
    }

    /* 失能。PP 下目标已经钉在原地，失能不会冲向任何点。 */
    Adm_Wr(OD_CONTROLWORD, 0u, CW_SHUTDOWN, 2u);

    Dev_Joint_SetPosFreeze(0u);          /* 交还 SDO 给 app_nj70 */

    printf("\r\n[ADM] ---- 已停止，原因: ");
    switch (s_adm_stop_rsn)
    {
    case ADM_STOP_KEY:  printf("按键\r\n");                          break;
    case ADM_STOP_TAU:  printf("外力矩超 %.1f N.m\r\n", (double)ADM_TAU_STOP); break;
    case ADM_STOP_ERR:  printf("跟随误差超 %ld counts\r\n", (long)ADM_FOLLOW_ERR_MAX); break;
    case ADM_STOP_PP:   printf("PP 换点连续 %u 次失败（SDO 不通 / 驱动器拒绝）\r\n",
                               (unsigned)ADM_PP_FAIL_STOP); break;
    case ADM_STOP_FDB_LOST: printf("位置反馈断流（TPDO 不来帧）\r\n");  break;
    case ADM_STOP_DRV_FAULT:
        printf("驱动器自身故障 0x%04lX (%s)\r\n",
               (unsigned long)(s_adm_drv_err & 0xFFFFu), Adm_DrvFaultName(s_adm_drv_err));
        break;
    case ADM_STOP_STEP: printf("启动流程失败\r\n");                  break;
    default:            printf("未知\r\n");                          break;
    }

    if (was_run != 0u)
    {
        printf("[ADM] 递推 %lu 拍(100 Hz) / 换点成功 %lu 次 / 失败 %lu 次\r\n",
               (unsigned long)s_adm_beat, (unsigned long)s_adm_pp_ok,
               (unsigned long)s_adm_pp_bad);
        /* 上限 = 限速贡献 VEL_MAX*DT + 超前项贡献 2*VEL_MAX*LEAD_S*LPF
         *      = 130 + 183 = 313 counts。
         * 超前项前面那个 2 不能漏：换向时 delta_vel 是跨零翻转（-V 直接到 +V），
         * 超前量单拍变化量是 2*V*LPF，不是 V*LPF。少乘 2 会误判"超限"。 */
        printf("[ADM] 单拍位置指令跳变峰值 = %ld counts（理论上限 %ld）\r\n",
               (long)s_adm_jump_pk,
               (long)(int32_t)(ADM_VEL_MAX * ADM_DT
                               + 2.0f * ADM_VEL_MAX * ADM_LEAD_S * ADM_LEAD_LPF));
        printf("[ADM] |指令-反馈| 峰值 = %ld counts (%.1f deg)\r\n",
               (long)s_adm_err_pk, (double)COUNT_TO_DEG(s_adm_err_pk));
        printf("[ADM] 净漂移 %+ld counts (%.3f deg)\r\n",
               (long)(s_adm_pos_fdb - s_adm_pos_home),
               (double)COUNT_TO_DEG(s_adm_pos_fdb - s_adm_pos_home));
    }

    (void)Dev_Joint_SdoPoll(&v);
    s_adm_step     = ADM_STEP_IDLE;
    s_adm_stop_rsn = ADM_STOP_NONE;
}

uint8_t App_Joint_Adm_IsBusy(void)
{
    return (s_adm_step != ADM_STEP_IDLE) ? 1u : 0u;
}

/* =====================================================================
 *  导纳控制 · 主循环任务
 *  1) 启动阶段：20 ms 一步
 *  2) 运行阶段：20 ms 一拍 —— 取 TPDO -> 换点 -> 看门狗 -> 定期打印
 * ===================================================================== */
void App_Joint_Adm_Task(void)
{
    uint32_t v  = 0u;
    int32_t  err;
    int32_t  tgt;
    float    te;

    if (s_adm_step == ADM_STEP_IDLE) return;

    /* ---- 1. ISR 自己停了（不可能，留个兜底）---- */
    if (s_adm_step == ADM_STEP_RUN && s_adm_run == 0u)
    {
        App_Joint_Adm_Stop();
        return;
    }

    /* ---- 2. 启动阶段 ---- */
    if (s_adm_step != ADM_STEP_RUN)
    {
        if ((uwTick - s_adm_step_t) < 20u) return;
        s_adm_step_t = uwTick;
        App_Joint_Adm_Step();
        return;
    }

    /* ---- 3. 运行阶段：ADM_STR_MS 一拍（40 ms，见头文件 v_max 推导）---- */
    if ((uwTick - s_adm_mon_t) < ADM_STR_MS) return;
    s_adm_mon_t = uwTick;

    Adm_PollTpdo();

    /* ---- 3a. PP 换点 ----
     * 目标直接用递推出来的 pos_cmd，取整后下发。
     * 死区 2 counts：指令几乎不变时不用重复换点（省 SDO、也少一次 bit4 翻转）。 */
    tgt = (int32_t)s_adm_pos_tgt;
    {
        int32_t d = tgt - s_adm_pp_sent;
        if (d < 0) d = -d;

        if (d >= ADM_STR_DEADBAND)
        {
            if (Adm_PpSend(tgt) != 0u)
            {
                s_adm_pp_ok++;
                s_adm_pp_fail = 0u;
            }
            else
            {
                s_adm_pp_bad++;
                if (s_adm_pp_fail < 200u) s_adm_pp_fail++;
                if (s_adm_pp_fail >= ADM_PP_FAIL_STOP)
                {
                    s_adm_stop_rsn = ADM_STOP_PP;
                    App_Joint_Adm_Stop();
                    return;
                }
            }
        }
    }

    /* ---- 静态保持自检到期：把环交还给导纳 ---- */
    if (s_adm_hold != 0u && (uwTick - s_adm_hold_t) >= ADM_SELFTEST_MS)
    {
        s_adm_hold = 0u;
        printf("[ADM] 静态保持自检通过：%u ms 内目标未动，驱动器无故障（0x6041=0x%04X）\r\n",
               (unsigned)ADM_SELFTEST_MS, (unsigned)s_adm_sw);
        printf("[ADM]   |err| 峰值 %ld counts (%.1f deg)，单拍跳变 %ld counts\r\n",
               (long)s_adm_err_pk, (double)COUNT_TO_DEG(s_adm_err_pk), (long)s_adm_jump_pk);
        s_adm_tgt_prev = (int32_t)s_adm_pos_tgt;
        s_adm_jump_pk  = 0;
        s_adm_err_pk   = 0;
    }

    /* ---- 看门狗 0：驱动器自己还活着吗（状态字来自 TPDO，不占 SDO）---- */
    if (s_adm_sw != 0xFFFFu)
    {
        if (s_adm_sw_last == 0xFFFFu)
        {
            s_adm_sw_last = s_adm_sw;
        }
        else if (s_adm_sw != s_adm_sw_last)
        {
            if ((s_adm_sw & SW_STATE_FAULT) != 0u)
            {
                if (Adm_Rd(OD_ERROR_CODE, 0u, &v))
                {
                    s_adm_drv_err = v;
                    printf("[ADM] 驱动器故障 0x603F = 0x%04lX (%s)\r\n",
                           (unsigned long)(v & 0xFFFFu), Adm_DrvFaultName(v));
                }
                printf("[ADM] 运行中状态字 0x%04X -> 0x%04X\r\n",
                       (unsigned)s_adm_sw_last, (unsigned)s_adm_sw);
                s_adm_stop_rsn = ADM_STOP_DRV_FAULT;
                App_Joint_Adm_Stop();
                return;
            }
            printf("[ADM] 运行中状态字 0x%04X -> 0x%04X\r\n",
                   (unsigned)s_adm_sw_last, (unsigned)s_adm_sw);
            s_adm_sw_last = s_adm_sw;
        }
    }

    /* 重力矩按【实测位置】算：重力矩只取决于真实姿态，
     * 用 pos_cmd 算会把"指令与实际不一致"变成虚假外力灌进导纳环。 */
    s_adm_tau_grav = App_Joint_GetGravityTorque(s_adm_pos_fdb);

    /* ---- 看门狗 1：外力矩 ----
     * NJ70 夹在关节与连杆之间，读的是关节输出轴作用在连杆上的力矩，
     * 里面包含驱动器自己的出力。所以阈值放到 12 N.m 并且要连续 10 拍。 */
    te = torque - s_adm_tau_grav;
    if (te > ADM_TAU_STOP || te < -ADM_TAU_STOP)
    {
        if (s_adm_tau_cnt < 200u) s_adm_tau_cnt++;
        if (s_adm_tau_cnt >= ADM_TAU_STOP_CNT)
        {
            s_adm_stop_rsn = ADM_STOP_TAU;
            App_Joint_Adm_Stop();
            return;
        }
    }
    else
    {
        s_adm_tau_cnt = 0u;
    }

    /* ---- 看门狗 2：位置反馈年龄（TPDO 断流）---- */
    if ((uwTick - s_adm_pos_t) > ADM_POS_AGE_STOP)
    {
        s_adm_stop_rsn = ADM_STOP_FDB_LOST;
        App_Joint_Adm_Stop();
        return;
    }

    /* ---- 看门狗 3：跟随误差（连续 3 拍）---- */
    err = (int32_t)s_adm_pos_tgt - s_adm_pos_fdb;   /* 驱动器真正感受到的偏差 */
    if (err > ADM_FOLLOW_ERR_MAX || err < -ADM_FOLLOW_ERR_MAX)
    {
        if (s_adm_err_cnt < 200u) s_adm_err_cnt++;
        if (s_adm_err_cnt >= 3u)
        {
            s_adm_stop_rsn = ADM_STOP_ERR;
            App_Joint_Adm_Stop();
            return;
        }
    }
    else
    {
        s_adm_err_cnt = 0u;
    }

    /* ---- 定期打印（500 ms）---- */
    if ((uwTick - s_adm_print_t) >= 500u)
    {
        s_adm_print_t = uwTick;

        if (Adm_Rd(OD_TORQUE_ACTUAL, 0u, &v))
            s_adm_tq_pm = (int32_t)(int16_t)(v & 0xFFFFu);
        if (Adm_Rd(OD_VELOCITY_ACTUAL, 0u, &v))
            s_adm_vel_fb = (int32_t)v;
        s_adm_tau_echo = (int32_t)(te * ADM_TAU_SIGN * 1000.0f);

        printf("[ADM] %6lu  %7ld  %7ld  %5ld  %7ld  %+6.3f  %4lu %4lu  %5ld %7ld\r\n",
               (unsigned long)(uwTick - s_adm_run_t0),
               (long)(int32_t)s_adm_pos_cmd,
               (long)s_adm_pos_fdb,
               (long)err,
               (long)s_adm_pp_sent,
               (double)((float)s_adm_tau_echo / 1000.0f),
               (unsigned long)s_adm_pp_ok,
               (unsigned long)s_adm_pp_bad,
               (long)s_adm_tq_pm, (long)s_adm_vel_fb);
    }
}

/* =====================================================================
 *  运行时增益档位
 *  KEY1 -> App_Joint_Adm_CycleGain() -> 只写请求位 -> ISR 拍边界生效
 * ===================================================================== */
static void Adm_ApplyGain(uint8_t idx)
{
    float sc;

    if (idx >= ADM_GAIN_LEVELS) idx = 0u;
    sc = s_adm_gain_scale[idx];

    s_adm_m     = ADM_M * sc;
    s_adm_b     = ADM_B * sc;
    s_adm_k     = ADM_K * sc;
    s_adm_inv_m = 1.0f / s_adm_m;      /* 一次除法，只在切档那一拍做 */

    s_adm_gain_cur = idx;
}

uint8_t App_Joint_Adm_GetGain(void)
{
    return s_adm_gain_req;
}

uint8_t App_Joint_Adm_SetGain(uint8_t idx)
{
    if (idx >= ADM_GAIN_LEVELS) idx = 0u;
    s_adm_gain_req = idx;
    return idx;
}

/* 循环切到下一档，打印一行。返回新档位（1..3，给人看的编号）。
 * 运行中切档不重置 delta_pos / delta_vel：位置连续、力矩突变，
 * 手感觉到的就是"弹簧突然变硬/变软"，正是要对比的东西。 */
uint8_t App_Joint_Adm_CycleGain(void)
{
    static const char *nm[ADM_GAIN_LEVELS] = { "软", "中", "硬" };
    /* 按 中 -> 软 -> 硬 走，不是按编号走。默认停在中档，
     * 第一下更省力、第二下更硬，比"第一下直接跳到最硬"好对比。 */
    static const uint8_t ord[ADM_GAIN_LEVELS] = { 1u, 0u, 2u };
    uint8_t i;
    uint8_t idx;
    float   kk;

    for (i = 0u; i < ADM_GAIN_LEVELS; i++)
    {
        if (ord[i] == s_adm_gain_req) break;
    }
    if (i >= ADM_GAIN_LEVELS) i = 0u;
    idx = ord[(uint8_t)((i + 1u) % ADM_GAIN_LEVELS)];

    s_adm_gain_req = idx;

    kk = ADM_K * s_adm_gain_scale[idx] * 83443.0f / 57.2958f;   /* N.m/deg */

    printf("[ADM] 档位 %u/%u = %s  K=%.3f N.m/deg（45 deg 对应 %.1f N.m）"
           "  M=%.2e B=%.2e%s\r\n",
           (unsigned)(idx + 1u), (unsigned)ADM_GAIN_LEVELS, nm[idx],
           (double)kk, (double)(kk * ADM_LIMIT_DEG),
           (double)(ADM_M * s_adm_gain_scale[idx]),
           (double)(ADM_B * s_adm_gain_scale[idx]),
           (App_Joint_Adm_IsBusy() != 0u) ? "  [运行中，下一拍生效]" : "");

    return (uint8_t)(idx + 1u);
}
