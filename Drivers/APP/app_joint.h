#ifndef __APP_JOINT_H
#define __APP_JOINT_H

#include "main.h"
#include "dev_joint.h"

/* ---- 位置定标（一圈 = 524288 counts，19 位多圈绝对值编码器）---- */
#define POS_PER_REV       524288.0f
#define DEG_TO_COUNT(d)   ((int32_t)((d) * POS_PER_REV / 360.0f))
#define COUNT_TO_DEG(c)   ((float)(c) * 360.0f / POS_PER_REV)
#define POS_TO_RAD(p)     ((float)(p) * (6.283185307f / POS_PER_REV))
#define DEG_TO_RAD(d)     ((d) * 3.14159265358979f / 180.0f)

/* ---- 重力矩补偿参数 tau_g(th) = a*cos(th) + b*sin(th) + c ----
 * 2026-10-23 拟合，输入为绝对角度（0x6064），不含任何偏置。
 */
#define GRAV_A          (-0.191777f)
#define GRAV_B          (-0.347195f)
#define GRAV_C          ( 0.004957f)

float App_Joint_GetGravityTorque(int32_t pos);

/* ---- 周期任务（主循环里调）---- */
void App_Joint_Task(void);

/* ---- 扫角自检（PP 模式 8 点采集，重拟合重力矩时用）---- */
void App_Joint_GravScan_Task(void);

/* =====================================================================
 *  导纳控制 · PP 位置流版
 *
 *  方程： M*acc_cmd + B*vel_cmd + K*(pos_cmd - pos_ref) = tau_ext
 *
 *  变量命名（2026-10-29 统一）：
 *      s_adm_pos_ref    平衡位置，启动时取使能后实际位置，运行中不变
 *      s_adm_pos_cmd    导纳模型算出的位置指令，每 20 ms 写进 0x607A
 *      s_adm_pos_fdb    驱动器回报的实际位置 0x6064（TPDO）
 *      s_adm_delta_vel  位移量的速度，不是电机实际速度
 *      s_adm_tau_grav   当前姿态下的重力矩
 *
 *  单位：pos = counts，vel = counts/s，tau = N.m
 *
 *  三档预设，改 ADM_PRESET 一个数就切换软硬：
 *      1 = 默认（1 N.m 退 2 deg，wn 1.51 Hz）—— 唯一推荐档
 *      2 = 软（1 N.m 退 11.4 deg）
 *      3 = 极软（1 N.m 退 114 deg，仅作对照）
 *
 *  为什么 2/3 不推荐：软限位只有 +-45 deg，导纳的稳态位移 d = tau/K。
 *  预设 2/3 的 K 太小，随便一推就要退几十上百度，直接撞限位；
 *  而且速度上限 6.9 deg/s 摆在那，平衡点在限位外就永远走不到，
 *  手感会变成"匀速爬行、推不动"。实测预设 1 的旧值(K=0.219)推 9.5 N.m
 *  需要退 43.4 deg、实际 1 s 只走了 22.8 deg，就是这个问题。
 * ===================================================================== */
#ifndef ADM_PRESET
#define ADM_PRESET          1
#endif

/* 阻尼比 1（临界阻尼），三档只有刚度不同：
 *   M = K / wn^2      B = 2*sqrt(K*M)
 *   counts/rad = 524288/(2*pi) = 83443 */
#if   ADM_PRESET == 1
#define ADM_M               (3.80e-6f)              /* J = 0.32 kg.m^2 */
#define ADM_B               (7.22e-5f)              /* 2*sqrt(K*M)，zeta=1.0 */
#define ADM_K               (3.43e-4f)              /* 0.5 N.m/deg，1 N.m 退 2 deg */
#elif ADM_PRESET == 2
#define ADM_M               (1.52e-6f)              /* J = 0.13 kg.m^2 */
#define ADM_B               (1.91e-5f)
#define ADM_K               (6.0e-5f)               /* 0.087 N.m/deg */
#else
#define ADM_M               (1.52e-7f)              /* J = 0.013 kg.m^2 */
#define ADM_B               (1.91e-6f)
#define ADM_K               (6.0e-6f)               /* 0.009 N.m/deg */
#endif

#define ADM_INV_M           (1.0f / ADM_M)
#define ADM_DT              (0.01f)                 /* 导纳递推周期 10 ms */

/* ---- 递推与换点的节拍 ----
 * TIM3 PSC=71 -> 1 MHz；进 RUN 前把 ARR 从 9999 改成 999 -> 1 kHz 中断。
 * 每 ADM_SUB(10) 拍做一次递推 = 100 Hz；换点走 ADM_STR_MS(20 ms) = 50 Hz。 */
#define ADM_ISR_HZ          (1000u)
#define ADM_SUB             (10u)
#define ADM_TIM_ARR         (1000u)
#define ADM_STR_MS          (40u)                   /* PP 换点周期，v_max 的分子，见下方推导 */

/* 换点死区：指令变化不到这么多 counts 就不发新点。
 * 省 SDO，也少一次 bit4 翻转；代价是 2 counts 的量化误差（0.0014 deg）。 */
#define ADM_STR_DEADBAND    (2)

/* ---- PP 轮廓参数（0x6081 / 0x6083 / 0x6084）----
 * ADM_PROF_VEL 只是给规划器的上限，真正的速度约束是 ADM_VEL_MAX。
 * 加速度必须满足 a >= ADM_VEL_MAX / (ADM_STR_MS/1000) = 30000/0.02 = 1.5e6，
 * 否则 PP 在 20 ms 内追不上点。取 3e6，2 倍余量。 */
#define ADM_PROF_VEL        (45000L)                /* counts/s = 30.9 deg/s，必须 > VEL_MAX */
#define ADM_PROF_ACC        (1800000L)              /* counts/s^2，见下方惯性力矩推导 */

#define ADM_LIMIT_DEG       (45.0f)                 /* 软限位，相对启动位置 */
#define ADM_LIMIT_CNT       ((int32_t)(ADM_LIMIT_DEG * POS_PER_REV / 360.0f))

/* 速度限幅 counts/s。18000 = 12.4 deg/s。
 *
 * 2026-10-30 第四次定标，关键发现：v_max = a*dt/4 里的 dt 也是杠杆。
 * 之前几轮一直想靠抬 a 换速度，但 a 恰恰就是惯性力矩的旋钮
 * （tau = J*a/83443，J=0.317），抬 a 必震，于是卡在 6.9 deg/s 出不来。
 * 而把换点周期 dt 加大一倍，速度上限直接翻倍，力矩一点不多：
 *     dt=20ms a=1.8e6 -> v_max= 9000 (6.2 deg/s) 单拍台阶 0.12 deg
 *     dt=40ms a=1.8e6 -> v_max=18000 (12.4 deg/s) 单拍台阶 0.49 deg
 * 代价只是指令更新从 50 Hz 降到 22 Hz，而导纳带宽 1.51 Hz，余量 7 倍，够。
 *
 * 历史（这条线夹了四跑）：
 *   8000  (5.5 deg/s)  不震但顶力大 -> "费力"（当时还没有超前补偿）
 *   30000 (20.6 deg/s) a=3e6 追不上 -> 极限环 -> 震动
 *   10000 (6.9 deg/s)  a=1.8e6 -> tq 掉到 -16~-88 permille，震动消失，但"推不动"
 *   18000 (12.4 deg/s) 本版，dt 拉到 40 ms、a 保持 1.8e6，所以仍然不震
 */
#define ADM_VEL_MAX         (18000.0f)

/* ---- 换点超前补偿 ----
 * 位置型导纳里，手上感到的力 = 驱动器位置环刚度 × (tgt - fdb)。
 * 驱动器刚度按 tq 反算约 20~30 N.m/deg，是导纳刚度 0.219 的 100 倍量级，
 * 所以 0.15 deg 的跟踪滞后就能变成 3~6 N.m 的顶力，把导纳完全盖住。
 * 把目标按当前速度前移一点，让驱动器"追得正好"，err 压下去，顶力就没了。
 *
 * 但超前量本身也是误差：驱动器会把它当成偏差去追，所以补过头一样出力、一样震。
 * 实测：v=10000/dt=22.5ms 时真滞后 12 ms；dt=40ms 时真滞后约 30 ms
 * （台阶变大，驱动器追得更久），取 32。
 *   0    = 关闭
 *   大   = err 变负、力矩反向、开始震（2026-10-30 用 30 在 29000 速度下就是这样）
 *   小   = err 恒正、顶力回潮 */
#define ADM_LEAD_MS         (32.0f)
#define ADM_LEAD_S          (ADM_LEAD_MS * 0.001f)

/* 超前量用的一阶低通（TIM3 100 Hz 下 0.3 = 33 ms 时间常数）。
 * 换向瞬间 delta_vel 会从 -16000 跳到 +16000，超前量如果跟着硬跳，
 * 目标位置会瞬移 500+ counts，那一下就是一次可听见的"哒"。
 * 滤一下，换向冲击没了，代价是换向后 30 ms 内补偿略欠。 */
#define ADM_LEAD_LPF        (0.30f)

/* 外力矩保护。原值 12 N.m 太低：传感器满量程 25 N.m，正常一推就 11.3 N.m，
 * 余量只剩 0.7 N.m，稍一使劲就跳闸停机；停机后目标被钉死，再推就是硬顶，
 * 手感变成"完全推不动"。2026-10-30 实测就是这个原因，不是阻尼太大。
 * 放到 20，对满量程留 5 N.m 余量。 */
#define ADM_TAU_STOP        (20.0f)                 /* 外力矩保护 N.m */
#define ADM_TAU_STOP_CNT    (10u)                   /* 连续 10 拍(100ms，ISR 100 Hz)才停 */

/* 力矩方向：2026-10-30 实测确定为 -1。
 * 传感器夹在关节与连杆之间，读的是"关节作用在连杆上"的力矩，
 * 关节受到的力与之相反。 */
#define ADM_TAU_SIGN        (-1.0f)
#define ADM_FOLLOW_ERR_MAX  (25000)                 /* 指令-反馈超此值停机（17 deg）*/

#define ADM_POS_AGE_STOP    (200u)                  /* TPDO 200 ms 不来帧 -> 停机 */
#define ADM_PP_FAIL_STOP    (5u)                    /* 换点连续失败 5 次 -> 停机 */

/* 使能后的"静态保持自检"时长 ms，0 = 关闭。
 * 目标钉死在锚点，看驱动器会不会自己报错。
 * 验证没问题后改成 0 即可。 */
#define ADM_SELFTEST_MS     (2000u)

void    App_Joint_Adm_Start(void);
void    App_Joint_Adm_Stop(void);
void    App_Joint_Adm_StopKey(void);
void    App_Joint_Adm_Task(void);
uint8_t App_Joint_Adm_IsBusy(void);

#endif
