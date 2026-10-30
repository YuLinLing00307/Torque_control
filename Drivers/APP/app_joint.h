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

/* 阻尼比 0.7（欠阻尼一档），三档只有刚度不同：
 *   M = K / wn^2      B = 2*zeta*sqrt(K*M)   zeta = 0.7
 *   counts/rad = 524288/(2*pi) = 83443
 *
 * 2026-10-30 第二次手感标定。上一版 K=0.5 / ζ=0.7 上机实测：
 *   推力 15.9 N.m 走 30.7 deg，比例 0.518 N.m/deg —— 导纳完全按 K 走，环是准的；
 *   err 峰值仅 1.4 deg、tq 稳在 +-400 permille —— 顶力很小。
 *   用户仍觉得“难推”，那推的不是摩擦、也不是顶力，就是 0.5 N.m/deg 这个力/角度比。
 * 降到 0.40：同样 15.9 N.m 走 39.8 deg（原 30.7，多走 29%）；
 * 45 deg 限位对应 18 N.m，仍在 ADM_TAU_STOP=20 之下，留 2 N.m 余量。
 * M、B 按同比例缩 0.8，wn(1.51 Hz) 和 zeta(0.7) 都不变 —— 纯粹变软，响应快慢不变。
 * 第三版把 K 定到 0.30：按人的舒适持续推力 30~50 N、力臂 0.3 m 算，
 * 舒适力矩 9~15 N.m，对应“45 deg 限位处刚好 10~15 N.m”-> K = 0.22~0.33。
 * 0.40 让 45 deg 要 18 N.m，偏硬；0.30 给 13.5 N.m，落在舒适区中间。
 * 降 K 的含义：推到同一个角度，用力少 25%（12 -> 9 N.m @ 30 deg）；
 * 花同样的力，走得更远 33%。两者是同一件事的两种说法。
 * 若还嫌“跟不上”（不是嫌重），下一招是提 wn：M 再减半、B 同步减，
 * 但 M 太小会“发飘”，推起来没有重量感，是另一个极端。 
 */
#if   ADM_PRESET == 1
#define ADM_M               (1.0e-6f)              
#define ADM_B               (2.0e-5f)              
#define ADM_K               (2.5e-5f)              /* 极软，非常容易推 */
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
/* 换点周期。
 * PP 下它是速度上限的分子：v_max = a*dt/4，40 ms 配 a=1.8e6 得 18000。
 * IP 下这个约束不成立（驱动器不做"到点停"重规划，每拍不用从 0 起步），
 * 所以如果 IP 跑顺了，可以试 40 -> 20 ms，速度还能再翻一倍，
 * 而且不再有台阶大小和 a 的耦合。先按 40 跑通再说。 */
#define ADM_STR_MS          (40u)

/* 换点死区：指令变化不到这么多 counts 就不发新点。
 * 省 SDO，也少一次 bit4 翻转；代价是 2 counts 的量化误差（0.0014 deg）。 */
#define ADM_STR_DEADBAND    (2)

/* ==== 换点机制：IP 还是 PP ====
 * 0 = PP 轮廓位置（0x6060=1）：0x0F -> 写 0x607A -> 0x3F，靠 bit4 上升沿采信。
 *     已验证可用：净漂移 +-40 counts、760/385 次换点 0 失败、0x603F 全 0。
 *     短板：每点按"到点停"重规划 -> 三角速度剖面，40 ms 台阶 720 counts
 *     -> 峰值 sqrt(a*d)=36000 counts/s，0x606C 上 +-24000 脉冲，25 Hz 一次
 *     -> 手上"一冲一停、不顺应"。这个剖面在 PP 里调参数调不掉，只能换机构。
 *
 * 1 = IP 插补位置（0x6060=7）：只写 0x607A，驱动器按 0x60C2(1 ms) 自己插补。
 *     0x6502 = 0x3ED 确认支持（bit5 = 1）。硬件确实支持。
 *
 *     【2026-10-30 实测：跑不通，已回退。别把这条当"更好的方案"再捡回来】
 *     表面数据很漂亮：静态保持 err 20~45 counts、tq 个位数 permille、
 *     1453 拍 / 191 次换点 0 失败、净漂移 +0 counts。
 *
 *     但一上手推就废，两次运行同一个死法：
 *       A) 推 4.5 N.m，cmd 212228 -> 214295（走 1.6 deg），
 *          fdb 只走 +50 counts 就卡住 -> err 2305 -> 一路冲到 9609
 *       B) cmd 229115 -> 228484，fdb 229065 -> 229087 纹丝不动
 *     两次 |cmd-fdb| 峰值 9609 / 9585 counts（6.6 deg），一致得离谱，
 *     然后 8000 阈值的跟随误差保护把它掐掉 —— 用户看到的"一推就掉线"。
 *
 *     不是驱动器故障：停机时 0x6041=0x0637（使能态）、0x603F=0x0000、
 *     0x607A 与 0x6064 差 0 counts。是驱动器在 IP 下压根不跟。
 *     每 40 ms 只改几十~几百 counts 的目标，它当噪声不动；
 *     等 err 堆到 5 deg 以上才一次发狠。观感 = 推不动 + 一推就停。
 *
 *     怀疑（未验证，想再试先做这一步）：IP 插补节拍可能依赖 SYNC
 *     （0x1005 bit30 + 0x1006 周期），当前 TPDO 走事件定时 2 ms、没配 SYNC。
 *     先抓 0x1005/0x1006 确认，再验证 fdb 是否跟着每个点走。
 *     没做完这步之前 ADM_USE_IP 保持 0。 */
#define ADM_USE_IP          (0u)

/* 0x60C2 插补周期（ms）。IP 模式下驱动器按这个周期插补，1 ms 是出厂值。
 * 千万别改成 10 —— 详见 dev_joint_od.h 里 OD_INTERP_TIME 的注释。 */
#define ADM_IP_PERIOD_MS    (1u)

/* ---- PP 轮廓参数（0x6081 / 0x6083 / 0x6084）----
 * IP 模式下这两条仍然有效：0x6081 作为插补速度上限、0x6083/0x6084 作为
 * 插补加减速（部分驱动器在 IP 下仍用它限速），所以照写不误。
 * ADM_PROF_VEL 只是给规划器的上限，真正的速度约束是 ADM_VEL_MAX。
 * 加速度必须满足 a >= ADM_VEL_MAX / (ADM_STR_MS/1000) = 30000/0.02 = 1.5e6，
 * 否则 PP 在 20 ms 内追不上点。取 3e6，2 倍余量。 */
#define ADM_PROF_VEL        (45000L)                /* counts/s = 30.9 deg/s，必须 > VEL_MAX */
#define ADM_PROF_ACC        (1300000L)              /* counts/s^2，与 VEL_MAX 配套：t_walk = 2*sqrt(d/a) = 40 ms */

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
 *   18000 (12.4 deg/s) dt=40ms a=1.8e6，K=0.40 时手感最好、只算微震
 *   13000 ( 8.9 deg/s) 本版，K 降到 0.30 之后震感变强，配套降速压剖面
 *
 * 2026-10-30 第五次定标（K=0.30 那一跑）。K 从 0.40 掉到 0.30，位移增益
 * 涨 1/0.75 = 1.33：同样的扰动、同样的剖面脉冲，手感觉到的摆幅大 33%。
 * 所以"更好推"和"震一点"是同一枚硬币，不是环坏了。
 * 压震动的正确姿势是压剖面本身，而不是回头抬 K：
 *   v_peak = sqrt(a*d)        剖面三角尖端速度
 *   t_walk = 2*sqrt(d/a)      走完一个点要多久
 * 要保持 t_walk 正好填满换点周期 40 ms，必须 d = a*dt^2/4 与 a 同比例变。
 * a 1.8e6 -> 1.3e6、d 720 -> 520，则
 *   v_peak 36000 -> 26000（-28%），换向冲击 J*alpha 同比降
 *   t_walk 仍是 40 ms，点与点之间不留空档，不会变成"走走停停"
 */
#define ADM_VEL_MAX         (13000.0f)              /* 8.9 deg/s，见上方第五次定标 */

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

/* 超前量用的一阶低通（TIM3 100 Hz 下 0.22 = 45 ms 时间常数）。
 *
 * 换向瞬间 delta_vel 会从 -VEL_MAX 跳到 +VEL_MAX，超前量如果跟着硬跳，
 * 目标位置会瞬移 2*VEL_MAX*LEAD_S = 1152 counts，那一下就是一次可听见的"哒"。
 *
 * 注意这个 2 倍：delta_vel 是跨零翻转，不是从 0 起步，所以超前量的单拍
 * 变化量是 2*VEL_MAX*LPF*LEAD_S，不是 VEL_MAX*LPF*LEAD_S。之前打印的
 * "理论上限"少乘了这个 2，才误报 420 > 352 这种"超限"。
 *
 * 这里是整套参数里对"哒"最敏感的旋钮，比 VEL_MAX 还直接：
 *   LPF 0.30 -> 单拍超前跳变 2*18000*0.30*0.032 = 346 counts
 *   LPF 0.22 -> 2*13000*0.22*0.032          = 183 counts
 * 代价是换向后 45 ms 内补偿略欠，顶力会回来一点（实测仍在 400 permille 内）。 */
#define ADM_LEAD_LPF        (0.22f)

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
/* 指令-反馈超此值停机。
 * IP 模式首次上机时收紧到 8000(5.5 deg)：如果驱动器其实在等 SYNC
 * 根本不跟随，err 会线性涨到 8000 然后停 —— 一眼就知道 IP 走不通，
 * 而不是"目标在跑、电机不动"地干看着。正常工作时 err 峰值约 2000，
 * 8000 有 4 倍余量，不会误停。
 * PP 模式（已验证）保持 25000(17 deg)。 */
#if (ADM_USE_IP != 0u)
/* IP 一推就到 9600 counts，8000 必然秒停。放开也没用 —— 阈值不是病根
 * （驱动器压根不跟），只会让它滑得更远。真要再试 IP 先解决 SYNC。 */
#define ADM_FOLLOW_ERR_MAX  (25000)
#else
#define ADM_FOLLOW_ERR_MAX  (25000)                 /* 17 deg，PP 已验证 */
#endif

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

/* ---- 运行时增益档位（KEY1 切换，A/B 对比手感用）----
 * 三档 wn(1.51 Hz)、zeta(0.7) 都不变，只有 K 变，所以切的是"多硬"：
 *   档 1 软  K=0.225 N.m/deg  45 deg 对应 10.1 N.m
 *   档 2 中  K=0.300 N.m/deg  45 deg 对应 13.5 N.m   <- 默认
 *   档 3 硬  K=0.400 N.m/deg  45 deg 对应 18.0 N.m
 * 运行中可切，位置连续、力矩突变。 */
uint8_t App_Joint_Adm_CycleGain(void);      /* 切下一档，返回 1..3 */
uint8_t App_Joint_Adm_SetGain(uint8_t idx); /* 0..2 指定，返回实际档位 */
uint8_t App_Joint_Adm_GetGain(void);        /* 0..2 */

#endif
