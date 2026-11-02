#ifndef __APP_JOINT_H
#define __APP_JOINT_H

#include "main.h"
#include "dev_joint.h"

/* =====================================================================
 *  关节控制 APP 层
 *
 *  内容：重力矩补偿、导纳控制（PP 位置流）、增益档位、扫角自检
 *  调参历史与踩坑记录：见项目根目录《导纳控制调参记录.md》
 *
 *  数据流：
 *    TIM3 1 kHz ISR --每 10 拍--> 导纳递推 100 Hz（只算 pos_cmd，不发帧）
 *    主循环 40 ms  --> PP 换点（0x0F -> 0x607A -> 0x3F 三帧 SDO）
 *    TPDO1 2 ms    --> {0x6041 状态字, 0x6064 实际位置}，运行期不占 SDO
 *    RS485 100 Hz  --> NJ70 力矩 -> 导纳环输入
 *
 *  硬约束（违反了必然出事）：
 *    - ISR 里只做递推：不访问 CAN、不发 SDO、不 printf、不 HAL_Delay
 *    - 位置反馈只走 TPDO，运行期绝不用 SDO 轮询 0x6064
 *    - 0x60C2 插补周期保持 1 ms（改成 10 实测 11.4 deg 稳态误差）
 * ===================================================================== */

/* ---- 角度 / 位置换算（524288 counts 一圈）---- */
#define POS_PER_REV       524288.0f
#define DEG_TO_COUNT(d)   ((int32_t)((d) * POS_PER_REV / 360.0f))
#define COUNT_TO_DEG(c)   ((float)(c) * 360.0f / POS_PER_REV)
#define POS_TO_RAD(p)     ((float)(p) * (6.283185307f / POS_PER_REV))
#define DEG_TO_RAD(d)     ((d) * 3.14159265358979f / 180.0f)

/* =====================================================================
 *  一、导纳参数（决定手感，最常调）
 *
 *  M*acc + B*vel + K*(pos - pos_ref) = tau_ext
 *  pos: counts   vel: counts/s   tau: N.m
 *
 *    K = 刚度，N.m/deg = K * 1456.36   （45 deg 处的力矩 = 45*K）
 *    B = 阻尼，B = 2*zeta*sqrt(K*M)
 *    M = 虚拟质量，M = K/wn^2
 *
 *  换软硬的正确做法：M/B/K 三个同比例缩放，wn 和 zeta 不变，
 *  只改"多硬"不改"多快"。运行时切档见 ADM_GAIN_LEVELS。
 * ===================================================================== */

/* 预设立档。日常改这个数就够，1 = 当前手感。
 * 2/3 是早期对照档，K 太小会撞 45 deg 限位走不到平衡点。 */
#ifndef ADM_PRESET
#define ADM_PRESET          1
#endif

#if   ADM_PRESET == 1
#define ADM_M               (1.0e-6f)               /* 虚拟质量 */
#define ADM_B               (2.0e-5f)               /* 阻尼 */
#define ADM_K               (2.5e-5f)               /* 刚度 N.m/count（= 0.036 N.m/deg） */
#elif ADM_PRESET == 2
#define ADM_M               (1.52e-6f)
#define ADM_B               (1.91e-5f)
#define ADM_K               (6.0e-5f)               /* 0.087 N.m/deg */
#else
#define ADM_M               (1.52e-7f)
#define ADM_B               (1.91e-6f)
#define ADM_K               (6.0e-6f)               /* 0.009 N.m/deg */
#endif

#define ADM_DT              (0.01f)                 /* 递推周期 10 ms = 100 Hz */

/* 增益档位：KEY1 循环切，M/B/K 同比例缩放（wn、zeta 不变）
 * 序号按 ADM_GAIN_SCALE 排：0=软 1=中(基准) 2=硬 */
#define ADM_GAIN_LEVELS     3u

/* 软限位，相对启动位置 */
#define ADM_LIMIT_DEG       (45.0f)
#define ADM_LIMIT_CNT       ((int32_t)(ADM_LIMIT_DEG * POS_PER_REV / 360.0f))

/* =====================================================================
 *  二、换点时序与速度
 *
 *  PP 每点按"到点停"重规划，是三角速度剖面：
 *      d      = VEL_MAX * STR_MS/1000     换点台阶 counts
 *      v_peak = sqrt(a*d)                 三角尖端速度（手感觉到的"冲"）
 *      t_walk = 2*sqrt(d/a)               走完一个点的时间
 *  要让 t_walk 正好填满换点周期，d 必须和 a 同比例变：
 *      d = a*dt^2/4
 *  只降 VEL_MAX 不降 a -> 走完干等 -> "走走停停"，更震。
 * ===================================================================== */

/* TIM3：PSC=71 -> 1 MHz，ARR=1000 -> 1 kHz；每 ADM_SUB 拍递推一次 */
#define ADM_ISR_HZ          (1000u)
#define ADM_SUB             (10u)
#define ADM_TIM_ARR         (1000u)

/* 换点周期 ms。也是速度上限的分子：v_max = a*dt/4 */
#define ADM_STR_MS          (40u)

/* 换点死区 counts：指令变化不到这么多就不发新点，省 SDO 和 bit4 翻转 */
#define ADM_STR_DEADBAND    (2)

/* 速度限幅 counts/s。当前 13000 = 8.9 deg/s，与 0x6083=1.3e6 配套 */
#define ADM_VEL_MAX         (13000.0f)

/* 0x6081 轮廓速度（给 PP 规划器的上限，真正的约束是 ADM_VEL_MAX） */
#define ADM_PROF_VEL        (45000L)

/* 0x6083/0x6084 轮廓加减速度。a 就是惯性力矩的旋钮（tau = J*a/83443），
 * 抬高必震。当前与 VEL_MAX 配套：d=520、t_walk=40 ms */
#define ADM_PROF_ACC        (1300000L)

/* =====================================================================
 *  三、超前补偿
 *
 *  手上感到的力 = 驱动器位置环刚度 * (tgt - fdb)。驱动器刚度 20~38 N.m/deg，
 *  是导纳刚度的 100 倍量级，0.15 deg 滞后就能变成 3~6 N.m 顶力把导纳盖死。
 *  把目标按当前速度前移一点，让驱动器"追得正好"。
 *  实测 err/vel 恒定 59 ms = 驱动器执行指令的总延迟；取 32 补掉一半多。
 * ===================================================================== */
#define ADM_LEAD_MS         (32.0f)
#define ADM_LEAD_S          (ADM_LEAD_MS * 0.001f)

/* 超前量的一阶低通。换向时 delta_vel 跨零翻转（-V 直接到 +V），
 * 不滤的话目标瞬移 2*V*LEAD_S。这是对换向"哒"最敏感的旋钮。
 * 单拍超前后跳变量 = 2*VEL_MAX*LPF*LEAD_S，别漏乘 2。 */
#define ADM_LEAD_LPF        (0.22f)

/* =====================================================================
 *  四、换点机制选择
 *
 *  0 = PP（0x6060=1）：0x0F -> 写 0x607A -> 0x3F，靠 bit4 上升沿采信。
 *      已验证：200 点连跑、净漂移 0.011~0.03 deg、0x603F 全 0。
 *      短板：三角剖面，0x606C 上 +-24000 脉冲，25 Hz，手上"一冲一停"。
 *
 *  1 = IP（0x6060=7）：只写 0x607A，驱动器按 0x60C2 自己插补。
 *      【实测跑不通，已回退，别捡回来】0x6502=0x3ED 说硬件支持，但驱动器
 *      对每 40 ms 只改几百 counts 的目标当噪声不理，fdb 卡死不动，
 *      |cmd-fdb| 峰值 9609/9585 counts 后被跟随误差保护掐停。
 *      想再试先验证 SYNC 假设（0x1005 bit30 + 0x1006），当前没配。
 * ===================================================================== */
#define ADM_USE_IP          (0u)

/* 0x60C2 插补周期 ms，出厂 1。只读确认，别写（改成 10 手感直接消失） */
#define ADM_IP_PERIOD_MS    (1u)

/* =====================================================================
 *  五、保护阈值
 * ===================================================================== */

/* 外力矩保护 N.m。传感器满量程 25，正常一推 11~12；
 * 原值 12 余量只剩 0.7，一使劲就跳闸，停机后目标钉死 -> "完全推不动" */
#define ADM_TAU_STOP        (20.0f)
#define ADM_TAU_STOP_CNT    (10u)                   /* 连续 10 拍(100 ms)才停 */

/* 力矩方向：传感器读到的是"关节作用在连杆上"的力矩，关节受力相反 */
#define ADM_TAU_SIGN        (-1.0f)

/* 指令-反馈偏差超此值停机 counts（25000 = 17 deg，正常运行时峰值约 2000） */
#define ADM_FOLLOW_ERR_MAX  (25000)

#define ADM_POS_AGE_STOP    (200u)                  /* TPDO 200 ms 不来帧 -> 停机 */
#define ADM_PP_FAIL_STOP    (5u)                    /* 换点连续失败 5 次 -> 停机 */

/* 使能后静态保持自检时长 ms，0 = 关闭。目标钉死看驱动器会不会自己报错 */
#define ADM_SELFTEST_MS     (2000u)

/* =====================================================================
 *  六、对外接口
 * ===================================================================== */

float App_Joint_GetGravityTorque(int32_t pos);      /* 重力矩 tau_g(th) */
void  App_Joint_Task(void);                         /* 周期任务，主循环调 */
void  App_Joint_GravScan_Task(void);                /* 扫角采集，重拟合重力矩用 */

void    App_Joint_Adm_Start(void);
void    App_Joint_Adm_Stop(void);
void    App_Joint_Adm_StopKey(void);                /* 停机并标记原因=按键 */
void    App_Joint_Adm_Task(void);
uint8_t App_Joint_Adm_IsBusy(void);

/* 增益档位：三档 wn/zeta 不变，只改 K（45 deg 处的力矩跟着变） */
uint8_t App_Joint_Adm_CycleGain(void);              /* 切下一档，返回 1..3 */
uint8_t App_Joint_Adm_SetGain(uint8_t idx);         /* 0..2 指定 */
uint8_t App_Joint_Adm_GetGain(void);                /* 0..2 */

/* ---- 重力矩补偿参数 tau_g(th) = a*cos(th) + b*sin(th) + c，th 用绝对角度 ---- */
#define GRAV_A          (-0.191777f)
#define GRAV_B          (-0.347195f)
#define GRAV_C          ( 0.004957f)

#endif
