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
#define ADM_M               (1.0e-6)               /* 虚拟质量 */
#define ADM_B               (7.0e-6)               /* 阻尼 */
#define ADM_K               (1.8e-5)               /* 刚度 */
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

/* 0x6083 轮廓加速度。a 就是惯性力矩的旋钮（tau = J*a/83443），抬高必震。
 * 1.3e6 是这套几何的下限：d=520 要在 40 ms 内走完，最少就要 4d/T^2 = 1.3e6。 */
#define ADM_PROF_ACC        (1300000L)

/* 0x6084 轮廓减速度，单独设。**必须 >= ADM_PROF_ACC**：
 * 低了这点行程在 40 ms 内走不完，下一点叠上来，change-set-immediately 会让
 * 位置一环一环往后拖，err 线性涨到跟随误差保护。所以"刹车更软"这条路是死的。
 *   1.30e6  与加速相同（原始行为），t_walk 40.0 ms，刚好不剩
 *   1.50e6  t_walk 38.6 ms，余 1.4 ms，制动段惯性力矩 +15%（当前）
 *   2.00e6  t_walk 36.3 ms，余 3.7 ms，但尖端速度升到 28600，不值得 */
#define ADM_PROF_DECEL      (1500000L)

/* =====================================================================
 *  三、保护阈值
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
 *  四、力控模式（2026-11-04，导师要求：给力矩指令，看稳态误差）
 *
 *  零力顺应是"力进、位置出"；力控是"力误差进、位置出"。
 *  两者共用同一个执行器（PP 位置流），区别只在 K 项：
 *      K != 0 : 稳态 K*x = tau_cmd - tau_ext，出力由挡块位置决定 -> 不可控
 *      K == 0 : 稳态 tau_ext = tau_cmd，出力就是指令
 *  所以力控必须把 K 置 0（M/B 也一并旁路），只留一个积分器：
 *      pos_cmd = pos_ref + Ki_pos * INT(tau_cmd - tau_ext) dt
 *
 *  为什么 K 必须去掉：K 是"位置回正弹簧"，顶住挡块时它把位置拉回参考，
 *  力误差被位置偏移吸收。同一个 1.0 N.m 指令，挡块位置不同出力能差 3 倍
 *  （见《导纳控制调参记录.md》第 13 节）。加积分也救不了：K 和积分在
 *  同一回路里会形成慢振荡。
 *
 *  APP_JOINT_FC = 0 时整段力控不参与编译，行为回到清理后的零力顺应版本
 *  （只多一个 KEY1 转发函数，Code 约 +160 B）。
 * ===================================================================== */

#define APP_JOINT_FC        (1u)        /* 1 = 力控模式，0 = 零力顺应 */

#if (APP_JOINT_FC != 0u)

/* 命名规则：FC_ = force control；单位后缀 _CNT = counts、_MS = 毫秒，
 * 力矩类不加后缀，统一 N.m（与 tau_ext / ADM_TAU_STOP 保持一致）。 */

/* 位置积分增益 [counts/(N.m*s)]：1 N.m 误差 -> 每秒挪 2000 counts = 1.37 deg */
#define FC_KI_POS           (2000.0f)

/* 积分器绝对上限 [counts]：42.6 deg。软限位 45deg(65536) 前留 3500 counts。
 *
 * 2026-11-07 实测教训：原来给 3000(2.06deg)，是把"接触后余量"当成了全部行程。
 * 实际手臂要先走完自由行程才碰得到挡块，3000 全被自由行程吃掉，
 * 日志里 pos_cmd 一涨到 214254(=211254+3000) 就彻底冻结 5.8 s，
 * F_cmd 从 0.5 加到 1.0 一动不动。**行程预算必须包含自由行程。**
 *
 * 2026-11-07 第二次放大：30000(20.6deg) 也只有半个机械行程，挡块摆远一点
 * 就够不到。现在顶到软限位下面，等于"力控能跑到机械能到的地方"。 */
#define FC_DELTA_MAX        (62000.0f)

/* 越界窗口 [counts]：2.75 deg。指令已比实际超前这么多，停止同向推进 */
#define FC_DEV_WIN          (4000.0f)

/* 力矩保护 [N.m]：目标是 0.3~1.5，超过 3 说明失控（兜底，不是工作点） */
#define FC_TAU_MAX          (3.0f)

/* "找不到挡块"判据 [counts]：位移超过 41.2 deg 还读不到力 -> 停机。
 * 力控在自由空间里积分器只会一直爬，没有挡块就永远到不了目标力。
 * 阈值跟着 FC_DELTA_MAX 走，留 2000 counts 让它在打到软限位前先报错，
 * 别让"钳在软限位上不出力"这种无声卡死重现。 */
#define FC_NO_CT_CNT        (60000.0f)

/* "找不到挡块"判据 [N.m]：位移超限时，实测力低于这个值就算没顶到 */
#define FC_NO_CT_TAU        (0.10f)

/* KEY1 力矩档位数。第 0 档必须是 0，启动默认不加力 */
#define FC_CMD_NUM          (5u)

/* 运行期打印周期 [ms]。力环建立 0.5~1 s，200 ms 够看趋势 */
#define FC_PRINT_MS         (200u)

/* 运行期力矩保护阈值：力控用 3 N.m（目标 0.3~1.5），零力顺应仍用 20 */
#define ADM_TAU_LIMIT       (FC_TAU_MAX)
#else
#define ADM_TAU_LIMIT       (ADM_TAU_STOP)
#endif

/* =====================================================================
 *  五、对外接口
 * ===================================================================== */

float App_Joint_GetGravityTorque(int32_t pos);      /* 重力矩 tau_g(th) */
void  App_Joint_Task(void);                         /* 周期任务，主循环调 */
void  App_Joint_GravScan_Task(void);                /* 扫角采集，重拟合重力矩用 */

void    App_Joint_Adm_Start(void);
void    App_Joint_Adm_Stop(void);
void    App_Joint_Adm_StopKey(void);                /* 停机并标记原因=按键 */
void    App_Joint_Adm_Task(void);
uint8_t App_Joint_Adm_IsBusy(void);

/* KEY1 统一入口：力控模式切 F_cmd，零力顺应模式切增益档 */
void  App_Joint_Adm_Key1(void);

/* 增益档位：三档 wn/zeta 不变，只改 K（45 deg 处的力矩跟着变） */
uint8_t App_Joint_Adm_CycleGain(void);              /* 切下一档，返回 1..3 */
uint8_t App_Joint_Adm_SetGain(uint8_t idx);         /* 0..2 指定 */
uint8_t App_Joint_Adm_GetGain(void);                /* 0..2 */

/* ---- 重力矩补偿参数 tau_g(th) = a*cos(th) + b*sin(th) + c，th 用绝对角度 ---- */
#define GRAV_A          (-0.191777f)
#define GRAV_B          (-0.347195f)
#define GRAV_C          ( 0.004957f)

#endif
