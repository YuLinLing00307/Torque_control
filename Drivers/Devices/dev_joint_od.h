#ifndef __DEV_JOINT_OD_H
#define __DEV_JOINT_OD_H

/* =====================================================================
 *  MXBS-14 / DiGE X200A_V2 · 对象字典与 COB-ID 定义
 *  来源：MG-C-EDS001001.EDS（VendorNumber 0x00666999, Product 0x00004806）
 *
 *  说明：
 *    索引:子索引 只出现在 CAN 数据段里，不在 ID 里
 *    SDO 帧第 2、3 字节 = 索引（小端），第 4 字节 = 子索引
 * ===================================================================== */

/* ---------------- 通信参数 ---------------- */
#define OD_DEVICE_TYPE          0x1000u     /* u32  ro   设备类型 */
#define OD_ERROR_REGISTER       0x1001u     /* u8   ro   错误寄存器 */
#define OD_IDENTITY             0x1018u     /* 身份对象：01厂商 02产品 03版本 04序列号 */
#define OD_SUB_VENDOR           1u
#define OD_SUB_PRODUCT          2u
#define OD_SUB_REVISION         3u
#define OD_SUB_SERIAL           4u
#define OD_COMM_CYCLE           0x1006u     /* u32  rw   通信周期 us（0 = 只由主站发 SYNC） */
#define OD_HEARTBEAT_TIME       0x1017u     /* u16  rw   心跳周期 ms（0 = 关闭） */

/* ---------------- PDO 配置 ---------------- */
#define OD_TPDO1_COB            0x1800u     /* 子 01 = COB-ID（bit30=1 表示未启用），02 = 传输类型 */
#define OD_TPDO1_MAP            0x1A00u

/* ---------------- CiA402 伺服对象 ---------------- */
#define OD_ERROR_CODE           0x603Fu     /* u16  ro   故障码 */
#define OD_CONTROLWORD          0x6040u     /* u16  rw   控制字 */
#define OD_STATUSWORD           0x6041u     /* u16  ro   状态字 */
#define OD_MODES_OF_OPERATION   0x6060u     /* i8   rw   运行模式 */
#define OD_MODE_DISPLAY         0x6061u     /* i8   ro   当前模式显示 */
#define OD_SUPPORTED_MODES      0x6502u     /* u32  ro   支持的工作模式位图 */
#define OD_POSITION_DEMAND      0x6062u     /* i32  ro   位置指令（内部） */
#define OD_POSITION_ACTUAL      0x6064u     /* i32  ro   实际位置  ← 重力补偿要用 */
#define OD_VELOCITY_ACTUAL      0x606Cu     /* i32  ro   实际速度 */
#define OD_TARGET_TORQUE        0x6071u     /* i16  rw   目标力矩 */
#define OD_TORQUE_ACTUAL        0x6077u     /* i16  ro   实际力矩 */
#define OD_TARGET_POSITION      0x607Au     /* i32  rw   目标位置  ← CSP 下发这个 */
#define OD_TARGET_VELOCITY      0x60FFu     /* i32  rw   目标速度  ← 默认 RPDO1 里装的是它 */
#define OD_MAX_PROFILE_VEL      0x607Fu     /* u32  rw   最大轮廓速度 counts/s，会钳制 0x6081 */
#define OD_MAX_MOTOR_SPEED      0x6080u     /* u32  rw   最大电机转速 rpm */
#define OD_PROFILE_VELOCITY     0x6081u     /* u32  rw   轮廓速度 */
#define OD_PROFILE_ACCEL        0x6083u     /* u32  rw   轮廓加速度 */
#define OD_PROFILE_DECEL        0x6084u

/* ---------------- 企业《CANopen字典操作示例》补充 ---------------- */
#define OD_TORQUE_SLOPE         0x6087u     /* u32  rw   力矩上升斜率，‰额定/s，默认 1000 */
#define OD_CURRENT_ACTUAL       0x6078u     /* i16  ro   电流当前值，‰额定电流 */
#define OD_BUS_VOLTAGE          0x6079u     /* u32  ro   母线电压 mV */
#define OD_RATED_CURRENT        0x6075u     /* u32  rw   电机额定电流 mA */
#define OD_RATED_TORQUE         0x6076u     /* u32  rw   电机额定力矩 mNm  <-- ‰ 的换算基准 */
#define OD_MAX_TORQUE           0x6072u     /* u16  rw   最大力矩 ‰，默认 2000 */
#define OD_FOLLOW_ERR_EN        0x35DBu     /* u8   rw   位置跟随误差检测开关 */
#define OD_INTERP_MODE          0x60C0u     /* i16  rw   插值方法 */
#define OD_LOAD_POS_ERROR       0x60F4u     /* i32  ro   负载端位置误差（驱动器自己算的） */
#define OD_INTERP_PERIOD        0x3675u     /* 子1=插值周期实测 子2=估计值 cycle 子3=相位 us */
#define OD_SOFT_POS_LIMIT       0x607Du     /* i32  rw   子1=最小  子2=最大（软位置限位） */
#define OD_MAX_CURRENT          0x6073u     /* u16  rw   最大电流 ‰，默认 2000 */
#define OD_FOLLOWING_ERR        0x6065u     /* u32  rw   位置跟随误差窗口，默认 10000 counts */
#define OD_FOLLOWING_ERR_TIME   0x6066u     /* u16  rw   跟随误差超时 ms，默认 20 */
#define OD_INTERP_TIME          0x60C2u     /* u8   rw   插补周期：子1=周期值，子2=指数(有符号)
                                             * 实测 子1=1 子2=-3 -> 1 ms。
                                             * 【别再改】曾改成 10 想"与写点周期对齐"，
                                             * 依据只是"违约 10 倍"的推断，实测直接崩：
                                             * 11.4 deg 稳态跟随误差、驱动器出力约 0、手感消失。
                                             * 而 0x60C2=1 时 0x6062-目标 = 0、0x6064 只差
                                             * 27~38 counts，本身就反驳了这个推断。
                                             * **保持 1 ms。** */
#define OD_POSITION_FACTOR      0x6093u     /* 位置单位换算分子/分母，默认 1/1 */
#define OD_POSITION_INC         0x6063u     /* i32  ro   位置增量（内部单位） */

/* ---------------- 通信监控（默认全 0 = 不监控，别乱开） ---------------- */
#define OD_HB_CONSUMER          0x1016u     /* 心跳消费者：高16位=主站节点号 低16位=超时ms */
#define OD_ERROR_BEHAVIOUR      0x1029u     /* NMT 错误行为 */
#define OD_ABORT_CONNECTION     0x6007u     /* u16 rw 通信中断后的动作，默认 1 */

/* 注意：0x1016 六个子索引默认全 0（EDS 实读）。一旦写入，主站必须周期发
 *      0x700+主站节点号 的心跳，否则超时后驱动器自动失能。 */     /* u32  rw   轮廓减速度 */

/* ---------------- 位置单位 ---------------- */
#define OD_ENCODER_RES          0x608Fu     /* 子 01 = 编码器线数，02 = 电机转数 */
#define OD_GEAR_RATIO           0x6091u     /* 子 01 = 电机转数，02 = 输出轴转数 */
#define OD_SUB_NUM              1u
#define OD_SUB_DEN              2u

/* ---------------- 厂商自定义 ---------------- */
#define OD_CAN_BAUDRATE         0x3519u     /* u16  rw   单位 kbps，默认 0x3E8 = 1000 */
#define OD_CAN_NODE_ID          0x351Au     /* u16  rw   默认 1 */
#define OD_FORCE_SENSOR_TYPE    0x358Fu     /* u16  rw   0=电流估算 1=ADC1 2=ADC2 3=扩展 */
#define OD_FORCE_CONTROL_MODE   0x3590u     /* u16  rw   0=关 1=电流 2=位置(导纳) */
#define OD_FORCE_CONTROL_CYCLE  0x3591u     /* u16  rw   力控周期 */
#define OD_IMPEDANCE_KBM        0x3588u     /* 子 01=K 02=B 03=M，i32 */
#define OD_ADMITTANCE_KBM       0x3592u     /* 子 01=K 02=B 03=M，i32 */
#define OD_SUB_K                1u
#define OD_SUB_B                2u
#define OD_SUB_M                3u

/* ---------------- 状态字 / 控制字 ---------------- */
#define SW_MASK_STATE           0x006Fu
#define SW_STATE_READY          0x0021u     /* Ready to switch on */
#define SW_STATE_SWITCHED_ON    0x0023u     /* Switched on */
#define SW_STATE_ENABLED        0x0027u     /* Operation enabled */
#define SW_MASK_FAULT           0x004Fu
#define SW_STATE_FAULT          0x0008u

#define CW_SHUTDOWN             0x0006u
#define CW_SWITCH_ON            0x0007u
#define CW_ENABLE_OPERATION     0x000Fu
#define CW_FAULT_RESET          0x0080u     /* bit7 上升沿 */
#define CW_NEW_SETPOINT         0x0010u     /* CSP 下 bit4：新设定点 */

/* bit5 = Change set immediately（CiA402 CSP）。
 * 这一位是 2026-10-30 从厂商上位机源码里翻出来的：
 *   utils/device_control.py  ControlWord.CHANGE_SET_IMMEDIATELY = 32
 *                            ControlWord.ENABLE_POS            = 63 (0x3F)
 * 我们之前一直只写 0x000F，bit5 恒 0 —— 意思是"等上一个设定点
 * 完全处理完（到位）再接受新目标"。位置型导纳的目标一直在动，
 * 永远"没处理完"，于是新目标被无限期推迟：
 *   0x607A 回读 = 我们写的值（写落地了），
 *   但 0x6062 内部指令停在原地甚至变成垃圾，
 *   驱动器按 |0x6062-0x6064| 判跟随误差 -> 0x8611。
 * 而静态保持 4 s 里目标不变，第一个点早就"到位"了，所以从不报错。
 * 这就是"静止没事、一动就挂、有时行有时不行"的完整解释。
 * 配 bit4 翻转使用：基准 0x2F，翻转后 0x3F，与厂商 ENABLE_POS 一致。 */
#define CW_CHANGE_SET_IMMEDIATELY 0x0020u   /* bit5：立即改写设定点 */
#define CW_CSP_BASE             (CW_ENABLE_OPERATION | CW_CHANGE_SET_IMMEDIATELY)

/* 运行段控制字 = 0x003F，与厂商上位机 device_control.py 的 ENABLE_POS = 63 一致。
 * 关键：bit4 常置 1，**不再翻转**。
 *   bit5 = 1（change set immediately）时，驱动器每收到一帧 0x607A 就立即采用，
 *   bit4 的上升沿语义此时是冗余的。上一版每来一个新目标就翻转 bit4
 *   (0x2F <-> 0x3F)，等于在驱动器已经"立即取点"的状态下再塞一个握手脉冲，
 *   两个语义互相打架 —— 这是"有时行有时不行"的一个来源。
 * 厂商自己跑 CSP 就恒定发 63，从不翻转，这里照抄。 */
#define CW_CSP_RUN              (CW_CSP_BASE | CW_NEW_SETPOINT)

/* RPDO1 映射条目（对象字典索引 16 位 | 长度 8 位） */

/* ---- CSV 速度模式（2026-10-30 夜，实测后的决定）----
 * 三条实测事实：
 *   1. 出厂 RPDO1 映射就是 {0x6040, 0x60FF} —— 这台驱动器是按速度模式
 *      配置出厂的，是我们把它改成了 0x607A 去跑 CSP。
 *   2. CSP 下 0x8611 的真实门限约 2~3 deg（3990/4491 counts 就跳），
 *      而 0x6065 读回 52428(36 deg)、0x35DB=0，它根本不按这个判。
 *   3. 同一条 1.37 deg/s 的慢速直线，一次 err=302 counts 走完，
 *      两次关节直接掉 10 deg 后跳闸 —— 慢速直线都不可重复。
 * 结论：位置口在这台驱动器上没有可用余量。CSV 没有跟随误差窗口，
 * 而且导纳控制本来就是"外力 -> 速度"的映射，输出速度是自然的接口。
 * 0x6060 是运行模式选择（和 0x6040 同属命令），不是电机出厂参数。 */
/* ---- IP 插补位置模式（0x6060 = 7）----
 * 2026-10-30 确认 0x6502 = 0x3ED，bit5(IP) = 1，这台驱动器支持。
 *
 * 为什么从 PP 换到 IP：
 *   PP 每收一个新目标都按"到点停"重新规划 —— 从 0 加速到峰值再刹回 0。
 *   40 ms 一个台阶 720 counts，剖面必然是峰值 sqrt(a*d) = 36000 counts/s 的
 *   三角波，25 Hz 循环一次。功能全对（净漂移 0.027 deg、0 失败、不震），
 *   但手上"一冲一停、不顺应"，而且这个三角剖面在 PP 里调参数调不掉。
 *   IP 模式下驱动器按 0x60C2 自己插补，不做重规划，速度可以连续。
 *
 * 控制字：IP 不走 PP 那套 bit4/bit5 握手，恒 0x000F 即可。
 *   若实测目标不跟随（fdb 不动），再试 0x1F 或 bit4 翻转。 */
#define CW_IP_RUN               (CW_ENABLE_OPERATION)   /* 0x000F */

#define JOINT_MODE_CSV          9
#define CW_CSV_RUN              (CW_ENABLE_OPERATION)   /* 0x000F，CSV 不用 bit4/bit5 */

/* 状态字 bit12：set-point acknowledge。
 * 厂商状态字分析里写得很清楚：
 *   bit12=1 "Previous setpoint still in process, setpoint overwriting
 *            shall be accepted."  -> 还在处理，可以覆盖
 *   bit12=0 "Previous setpoint already processed, waiting for new setpoint."
 * 停机诊断打印它，用来判断握手有没有真的在走。 */
#define SW_BIT_SETPOINT_ACK     0x1000u     /* bit12 */
#define SW_BIT_FOLLOWING_ERR    0x2000u     /* bit7(CSP) = 跟随误差 */

/* ---------------- COB-ID：11 位 ID = 高 4 位功能码 + 低 7 位节点号 ---------------- */
#define COB_NMT                 0x000u
#define COB_SYNC                0x080u      /* 功能码 1，节点位为 0 → 广播 */
#define COB_EMCY(n)             (0x080u + (n))      /* 功能码 1 */
#define COB_TPDO1(n)            (0x180u + (n))      /* 功能码 3 */
#define COB_RPDO1(n)            (0x200u + (n))      /* 功能码 4 */
#define COB_SDO_TX(n)           (0x580u + (n))      /* 功能码 11，从站→主站 */
#define COB_SDO_RX(n)           (0x600u + (n))      /* 功能码 12，主站→从站 */
#define COB_HEARTBEAT(n)        (0x700u + (n))      /* 功能码 14 */

/* ---------------- SDO 命令字 ---------------- */
#define SDO_READ                0x40u
#define SDO_WRITE_1B            0x2Fu
#define SDO_WRITE_2B            0x2Bu
#define SDO_WRITE_3B            0x27u
#define SDO_WRITE_4B            0x23u

#endif
