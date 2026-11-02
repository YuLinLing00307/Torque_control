#ifndef __DEV_JOINT_OD_H
#define __DEV_JOINT_OD_H

/* =====================================================================
 *  MXBS-14 / DiGE X200A_V2 对象字典与 COB-ID
 *  来源：MG-C-EDS001001.EDS（Vendor 0x00666999，Product 0x00004806）
 *
 *  索引/子索引只出现在数据段，不进 ID：
 *      SDO 帧第 2、3 字节 = 索引（小端），第 4 字节 = 子索引
 * ===================================================================== */

/* ================ 一、COB-ID：11 位 = 4 位功能码 + 7 位节点号 ================ */
#define COB_NMT                 0x000u
#define COB_SYNC                0x080u      /* 功能码 1，节点位为 0 = 广播 */
#define COB_EMCY(n)             (0x080u + (n))
#define COB_TPDO1(n)            (0x180u + (n))
#define COB_RPDO1(n)            (0x200u + (n))
#define COB_SDO_TX(n)           (0x580u + (n))      /* 从站 → 主站 */
#define COB_SDO_RX(n)           (0x600u + (n))      /* 主站 → 从站 */
#define COB_HEARTBEAT(n)        (0x700u + (n))

/* ================ 二、SDO 命令字 ================ */
#define SDO_READ                0x40u
#define SDO_WRITE_1B            0x2Fu
#define SDO_WRITE_2B            0x2Bu
#define SDO_WRITE_3B            0x27u
#define SDO_WRITE_4B            0x23u

/* ================ 三、控制字 / 状态字（CiA402） ================ */
#define CW_SHUTDOWN             0x0006u
#define CW_SWITCH_ON            0x0007u
#define CW_ENABLE_OPERATION     0x000Fu
#define CW_FAULT_RESET          0x0080u     /* bit7，上升沿有效 */
#define CW_NEW_SETPOINT         0x0010u     /* bit4，新设定点 */
/* bit5 = change set immediately。厂商上位机里叫 ControlWord.CHANGE_SET_IMMEDIATELY = 32。
 * 只写 0x000F（bit5=0）时，驱动器要等上一个设定点"完全到位"才接受新目标；
 * 位置型导纳的目标一直在动、永远到不了位 → 新目标被无限推迟 → |0x6062-0x6064|
 * 越滚越大 → 报 0x8611。静态保持时目标不变、第一个点早就到位，所以从不报错。 */
#define CW_CHANGE_SET_IMMEDIATELY 0x0020u   /* bit5 */
#define CW_CSP_BASE             (CW_ENABLE_OPERATION | CW_CHANGE_SET_IMMEDIATELY)
/* 运行段恒发 0x3F，bit4 不再翻转（bit5=1 时 bit4 的上升沿语义是冗余的，
 * 两套握手叠在一起会互相打架）。与厂商 ENABLE_POS = 63 一致。 */
#define CW_CSP_RUN              (CW_CSP_BASE | CW_NEW_SETPOINT)
#define CW_IP_RUN               (CW_ENABLE_OPERATION)   /* IP / CSV 不吃 bit4、bit5 */

#define SW_MASK_STATE           0x006Fu
#define SW_STATE_READY          0x0021u     /* Ready to switch on */
#define SW_STATE_SWITCHED_ON    0x0023u
#define SW_STATE_ENABLED        0x0027u     /* Operation enabled */
#define SW_MASK_FAULT           0x004Fu
#define SW_STATE_FAULT          0x0008u
#define SW_BIT_SETPOINT_ACK     0x1000u     /* bit12：1 = 上一点还在处理，可覆盖 */
#define SW_BIT_FOLLOWING_ERR    0x2000u     /* bit7：跟随误差超限 */

/* ================ 四、CiA402 伺服对象（日常最常用） ================ */
#define OD_CONTROLWORD          0x6040u     /* u16  rw   控制字 */
#define OD_STATUSWORD           0x6041u     /* u16  ro   状态字 */
#define OD_MODES_OF_OPERATION   0x6060u     /* i8   rw   运行模式 */
#define OD_MODE_DISPLAY         0x6061u     /* i8   ro   当前模式回读 */
#define OD_TARGET_POSITION      0x607Au     /* i32  rw   目标位置（PP / CSP 下发这个） */
#define OD_POSITION_ACTUAL      0x6064u     /* i32  ro   实际位置 */
#define OD_POSITION_DEMAND      0x6062u     /* i32  ro   内部位置指令 */
#define OD_VELOCITY_ACTUAL      0x606Cu     /* i32  ro   实际速度 */
#define OD_TARGET_VELOCITY      0x60FFu     /* i32  rw   目标速度（出厂 RPDO1 装的就是它） */
#define OD_TORQUE_ACTUAL        0x6077u     /* i16  ro   实际力矩，‰额定 */
#define OD_TARGET_TORQUE        0x6071u     /* i16  rw   目标力矩 */
#define OD_ERROR_CODE           0x603Fu     /* u16  ro   故障码 */
#define OD_SUPPORTED_MODES      0x6502u     /* u32  ro   支持的模式位图 */

/* 轮廓参数：PP 换点的速度上限由 0x6083 × 换点周期 决定 */
#define OD_PROFILE_VELOCITY     0x6081u     /* u32  rw   轮廓速度 counts/s */
#define OD_MAX_PROFILE_VEL      0x607Fu     /* u32  rw   最大轮廓速度，会钳制 0x6081 */
#define OD_PROFILE_ACCEL        0x6083u     /* u32  rw   轮廓加速度 */
#define OD_PROFILE_DECEL        0x6084u     /* u32  rw   轮廓减速度 */
#define OD_MAX_MOTOR_SPEED      0x6080u     /* u32  rw   最大电机转速 rpm */

/* 跟随误差保护 */
#define OD_FOLLOWING_ERR        0x6065u     /* u32  rw   窗口 counts，默认 10000 */
#define OD_FOLLOWING_ERR_TIME   0x6066u     /* u16  rw   超时 ms，默认 20 */
#define OD_FOLLOW_ERR_EN        0x35DBu     /* u8   rw   跟随误差检测开关 */
#define OD_LOAD_POS_ERROR       0x60F4u     /* i32  ro   负载端位置误差（驱动器自己算的） */

/* 插补：【0x60C2 保持 1 ms，别再改】曾改成 10 想"与写点周期对齐"，
 * 实测直接崩：11.4° 稳态跟随误差、驱动器出力约 0、手感消失。详见《导纳控制调参记录.md》 */
#define OD_INTERP_TIME          0x60C2u     /* u8   rw   子1 周期值，子2 指数（1 / -3 = 1 ms） */
#define OD_INTERP_PERIOD        0x3675u     /* 子1 实测周期 子2 估计值 cycle 子3 相位 us */
#define OD_INTERP_MODE          0x60C0u     /* i16  rw   插值方法 */

/* ================ 五、PDO 配置 ================ */
#define OD_TPDO1_COB            0x1800u     /* 子01 COB-ID（bit30=1 = 未启用），02 传输类型，
                                             * 03 抑制时间（出厂 1000 = 100 ms，必须清零），
                                             * 05 事件定时 ms */
#define OD_TPDO1_MAP            0x1A00u

/* ================ 六、通信参数与监控 ================ */
#define OD_DEVICE_TYPE          0x1000u     /* u32  ro   设备类型 */
#define OD_ERROR_REGISTER       0x1001u     /* u8   ro   错误寄存器 */
#define OD_IDENTITY             0x1018u     /* 身份对象：01 厂商 02 产品 03 版本 04 序列号 */
#define OD_SUB_VENDOR           1u
#define OD_SUB_PRODUCT          2u
#define OD_SUB_REVISION         3u
#define OD_SUB_SERIAL           4u
#define OD_COMM_CYCLE           0x1006u     /* u32  rw   通信周期 us（0 = 只听主站 SYNC） */
#define OD_HEARTBEAT_TIME       0x1017u     /* u16  rw   心跳周期 ms（0 = 关闭） */
/* 0x1016 六个子索引 EDS 实读全 0。一旦写入，主站必须周期发 0x700+主站节点号 的心跳，
 * 否则超时后驱动器自动失能。默认全 0 = 不监控，别乱开。 */
#define OD_HB_CONSUMER          0x1016u     /* 高16位 主站节点号，低16位 超时 ms */
#define OD_ERROR_BEHAVIOUR      0x1029u     /* NMT 错误行为 */
#define OD_ABORT_CONNECTION     0x6007u     /* u16  rw   通信中断后的动作，默认 1 */

/* ================ 七、位置单位 ================ */
#define OD_ENCODER_RES          0x608Fu     /* 子01 编码器线数，02 电机转数 */
#define OD_GEAR_RATIO           0x6091u     /* 子01 电机转数，02 输出轴转数 */
#define OD_POSITION_FACTOR      0x6093u     /* 位置换算分子/分母，默认 1/1 */
#define OD_POSITION_INC         0x6063u     /* i32  ro  位置增量（内部单位） */
#define OD_SOFT_POS_LIMIT       0x607Du     /* i32  rw  子1 最小 / 子2 最大（软限位） */
#define OD_SUB_NUM              1u
#define OD_SUB_DEN              2u

/* ================ 八、厂商自定义 ================ */
#define OD_CAN_BAUDRATE         0x3519u     /* u16  rw   kbps，默认 0x3E8 = 1000 */
#define OD_CAN_NODE_ID          0x351Au     /* u16  rw   默认 1 */
#define OD_FORCE_SENSOR_TYPE    0x358Fu     /* u16  rw   0 电流估算 / 1 ADC1 / 2 ADC2 / 3 扩展 */
#define OD_FORCE_CONTROL_MODE   0x3590u     /* u16  rw   0 关 / 1 电流 / 2 位置（导纳） */
#define OD_FORCE_CONTROL_CYCLE  0x3591u     /* u16  rw   力控周期 */
#define OD_IMPEDANCE_KBM        0x3588u     /* 子 01 K / 02 B / 03 M，i32 */
#define OD_ADMITTANCE_KBM       0x3592u     /* 子 01 K / 02 B / 03 M，i32 */
#define OD_SUB_K                1u
#define OD_SUB_B                2u
#define OD_SUB_M                3u

/* ================ 九、力矩换算基准（企业文档补充） ================ */
#define OD_RATED_TORQUE         0x6076u     /* u32  rw   额定力矩 mNm  ← ‰ 的换算基准 */
#define OD_RATED_CURRENT        0x6075u     /* u32  rw   额定电流 mA */
#define OD_MAX_TORQUE           0x6072u     /* u16  rw   最大力矩 ‰，默认 2000 */
#define OD_MAX_CURRENT          0x6073u     /* u16  rw   最大电流 ‰，默认 2000 */
#define OD_TORQUE_SLOPE         0x6087u     /* u32  rw   力矩上升斜率 ‰额定/s，默认 1000 */
#define OD_CURRENT_ACTUAL       0x6078u     /* i16  ro   实际电流 ‰额定 */
#define OD_BUS_VOLTAGE          0x6079u     /* u32  ro   母线电压 mV */

#endif
