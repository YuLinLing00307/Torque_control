# -*- coding:utf-8 -*-
"""
@Time: 2020/6/25 15:51
@Author: 谭国榕
@Email: tanguorong@mail.dige.com
@File: base.py
"""
from enum import Enum


# 辨识枚举
from ui.config import translate

NONE = None
UNS8 = ("<4B", [0, 255], 8)
UNS16 = ("<2H", [0, 65536], 16)
UNS32 = ("<I", [0, 4294967295], 32)
INTEGER8 = ("<4b", [-128, 127], 8)
INTEGER16 = ("<2h", [-32768, 32767], 16)
INTEGER32 = ("<i", [-2147483648, 2147483647], 32)

ElectricalIdentification = -1       # 电气辨识
DirectionIdentification = -2        # 方向辨识
HallIdentification = -3             # 霍尔辨识
PolarLogarithmIdentification = -4   # 极对数辨识
ZPhaseIdentification = -5           # Z相辨识
ElectricAngleIdentification = -6    # 电角度辨识
MechanicalIdentification = -7       # 机械辨识
TorqueRippleIdentification = -8     # 转矩脉动辨识
MechanicalCharacteristicIdentity = -9  # 机械特性辨识
ForceControlIdentification = -10    # 力控特性辨识
ElectricalIdentification_2 = -11    # 电气辨识2
InertiaCalibration = -12            # 惯量校准

# 模式枚举
NoneMode = 0
PPM = 1
PVM = 3
PTM = 4
AnaCtrlVelMode = 22
AnaCtrlTorqueMode = 23
PulseCtrlPosMode = 31
HomingMode = 6
PWMCtrlMode = 42
ReciprocalMotion = 60
ReferenceMode = 61
ForceCtrlMode = 11
MultiMotionMode = 62
CSPMode = 8
CSVMode = 9
CSTMode = 10

# 辨识状态
FreeState = 0
IdentificationSuccess = 1
ElectricalIdentificationStatus = 2
DirectionIdentificationStatus = 3
InertiaIdentificationState = 4
TorqueRippleIdentificationState = 5
ErrorIdentificationState = 255


# 读写权限
class RWPermission(Enum):
    RO = 0  # 只读
    RW = 1  # 读写


class ControlCmd(Enum):
    REST = 0  # 驱动器复位
    SAVE = 1  # 保存所有参数到Flash
    REQUEST = 2  # 请求驱动器参数（暂未启用）
    TEST_LED = 3  # 测试设备闪灯（暂未启用）
    SET_CAN_OPEN = 4  # CanOpen配置（暂未启用）
    VERSION = 5  # 固件版本号（暂未启用）
    RESTORE = 255  # 恢复出厂设置，并系统自动复位


class ControlWord(Enum):
    DISABLE_OPERATION = 7
    SWITCH_ON = 7
    SWITCH_ON_ENABLE = 15
    ENABLE_OPERATION = 15
    STANDARD_MARK = 143

    SHUTDOWN_MARK = 135
    SHUTDOWN = 6

    DISABLE_VOLTAGE_MARK = 130
    DISABLE_VOLTAGE = 0

    QUICK_STOP_MARK = 134
    QUICK_STOP = 2

    EXTERN_MARK = 63488

    FAULT_RESET = 128  # 错误复位
    HALT = 271  # 暂停使能位
    HALT_QUICK_STOP = 267  # Quick Stop 暂停指令
    DISABLE_HALT = 15
    NEW_SET_POINT = 16
    HOMING_OPERATION_START = 16
    ENABLE_INTERPOLATION = 16
    CHANGE_SET_IMMEDIATELY = 32
    RELATIVE = 64  # 使能相对位置

    ENABLE_POS = 63
    ENABLE_HOME = 31


class ModesOfIdentify(Enum):
    ONE_ID = -7
    ELECT_ID = -8
    MEC_ID = -9
    HPE_ID = -10
    ELECT_ANGLE_ID = -11
    TQ_ID = -12
    SIM_FOC = -13


class ModesOfOperation(Enum):
    NO_MODE = 0
    PPM = 1
    VL = 2
    PVM = 3
    TPM = 4
    RESERVED = 5
    HM = 6
    IPM = 7
    CSP = 8
    CSV = 9
    CST = 10

    # User define
    SCAN = -15


class UpdateCmd(Enum):
    UpdateStart = "UpdateStart"
    UpdateFrameCount = "UpdateFrameCount"
    UpdateChecksums = "UpdateChecksums"
    UpdateVersion = "UpdateVersion"


UpdateError = {
    0: "Normal",
    1: "App self-start checksum error",
    2: "AppAddrError",
    3: "App stack address error",
    4: "Failed to save firmware transfer size",
    5: "Firmware transfer size exceeds limit",
    6: "Error in firmware transfer checksum",
    7: "Firmware transfer failed",
    8: "Wrong number of firmware transmission packets",
    9: "5s timeout for firmware transmission",
    10: "A SPI Flash Check fail"
}

DeviceConnectState = {
    translate("Form", "设备未连接"): 0,
    translate("Form", "设备已连接"): 1
}

DeviceCommState = {
    translate("Form", "正在通讯检测"): -1,
    translate("Form", "设备通讯异常"): 0,
    translate("Form", "设备通讯正常"): 1,
}

SerialPortCount = {
    # "COM0": 0,   # 0表示串口异常
    # "COM2": 1   # 1表示串口正常
    # "COM3": 0
}

SerialPortChoose = {
    translate("Form", "显示正常串口"): 0,
    translate("Form", "显示全部串口"): 1,
}

HallState = {
    translate("Form", "正常"): 0,
    translate("Form", "错误"): 1,
    translate("Form", "霍尔未连接"): 2
}

ElcAngleAlignment = {
    "0": 0,
    "60": 1,
    "120": 2,
    "180": 3,
    "240": 4,
    "300": 5
}

IdentifyState = {
    translate("Form", "启动开始空闲状态"): FreeState,
    translate("Form", "辨识成功"): IdentificationSuccess,
    translate("Form", "正在进行电气辨识..."): ElectricalIdentificationStatus,
    translate("Form", "正在进行换向辨识..."): DirectionIdentificationStatus,
    translate("Form", "正在进行惯量辨识..."): InertiaIdentificationState,
    translate("Form", "正在进行转矩脉动辨识..."): TorqueRippleIdentificationState,
    translate("Form", "错误状态"): ErrorIdentificationState
}

IdentifyModes = {
    translate("Form", "电气辨识"): ElectricalIdentification,
    translate("Form", "方向辨识"): DirectionIdentification,
    translate("Form", "霍尔辨识"): HallIdentification,
    translate("Form", "极对数辨识"): PolarLogarithmIdentification,
    translate("Form", "Z相辨识"): ZPhaseIdentification,
    translate("Form", "电角度辨识"): ElectricAngleIdentification,
    translate("Form", "机械辨识"): MechanicalIdentification,
    translate("Form", "转矩脉动辨识"): TorqueRippleIdentification
    # "模拟旋转磁场": -13
}

ElecAngIdentModes = {
    translate("Form", "强拉寻相模式"): 0,
    # "电角度辨识模式2": 1,
    # "电角度辨识模式3": 2,
    translate("Form", "短距离寻相模式"): 3,
    translate("Form", "矢量脉冲寻相模式"): 4,
}

ElecAngIdentModesTip = {
    0: translate("Form", "电角度辨识模式1"),
    3: translate("Form", "电角度辨识模式4"),
}

ElecIdentifyModes = {
    translate("Form", "本地电气特性辨识"): 1,
    translate("Form", "云端电气特性辨识"): 2,
}

MechanicalIdentifyModes = {
    translate("Form", "本地机械特性辨识"): 1,
    translate("Form", "云端机械特性辨识"): 2,
}

encoder_num_map = {str(i): i for i in range(1, 23)}

bin_version_map = {}


ProductSelect = {
    "HID": 0,
    "MH_CAN_14-17": 1
}

LanguageType = {
    translate("Form", "中文（简体）"): 0,
    translate("Form", "英语(English)"): 1,
}

GainMap = {"低增益": 30, "中增益": 50, "高增益": 100}

# IdentifyModes = {
#     "一键整定": -7,
#     "电气辨识": -8,
#     "换向辨识": -10,
#     "电角度辨识": -11,
#     "机械辨识": -9,
#     "转矩脉动辨识": -12,
#     # "模拟旋转磁场": -13
# }

HallType = {
    translate("Form", "无霍尔"): 0,
    translate("Form", "120°"): 1,
    # "60°": 2
}

MotorType = {
    translate("Form", "直流有刷电机"): 1,
    translate("Form", "永磁同步旋转电机"): 3,
    # "开关磁阻电机": 5,
    # "步进电机": 8,
    # "虚拟电机": 32768,
    translate("Form", "永磁同步直线电机"): 32769,
    translate("Form", "音圈电机"): 32770,
    # "绕组分段永磁同步电机": 32771
}

MotorCurrentType = {
    translate("Form", "有效值(Arms)"): 0,
    translate("Form", "峰值(Amp)"): 1
}

INC_ABZ = 0
INC_AB = 1
INC_UDC_PA = 8
ABS_BISS_C = 2
ABS_TAMAGAWA = 3
ABS_SSI = 4
HALL_ONLY = 5
ABS_UDC_PA = 6
ABS_NIKON = 7
ABS_ENDAT22 = 9
ANA_SINCOS = 10
ANA_HIPERFACE = 11
ANA_LINEAR_HALL = 12
ANA_RESOLVER = 13
NONE_ENCODER = 255

# 内环编码器类型
EncodeType = {
    translate("Form", "增量式 - ABZ"): INC_ABZ,
    translate("Form", "增量式 - AB"): INC_AB,
    translate("Form", "增量式 - UDC.Pa"): INC_UDC_PA,
    translate("Form", "绝对式 - BISS-C"): ABS_BISS_C,
    translate("Form", "绝对式 - Tamagawa"): ABS_TAMAGAWA,
    translate("Form", "绝对式 - SSI"): ABS_SSI,
    translate("Form", "绝对式 - UDC.Pa"): ABS_UDC_PA,
    translate("Form", "绝对式 - EnDat22"): ABS_ENDAT22,
    translate("Form", "绝对式 - Nikon"): ABS_NIKON,
    translate("Form", "模拟量 - SIN/COS"): ANA_SINCOS,
    translate("Form", "模拟量 - Hiperface"): ANA_HIPERFACE,
    translate("Form", "模拟量 - 线性霍尔"): ANA_LINEAR_HALL,
    translate("Form", "模拟量 - Resolver"): ANA_RESOLVER,
    translate("Form", "Halls Only"): HALL_ONLY,
    translate("Form", "无编码器"): NONE_ENCODER
}

# 外环编码器类型
OutsideEncoderType = {
    translate("Form", "增量式 - ABZ"): INC_ABZ,
    translate("Form", "增量式 - AB"): INC_AB,
    translate("Form", "增量式 - UDC.Pa"): INC_UDC_PA,
    translate("Form", "绝对式 - BISS-C"): ABS_BISS_C,
    translate("Form", "绝对式 - Tamagawa"): ABS_TAMAGAWA,
    translate("Form", "绝对式 - SSI"): ABS_SSI,
    translate("Form", "绝对式 - UDC.Pa"): ABS_UDC_PA,
    translate("Form", "绝对式 - EnDat22"): ABS_ENDAT22,
    translate("Form", "绝对式 - Nikon"): ABS_NIKON,
    translate("Form", "模拟量 - SIN/COS"): ANA_SINCOS,
    translate("Form", "模拟量 - Hiperface"): ANA_HIPERFACE,
    translate("Form", "模拟量 - 线性霍尔"): ANA_LINEAR_HALL,
    translate("Form", "模拟量 - Resolver"): ANA_RESOLVER,
    translate("Form", "Halls Only"): HALL_ONLY,
    translate("Form", "无编码器"): NONE_ENCODER
}

SpeedMap = {
    translate("Form", "编码器1"): 1,
    translate("Form", "编码器2"): 2,
}

DualFBControlMode = {
    translate("Form", "全闭环"): 0,
    translate("Form", "半闭环"): 1,
}

EventType = {
    "参数": 0,
    "地址": 1,
}

EventValue = {
    "发生报警触发": 1,
    "上升阈值触发": 2,
    "下降阈值触发": 3,
}

EventChannel = {str(i): i for i in range(1, 17)}

EventDataType = {
    "bool": 0,
    "uint8": 1,
    "uint16": 3,
    "uint32": 5,
    "int8": 2,
    "int16": 4,
    "int32": 6,
    "float32": 7,
}

SamplingDelayPeriod = {str(i): i for i in range(51)}

EncoderIncremental = [0, 1]


EncoderAbsolute = [2, 3, 4, 6, 7, 8, 9, 10, 11, 12]

AckBits = {str(i): i for i in range(37)}
DelayedResponse = {str(i): i for i in range(17)}

AbsSSIEncoderBaudRate = {
    "0.5": 500000,
    "0.625": 625000,
    "1.0": 1000000,
    "1.25": 1250000,
    "1.6": 1600000,
    "2.0": 2000000,
    "2.5": 2500000,
    "5.0": 5000000
}

AbsBISSEncoderBaudRate = {
    "0.625": 625000,
    "1.25": 1250000,
    "2.5": 2500000,
    "5.0": 5000000
}

AbsTamagawEncoderBaudRate = {
    "2.5": 2500000
}

ErrorActiveLevel = {
    "0": 0,
    "1": 1,
    "None": 2
}

DataPresentation = {
    translate("Form", "二进制"): 0,
    translate("Form", "格雷码"): 1
}

EncoderSignalType = {
    translate("Form", "差分信号"): 0,
    translate("Form", "单端信号"): 1
}

EncoderResolverMultiplicationFactor = {str(i): i for i in range(6, 13)}
EncoderResolverFrq = {
    "10": 10000,
    "12": 12000,
    "15": 15000,
    "20": 20000,
}

CurrentLoopFrq = {
    "20000": 20000,
    "40000": 40000
}

PosLoopFrq = {
    "10000": 10000,
    "20000": 20000
}

EncoderZeroBits = {str(i): i for i in range(37)}

EncoderResolutionList = {str(i): 2 ** i for i in range(37)}

AbsEncoderResolutionList = {"{} bits".format(str(i + 8)): 2 ** (i + 8) for i in range(29)}
AbsEncoderResolutionList.update({"32": 0})
# 新增编码器参数map
EncoderHardWareRes = {str(i + 8): 2 ** (i + 8) for i in range(29)}
EncoderHardWareRes.update({"32": 0})
EncoderProtocolTotalBits = {str(i): i for i in range(65)}
EncoderPosLSBBit = {str(i): i for i in range(37)}
EncoderHighMaskBits = {str(i): i for i in range(37)}
EncoderErrorBit = {str(i): i for i in range(37)}
EncoderDataPollingTime = {str(i + 1): i for i in range(8)}
EncoderCdsState = {
    "0": 0,
    "1": 1,
    "None": 2
}

CommType = {
    "CAN": 0,
    "EtherCat": 1,
    "ModBus": 2,
    "Pulse": 3
    # "ANALOG SIGNAL": 3
}

MotorShowState = {
    translate("Form", "电机失能"): 0,
    translate("Form", "电机使能"): 1,
    translate("Form", "电机使能中"): 2,
    # "STO2保护": 3
}

EncodeState = {
    translate("Form", "正常"): 0,
    translate("Form", "错误"): 1,
    translate("Form", "编码器未连接"): 2,
    translate("Form", "编码器未准备好"): 3
}

QuickStopCode = {
    "Quick Stop 0": 0,
    "Quick Stop 1": 1,
    "Quick Stop 2": 2,
    "Quick Stop 3": 3,
    "Quick Stop 5": 5,
    "Quick Stop 6": 6,
    "Quick Stop 7": 7,
}

Planner = {
    translate("Form", "位置模式"): 1,
    translate("Form", "速度模式"): 3,
    translate("Form", "力矩模式"): 4,
    # "PWM控制": -12,
    translate("Form", "模拟量控速度"): 22,
    translate("Form", "模拟量控力矩"): 23,
    translate("Form", "脉冲控位置"): 31,
    translate("Form", "PWM控制速度"): 42,
    # "脉冲控速度": 32,
    # "脉冲控力矩": 33,
    translate("Form", "回零模式"): 6,
    translate("Form", "参考信号"): 61,
    translate("Form", "力控模式"): 11,
    # "PWM控位置": 41,
    # "PWM控速度": 42,
    # "PWM控力矩": 43
    # "扫频模式": -15
    translate("Form", "往复运动模式"): 60,
    translate("Form", "多段运行模式"): 62,
    "CSP": 8,
    "CSV": 9,
    "CST": 10,
    # "空模式": 0
}


DisplayPlanner = {
    translate("Form", "位置模式"): 1,
    translate("Form", "速度模式"): 3,
    translate("Form", "力矩模式"): 4,
    # "PWM控制": -12,
    translate("Form", "模拟量控速度"): 22,
    translate("Form", "模拟量控力矩"): 23,
    translate("Form", "脉冲控位置"): 31,
    translate("Form", "PWM控制速度"): 42,
    # "脉冲控速度": 32,
    # "脉冲控力矩": 33,
    translate("Form", "回零模式"): 6,
    translate("Form", "参考信号"): 61,
    translate("Form", "力控模式"): 11,
    # "PWM控位置": 41,
    # "PWM控速度": 42,
    # "PWM控力矩": 43
    # "扫频模式": -15
    translate("Form", "往复运动模式"): 60,
    translate("Form", "多段运行模式"): 62,
    "CSP": 8,
    "CSV": 9,
    "CST": 10,
    translate("Form", "电气辨识"): ElectricalIdentification,     # 电气辨识
    translate("Form", "方向辨识"): DirectionIdentification,        # 方向辨识
    translate("Form", "霍尔辨识"): HallIdentification,             # 霍尔辨识
    translate("Form", "极对数辨识"): PolarLogarithmIdentification,   # 极对数辨识
    translate("Form", "Z相辨识"): ZPhaseIdentification,          # Z相辨识
    translate("Form", "电角度辨识"): ElectricAngleIdentification,   # 电角度辨识
    translate("Form", "机械辨识"): MechanicalIdentification,      # 机械辨识
    translate("Form", "转矩脉动辨识"): TorqueRippleIdentification,     # 转矩脉动辨识
    translate("Form", "机械特性辨识"): MechanicalCharacteristicIdentity,  # 机械特性辨识
    translate("Form", "力控特性辨识"): ForceControlIdentification,    # 力控特性辨识
    translate("Form", "电气辨识_"): ElectricalIdentification_2,
    translate("Form", "惯量校准"): InertiaCalibration,
    translate("Form", "空模式"): 0
}
PositionOption = {
    translate("Form", "相对位置目标值"): 0,
    translate("Form", "相对位置规划值"): 1,
    translate("Form", "相对位置当前值"): 2
}

DisplayMode = {
    ElectricalIdentification: translate("Form", "电气辨识"),
    DirectionIdentification: translate("Form", "方向辨识"),
    HallIdentification: translate("Form", "霍尔辨识"),
    PolarLogarithmIdentification: translate("Form", "极对数辨识"),
    ZPhaseIdentification: translate("Form", "Z相辨识"),
    ElectricAngleIdentification: translate("Form", "电角度辨识"),
    MechanicalIdentification: translate("Form", "机械辨识"),
    TorqueRippleIdentification: translate("Form", "转矩脉动辨识"),

}

VelocityControlMode = {
    translate("Form", "位置指令控制"): 0,
    translate("Form", "速度指令控制"): 1
}

PulseOutputMode = {
    translate("Form", "关闭输出"): 0,
    translate("Form", "正交输出模式+软件Z相"): 1,
    translate("Form", "正交输出模式+硬件Z相"): 2
}

PulseOutputMode2 = {
    translate("Form", "关闭输出"): 0,
    translate("Form", "正交输出模式+软件Z相"): 1,
    # "index per Feedback": 2
}

MotionProfileType = {
    translate("Form", "不滤波"): 0,
    translate("Form", "TD滤波"): 1,
    translate("Form", "滑动均值滤波"): 2
}

SmoothingFactorIndex = {str(i): i for i in range(-3, 4)}

# 测量模式
MeasureMode = {
    translate("Form", '到位整定时间测量'): 0
}

# # 输入指令
# InputCommand = {
#     "速度指令值": 0,
#     "位置指令值": 1,
# }
#
# # 输出响应
# OutputResponse = {
#     "速度实际值": 0,
#     "位置实际值": 1
# }

PositionType = {
    translate("Form", "绝对位置"): 63,  # 0x3F
    translate("Form", "相对位置"): 127  # 0x7F
}

SpeedUnit_RPM_P = {
    "RPM": 0,
    "P/s": 1,
    "deg/s": 2,
    translate("Form", "脉冲当量"): 3,
    "mm/s": 4,
    # "rad/s": 5,
    # "RPS": 6,
    # "um/s": 7,
}


PosRotary = {
    "P": 0,
    "deg": 1,
    translate("Form", "脉冲当量"): 2,
    "mm": 3,
    # "rad": 4,
    # "um": 5,
}

Unit_MM_P = {
    "P": 0,
    "mm": 1,
}

ForceUnit = {
    "unit": 0,
    "g": 1,
    # "kg": 2
}

MultiSegments = {"{}{}".format(i, translate("Form", "段")): i for i in range(1, 17)}

Cycle_Model = {
    translate("Form", "顺序执行"): 0,
    translate("Form", "循环执行"): 1,
    translate("Form", "DI切换"): 2,
    translate("Form", "指令切换"): 3,
}

Time_Unit = {
    "ms": 0,
    "s": 1,
}

Jump_Enable = {
    translate("Form", "否"): 0,
    translate("Form", "是"): 1,
}

Res_Action = {
    translate("Form", "继续当前"): 0,
}
Fault_Res_Action = {
    translate("Form", "停机"): 0,
    translate("Form", "自动清除,回零"): 1,
}

Start_CMD_Option = {
    translate("Form", "使能直接启动"): 0,
    translate("Form", "控制字指令启动"): 1,
}

Opera_Mode = {
    translate("Form", "位置模式"): 0,
    translate("Form", "速度模式"): 1,
    translate("Form", "力矩模式"): 2,
    translate("Form", "力控模式"): 3,
}

Target_Attr = {
    translate("Form", "绝对位置"): 0,
    translate("Form", "相对位置"): 1,
}

Force_Type = {
    translate("Form", "开环力控"): 0,
    translate("Form", "闭环力控"): 1,
}

# 这个暂时用不着,后期要加上
Target_Attr2 = {
    translate("Form", "不归零"): 0,
    translate("Form", "归零"): 1,
}

ModesOfHoming = {
    "-1.Motor locked rotor positive side + index": -1,
    "-2.Motor locked rotor negative side + index": -2,
    "1.RLS falling edge + index": 1,
    "2.FLS falling edge + index": 2,
    "3.Home positive side, falling edge + index": 3,
    "4.Home positive side, rising edge + index": 4,
    "5.Home negative side, falling edge + index": 5,
    "6.Home negative side, rising edge + index": 6,
    "7.Home pulse negative side, falling edge + index, FLS detection": 7,
    "8.Home pulse negative side, rising edge + index, FLS detection": 8,
    "9.Home pulse positive side, rising edge + index, FLS detection": 9,
    "10.Home pulse positive side, falling edge + index, FLS detection": 10,
    "11.Home pulse position side, falling edge + index, RLS detection": 11,
    "12.Home pulse position side, rising edge + index, RLS detection": 12,
    "13.Home pulse negative side, rising edge + index, RLS detection": 13,
    "14.Home pulse negative side, falling edge + index, RLS detection": 14,
    "15.Positive Direction Stall": 15,
    "16.Negative Direction Stall": 16,
    "17.RLS falling edge": 17,
    "18.FLS falling edge": 18,
    "19.Home positive side, falling edge": 19,
    "20.Home positive side, rising edge": 20,
    "21.Home negative side, falling edge": 21,
    "22.Home negative side, rising edge": 22,
    "23.Home pulse negative side, falling edge, FLS detection": 23,
    "24.Home pulse negative side, rising edge, FLS detection": 24,
    "25.Home pulse positive side, rising edge, FLS detection": 25,
    "26.Home pulse positive side, falling edge, FLS detection": 26,
    "27.Home pulse positive side, falling edge, RLS detection": 27,
    "28.Home pulse positive side, rising edge, RLS detection": 28,
    "29.Home pulse negative side, rising edge, RLS detection": 29,
    "30.Home pulse negative side, falling edge, RLS detection": 30,
    "33.Home on index negative move": 33,
    "34.Home on index positive move": 34,
    "35.Home immediate on current position": 35,
}

# 位置补偿单位
ErrorMappingUnit = {
    "um": 0,
    "mm": 1,
    "P": 2,
    # "degress": 2,
    # "udegress": 3,
    # "counts": 4
}

# ADC电压采样范围
ADC_VREF = {
    "-{a} ~ +{a}".format(a="12.288"): 0x00,
    "-{a} ~ +{a}".format(a="10.24"): 0x01,
    "-{a} ~ +{a}".format(a="6.14"): 0x02,
    "-{a} ~ +{a}".format(a="5.12"): 0x03,
    "-{a} ~ +{a}".format(a="2.56"): 0x04,
    "0 ~ +{a}".format(a="12.288"): 0x08,
    "0 ~ +{a}".format(a="10.24"): 0x09,
    "0 ~ +{a}".format(a="6.14"): 0x0A,
    "0 ~ +{a}".format(a="5.12"): 0x0B,
}

ADC_Resolution = {str(i) + " bit": i for i in range(32)}

Direction = {
    translate("Form", "正方向"): 1,
    translate("Form", "负方向"): -1
}

BreakSettingMap = {
    translate("Form", "手动控制"): 0,
    translate("Form", "自动控制"): 1,
}

ForceControlSensorType = {
    translate("Form", "当前电流"): 0,
    translate("Form", "ADC通道-1"): 1,
    translate("Form", "ADC通道-2"): 2,
    translate("Form", "扩展通道"): 3,
}

AnalogCtrlSpeedAdcSource = {
    translate("Form", "模拟量通道-1"): 1,
    translate("Form", "模拟量通道-2"): 2,
}

ForceUnitIndex = {
    '0': 0,
    '-1': -1,
    '-2': -2,
    '-3': -3,
    '-4': -4,
    '-5': -5,
    '-6': -6
}

# todo:暂时屏蔽阻抗控制
ForceControlMode = {
    translate("Form", "电流控制模式"): 1,
    translate("Form", "位置控制模式"): 2,
    # "速度控制模式": 3,
    # "开环控制": 0,
    # "闭环力矩": 1,
    # "闭环力位": 2
}

ForceReachesFlag = {
    translate("Form", "未到位"): 0,
    translate("Form", "已到位"): 1
}

ForceCurveFlag = {
    translate("Form", "运行结束"): 0,
    translate("Form", "到达A点"): 1,
    translate("Form", "满足返程条件"): 2,
    translate("Form", "完成曲线动作，快速返回"): 3,
}

CurrentLoopControlMode = {
    translate("Form", "P控制"): 0,
    translate("Form", "PIC控制"): 1
}

PulseCtrlSpeedRes = {str(i) + translate("Form", "位"): i for i in range(0, 32)}

PulsePositionCtrlMode = {
    translate("Form", "正交控制"): 0,
    translate("Form", "方向脉冲控制"): 1,
    translate("Form", "正负双脉冲控制"): 2,
}

PulseInputChannel = {
    translate("Form", "高速输入"): 0,
    translate("Form", "低速输入"): 1
}

# 霍尔状态对应的电角度
HallStateElecAngle = {
    "0°": 0,
    "60°": 1,
    "120°": 2,
    "180°": 3,
    "240°": 4,
    "300°": 5
}

"""
离线数据分析使用参数：
"""
DataSampleCmd_Invalid = 0x00  # 无效控制字
DataSampleCmd_ScopeConfig = 0x01  # 配置示波器
DataSampleCmd_Start = 0x02  # 采样开始
DataSampleCmd_Immediately = 0x03  # 立即采集
DataSampleCmd_Stop = 0x04  # 采样停止
DataSampleCmd_Reset = 0x06  # 采样复位
DataSampleCmd_Clear = 0x07  # 采样清空
DataSampleCmd_Open = 0x08  # 打开示波器
DataSampleCmd_ReadyConfig = 0x09  # 进入示波器预配置状态
DataSampleCmd_ErrorClear = 0x80  # 清除错误，回到SafeOp状态

DataFullFlag_InitStatus = 0x00  # 未初始化成功
DataFullFlag_PreOpStatus = 0x01  # 初始化完成未配置参数
DataFullFlag_SafeOpStatus = 0x03  # 可以配置示波器参数状态，不能开启采集
DataFullFlag_OpStatus = 0x07  # 示波器参数配置完成，可以开启采集
DataFullFlag_CompleteStatus = 0x17  # 示波器采集已完成
DataFullFlag_CompleteTrans = 0x37  # 数据转换完成
DataFullFlag_ErrorStatus = 0x80  # 示波器处于错误状态，需要复位操作，否则不能开启采集

# 参考信号类型
RefSignalType = {
    "Step": 2,
    "Sine": 1,
    "Triangle": 3,
}

# 参考信号对象
RefSignalObject = {
    translate("Form", "位置参考信号"): 1,
    translate("Form", "速度参考信号"): 2,
    translate("Form", "力矩参考信号"): 3,
    translate("Form", "AB轴电压参考信号"): 4,
    translate("Form", "D轴电流参考信号"): 5
}

# 离线数据状态
OfflineDataState = {
    translate("Form", "未初始化"): 0x00,
    translate("Form", "正在初始化"): 0x01,
    translate("Form", "可配置参数"): 0x03,
    translate("Form", "参数配置完成"): 0x07,
    translate("Form", "采集完成"): 0x17,
    translate("Form", "数据转换完成"): 0x37,
    translate("Form", "错误状态"): 0x80
}

# 采样数据类型定义
SamplingDataType = {
    translate("Form", "位置实时规划值"): 0,
    translate("Form", "位置实际运行值"): 1,
    translate("Form", "速度实时规划值"): 2,
    translate("Form", "速度实际运行值"): 3,
    translate("Form", "力矩实时规划值"): 4,
    translate("Form", "力矩实际运行值"): 5
}

# 采样次数标志
SamplingTimeFlag = {
    "Signal": 0x01,
    "Normal": 0x02
}

# 采样周期
SamplingRes = {
    translate("Form", "1个控制周期"): 1,
    translate("Form", "2个控制周期"): 2,
    translate("Form", "3个控制周期"): 3,
    translate("Form", "4个控制周期"): 4,
    translate("Form", "5个控制周期"): 5,
    translate("Form", "6个控制周期"): 6
}

# 采样触发方式
TriggerMethod = {
    translate("Form", "上升沿触发"): 0,
    translate("Form", "下降沿触发"): 1,
    # "窗口触发": 0x02,
}

# 采样延时
SamplingDelay = {
    "0 %": 0,
    "10 %": 10,
    "20 %": 20,
    "30 %": 30,
    "40 %": 40,
    "50 %": 50,
    "60 %": 60,
    "70 %": 70,
    "80 %": 80,
    "90 %": 90,
    "100 %": 100
}

SamplingDirection = {
    translate("Form", "触发时间点前采样数据"): 0,
    translate("Form", "触发时间点后采样数据"): 1
}

"""
性能评估模块使用参数：
"""
# 评估模块指令
BodeDiagram_Stop = 0
BodeDiagram_Start = 1
# 评估模块完成标志位
BodeDiagramFlag_Normal = 0  # 空闲状态
BodeDiagramFlag_Success = 1  # 成功状态
BodeDiagramFlag_Running = 2  # 进行中
BodeDiagramFlag_Error = 3  # 错误状态
# 评估模式
BodeDiagramScanMode = {
    # "电流开环": 0,
    translate("Form", "电流闭环"): 1,
    translate("Form", "速度开环"): 2,
    translate("Form", "速度闭环"): 3,
    # "位置开环": 4,
    # "位置闭环": 5
}

# 产品代号
ProductCode = {
    0x10001: "USERVO-U60A{}",
    0x20001: "U-48V{}AC",
    0x20002: "U-48V{}AE",
    0x20003: "U-48V{}AM",
    0x20004: "U-48V{}AP",
    0x30001: "SERVOTOP-220V{}AC",
    0x30002: "SERVOTOP-220V{}AE",
    0x30003: "SERVOTOP-220V{}AM",
    0x30004: "SERVOTOP-220V{}AP",
    0x50001: "UF-48V{}AC",
    0x50002: "UF-48V{}AE",
    0x50003: "UF-48V{}AM",
    0x50004: "UF-48V{}AP",
    0x70002: "UF-48V{}AEDR",
}

# 速度参数
velocity_param_list = ["Velocity_demand_value", "Velocity_actual_value", "CurSpeedError", "Velocity_window",
                       "Velocity_threshold", "Profile_velocity", "Max_Profile_velocity",
                       "Homing_speeds_Speed_for_zero_search", "Homing_speeds_Speed_for_switch_search",
                       "Target_velocity", "CurrentLoopCtlSpeedMax",
                       # "VelocityReferenceSignal_Amplitude",
                       # "VelocityReferenceSignal_Offset",
                       # "OutloopSpeedActualValue",
                       # "OutloopSpeedError"
                       # "MotorSpeedObserverActualValue",
                       "MotorStuckVelocityThreshold",
                       "BreakSetting_activation_velocity",
                       "ForceControlSearchSpeed",
                       ]

# 加速度参数
acceleration_param_list = ["Profile_acceleration", "Profile_deceleration", "Homing_acceleration", "Max_acceleration",
                           "Max_deceleration", "Quick_stop_deceleration"]

read_only_list = ['MotorShowState', 'EncodeState', 'HallState', 'IndetifyState', 'CurSpeedError', 'CurCurrentError',
                  'DeviceTemp', 'MotorTemp', 'Error_code', 'Statusword', 'Modes_of_operation_display',
                  'Position_demannd_value', 'Position_actual_value_inc', 'Position_actual_value',
                  'Velocity_demand_value', 'Velocity_actual_value', 'Torque_demand_value',
                  'Torque_actual_value', 'Current_actual_value', 'DC_link_circuit_voltage',
                  'Interpolation_data_configuration_Maxinmum_buffer_size', 'Following_error_actual_value',
                  'Supported_drive_modes']

# 多段位移中自定义参数转换
# 使用多段位移的对象
multi_step_vel_obj = []
multi_step_acc_obj = []
multi_step_pos_obj = []
multi_step_time_obj = []
# 速度
custom_velocity_list = ["TargetVelocityValue", "ProfileVelocity", "JogVelocity"]
# 加速度
custom_acc_list = ["ProfileAcc"]
# 位置
custom_position_list = ["TargetPositionValue"]
# 时间
custom_time_list = ["WaitTime"]
# cmd_enable = {"ControlWord": ControlWord.SHUTDOWN.value}

# 多段运行模式选择
multi_motion_mode = {
    translate("Form", '绝对位置模式'): 0,
    translate("Form", '相对位置模式'): 1,
    translate("Form", '速度模式'): 2,
    translate("Form", '力矩模式'): 3,
    translate("Form", '力控模式'): 4,
}

multi_search_mode = {
    translate("Form", '不搜索'): 255,
    translate("Form", '绝对位置'): 0,
    translate("Form", '相对位置'): 1,
    translate("Form", '速度'): 2,
    translate("Form", '力矩'): 3,
    translate("Form", '力控'): 4,
    translate("Form", '绝对位置，大于目标值'): 5,
    translate("Form", '相对位置，大于目标值'): 6,
    translate("Form", '速度，大于目标值'): 7,
    translate("Form", '力矩，大于目标值'): 8,
    translate("Form", '力控，大于目标值'): 9,
    translate("Form", '绝对位置，小于目标值'): 10,
    translate("Form", '相对位置，小于目标值'): 11,
    translate("Form", '速度，小于目标值'): 12,
    translate("Form", '力矩，小于目标值'): 13,
    translate("Form", '力控，小于目标值'): 14,
}

# 基础状态参数
base_read = {
    "HallState": None,
    "EncodeState": None,
    "DC_link_circuit_voltage": None,
    "DeviceTemp": None,
    "Error_code": None,  # 错误代码
    "Statusword": None,  # 状态字
    "MotorShowState": None,
    "Encoder2State": None,
    "Digital_inputs": None,
    "Digital_outputs_Physical_outputs": None,
    "Digital_outputs_Bit_mask": None,
    "Modes_of_operation_display": None,
    "DataFullFlag": None,
    "MultiSegmentRunningSegment": None,
    "MultiSegmentRunningState": None,
    "PositionErrorCorrectionStatus": None,
    "WarningCodeSegment1": None,
    "WarningCodeSegment2": None
}

# 版本参数
version_read = {
    "ProductCode": None,
    "DriverBoardVersionHW": None,
    "DriverBoardVersionSW": None,
    "InterfaceBoardVersionSW": None
}

version_type = {
    0: "Release",
    1: "Rev1",
    2: "Rev2",
    3: "Rev3",
    4: "Rev4",
    255: "Alpha",
    254: "Beta",
    253: "Weekly build"
}

sw_bin_type = {
    '0': "Release",
    '2': "Alpha",
    '4': "Beta",
    '6': "RC",
    '8': "Delta",
}

# 曲线参数
curve_read = {
    "Position_actual_value": None,  # 位置当前值
    "Following_error_actual_value": None,  # 跟随误差当前值
    "Velocity_actual_value": None,  # 速度当前值
    "CurSpeedError": None,  # 速度当前误差
    "Current_actual_value": None,  # 电流当前值
    "CurCurrentError": None,  # 当前电流误差
    # "FlashStorageStatus": None
}

# 曲线参数增加计数,适应多场景同时使用，初始化为空。
# 示例：{"Position_actual_value": [None, 1], "Velocity_actual_value": [None, 1]}
curve_params = {}


# 辨识参数
identify_param_read = {
    "Inductance": None,
    "Resistance": None,
    "PolarPositives": None,
    "Inertia": None,
    "Damping": None,
    "TorqueConstant": None,
    "ZPhaseElecAngle": None,
    "ElecAngleOffsetValue": None,
    "EncoderDirection": None,
    "Encoder2Direction": None,
    "HallStateElecAngle": None,
    "HallStateElecAngle2": None,
    "HallStateElecAngle3": None,
    "HallStateElecAngle4": None,
    "HallStateElecAngle5": None,
    "HallStateElecAngle6": None,
    "HallStateElecAngle7": None,
    "HallStateElecAngle8": None,
}

# 控制器参数
ctrl_params = ["CurrentBandwidth", "PosBandwidth"]

# 其他位置模式参数
modes_position_other_params = [
    "Motion_profile_type",
    "SmoothFactor",
    "TargetPositionBuff",
    "TargetPositionBuff_2",
    "RepetitiveEnable"
]

# 位置模式参数
modes_position_params = [
    "Target_position",
    "Max_Profile_velocity",
    "Profile_velocity",
    "Profile_acceleration",
    "Profile_deceleration",
    # "Software_position_limit_Minimal_position_limit",
    # "Software_position_limit_Maximal_position_limit"
]

# 速度模式参数
modes_velocity_params = [
    "Max_Profile_velocity",
    "Target_velocity",
    "Profile_acceleration",
    "Profile_deceleration"
]

# 力矩模式参数
modes_torque_params = [
    "Max_current",
    "Target_torque",
    "Torque_slope"
]

# 回零模式参数
modes_homing_params = [
    "Homing_method",
    "Home_offset",
    "Homing_acceleration",
    "Homing_speeds_Speed_for_switch_search",
    "Homing_speeds_Speed_for_zero_search"
]

# 速度单位转换的参数
change_unit_list = [
    "Velocity_demand_value",
    "Velocity_actual_value",
    "Velocity_window",
    "Velocity_threshold",
    "Max_Profile_velocity",
    "Profile_velocity",
    "Profile_acceleration",
    "Profile_deceleration",
    "Homing_speeds_Speed_for_zero_search",
    "Homing_speeds_Speed_for_switch_search",
    "Homing_acceleration",
    "Max_acceleration",
    "Max_deceleration",
    "Target_velocity"
]

# GPI数字IO边沿触发事件
IOFallingTriggerEvents = {
    translate("Form", "忽略输入"): 0,
    translate("Form", "电机使能"): 1,
    translate("Form", "电机失能"): 2,
    translate("Form", "清除错误"): 3,
    translate("Form", "暂停指令"): 4,
    translate("Form", "急停"): 5,
    translate("Form", "切换工作模式"): 6,
    # "切换目标位置": 7,
    # "切换目标速度": 8,
    # "切换目标力矩": 9,
    translate("Form", "回零开始"): 10,
    # "脉冲输入禁止": 11,
    # "电机旋转方向切换": 12,
    # "增益切换": 13,
    # "磁极检测": 14,
    translate("Form", "切换位置模式"): 15,
    translate("Form", "切换速度模式"): 16,
    translate("Form", "切换力矩模式"): 17,
    translate("Form", "切换模拟量控制速度模式"): 18,
    translate("Form", "切换模拟量控制力矩"): 19,
    translate("Form", "切换脉冲控位置"): 20,
    translate("Form", "切换力控模式"): 21,
    translate("Form", "切换多段运行模式"): 22,
    translate("Form", "切换回零模式"): 26,
    translate("Form", "多段运行模式段触发"): 27,
    translate("Form", "多段段选Bit0置1"): 28,
    translate("Form", "多段段选Bit0置0"): 29,
    translate("Form", "多段段选Bit1置1"): 30,
    translate("Form", "多段段选Bit1置0"): 31,
    translate("Form", "多段段选Bit2置1"): 32,
    translate("Form", "多段段选Bit2置0"): 33,
    translate("Form", "多段段选Bit3置1"): 34,
    translate("Form", "多段段选Bit3置0"): 35,
    translate("Form", "取消暂停"): 36,
    translate("Form", "清除所有警告"): 37,
    translate("Form", "探针1"): 38,
    translate("Form", "探针2"): 39,
    translate("Form", "开始记录力矩最大值"): 40,
    translate("Form", "停止记录力矩最大值"): 41,
    translate("Form", "记录位置1"): 42,
    translate("Form", "记录位置2"): 43,
    translate("Form", "记录位置3"): 44,
    translate("Form", "记录位置4"): 45,

}


# GPI数字IO边沿触发事件
IORisingTriggerEvents = {
    translate("Form", "忽略输入"): 0,
    translate("Form", "电机使能"): 1,
    translate("Form", "电机失能"): 2,
    translate("Form", "清除错误"): 3,
    translate("Form", "暂停指令"): 4,
    translate("Form", "急停"): 5,
    translate("Form", "切换工作模式"): 6,
    # "切换目标位置": 7,
    # "切换目标速度": 8,
    # "切换目标力矩": 9,
    translate("Form", "回零开始"): 10,
    # "脉冲输入禁止": 11,
    # "电机旋转方向切换": 12,
    # "增益切换": 13,
    # "磁极检测": 14,
    translate("Form", "切换位置模式"): 15,
    translate("Form", "切换速度模式"): 16,
    translate("Form", "切换力矩模式"): 17,
    translate("Form", "切换模拟量控制速度模式"): 18,
    translate("Form", "切换模拟量控制力矩"): 19,
    translate("Form", "切换脉冲控位置"): 20,
    translate("Form", "切换力控模式"): 21,
    translate("Form", "切换多段运行模式"): 22,
    translate("Form", "正极限开关"): 23,
    translate("Form", "回零开关"): 24,
    translate("Form", "负极限开关"): 25,
    translate("Form", "切换回零模式"): 26,
    translate("Form", "多段运行模式段触发"): 27,
    translate("Form", "多段段选Bit0置1"): 28,
    translate("Form", "多段段选Bit0置0"): 29,
    translate("Form", "多段段选Bit1置1"): 30,
    translate("Form", "多段段选Bit1置0"): 31,
    translate("Form", "多段段选Bit2置1"): 32,
    translate("Form", "多段段选Bit2置0"): 33,
    translate("Form", "多段段选Bit3置1"): 34,
    translate("Form", "多段段选Bit3置0"): 35,
    translate("Form", "取消暂停"): 36,
    translate("Form", "清除所有警告"): 37,
    translate("Form", "探针1"): 38,
    translate("Form", "探针2"): 39,
    translate("Form", "开始记录力矩最大值"): 40,
    translate("Form", "停止记录力矩最大值"): 41,
    translate("Form", "记录位置1"): 42,
    translate("Form", "记录位置2"): 43,
    translate("Form", "记录位置3"): 44,
    translate("Form", "记录位置4"): 45,
}

IOInputAutoTriggerEnable = {
    translate("Form", "边沿触发"): 0,
    translate("Form", "电平触发"): 1,
}

# 数字io上升沿/下降沿状态
OutputStatus = {
    "Low": 0,
    "High": 1,
}

# 高低有效
OutputEffective = {
    translate("Form", "低有效"): 0,
    translate("Form", "高有效"): 1
}

InputEffective = {
    translate("Form", "低有效"): 0,
    translate("Form", "高有效"): 1
}

# 数字输出触发事件
OutputEvents = {
    translate("Form", "目标到达"): 1,
    translate("Form", "通用输出"): 0,
    translate("Form", "伺服准备好"): 2,
    translate("Form", "位置到达"): 3,
    translate("Form", "零速到达"): 4,
    translate("Form", "速度到达"): 5,
    translate("Form", "回零完成"): 6,
    translate("Form", "驱动器错误"): 7,
    translate("Form", "警告输出"): 9,
    translate("Form", "多段运行状态"): 27,
    translate("Form", "Z相脉冲信号"): 28,
    # "力到达": 10,
    # "转矩限制检出": 11,
    # "速度限制检出": 12,
    # "制动器控制输出": 13,
    # "目标规划到达": 14,
    # "位置规划到达": 15,
    # "速度规划到达": 16,
    # "力矩规划到达": 17,
    # "力规划到达": 18,
    # "电池电压过低错误": 19,
    # "电池电压过低警告": 20,
    # "寻相成功": 21,
    # "错误或者失能": 22,
    # "过流错误": 23,
    # "母线过压": 24,
    # "母线欠压": 25,
    # "需要寻相": 26,
}

# 硬件通讯支持
HardWareCommType = {
    0: "CAN",
    1: "ECAT",
    2: "Modbus",
    3: "Pulse"
}

# 硬件支持编码器类型
HardWareEncoderType = {
    0: translate("Form", "单编码器"),
    1: translate("Form", "双编码器")
}

# 细分位数
SubdivisionNumber = {str(i): i for i in range(2, 17)}

# 模拟量控制
AnalogCtrlMode = {
    translate("Form", "关闭模拟量控制"): 0,
    translate("Form", "模拟量控速度"): 1,
    translate("Form", "模拟量控力矩"): 2
}

# 用户单位参数
device_unit_param = [
    "Polarity",
    "Position_factor_Numerator",
    "Position_factor_Feed_constant",
    "Velocity_factor_1_Numerator",
    "Velocity_factor_1_Divisor",
    "Acceleration_factor_Numerator",
    "Acceleration_factor_Divisor",
    "ForceFactorNum",
    "ForceFactorDen"
]

# 数据缓存buff
GraphDataBuff = {
    "1.5s": 500,
    "3.0s": 1000,
    "9.0s": 3000,
    "12.0s": 4000
}

# 参数设置的所有参数
item_list = [
    "MaxCurrentTime",
    "MotorBlockTimer",
    "HardwareVolMaxLimit",
    "HardwareVolMinLimit",
    "HardwareCurMaxLimit",
    "SpeedObserveBandwith",
    "StaticFriction",
    "TorqueConstant",
    # "SmoothFactor",
    "PositionLimitEnable",
    "SpeedMaxLimitEnable",
    "Following_error_window",
    "Following_error_time_Out",
    "Position_window",
    "Position_window_time",
    "Velocity_window",
    "Velocity_window_time",
    "Velocity_threshold",
    "Velocity_threshold_time",
    "Position_range_limit_Maximal_position_limit",
    "Position_range_limit_Minimal_position_limit",
    "CommType",
    "CanOpenBaudrate",
    "CanOpenId",
    "FixCurrentFre",
    "FixCurrentAmp",
    "FixMacFre",
    "FixMacSpeed",
    "FixMacCurrent",
    "FixMacPosP",
    "FixMacPosN",
    "Interpolation_data_record_Data1",
    "Interpolation_data_record_Data2",
    "Interpolation_time_period_ip_time_units",
    "Interpolation_time_period_ip_time_index",
    "Interpolation_sync_definition_Synchronize_on_group",
    "Interpolation_sync_definition_ip_sync_every_n_event",
    "Interpolation_data_configuration_Maxinmum_buffer_size",
    "Interpolation_data_configuration_Actual_buffer_size",
    "Interpolation_data_configuration_Buffer_organization",
    "Interpolation_data_configuration_Buffer_position",
    "Interpolation_data_configuration_Size_of_data_record",
    "Interpolation_data_configuration_Buffer_clear",
    "Motor_type",
    "Inductance",
    "Resistance",
    "PolarPositives",
    "Inertia",
    "Damping",
    "Magnetic",
    "Motor_rated_current",
    "Motor_rated_torque",
    "EncodeType",
    "Position_encoder_resolution_Encoder_increments",
    # "Position_encoder_resolution_Motor_revolutions",
    "HallType",
    "DeviceMaxTemp",
    "DeviceMinTemp",
    "MotorMaxTemp",
    "DeviceWarningTemp",
    # "TestAmp",
    # "TestFre",
    "Max_motor_speed",
    "Encoder1Baudrate",
    # "Encoder1Zerobits",
    "Encoder1Ackbits",
    "Encoder1MulRes",
    "STO1Enable",
    "PSWProtectEnable",
    "NSWProtectEnable",
    "Encoder1ErrorActiveLevel",
    "Encoder1SensorDataPresentation",
    "SpeedMeasurementCycle",
    "Max_acceleration",
    "Max_deceleration",
    "Max_current",
    "SpeedFeedForward",
    "ElcAngleAlignment",
    "PosBandwidth",
    "CurrentBandwidth",
    "TRCEnable", "FCEnable", "PolePairsIdElecCycles",
    "PolePairsIdWaitTime", "DirctionIdDistance", "CurrentLoopFrq", "PositionLoopFrq",
    "Software_position_limit_Minimal_position_limit",
    "Software_position_limit_Maximal_position_limit",
    "Homing_method",
    "Home_offset",
    "Homing_acceleration",
    "Homing_speeds_Speed_for_switch_search",
    "Homing_speeds_Speed_for_zero_search",
    "HomingStallTime",
    "AbsEncoderErrorDetectWin",
    "HallErrorDetectWin",
    "CurrentLoopCtlSpeedMax",
    "FPCtlMaxForce",
    "FPCtlForceSlope",
    "FPCtlMinForce",
    "FVCtlMaxForce",
    "FVCtlForceSlope",
    "FVCtlMinForce",
    # "FPCtlEnable",
    "FVCtlEnable",
    "Encoder2MulRes",
    "Encoder2Baudrate",
    # "Encoder2Zerobits",
    "Encoder2Ackbits",
    "Encoder2ErrorActiveLevel",
    "Encoder2SensorDataPresentation",
    "OutLoopEncoderType",
    "OutLoopEncoderResolution",
    "ReductionRatioNumerator",
    "ReductionRatioDenominator",
    "ADC1VoltageRange",
    "ADC2VoltageRange",
    "FieldWeakeningControlCurrent",
    "DeviceTempProtectTimeWindow",
    "ImpedanceControl_K",
    "ImpedanceControl_B",
    "ImpedanceControl_M",
    # "TargetPressure",
    "ForceControlSensorType",
    "ForceControlMode",
    "ForceControlCycle",
    "AdmittanceControl_K",
    "AdmittanceControl_B",
    "AdmittanceControl_M",
    "ForceSensorLPFCutOffFrq",
    # "ForceSensorSamplingOffset",
    "ForceSensorDirection",
    # "ForceSpaceParaUnit",
    "EncoderDirection",
    "Encoder2Direction",
    "HallStateElecAngle",
    "HallStateElecAngle2",
    "HallStateElecAngle3",
    "HallStateElecAngle4",
    "HallStateElecAngle5",
    "HallStateElecAngle6",
    "HallStateElecAngle7",
    "HallStateElecAngle8",
    "ZPhaseElecAngle",
    "ElecAngleOffsetValue",
    "PwmCtlSpeedBasicFrq",
    "PulseControlSpeed_EnableFlag",
    "PulseControlSpeed_FilterTime",
    "PulseControlSpeed_Resolution",
    # "AnalogCtlSpeedEnable",
    # "AnalogCtlSpeedADCSource",
    "ExtADC1ReduceResolution",
    "ExtADC2ReduceResolution",
    # "BreakSetting_activation_velocity",
    # "BreakSetting_disengage_time",
    # "BreakSetting_engage_time",
    # "BreakSetting_enable",
    # "BreakSetting_disengage_software_delay_time",
    # "BreakSetting_engage_software_delay_time",
    # "BreakSetting_activation_delay_time",
    "PulsePositionControl_Enable",
    "PulsePositionControl_Mode",
    "DeadZoneCompensationEnable",
    "ADCSamplingOffset_ADC1",
    "ADCSamplingOffset_ADC2",
    "DriveOverloadProtection_RatedCurrent",
    "DriveOverloadProtection_PeakCurrent",
    "DriveOverloadProtection_PeakCurrentDuration",
    "DriveOverloadProtection_OverloadProtectionTime",
    "Encoder1HardwareEncoderResolution",
    "Encoder1ProtocolTotalBits",
    "Encoder1PositionLsbBitNb",
    "Encoder1HighMaskBits",
    "Encoder1ErrorBitNb",
    "Encoder1DataPollingTime",
    "Encoder2HardwareEncoderResolution",
    "Encoder2ProtocolTotalBits",
    "Encoder2PositionLsbBitNb",
    "Encoder2HighMaskBits",
    "Encoder2ErrorBitNb",
    "Encoder2DataPollingTime"
]

# 精简模式下需要隐藏的控件
hide_param_list = [
    "Inductance",
    "Resistance",
    "PolarPositives",
    "Inertia",
    "Damping",
    "Magnetic",
    "Motor_rated_torque",
    "TorqueConstant",
    "TRCEnable",
    "FCEnable"
]

# 离线数据参数
offline_data_param_list = [
    "PositionDemandValue",
    "PositionActualValue",
    "VelocityDemandValue",
    "VelocityActualValue",
    "TorqueDemandValue",
    "TorqueActualValue"
]

DriverBoardFileName = "Uservo_enc.bin"
InterFaceBoardFileName = "Uservo_interface.bin"

# curve_param_list = {
#     "Velocity_actual_value",  # 速度当前值
#     "CurSpeedError",  # 速度当前误差
#     "Position_actual_value",  # 位置当前值
#     "Following_error_actual_value",  # 跟随误差当前值
#     "Current_actual_value",  # 电流当前值
#     "CurCurrentError",  # 当前电流误差
#     "ZPhaseDetectError",  # Z相误差
#     "OutloopPositionActualValue",  # 外环当前位置
#     "OutloopFollowingError",  # 外环位置误差
#     "OutloopSpeedActualValue",  # 外环速度当前值
#     "OutloopSpeedError",  # 外环速度误差
# }


# IO输入头部标签
input_header_dict = {
    "input_no": "",
    "high": translate("Form", "上升沿触发事件"),
    "low": translate("Form", "下降沿触发事件"),
    "level": translate("Form", "极性设置"),
    "events": translate("Form", "触发事件"),
    "filter_time": translate("Form", "IO输入滤波时间/ms"),
    "level_status": translate("Form", "有效状态"),
}

# IO输入头部标签
input_header_dict_none = {
    "input_no": "",
    "high": "              ",
    "low": "              ",
    "level": "      ",
    "events": "        ",
    "filter_time": "                  ",
    "level_status": "        ",
}

# IO输出头部标签
output_header_dict = {
    "output_no": "",
    "high_low": translate("Form", "上升沿/下降沿"),
    "events": translate("Form", "触发事件"),
    "level_status": translate("Form", "有效状态"),
}

output_header_dict_none = {
    "output_no": "",
    "high_low": "             ",
    "events": "        ",
    "level_status": "        "
}

NoDisplayModes = ["CSP", "CSV", "CST", translate("Form", "电气辨识"), translate("Form", "方向辨识"),
                          translate("Form", "霍尔辨识"), translate("Form", "极对数辨识"), translate("Form", "Z相辨识"),
                          translate("Form", "电角度辨识"), translate("Form", "机械辨识"), translate("Form", "转矩脉动辨识"),
                          translate("Form", "机械特性辨识"),
                          translate("Form", "力控特性辨识"), translate("Form", "空模式"), translate("Form", "电气辨识_"),
                          translate("Form", "惯量校准")]

# 电流指令滤波器类型
CurrentFilterType = {
    translate("Form", "不滤波"): 0,
    translate("Form", "低通滤波器"): 1,
    # "高通滤波器": 2,
    # "带通滤波器": 3,
    translate("Form", "带阻滤波器(陷波滤波器)"): 4
}

# 低频振动滤波器类型
VibraFilterType = {
    translate("Form", "不滤波"): 0,
    translate("Form", "低通滤波器"): 1,
    # "高通滤波器": 2,
    # "带通滤波器": 3,
    translate("Form", "带阻滤波器(陷波滤波器)"): 4
}

# 使能状态下需要禁止写操作的控件
need_readOnly = [
    "EncodeType",
    "Inductance",
    "Resistance",
    "PolarPositives",
    "HardwareVolMaxLimit",
    "HardwareVolMinLimit",
    "HardwareCurMaxLimit",
    "OutLoopEncoderType",
    "OutLoopEncoderResolution",
    # "ControlCmd",
    "Abort_connection_option_code",
    "Shutdown_option_code",
    "Fault_reaction_option_code",
    # "Modes_of_operation",
    "Max_torque",
    "Max_current",
    "Motor_rated_current",
    "Motor_rated_torque",
    "Position_range_limit_Minimal_position_limit",
    "Position_range_limit_Maximal_position_limit",
    "Software_position_limit_Minimal_position_limit",
    "Software_position_limit_Maximal_position_limit",
    "Polarity",
    "Max_motor_speed",
    "Position_encoder_resolution_Encoder_increments",
    "Position_encoder_resolution_Motor_revolutions",
    "Position_factor_Numerator",
    "Position_factor_Feed_constant",
    "Velocity_factor_1_Numerator",
    "Velocity_factor_1_Divisor",
    "Acceleration_factor_Numerator",
    "Acceleration_factor_Divisor",
    "Interpolation_time_period_ip_time_units",
    "Interpolation_time_period_ip_time_index",
    "btn_save",
    "btn_input",
    "btn_output",
    "Damping",
    "Magnetic",
    "TorqueConstant",
    "Motor_type",
    "VelocityFeedbackMapping",
    "Motion_profile_type",
    "BreakSetting_enable",
    # "RepetitiveEnable",
    "MotorPolePairPitch",
    "ElecAngleIdentMode",
    "FCEnable",
    "TRCEnable",
    "FixCurrentFre",
    "FixCurrentAmp",
    "FixMacFre",
    "FixMacSpeed",
    "FixMacCurrent",
    "PolePairsIdElecCycles",
    "PolePairsIdWaitTime",
    "DirctionIdDistance",
    "ElcAngleAlignment",
    "ForceControlSensorType",
    "ForceControlMode",
    "MotorDirectionReverse",
    # "TargetPressure",
    # "ForcePosFuncWidget"
    # "NooAppParameters_RunningSpeed",
    # "NooAppParameters_UpwardPosition",
    # "NooAppParameters_DownwardPosition",
    # "NooAppParameters_PressureHoldingTime",
    # "NooAppParameters_PressureReachesFlag",
    # "NooAppParameters_AutomaticRunEnable",
    # "NooAppParameters_TopWaitingTime",
    # "PressureProtectionValue_Max",
    # "PressureProtectionValue_Min",
    # "TargetPressure"
]

VersionInfoList = [
    "HWSnNumber_1",
    "HWSnNumber_2",
    "HWSnNumber_3",
    "HWSnNumber_4",
    "HWSnNumber_5",
    "HWSnNumber_6",
    "HWSnNumber_7",
    "HWSnNumber_8",
    "HWSnNumber_9",
    "HWSnNumber_10",
    "HWSnNumber_11",
    "HWSnNumber_12",
    "HWSnNumber_13",
    "HWSnNumber_14",
    "HWSnNumber_15",
    "HWSnNumber_16",
    "HWSnNumber_17",
    "HWSnNumber_18",
    "HWSnNumber_19",
    "HWSnNumber_20",
    "HWSnNumber_21",
    "HWSnNumber_22",
    "HWSnNumber_23",
    "HWSnNumber_24",
    "HWSnNumber_25",
    "HWSnNumber_26",
    "HWSnNumber_27",
    "HWSnNumber_28",
    "HWSnNumber_29",
    "HWSnNumber_30",
    "HWSnNumber_31",
    "HWSnNumber_32",
    "Motor_rated_current",
    "Position_encoder_resolution_Encoder_increments",
    "OutLoopEncoderResolution",
    "EncodeType",
    "OutLoopEncoderType",
    "DriverBoardVersionHW",
    "DriverBoardVersionSW",
    "InterfaceBoardVersionSW",
    "HardwareCommunicationType",
    "ProductCode",
    "DriveOverloadProtection_RatedCurrent",
    "BSPSoftwareVersion",
    "BootloaderVersion",
]

RemoteType = {
    translate("Form", "被控端"): 0,
    translate("Form", "控制端"): 1
}

DeviceCommType = {
    "USB-HID": 0,
    "SERIAL": 1,
    "REMOTE": 2
}

OfflineDataType = {
    "5": 5
}

OfflineDataUnit = {
    "None": 0,
    "mA": 1,
    "A": 2,
    "mV": 3,
    "V": 4,
    "P": 5,
    "P/s": 6,
    "P/s²": 7,
    "RPM": 8,
    "RPM/s": 9,
    "unit": 10,
    "ms": 11,
}

HostCommType = {
    "USB-HID": 0,
    "RS-232": 1,
    "RS-485": 2,
    "Ws": 3,
}

DriverType = {
    "UF-48V**AEDx": 1,
    "UF-48V**ACDN": 2,
    "UF-48V**AMDN": 11,
    "UF-48V**APDN": 3,
    "DE1-E48***": 4,
    "DE1-G48***": 5,
    "DE2-E48***DD": 6,
    "DE2-G48**NDD": 7,
    "DE2-G48***DA": 8,
    "D-A030*DNN": 9,
    "D-A060*DNN": 12,
    "D-A100*DNN": 10,
}

IdentifyStep = IdentifyModes.copy()


EncodeChoose = {
    translate("Form", "编码器1"): 0,
    translate("Form", "编码器2"): 1,
}

EncodeChooseType = {
    translate("Form", "单编码器"): 0,
    translate("Form", "双编码器"): 1,
}

wizard_params_title = [
    translate("Form", "通讯连接"),
    translate("Form", "电机参数"),
    translate("Form", "抱闸设置"),
    translate("Form", "编码器选择"),
    translate("Form", "限制参数设置"),
    translate("Form", "设置完成"),
]

protocol_type = {
    translate("Form", "版本B"): 0,
    translate("Form", "版本A"): 1,
}

current_filter_index = {
    translate("Form", "1号滤波器"): 1,
    translate("Form", "2号滤波器"): 2,
    translate("Form", "3号滤波器"): 3,
    translate("Form", "4号滤波器"): 4
}

CANBaudRate = {
    "1000": 1000,
    "800": 800,
    "500": 500,
    "250": 250,
    "125": 125,
}

SyncModelType = {
    "FreeRun": 0,
    "SM": 1,
    "DC": 2,
    "按照主站配置": 255,
}

ModbusBaudRate = {
    "115200": 115200,
    "57600": 57600,
    "56000": 56000,
    "43000": 43000,
    "38400": 38400,
    "19200": 19200,
    "9600": 9600,
    "4800": 4800,
    "2400": 2400,
    "1200": 1200,
}


if __name__ == '__main__':
    print(list(MotorType.keys()))
ModeOfOperation = {
    "advance_mode": translate("Form", "高级模式"),
    "simple_mode": translate("Form", "精简模式"),
}
SwitchMap = {
    translate("Form", "关闭"): 0,
    translate("Form", "开启"): 1
}


BitDefine = {
    "Bit15": "暂无定义！",
    "Bit14": "暂无定义！",
    "Bit13": "根据模式定义！",
    "Bit12": "根据模式定义！",
    "Bit11": "如果Bit11置1，则表明触发位置限制（软件限位或硬件限位）！",
    "Bit10": "如果Bit10置1，则表明目标到达！",
    "Bit9": "暂无定义！",
    "Bit8": "暂无定义！",
    "Bit7": "如果Bit7置1，则表明警告！",
    "Bit6": "如果Bit6置1，则表明伺服未接通PWM！",
    "Bit5": "如果Bit5置0，则表明急停！",
    "Bit4": "如果Bit4置1，则表明驱动器已接通电源！",
    "Bit3": "如果Bit3置1，则表明处于错误状态！",
    "Bit2": "如果Bit2置1，则表明伺服运行！",
    "Bit1": "如果Bit1置1，则表明伺服已接通PWM！",
    "Bit0": "如果Bit0置0，则表明伺服未准备好！",
    "Bit-1": "暂无定义！",
}