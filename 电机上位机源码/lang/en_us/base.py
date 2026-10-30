#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time        : 2023/3/29 16:17
# @Author      : WeiZujun@Motiong.com
# @Description :
# @File        : base_english.py
# @Software    : PyCharm
from enum import Enum
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




UpdateError = {
    "0": "Normal",
    "1": "App self-start checksum error",
    "2": "AppAddrError",
    "3": "App stack address error",
    "4": "Failed to save firmware transfer size",
    "5": "Firmware transfer size exceeds limit",
    "6": "Error in firmware transfer checksum",
    "7": "Firmware transfer failed",
    "8": "Wrong number of firmware transmission packets",
    "9": "5s timeout for firmware transmission",
    "10": "A SPI Flash Check fail"
}

DeviceConnectState = {
    "Device not connected": 0,
    "Device connected": 1
}

DeviceCommState = {
    "Detecting communication": -1,
    "Abnormal device communication": 0,
    "Normal device communication": 1
}

SerialPortCount = {}

SerialPortChoose = {
    "Display normal serial ports": 0,
    "Display all serial ports": 1
}

HallState = {
    "Normal": 0,
    "Error": 1,
    "Hall not connected": 2
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
    "Start idle state": FreeState,
    "Successful identification": IdentificationSuccess,
    "Electrical identifying...": ElectricalIdentificationStatus,
    "Direction identifying...": DirectionIdentificationStatus,
    "Inertia identifying...": InertiaIdentificationState,
    "Torque pulse identifying...": TorqueRippleIdentificationState,
    "Error state": ErrorIdentificationState
}

IdentifyModes = {
    "Electrical identification": ElectricalIdentification,
    "Direction identification": DirectionIdentification,
    "Hall identification": HallIdentification,
    "Pole-pair number identification": PolarLogarithmIdentification,
    "Z-phase identification": ZPhaseIdentification,
    "Electrical angle identification": ElectricAngleIdentification,
    "Mechanical identification": MechanicalIdentification,
    "Torque pulse identification": TorqueRippleIdentification
}

ElecAngIdentModes = {
    "Strong-pull phase finding mode": 0,
    "Short-distance phase finding mode": 3,
    "Vector pulse phase finding mode": 4
}

ElecAngIdentModesTip = {
    "0": "Electrical angle identification mode 1",
    "3": "Electrical angle identification mode 4"
}

ElecIdentifyModes = {
    "Local electrical characteristic identification": 1,
    "Cloud electrical characteristic identification": 2
}

MechanicalIdentifyModes = {
    "Local mechanical characteristic identification": 1,
    "Cloud mechanical characteristic identification": 2
}

encoder_num_map = {str(i): i for i in range(1, 23)}

bin_version_map = {}

ProductSelect = {
    "UF-48VxxACDN": 0,
    "UF-48VxxAEDx": 1,
    "UF-48VxxAMDN": 2,
    "UF-48VxxAPDN": 3,
    "DG1-E48xxx": 4,
    "DG1-G48xxx": 5,
    "DG2-E48xxxDD": 6,
    "DG2-G48xxNDD": 7,
    "DG2-G48xxxDA": 8,
    "DP2-E48xxxDD": 9,
    "DP2-G48xxNDD": 10,
    "DR1-E48xxx": 11,
    "DN1-G60xxN": 12,
    "D-AxxxEDNN": 13,
    "D-AxxxPDNN": 14,
    "DG3-G72xxNDD": 15,
    "DG3-G72xxNDD-M": 16,
}

LanguageType = {
    "Chinese (simplified)": 0,
    "English": 1
}


GainMap = {"Low Gain": 30, "Middle Gain": 50, "High Gain": 100}


HallType = {
    "No Hall": 0,
    "120°": 1
}

MotorType = {
    "DC brush motor": 1,
    "PMS Rotary Motor": 3,
    "PMS Linear Motor": 32769,
    "Voice Coil Motor": 32770
}

MotorCurrentType = {
    "Valid Value (Arms)": 0,
    "Peak Value (Amp)": 1
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

EncodeType = {
    "Incremental - ABZ": 0,
    "Incremental - AB": 1,
    "Incremental - UDC.Pa": 8,
    "Absolute - BISS-C": 2,
    "Absolute - Tamagawa": 3,
    "Absolute - SSI": 4,
    "Absolute - UDC.Pa": 6,
    "Absolute - EnDat22": 9,
    "Absolute - Nikon": 7,
    "Analog - SIN/COS": 10,
    "Analog - Hiperface": 11,
    "Analog - Linear Hall": 12,
    "Analog - Resolver": 13,
    "Halls Only": 5,
    "No encoder": 255
}

OutsideEncoderType = {
    "Incremental - ABZ": 0,
    "Incremental - AB": 1,
    "Incremental - UDC.Pa": 8,
    "Absolute - BISS-C": 2,
    "Absolute - Tamagawa": 3,
    "Absolute - SSI": 4,
    "Absolute - UDC.Pa": 6,
    "Absolute - EnDat22": 9,
    "Absolute - Nikon": 7,
    "Analog - SIN/COS": 10,
    "Analog - Hiperface": 11,
    "Analog - Linear Hall": 12,
    "Analog - Resolver": 13,
    "Halls Only": 5,
    "No encoder": 255
}

SpeedMap = {
    "Encoder 1": 1,
    "Encoder 2": 2
}

DualFBControlMode = {
    "Fully closed-loop": 0,
    "Semi-closed loop": 1
}


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
    "Binary": 0,
    "Gray code": 1
}

EncoderSignalType = {
    "Differential signal": 0,
    "Single-end signal": 1
}

EventType = {
    "Parameter": 0,
    "Address": 1,
}

EventValue = {
    "Occurrence Alarm Trigger": 1,
    "Rising Threshold Trigger": 2,
    "Falling Threshold Trigger": 3,
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

EncoderResolverMultiplicationFactor = {
    "6": 6,
    "7": 7,
    "8": 8,
    "9": 9,
    "10": 10,
    "11": 11,
    "12": 12
}

EncoderResolverFrq = {
    "10": 10000,
    "12": 12000,
    "15": 15000,
    "20": 20000
}

CurrentLoopFrq = {
    "20000": 20000,
    "40000": 40000
}

PosLoopFrq = {
    "10000": 10000,
    "20000": 20000
}

EncoderZeroBits = {
    "0": 0,
    "1": 1,
    "2": 2,
    "3": 3,
    "4": 4,
    "5": 5,
    "6": 6,
    "7": 7,
    "8": 8,
    "9": 9,
    "10": 10,
    "11": 11,
    "12": 12,
    "13": 13,
    "14": 14,
    "15": 15,
    "16": 16,
    "17": 17,
    "18": 18,
    "19": 19,
    "20": 20,
    "21": 21,
    "22": 22,
    "23": 23,
    "24": 24,
    "25": 25,
    "26": 26,
    "27": 27,
    "28": 28,
    "29": 29,
    "30": 30,
    "31": 31,
    "32": 32,
    "33": 33,
    "34": 34,
    "35": 35,
    "36": 36
}

EncoderResolutionList = {
    "0": 1,
    "1": 2,
    "2": 4,
    "3": 8,
    "4": 16,
    "5": 32,
    "6": 64,
    "7": 128,
    "8": 256,
    "9": 512,
    "10": 1024,
    "11": 2048,
    "12": 4096,
    "13": 8192,
    "14": 16384,
    "15": 32768,
    "16": 65536,
    "17": 131072,
    "18": 262144,
    "19": 524288,
    "20": 1048576,
    "21": 2097152,
    "22": 4194304,
    "23": 8388608,
    "24": 16777216,
    "25": 33554432,
    "26": 67108864,
    "27": 134217728,
    "28": 268435456,
    "29": 536870912,
    "30": 1073741824,
    "31": 2147483648,
    "32": 4294967296,
    "33": 8589934592,
    "34": 17179869184,
    "35": 34359738368,
    "36": 68719476736
}

AbsEncoderResolutionList = {
    "8": 256,
    "9": 512,
    "10": 1024,
    "11": 2048,
    "12": 4096,
    "13": 8192,
    "14": 16384,
    "15": 32768,
    "16": 65536,
    "17": 131072,
    "18": 262144,
    "19": 524288,
    "20": 1048576,
    "21": 2097152,
    "22": 4194304,
    "23": 8388608,
    "24": 16777216,
    "25": 33554432,
    "26": 67108864,
    "27": 134217728,
    "28": 268435456,
    "29": 536870912,
    "30": 1073741824,
    "31": 2147483648,
    "32": 0,
    "33": 8589934592,
    "34": 17179869184,
    "35": 34359738368,
    "36": 68719476736
}

EncoderHardWareRes = {
    "8": 256,
    "9": 512,
    "10": 1024,
    "11": 2048,
    "12": 4096,
    "13": 8192,
    "14": 16384,
    "15": 32768,
    "16": 65536,
    "17": 131072,
    "18": 262144,
    "19": 524288,
    "20": 1048576,
    "21": 2097152,
    "22": 4194304,
    "23": 8388608,
    "24": 16777216,
    "25": 33554432,
    "26": 67108864,
    "27": 134217728,
    "28": 268435456,
    "29": 536870912,
    "30": 1073741824,
    "31": 2147483648,
    "32": 0,
    "33": 8589934592,
    "34": 17179869184,
    "35": 34359738368,
    "36": 68719476736
}

EncoderProtocolTotalBits = {
    "0": 0,
    "1": 1,
    "2": 2,
    "3": 3,
    "4": 4,
    "5": 5,
    "6": 6,
    "7": 7,
    "8": 8,
    "9": 9,
    "10": 10,
    "11": 11,
    "12": 12,
    "13": 13,
    "14": 14,
    "15": 15,
    "16": 16,
    "17": 17,
    "18": 18,
    "19": 19,
    "20": 20,
    "21": 21,
    "22": 22,
    "23": 23,
    "24": 24,
    "25": 25,
    "26": 26,
    "27": 27,
    "28": 28,
    "29": 29,
    "30": 30,
    "31": 31,
    "32": 32,
    "33": 33,
    "34": 34,
    "35": 35,
    "36": 36,
    "37": 37,
    "38": 38,
    "39": 39,
    "40": 40,
    "41": 41,
    "42": 42,
    "43": 43,
    "44": 44,
    "45": 45,
    "46": 46,
    "47": 47,
    "48": 48,
    "49": 49,
    "50": 50,
    "51": 51,
    "52": 52,
    "53": 53,
    "54": 54,
    "55": 55,
    "56": 56,
    "57": 57,
    "58": 58,
    "59": 59,
    "60": 60,
    "61": 61,
    "62": 62,
    "63": 63,
    "64": 64
}

EncoderPosLSBBit = {
    "0": 0,
    "1": 1,
    "2": 2,
    "3": 3,
    "4": 4,
    "5": 5,
    "6": 6,
    "7": 7,
    "8": 8,
    "9": 9,
    "10": 10,
    "11": 11,
    "12": 12,
    "13": 13,
    "14": 14,
    "15": 15,
    "16": 16,
    "17": 17,
    "18": 18,
    "19": 19,
    "20": 20,
    "21": 21,
    "22": 22,
    "23": 23,
    "24": 24,
    "25": 25,
    "26": 26,
    "27": 27,
    "28": 28,
    "29": 29,
    "30": 30,
    "31": 31,
    "32": 32,
    "33": 33,
    "34": 34,
    "35": 35,
    "36": 36
}

EncoderHighMaskBits = {
    "0": 0,
    "1": 1,
    "2": 2,
    "3": 3,
    "4": 4,
    "5": 5,
    "6": 6,
    "7": 7,
    "8": 8,
    "9": 9,
    "10": 10,
    "11": 11,
    "12": 12,
    "13": 13,
    "14": 14,
    "15": 15,
    "16": 16,
    "17": 17,
    "18": 18,
    "19": 19,
    "20": 20,
    "21": 21,
    "22": 22,
    "23": 23,
    "24": 24,
    "25": 25,
    "26": 26,
    "27": 27,
    "28": 28,
    "29": 29,
    "30": 30,
    "31": 31,
    "32": 32,
    "33": 33,
    "34": 34,
    "35": 35,
    "36": 36
}

EncoderErrorBit = {
    "0": 0,
    "1": 1,
    "2": 2,
    "3": 3,
    "4": 4,
    "5": 5,
    "6": 6,
    "7": 7,
    "8": 8,
    "9": 9,
    "10": 10,
    "11": 11,
    "12": 12,
    "13": 13,
    "14": 14,
    "15": 15,
    "16": 16,
    "17": 17,
    "18": 18,
    "19": 19,
    "20": 20,
    "21": 21,
    "22": 22,
    "23": 23,
    "24": 24,
    "25": 25,
    "26": 26,
    "27": 27,
    "28": 28,
    "29": 29,
    "30": 30,
    "31": 31,
    "32": 32,
    "33": 33,
    "34": 34,
    "35": 35,
    "36": 36
}

EncoderDataPollingTime = {
    "1": 0,
    "2": 1,
    "3": 2,
    "4": 3,
    "5": 4,
    "6": 5,
    "7": 6,
    "8": 7
}

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
}

MotorShowState = {
    "Disable motor": 0,
    "Enable motor": 1,
    "Motor enabling": 2
}

EncodeState = {
    "Normal": 0,
    "Error": 1,
    "Encoder not connected": 2,
    "Encoder not prepared": 3
}

QuickStopCode = {
    "Quick Stop 0": 0,
    "Quick Stop 1": 1,
    "Quick Stop 2": 2,
    "Quick Stop 3": 3,
    "Quick Stop 5": 5,
    "Quick Stop 6": 6,
    "Quick Stop 7": 7
}

Planner = {
    "Position mode": 1,
    "Velocity mode": 3,
    "Torque mode": 4,
    "Analog control speed": 22,
    "Analog control torque": 23,
    "Pulse control position": 31,
    "PWM control speed": 42,
    "Homing mode": 6,
    "Reference Signal": 61,
    "Force control mode": 11,
    "Reciprocating Motion": 60,
    "Multi-segment Running Mode": 62,
    "CSP": 8,
    "CSV": 9,
    "CST": 10
}

DisplayPlanner = {
    "Position mode": 1,
    "Velocity mode": 3,
    "Torque mode": 4,
    "Analog control speed": 22,
    "Analog control torque": 23,
    "Pulse control position": 31,
    "PWM control speed": 42,
    "Homing mode": 6,
    "Reference Signal": 61,
    "Force control mode": 11,
    "Reciprocating Motion": 60,
    "Multi-segment Running Mode": 62,
    "CSP": 8,
    "CSV": 9,
    "CST": 10,
    "Electrical identification": -1,
    "Direction identification": -2,
    "Hall identification": -3,
    "Pole-pair number identification": -4,
    "Z-phase identification": -5,
    "Electrical angle identification": -6,
    "Mechanical identification": -7,
    "Torque pulse identification": -8,
    "Mechanical Characteristic Identification": -9,
    "Force Control Characteristic Identification": -10,
    "Electrical Identification_": -11,
    "Inertia calibration": -12,
    "Empty mode": 0
}

PositionOption = {
    "Relative position target value": 0,
    "Relative position planning value": 1,
    "Relative position current value": 2
}

DisplayMode = {
    "-1": "Electrical identification",
    "-2": "Direction identification",
    "-3": "Hall identification",
    "-4": "Pole-pair number identification",
    "-5": "Z-phase identification",
    "-6": "Electrical angle identification",
    "-7": "Mechanical identification",
    "-8": "Torque pulse identification"
}

VelocityControlMode = {
    "Position command control": 0,
    "Speed command control": 1
}

PulseOutputMode = {
    "Disable output": 0,
    "Quadrature output mode + software Z-phase": 1,
    "Quadrature output mode + hardware Z-phase": 2
}

PulseOutputMode2 = {
    "Disable output": 0,
    "Quadrature output mode + software Z-phase": 1
}

MotionProfileType = {
    "No filter": 0,
    "TD filter": 1,
    "Moving average filter": 2
}

SmoothingFactorIndex = {
    "-3": -3,
    "-2": -2,
    "-1": -1,
    "0": 0,
    "1": 1,
    "2": 2,
    "3": 3
}

MeasureMode = {
    "In-place settling time measurement": 0
}

PositionType = {
    "Absolute position": 63,
    "Relative position": 127
}

SpeedUnit_RPM_P = {
    "RPM": 0,
    "P/s": 1,
    "deg/s": 2,
    "Pulse equivalent": 3,
    "mm/s": 4
}

PosRotary = {
    "P": 0,
    "deg": 1,
    "Pulse equivalent": 2,
    "mm": 3
}

Unit_MM_P = {
    "P": 0,
    "mm": 1
}

ForceUnit = {
    "unit": 0,
    "g": 1
}

MultiSegments = {
    "1 segment": 1,
    "2 segments": 2,
    "3 segments": 3,
    "4 segments": 4,
    "5 segments": 5,
    "6 segments": 6,
    "7 segments": 7,
    "8 segments": 8,
    "9 segments": 9,
    "10 segments": 10,
    "11 segments": 11,
    "12 segments": 12,
    "13 segments": 13,
    "14 segments": 14,
    "15 segments": 15,
    "16 segments": 16
}

Cycle_Model = {
    "Sequential execution": 0,
    "Looping execution": 1,
    "DI switch": 2,
    "Command switch": 3
}

Time_Unit = {
    "ms": 0,
    "s": 1
}

Jump_Enable = {
    "No": 0,
    "Yes": 1
}

Res_Action = {
    "Continue": 0
}

Fault_Res_Action = {
    "Stop": 0,
    "Auto clear, return to origin": 1
}

Start_CMD_Option = {
    "Direct Start": 0,
    "Control word command start": 1,
}

Opera_Mode = {
    "Position mode": 0,
    "Velocity mode": 1,
    "Torque mode": 2,
    "Force control mode": 3
}

Target_Attr = {
    "Absolute position": 0,
    "Relative position": 1
}

Force_Type = {
    "Open-loop force control": 0,
    "Closed-loop force control": 1
}

Target_Attr2 = {
    "Non-return-to-zero": 0,
    "Return-to-zero": 1
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
    "35.Home immediate on current position": 35
}

ErrorMappingUnit = {
    "um": 0,
    "mm": 1,
    "P": 2
}

ADC_VREF = {
    "-12.288 ~ +12.288": 0,
    "-10.24 ~ +10.24": 1,
    "-6.14 ~ +6.14": 2,
    "-5.12 ~ +5.12": 3,
    "-2.56 ~ +2.56": 4,
    "0 ~ +12.288": 8,
    "0 ~ +10.24": 9,
    "0 ~ +6.14": 10,
    "0 ~ +5.12": 11
}

ADC_Resolution = {
    "0 bit": 0,
    "1 bit": 1,
    "2 bits": 2,
    "3 bits": 3,
    "4 bits": 4,
    "5 bits": 5,
    "6 bits": 6,
    "7 bits": 7,
    "8 bits": 8,
    "9 bits": 9,
    "10 bits": 10,
    "11 bits": 11,
    "12 bits": 12,
    "13 bits": 13,
    "14 bits": 14,
    "15 bits": 15,
    "16 bits": 16,
    "17 bits": 17,
    "18 bits": 18,
    "19 bits": 19,
    "20 bits": 20,
    "21 bits": 21,
    "22 bits": 22,
    "23 bits": 23,
    "24 bits": 24,
    "25 bits": 25,
    "26 bits": 26,
    "27 bits": 27,
    "28 bits": 28,
    "29 bits": 29,
    "30 bits": 30,
    "31 bits": 31
}

Direction = {
    "Forward": 1,
    "Reverse": -1
}

BreakSettingMap = {
    "Manual control": 0,
    "Auto control": 1
}

ForceControlSensorType = {
    "Current": 0,
    "ADC channel-1": 1,
    "ADC channel-2": 2,
    "Extended channel": 3
}

AnalogCtrlSpeedAdcSource = {
    "Analog channel-1": 1,
    "Analog channel-2": 2
}

ForceUnitIndex = {
    "0": 0,
    "-1": -1,
    "-2": -2,
    "-3": -3,
    "-4": -4,
    "-5": -5,
    "-6": -6
}

ForceControlMode = {
    "Current control mode": 1,
    "Position control mode": 2
}

ForceReachesFlag = {
    "Not in place": 0,
    "In place": 1
}

ForceCurveFlag = {
    "End of run": 0,
    "Reach point A": 1,
    "Meet return conditions": 2,
    "Complete curve action and return quickly": 3
}

CurrentLoopControlMode = {
    "P control": 0,
    "PIC control": 1
}

PulseCtrlSpeedRes = {
    "0 bit": 0,
    "1 bit": 1,
    "2 bits": 2,
    "3 bits": 3,
    "4 bits": 4,
    "5 bits": 5,
    "6 bits": 6,
    "7 bits": 7,
    "8 bits": 8,
    "9 bits": 9,
    "10 bits": 10,
    "11 bits": 11,
    "12 bits": 12,
    "13 bits": 13,
    "14 bits": 14,
    "15 bits": 15,
    "16 bits": 16,
    "17 bits": 17,
    "18 bits": 18,
    "19 bits": 19,
    "20 bits": 20,
    "21 bits": 21,
    "22 bits": 22,
    "23 bits": 23,
    "24 bits": 24,
    "25 bits": 25,
    "26 bits": 26,
    "27 bits": 27,
    "28 bits": 28,
    "29 bits": 29,
    "30 bits": 30,
    "31 bits": 31
}

PulsePositionCtrlMode = {
    "Orthotropic control": 0,
    "Direction pulse control": 1,
    "Positive/negative double-pulse control": 2
}

PulseInputChannel = {
    "High-speed input": 0,
    "Low-speed input": 1
}

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

RefSignalType = {
    "Step": 2,
    "Sine": 1,
    "Triangle": 3,
}

RefSignalObject = {
    "Position reference signal": 1,
    "Speed reference signal": 2,
    "Torque reference signal": 3,
    "AB-axis voltage reference signal": 4,
    "D-axis current reference signal": 5
}

OfflineDataState = {
    "Not initialized": 0,
    "Initializing": 1,
    "Configurable parameters": 3,
    "Parameter configuration completed": 7,
    "Acquisition completed": 23,
    "Data conversion completed": 55,
    "Error state": 128
}

SamplingDataType = {
    "Real-time position planning value": 0,
    "Actual position running value": 1,
    "Real-time speed planning value": 2,
    "Actual speed running value": 3,
    "Real-time torque planning value": 4,
    "Actual torque running value": 5
}

SamplingTimeFlag = {
    "Signal": 1,
    "Normal": 2
}

SamplingRes = {
    "1 control cycle": 1,
    "2 control cycles": 2,
    "3 control cycles": 3,
    "4 control cycles": 4,
    "5 control cycles": 5,
    "6 control cycles": 6
}

TriggerMethod = {
    "Rising edge trigger": 0,
    "Falling edge trigger": 1
}

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
    "Sampling data before the trigger time point": 0,
    "Sampling data after the trigger time point": 1
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

BodeDiagramScanMode = {
    "Current closed loop": 1,
    "Speed open loop": 2,
    "Speed closed loop": 3
}

ProductCode = {
    "65537": "USERVO-U60A{}",
    "131073": "U-48V{}AC",
    "131074": "U-48V{}AE",
    "131075": "U-48V{}AM",
    "131076": "U-48V{}AP",
    "196609": "SERVOTOP-220V{}AC",
    "196610": "SERVOTOP-220V{}AE",
    "196611": "SERVOTOP-220V{}AM",
    "196612": "SERVOTOP-220V{}AP",
    "327681": "UF-48V{}AC",
    "327682": "UF-48V{}AE",
    "327683": "UF-48V{}AM",
    "327684": "UF-48V{}AP",
    "458754": "UF-48V{}AEDR"
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

multi_motion_mode = {
    "Absolute position mode": 0,
    "Relative position mode": 1,
    "Velocity mode": 2,
    "Torque mode": 3,
    "Force control mode": 4
}

multi_search_mode = {
    "No search": 255,
    "Absolute position": 0,
    "Relative position": 1,
    "Speed": 2,
    "Torque": 3,
    "Force control": 4,
    "Absolute position, greater than the target value": 5,
    "Relative position, greater than the target value": 6,
    "Speed, greater than the target value": 7,
    "Torque, greater than the target value": 8,
    "Force control, greater than the target value": 9,
    "Absolute position, smaller than the target value": 10,
    "Relative position, smaller than the target value": 11,
    "Speed, smaller than the target value": 12,
    "Torque, smaller than the target value": 13,
    "Force control, smaller than the target value": 14
}

base_read = {
    "HallState": None,
    "EncodeState": None,
    "DC_link_circuit_voltage": None,
    "DeviceTemp": None,
    "Error_code": None,
    "Statusword": None,
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

version_read = {
    "ProductCode": None,
    "DriverBoardVersionHW": None,
    "DriverBoardVersionSW": None,
    "InterfaceBoardVersionSW": None
}

version_type = {
    "0": "Release",
    "1": "Rev1",
    "2": "Rev2",
    "3": "Rev3",
    "4": "Rev4",
    "255": "Alpha",
    "254": "Beta",
    "253": "Weekly build"
}

sw_bin_type = {
    "0": "Release",
    "2": "Alpha",
    "4": "Beta",
    "6": "RC",
    "8": "Delta"
}

curve_read = {
    "Position_actual_value":  None,
    "Following_error_actual_value":  None,
    "Velocity_actual_value":  None,
    "CurSpeedError":  None,
    "Current_actual_value":  None,
    "CurCurrentError":  None
}

curve_params = {}

identify_param_read = {
    "Inductance":  None,
    "Resistance":  None,
    "PolarPositives":  None,
    "Inertia":  None,
    "Damping":  None,
    "TorqueConstant":  None,
    "ZPhaseElecAngle":  None,
    "ElecAngleOffsetValue":  None,
    "EncoderDirection":  None,
    "Encoder2Direction":  None,
    "HallStateElecAngle":  None,
    "HallStateElecAngle2":  None,
    "HallStateElecAngle3":  None,
    "HallStateElecAngle4":  None,
    "HallStateElecAngle5":  None,
    "HallStateElecAngle6":  None,
    "HallStateElecAngle7":  None,
    "HallStateElecAngle8":  None
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

IOFallingTriggerEvents = {
    "Ignore input": 0,
    "Enable motor": 1,
    "Disable motor": 2,
    "Clear error": 3,
    "Halt": 4,
    "Emergency stop": 5,
    "Switch operation mode": 6,
    "Homing": 10,
    "Switch position mode": 15,
    "Switch velocity mode": 16,
    "Switch torque mode": 17,
    "Switch analog control speed mode": 18,
    "Switch analog control torque": 19,
    "Switch pulse control position": 20,
    "Switch force control mode": 21,
    "Switch multi-segment running mode": 22,
    "Switch homing mode": 26,
    "Segment trigger of multi-segment running mode": 27,
    "Set multi-segment selection Bit0 to 1": 28,
    "Set multi-segment selection Bit0 to 0": 29,
    "Set multi-segment selection Bit1 to 1": 30,
    "Set multi-segment selection Bit1 to 0": 31,
    "Set multi-segment selection Bit2 to 1": 32,
    "Set multi-segment selection Bit2 to 0": 33,
    "Set multi-segment selection Bit3 to 1": 34,
    "Set multi-segment selection Bit3 to 0": 35,
    "Cancel pause": 36,
    "Clear all warnings": 37,
    "Probe 1": 38,
    "Probe 2": 39,
    "Start recording max torque": 40,
    "Stop recording max torque": 41,
    "Record Position 1": 42,
    "Record Position 2": 43,
    "Record Position 3": 44,
    "Record Position 4": 45,
}

IORisingTriggerEvents = {
    "Ignore input": 0,
    "Enable motor": 1,
    "Disable motor": 2,
    "Clear error": 3,
    "Halt": 4,
    "Emergency stop": 5,
    "Switch operation mode": 6,
    "Homing": 10,
    "Switch position mode": 15,
    "Switch velocity mode": 16,
    "Switch torque mode": 17,
    "Switch analog control speed mode": 18,
    "Switch analog control torque": 19,
    "Switch pulse control position": 20,
    "Switch force control mode": 21,
    "Switch multi-segment running mode": 22,
    "Positive limit switch": 23,
    "Homing switch": 24,
    "Negative limit switch": 25,
    "Switch homing mode": 26,
    "Segment trigger of multi-segment running mode": 27,
    "Set multi-segment selection Bit0 to 1": 28,
    "Set multi-segment selection Bit0 to 0": 29,
    "Set multi-segment selection Bit1 to 1": 30,
    "Set multi-segment selection Bit1 to 0": 31,
    "Set multi-segment selection Bit2 to 1": 32,
    "Set multi-segment selection Bit2 to 0": 33,
    "Set multi-segment selection Bit3 to 1": 34,
    "Set multi-segment selection Bit3 to 0": 35,
    "Cancel pause": 36,
    "Clear all warnings": 37,
    "Probe 1": 38,
    "Probe 2": 39,
    "Start recording max torque": 40,
    "Stop recording max torque": 41,
    "Record Position 1": 42,
    "Record Position 2": 43,
    "Record Position 3": 44,
    "Record Position 4": 45,
}

IOInputAutoTriggerEnable = {
    "Edge trigger": 0,
    "Level trigger": 1
}

OutputStatus = {
    "Low": 0,
    "High": 1
}

OutputEffective = {
    "Active low": 0,
    "Active high": 1
}

InputEffective = {
    "Active low": 0,
    "Active high": 1
}

OutputEvents = {
    "Target reached": 1,
    "General output": 0,
    "Servo ready": 2,
    "Position reached": 3,
    "Zero-velocity reached": 4,
    "Velocity reached": 5,
    "Homing completed": 6,
    "Drive error": 7,
    "Warning output": 9,
    "Multi-segment running status": 27,
    "Z-phase pulse signal": 28
}

HardWareCommType = {
    "0": "CAN",
    "1": "ECAT",
    "2": "Modbus",
    "3": "Pulse"
}

HardWareEncoderType = {
    "0": "Single encoder",
    "1": "Dual encoder"
}

SubdivisionNumber = {
    "2": 2,
    "3": 3,
    "4": 4,
    "5": 5,
    "6": 6,
    "7": 7,
    "8": 8,
    "9": 9,
    "10": 10,
    "11": 11,
    "12": 12,
    "13": 13,
    "14": 14,
    "15": 15,
    "16": 16
}

AnalogCtrlMode = {
    "Disable analog control": 0,
    "Analog control speed": 1,
    "Analog control torque": 2
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

input_header_dict = {
    "input_no": "",
    "high": "Rising edge trigger event",
    "low": "Falling edge trigger event",
    "level": "Polarity settings",
    "events": "Trigger event",
    "filter_time": "IO input filter time/ms",
    "level_status": "Valid status"
}

input_header_dict_none = {
    "input_no": "",
    "high": "              ",
    "low": "              ",
    "level": "      ",
    "events": "        ",
    "filter_time": "                  ",
    "level_status": "        "
}

output_header_dict = {
    "output_no": "",
    "high_low": "Rising/falling edge",
    "events": "Trigger event",
    "level_status": "Valid status"
}

output_header_dict_none = {
    "output_no": "",
    "high_low": "             ",
    "events": "        ",
    "level_status": "        "
}

NoDisplayModes = [
    "CSP",
    "CSV",
    "CST",
    "Electrical identification",
    "Direction identification",
    "Hall identification",
    "Pole-pair number identification",
    "Z-phase identification",
    "Electrical angle identification",
    "Mechanical identification",
    "Torque pulse identification",
    "Mechanical Characteristic Identification",
    "Force Control Characteristic Identification",
    "Empty mode",
    "Electrical Identification_",
    "Inertia calibration"
]

CurrentFilterType = {
    "No filter": 0,
    "Low pass filter": 1,
    "Band stop filter(notching filter)": 4
}

VibraFilterType = {
    "No filter": 0,
    "Low pass filter": 1,
    "Band stop filter(notching filter)": 4
}

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
    "ControlCmd",
    "Abort_connection_option_code",
    "Shutdown_option_code",
    "Fault_reaction_option_code",
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
    "MotorDirectionReverse"
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
    "BootloaderVersion"
]

RemoteType = {
    "Controlled terminal": 0,
    "Control terminal": 1
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
    "ms": 11
}

HostCommType = {
    "USB-HID": 0,
    "RS-232": 1,
    "RS-485": 2,
    "Ws": 3
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
    "D-A100*DNN": 10
}

IdentifyStep = {
    "Electrical identification": -1,
    "Direction identification": -2,
    "Hall identification": -3,
    "Pole-pair number identification": -4,
    "Z-phase identification": -5,
    "Electrical angle identification": -6,
    "Mechanical identification": -7,
    "Torque pulse identification": -8
}

EncodeChoose = {
    "Encoder 1": 0,
    "Encoder 2": 1
}

EncodeChooseType = {
    "Single encoder": 0,
    "Dual encoder": 1
}

wizard_params_title = [
    "Communication connection",
    "Motor parameters",
    "Brake settings",
    "Encoder selection",
    "Limit parameter settings",
    "Setting completed",
]

protocol_type = {
    "Version B": 0,
    "Version A": 1
}

current_filter_index = {
    "Filter #1": 1,
    "Filter #2": 2,
    "Filter #3": 3,
    "Filter #4": 4
}

CANBaudRate = {
    "1000": 1000,
    "800": 800,
    "500": 500,
    "250": 250,
    "125": 125
}

SyncModelType = {
    "FreeRun": 0,
    "SM": 1,
    "DC": 2,
    "Default": 255,
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
    "1200": 1200
}

ModeOfOperation = {
    "advance_mode": "Advanced mode",
    "simple_mode": "Lite mode"
}

SwitchMap = {
    "Off": 0,
    "On": 1
}

BitDefine = {
    "Bit15": "undefined！",
    "Bit14": "undefined！",
    "Bit13": "Defined by schema！",
    "Bit12": "Defined by schema！",
    "Bit11": "If Bit11 is set to 1, it indicates a trigger position limit (software limit or hardware limit)!",
    "Bit10": "If Bit10 sets to 1, the target has been reached!",
    "Bit9": "undefined！",
    "Bit8": "undefined！",
    "Bit7": "If Bit7 is set to 1, it indicates a warning!",
    "Bit6": "If Bit6 is set to 1, then the servo is not connected to PWM!",
    "Bit5": "If Bit5 is set to 0, it indicates an emergency stop!",
    "Bit4": "If Bit4 is set to 1, the drive is powered on!",
    "Bit3": "If Bit3 is set to 1, it is in an error state!",
    "Bit2": "If Bit2 is set to 1, the servo is running!",
    "Bit1": "If Bit1 is set to 1, then the servo is connected to PWM!",
    "Bit0": "If Bit0 is set to 0, the servo is not ready!",
    "Bit-1": "undefined！",
}