# -*- coding:utf-8 -*-
"""
@Time: 2023/4/6 9:40
@Author: 谭国榕
@Email: guorong.tan@MotionG.com
@File: device_control.py
"""
from data import model_data
from utils import get_bit_val, set_bit_val, get_work_thread


class StatusWord:
    NotReadyToSwitchOn = "Not ready to switch on"
    SwitchOnDisabled = "Switch on disabled"
    FaultReactionActive = "Fault reaction active"
    Fault = "Fault"
    ReadyToSwitchOn = "Ready to switch on"
    SwitchedOn = "Switched on"
    OperationEnabled = "Operation enabled"
    QuickStopActive = "Quick stop active"


class ControlWord:
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


class DisplayPlanner:
    Position_mode = 1
    Velocity_mode = 3
    Torque_mode = 4
    Analog_control_speed = 22
    Analog_control_torque = 23
    Pulse_control_position = 31
    PWM_control_speed = 42
    Homing_mode = 6
    Reference_Signal = 61
    Force_control_mode = 11
    Reciprocating_Motion = 60
    Multi_segment_Running_Mode = 62
    CSP = 8
    CSV = 9
    CST = 10
    Electrical_identification = -1
    Direction_identification = -2
    Hall_identification = -3
    Pole_pair_number_identification = -4
    Z_phase_identification = -5
    Electrical_angle_identification = -6
    Mechanical_identification = -7
    Torque_pulse_identification = -8
    Mechanical_Characteristic_Identification = -9
    Force_Control_Characteristic_Identification = -10
    Electrical_Identification_ = -11
    Inertia_calibration = -12
    Empty_mode = 0


def enable():
    """
    驱动器使能
    @return:
    """
    status_word = get_status_word()
    if status_word == StatusWord.SwitchOnDisabled:
        send_data({model_data.Controlword.od_name: ControlWord.SHUTDOWN})
    send_data({model_data.Controlword.od_name: ControlWord.ENABLE_OPERATION})


def disable():
    """
    驱动器失能
    @return:
    """
    status_word = get_status_word()
    if status_word == StatusWord.QuickStopActive:
        send_data({model_data.Controlword.od_name: ControlWord.DISABLE_VOLTAGE})
    else:
        send_data({model_data.Controlword.od_name: ControlWord.SHUTDOWN})


def halt():
    """
    驱动器暂停
    @return:
    """
    halt_bit = get_bit_val(model_data.Controlword.data, 8)
    status_word = get_status_word()
    if halt_bit == 0:
        if status_word != StatusWord.QuickStopActive:
            halt_val = set_bit_val(model_data.Controlword.data, 8, 1)
        else:
            halt_val = ControlWord.HALT_QUICK_STOP
        send_data({model_data.Controlword.od_name: halt_val})


def cancel_halt():
    """
    驱动器取消暂停
    @return:
    """
    halt_bit = get_bit_val(model_data.Controlword.data, 8)
    if halt_bit == 1:
        halt_val = set_bit_val(model_data.Controlword.data, 8, 0)
        send_data({model_data.Controlword.od_name: halt_val})


def start_operation():
    """
    驱动器启动
    @return:
    """
    if model_data.Modes_of_operation_display.data in [DisplayPlanner.Position_mode, DisplayPlanner.Reciprocating_Motion] and model_data.MotorShowState.data == 1:
        if model_data.Controlword.data == 0:
            send_data({model_data.Controlword.od_name: 0x0F & 0xFFEF})
        else:
            send_data({model_data.Controlword.od_name: model_data.Controlword.data & 0xFFEF})


def set_mode(mode: int):
    """
    设置驱动器的运行模式
    @param mode: 运行模式
    @return:
    """
    send_data({model_data.Modes_of_operation.od_name: mode})


def map_control_word(data):
    """
    状态字解析
    @param data: 状态字原始数值
    @return:
    """
    word_1 = data & 0x4F
    word_2 = data & 0x6F
    status = ""
    if word_1 in [0x00, 0x40, 0x0F, 0x08]:
        if word_1 == 0:
            status = "Not ready to switch on"
        elif word_1 == 0x40:
            status = "Switch on disabled"
        elif word_1 == 0x0F:
            status = "Fault reaction active"
        elif word_1 == 0x08:
            status = "Fault"
    elif word_2 in [0x21, 0x23, 0x27, 0x07]:
        if word_2 == 0x21:
            status = "Ready to switch on"
        elif word_2 == 0x23:
            status = "Switched on"
        elif word_2 == 0x27:
            status = "Operation enabled"
        elif word_2 == 0x07:
            status = "Quick stop active"
    return status


def get_encoder_resolution_at_load():
    """
    获取负载端编码器分辨率
    @return:
    """
    if model_data.OutLoopEncoderType.data != 255 and model_data.DualFBControlMode.data == 0:
        return model_data.OutLoopEncoderResolution.data
    else:
        return model_data.Position_encoder_resolution_Encoder_increments.data


def get_encoder_type():
    """
    检测编码器使用类型单编码器或双编码器
    @return:
    """
    if model_data.OutLoopEncoderType.data != 255 and model_data.DualFBControlMode.data == 0:
        return model_data.OutloopPositionActualValue.od_name
    else:
        return model_data.Position_actual_value.od_name


def get_status_word():
    """
    获取状态字对应的驱动器状态
    @return:
    """
    return map_control_word(model_data.Statusword.raw_data)


def send_data(data):
    """
    发送数据
    @param data:
    @return:
    """
    get_work_thread().send_data(data)
    pass



