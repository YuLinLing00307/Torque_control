# -*- coding:utf-8 -*-
"""
@Time: 2020/9/20 11:45
@Author: 谭国榕
@Email: tanguorong@mail.dige.com
@File: CustomString.py
"""
from ui.config import translate

BaseControlParam = translate("Form", "基本控制参数")
ForceControlParam = translate("Form", "力控参数")
OtherControlParam = translate("Form", "其他控制参数")
PwmControlParam = translate("Form", "PWM控制参数")
McsControlParam = translate("Form", "模拟量控制参数")
PulseControlParam = translate("Form", "脉冲控制参数")

CurrentProtectLimit = translate("Form", "电流保护限制")
VoltageProtectLimit = translate("Form", "电压保护限制")
PositionProtectLimit = translate("Form", "位置保护限制")
TemperatureProtectLimit = translate("Form", "温度保护限制")
OffsetProtectLimit = translate("Form", "偏差保护限制")
MaxAccDecProtectLimit = translate("Form", "最大加减速度限制")
BreakSettingLimit = translate("Form", "抱闸保护限制")
ForceSensorParam = translate("Form", "力传感器设置")
DriverProtectionLimit = translate("Form", "驱动器保护限制")

# 传感器
EncoderParam = translate("Form", "内环编码器设置")
OutLoopEncoderParam = translate("Form", "外环编码器设置")
HallParam = translate("Form", "霍尔传感器设置")
ExternalForceParam = translate("Form", "外部力传感器设置")

# 离线波形
OfflineTitle = translate("Form", "离线数据采集")
OfflineComplete = translate("Form", "数据采集完成！")
OfflineTimeOut = translate("Form", "数据采集超时！")
OfflineNoFFTData = translate("Form", "缺少待转换的FFT数据，请先采集或导入！")
OfflineNoSampleData = translate("Form", "请先设置需要采集的数据！")
Offline_Tooltip_start = translate("Form", "开始采集")
Offline_Tooltip_imm = translate("Form", "立即采集")
Offline_Tooltip_stop = translate("Form", "停止采集")
Offline_Tooltip_pause = translate("Form", "暂停采集")
Offline_Tooltip_reset = translate("Form", "复位")
Offline_Tooltip_clear = translate("Form", "清除")

# 设备保护限制
DeviceCurrentProtocol = translate("Form", "驱动器电流保护限制")
DeviceVoltageProtocol = translate("Form", "驱动器电压保护限制")
DeviceTemperatureProtocol = translate("Form", "驱动器温度保护限制")

# 位置保护限制
PositionErrorSetting = translate("Form", "位置误差设置")
DevicePositionProtocol = translate("Form", "硬件位置保护限制")
ProgramPositionProtocol = translate("Form", "软件位置保护限制")

# 速度保护限制
VelocityErrorSetting = translate("Form", "速度误差设置")
MaxDeviceVelocityProtocol = translate("Form", "最大电机转速保护限制")
MaxAcDcVelocityProtocol = translate("Form", "最大加减速度限制")

# 电流保护限制
CurrentProtocolSetting = translate("Form", "电流保护限制")


FileLoad = translate("Form", "导入配置到驱动器")
FileSave = translate("Form", "从驱动器导出配置")
FileReload = translate("Form", "从驱动器重载配置")

