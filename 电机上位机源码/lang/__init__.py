#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time        : 2023/3/30 11:33
# @Author      : WeiZujun@Motiong.com
# @Description :
# @File        : __init__.py
# @Software    : PyCharm
import configparser
import os
current_path = os.path.abspath(os.path.dirname(__file__))
root_path = os.path.abspath(os.path.join(current_path, "../"))
ini_path = root_path + "/software_info.ini"

try:
    if not os.access(ini_path, os.F_OK) or not os.access(ini_path, os.R_OK):
        raise Exception("software_info.ini is not found!")
    config_obj = configparser.ConfigParser()
    config_obj.read(ini_path, encoding="utf-8")
    if not config_obj.has_section("config"):
        raise Exception("software_info.ini has not selection config!")
    else:
        language = config_obj.get("config", "language", fallback="zh_cn").upper()
except Exception as e:
    print(str(e))
    language = "ZH_CN"

if language == "ZH_CN":
    from lang.zh_cn.base import *
    from lang.zh_cn.CustomString import *
    from lang.zh_cn import base, CustomString
    curve_path = "/zh_cn/curve_classification.csv"
elif language == "EN_US":
    from lang.en_us.base import *
    from lang.en_us.CustomString import *
    from lang.en_us import base, CustomString
    curve_path = "/en_us/curve_classification.csv"
