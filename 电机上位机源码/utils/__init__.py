# -*- coding:utf-8 -*-
"""
@Time: 2023/2/10 10:14
@Author: 谭国榕
@Email: guorong.tan@MotionG.com
@File: __init__.py.py
"""
import json
from typing import Optional, TYPE_CHECKING

from .settings import *
from .path import *
from .comm_functions import *

if TYPE_CHECKING:
    import MotorHost
    from PyQt5.QtWidgets import QApplication

_application = Optional["QApplication"]
_root_dir = None
_main_window: Optional["MotorHost.MainWindow"] = None
_offline_service: Optional["MotorHost.offline_data_service"] = None
translate: Optional["MotorHost.translate"] = None
_write_service: Optional["MotorHost.ui.service"] = None
_work_thread: Optional["MotorHost.ui.service"] = None
_read_service = None


def get_application() -> "QApplication":
    """
    获取QApplication
    @return:
    """
    assert _application is not None
    return _application


def get_main_window() -> "MotorHost.MainWindow":
    """
    获取主窗口或者主控件。
    Returns:
    """
    assert _main_window is not None
    return _main_window


def get_offline_data_service() -> "MotorHost.offline_data_service":
    assert _offline_service is not None
    return _offline_service


def get_write_service() -> "MotorHost.ui.service":
    assert _write_service is not None
    return _write_service


def get_read_service():
    assert _read_service is not None
    return _read_service


def get_work_thread() -> "MotorHost.ui.service":
    assert _work_thread is not None
    return _work_thread


def get_work_dir() -> 'str':
    """
    获取工作目录
    Returns:
    """
    return get_settings_item_from_file("config.ini", "MAIN/PATH_WORKDIR")


def load_json_file(file_path):
    with open(file_path, "r", encoding="utf-8") as json_file:
        data_dict = json.load(json_file)
    return data_dict


# def open_url(url):
#     """
#     打开网址
#     """
#     try:
#         webbrowser.get('chrome').open_new_tab(url)
#     except Exception as e:
#         webbrowser.open_new_tab(url)
