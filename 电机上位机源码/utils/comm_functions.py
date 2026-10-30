# -*- coding:utf-8 -*-
"""
@Time: 2023/2/17 17:59
@Author: 谭国榕
@Email: guorong.tan@MotionG.com
@File: comm_functions.py
"""
import csv
import json
import zipfile


def crc8(data_list: list):
    """
    CRC8校验
    :param data_list: 需要校验的数据
    :return:
    """
    length = len(data_list)
    crc = 0
    for k in range(length):
        data = data_list[k]
        crc ^= data
        for i in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0xB8
            else:
                crc = crc >> 1
    return crc


def get_bit_val(byte, index):
    """
    得到某个字节中某一位（Bit）的值

    :param byte: 待取值的字节值
    :param index: 待读取位的序号，从右向左0开始，0-7为一个完整字节的8个位
    :returns: 返回读取该位的值，0或1
    """
    if byte & (1 << index):
        return 1
    else:
        return 0


def set_bit_val(byte, index, val):
    """
    更改某个字节中某一位（Bit）的值

    :param byte: 准备更改的字节原值
    :param index: 待更改位的序号，从右向左0开始，0-7为一个完整字节的8个位
    :param val: 目标位预更改的值，0或1
    :returns: 返回更改后字节的值
    """
    if val:
        return byte | (1 << index)
    else:
        return byte & ~(1 << index)


def judge_digit(data: str):
    """
    判断是否数值
    :param data:
    :return:
    """
    if '-' in data and data.count('-') == 1:
        left, right = data.split('-')
        if left != '':
            return
        else:
            data = right
    if '.' in data and data.count('.') == 1:
        left, right = data.split('.')
        return True if left.isdigit() and right.isdigit() else False
    else:
        return True if data.isdigit() else False


def load_csv(file_path):
    with open(file_path, "r", encoding="utf-8-sig") as csv_file:
        reader = csv.reader(csv_file)
        rows = [row for row in reader]
    return True, rows


#  鉴别中文
def check_chinese(text_str, is_all=False):
    if is_all:
        if '\u4e00' <= text_str <= '\u9fff':
            return True
        else:
            return False
    else:
        has_chinese = False
        for _str in text_str:
            if '\u4e00' <= _str <= '\u9fff':
                has_chinese = True
                break
        return has_chinese


def verify_crc(verify, data_dict, version):
    """
    校验dgc文件的CRC
    @param verify:
    @param data_dict:
    @param version:
    @return:
    """
    verify_dict = json.loads(verify.decode('utf-8'))
    verify_result = True
    if len(verify_dict) != len(data_dict) + 1:
        return False
    for key, value in data_dict.items():
        if verify_dict[str(key)] != crc8(list(value)):
            verify_result = False
            break
    if verify_dict["version"] != crc8(list(version)):
        verify_result = False

    return verify_result


def load_dgc_file(file_path):
    """
    加载配置文件
    @return:
    """
    if not file_path:
        info = "未选择配置文件!"
        return False, info

    data_dict = {}
    version = b''
    verify = b''

    with zipfile.ZipFile(file_path, 'r') as zip_file:
        for file in zip_file.namelist():
            if file == 'version{}'.format(".dige"):
                version = zip_file.read(file)
            elif file == 'verify{}'.format(".dige"):
                verify = zip_file.read(file)
            else:
                data_dict.update({int(file[:file.find(".")]): zip_file.read(file)})

    if not version:
        # 配置文件缺失
        info = "导入的配置文件缺失版本信息!"
        return False, info

    if verify:
        ver_res = verify_crc(verify, data_dict, version)
        if not ver_res:
            # 配置文件校验错误
            info = "导入的配置文件校验失败！"
            return False, info

    return data_dict, "读取文件成功"


def cal_version_data(data: int) -> int:
    """
    计算软件版本号对比
    @param data: 软件版本号原始值
    @return: 版本号数值  example: 6117
    """
    try:
        if not data:
            raise Exception("data is empty")
        first = data >> 24
        second = (data >> 16) & 0xFF
        fourth = data & 0xFF
        if fourth < 10:
            fourth = "0" + str(fourth)
        version_data = int("{}{}{}".format(str(first), str(second), str(fourth)))
    except Exception as e:
        print("cal version error", str(e))
        version_data = 6110
    finally:
        return version_data


