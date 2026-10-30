# -*- coding:utf-8 -*-
"""
@Time: 2023/1/11 14:04
@Author: 谭国榕
@Email: tanguorong@mail.dige.com
@File: serial_comm.py
"""
import struct
import time

import serial

import utils


class SerialComm:

    def __init__(self):
        super(SerialComm, self).__init__()
        self._device = None

    def connect(self, port: str):
        """
        通讯连接
        :param port:
        :return:
        """
        self._device = serial.Serial()
        self._device.port = port
        self._device.baudrate = 921600
        self._device.parity = "N"
        self._device.bytesize = 8
        self._device.stopbits = 1
        self._device.timeout = 3
        self._device.open()

    def write(self, data: list, read_flag=True):
        """
        写数据
        @param data:
        @param read_flag:
        @return
        """
        time.sleep(0.001)
        data += [0] * (64 - len(data))
        crc = utils.crc8(data)
        data.append(crc)
        self._device.write(data)
        return self.read() if read_flag else True

    def write_no_crc(self, data: list, read_flag=True):
        data += [0] * (64 - len(data))
        self._device.write(data)
        return self._device.read_all() if read_flag else True

    def read(self):
        """
        读数据
        :return:
        """
        rec = None
        retry_count = 0
        while not rec:
            rec = self._device.read(65)[:8]
            retry_count += 1
            if retry_count == 50:
                break
            time.sleep(0.001)
        return rec

    def disconnect(self):
        """
        断开连接
        :return:
        """
        if self._device:
            self._device.close()

    def is_opened(self):
        return self._device.isOpen()

    def update_device_list(self):
        """
        更新设备列表
        @return:
        """
        pass

    def __rw_data(self, rw, od, value, read_flag=True):
        """
        读写写数据
        @param rw:
        @param od:
        @return:
        """
        if rw == 0:
            _header = 0x40
            value = 0
        else:
            _header = 0x23

        od_type = od.unpack_data_type.replace("<", "").replace("4", "").replace("2", "")
        raw_data = struct.pack("<BHB{}".format(od_type), _header, od.index, od.sub_index, value)
        raw_data += b'\x00' * (64 - len(raw_data))
        raw_data = list(raw_data)
        if read_flag:
            re_data = self.write(raw_data)[4:8]
            re_data = struct.unpack(od.unpack_data_type, re_data)[0]
        else:
            self.write(raw_data, read_flag)
            re_data = True
        return re_data

    def write_od_data(self, od, value: int, read_flag=True):
        """
        写字典数据
        @param od:
        @param value:
        @param read_flag:
        @return:
        """
        return self.__rw_data(1, od, value, read_flag)

    def read_od_data(self, od):
        """
        读取数据字典
        @param od:
        @return:
        """
        return self.__rw_data(0, od, 0)

    def device_id(self):
        """
        获取设备ID
        @return:
        """
        if self._device:
            return self._device.port
        else:
            return None

    def clear(self):
        self._device.read_all()
