# -*- coding:utf-8 -*-
"""
@Time: 2023/1/11 14:04
@Author: 谭国榕
@Email: tanguorong@mail.dige.com
@File: hid_comm.py
"""
import struct
from queue import Queue
from pywinusb import hid


class HidComm:

    def __init__(self):
        super(HidComm, self).__init__()
        self._device = None
        self._report = None
        self._rec_que = Queue(1)

    def connect(self, hid_device):
        """
        连接设备
        :param hid_device: hid设备
        :return:
        """
        if isinstance(hid_device, str):
            _filter = hid.HidDeviceFilter(vendor_id=0x483, product_id=0x5750)
            devices = _filter.get_devices()
            for device in devices:
                if device.serial_number == hid_device:
                    hid_device = device
                    break
        self._device = hid_device
        self._device.open(shared=False)
        self._report = self._device.find_output_reports()
        self._device.set_raw_data_handler(self._read_data_handler)

    def write(self, data: list, read_flag=True):
        """
        写数据
        @param data:
        @param read_flag:
        @return
        """
        remind = 64 - len(data)
        if remind:
            data += [0]*remind
        data.insert(0, 0)
        self._report[0].set_raw_data(data)
        self._report[0].send()
        return self.read() if read_flag else True

    def read(self):
        """
        读数据
        :return:
        """
        data = self._rec_que.get(timeout=10)
        return data

    def disconnect(self):
        """
        断开连接
        :return:
        """
        if self._device:
            self._device.close()
        pass

    def is_opened(self):
        return self._device.is_opened()

    def _read_data_handler(self, data):
        """
        读取数据处理
        :param data:
        :return:
        """
        data = bytes(data[1:])
        self._rec_que.put(data)
        pass

    def update_device_list(self):
        """
        更新设备列表
        @return:
        """
        _filter = hid.HidDeviceFilter(vendor_id=0x483, product_id=0x5750)
        hid_device = _filter.get_devices()
        port_list = [device.serial_number for device in hid_device]
        return port_list

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
        raw_data += b'\x00' * 56
        raw_data = list(raw_data)
        if read_flag:
            re_data = self.write(raw_data)[4:8]
            re_data = struct.unpack(od.unpack_data_type, re_data)[0]
        else:
            self.write(raw_data, read_flag)
            re_data = True
        return re_data

    def write_od_data(self, od, value: int, read_flag=True) -> bool:
        """
        写字典数据
        @param od: 字典
        @param read_flag: 读取标志位
        @param value: 值
        @return: bool
        """
        return self.__rw_data(1, od, value, read_flag)

    def read_od_data(self, od) -> int:
        """
        读取字典数据
        @param od: 字典
        @return: 值(int)
        """
        return self.__rw_data(0, od, 0)

    def device_id(self):
        """
        获取设备ID
        @return:
        """
        if self._device:
            return self._device.serial_number
        else:
            return None

    def clear(self):
        while not self._rec_que.empty():
            self._rec_que.get()

