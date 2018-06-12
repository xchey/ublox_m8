"""
# Author: Chey
# -*- coding: utf-8 -*-
# @Time     : 6/12/2018, 9:31 PM
# @Author   : Chey
# @Email    : cheyu856@gmail.com
# @File     : read_raw.py
# @Software : PyCharm
"""
import sys
import serial
from struct import pack, unpack, unpack_from, calcsize


# protocol constants
PREAMBLE1 = 0xb5
PREAMBLE2 = 0x62

# ublox class ID
CLS_NAV = 0x01
CLS_RXM = 0x02
CLS_CFG = 0x06


# ublox messag ID
MSG_RAWX    = 0x15
MSG_POSECEF = 0x01
MSG_VELECEF = 0x11
MSG_TIMEGPS = 0x20

class StructField:
    '''
    Descriptor representing a simple structure field
    '''
    def __init__(self, format, offset):
        self.format = format
        self.offset = offset

    def __get__(self, instance, cls):
        if instance is None:
            return self
        else:
            r = unpack_from(self.format, instance._buffer, self.offset)

class decode_meta(type):
    '''
    Metaclass that automatically creates StructField descriptors
    '''
    def __init__(self, clsname, bases, clsdict):
        super().__init__(self)
        fields = getattr(self, '_fields_', [])

        byte_order = ''
        offset = 0
        for format, fieldname in fields:
            if format.startswith(('<', '>', '!', '@')):
                byte_order = format[0]

        format = format[1:]
        format = byte_order + format

        setattr(self, fieldname, StructField(format, offset))
        offset += calcsize(format)
        setattr(self, 'struct_size', offset)

class decode_msg(metaclass=decode_meta):
    def __init__(self, bytedata):
        self._buffer = bytedata

    @classmethod
    def from_file(cls, f):
        return cls(f.read(cls.struct_size))


class ubx_desp:
    '''
    class to describe the structure of ublox message
    '''
    def __init__(self, name, msg_format, fields=None, count_field=None, format2=None, fields2=None):
        self.name = name
        self.msg_format = msg_format
        if fields is None:
            self.fields = []
        else:
            self.fields = fields
        self.count_field = count_field
        self.format2 = format2
        self.fields2 = fields2

    # def decode(self, msg):
    #     format =

# message types
msg_type = {
    # (CLS_RXM, MSG_RAWX) : ubx_desp('RXM_RAWX',
    #                                    ['']),

    (CLS_NAV, MSG_POSECEF): ubx_desp('NAV_POSECEF',
                                     '<LlllL',
                                     ['iTOW', 'ecefX', 'ecefY', 'ecefZ', 'pAcc']),

    (CLS_NAV, MSG_VELECEF): ubx_desp('NAV_VELECEF',
                                     '<LlllL',
                                     ['iTOW', 'ecefVX', 'ecefVY', 'ecefVZ', 'sAcc']),

    (CLS_NAV, MSG_TIMEGPS): ubx_desp('NAV_TIMEGPS',
                                     '<LlhbBL',
                                     ['iTOW', 'fTOW', 'week', 'leapS', 'valid', 'tAcc'])
}

class ublox:
    def __init__(self, port, baudrate=115200, timeout=0, bytesize=8, stopsize=1,
                 parity='N'):
        self.port = port
        self.baud = baudrate
        self.tmout = timeout
        self.bysz = bytesize
        self.spsz = stopsize
        self.parity = parity

        if self.port.startswith('com'):
            self.ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=self.tmout,
                                bytesize=self.bysz, stopbits=self.spsz, parity=self.parity)
            self.ser.close()
            try:
                self.ser.open()
            except serial.SerialException as e:
                sys.stderr.write('Could not open serial port {}:{}\n'.format(self.port, e))
                sys.exit(1)

    def read_raw(self, n):
        '''
        read n bytes from serial port
        :param n:
        :return:
        '''
        return self.ser.read(n)

    def decode_prx(self):
        '''
        read serial message preamble
        :return: class id, message id, message length
        '''
        prex = self.ser.read(6)
        if prex[0] == PREAMBLE1 and prex[1] == PREAMBLE2:
            return prex[2], prex[3], unpack('<H', prex[4:])
        else:
            return None

    def decode_raw(self):
        '''
        decode ublox binary message
        :return: decoded data
        '''
        cls_id, msg_id, dlen = self.decode_prx()
        print(cls_id, msg_id, dlen)





if __name__ == '__main__':
    dev = ublox('com23', 115200)
    while True:
        dev.decode_raw()
