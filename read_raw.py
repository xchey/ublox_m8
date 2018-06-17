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
from struct import unpack, unpack_from, calcsize


# protocol constants
PREAMBLE1 = 0xb5
PREAMBLE2 = 0x62

# ublox class ID
CLS_NAV = 0x01
CLS_RXM = 0x02
CLS_CFG = 0x06


# ublox messag ID
MSG_RAWX    = 0x15
MSG_SFRBX   = 0x13
MSG_POSECEF = 0x01
MSG_VELECEF = 0x11
MSG_TIMEGPS = 0x20

class UbxStructField:
    '''
    Descriptor representing a simple struct field
    '''
    def __init__(self, format, offset):
        self.format = format
        self.offset = offset

    def __get__(self, instance, cls):
        if instance is None:
            return self
        else:
            r = unpack_from(self.format, instance._buffer, self.offset)
            return r[0] if len(r) == 1 else r

class UbxStructureMeta(type):
    '''
    Meta class that automatically creates StructureField descriptor
    '''
    def __init__(self, clsname, bases, clsdict):
        fields = getattr(self, '_fields_', [])
        byte_order = ''
        offset = 0
        for format, fieldname in fields:
            if format.startswith(('<', '>', '!', '@')):
                byte_order = format[0]
                format = format[1:]
            format = byte_order + format
            setattr(self, fieldname, UbxStructField(format, offset))
            offset += calcsize(format)
        setattr(self, 'struct_size', offset)

#
class Structure(metaclass=UbxStructureMeta):
    '''
    Structure to read and decode ublox data
    '''
    def __init__(self, bytedata):
        self._buffer = memoryview(bytedata)

    @classmethod
    def from_file(cls, f):
        return cls(f.read(cls.struct_size))

    @classmethod
    def from_ser(cls, ser):
        return cls(ser.read(cls.struct_size))

    @classmethod
    def from_buff(cls, buf):
        return cls(buf)


class UBloxError(Exception):
    '''Ublox error class'''
    def __init__(self, msg):
        Exception.__init__(self, msg)
        self.message = msg


class UbloxDesp():
    '''
    Class to describe the structure of ublox message
    '''
    def __init__(self, data, count, *args):

        self._data = data
        self.name = args[0][0]

        fmt = []; self._fields_ = []
        for i in range(1, len(args[0][1])):
            fmt.append(args[0][1][i])
        fmt[0] = '<' + fmt[0]

        self.msg_format = fmt
        if args[0][2] is None:
            self.fields = []
        else:
            self.fields = args[0][2]

        # generate fields member
        for flt, fld in zip(fmt, self.fields):
            self._fields_.append((flt, fld))

        # for multi measurements
        if len(args[0]) > 3:
            self.count_field = count
            self.format2 = args[0][4]
            self.fields2 = args[0][5]

            fd = []; fmt = []; cnt = 0
            while cnt < self.count_field:
                for i in range(1, len(self.format2)):
                    fmt.append(self.format2[i])
                    fd.append(self.fields2[i-1]+str(cnt))
                cnt += 1

            for flt, fld in zip(fmt, fd):
                self._fields_.append((flt, fld))
        else:
            self.count_field = None
            self.format2 = None
            self.fields2 = None

    def decode(self):
        '''
        unpack a UbloxMessage with specified fields
        TBD
        '''
        class DecHed(Structure):
            _fields_ = self._fields_
        phead = DecHed.from_buff(self._data)
        return self.name, phead




# message types
dmsg_type = {
    (CLS_RXM, MSG_RAWX) :   ('RXM_RAWX',
                             '<dHbBbbbb',
                             ['rcvTow', 'week', 'leapS', 'numMeas', 'recStat', 'resv11', 'resv12', 'resv13'],
                             'numMeas',
                             '<ddfBBBBHBbbbbB',
                             ['prMes', 'cpMes', 'doMes', 'gnssId', 'svId', 'resv2', 'freqId',
                              'locktime', 'cno', 'prStdev', 'cpStdev', 'doStdev', 'trkStat',
                              'resv3']),

    (CLS_RXM, MSG_SFRBX) : ('RXM_SFRBX',
                            '<BBBBBBBB',
                            ['gnssId', 'svId', 'resv1', 'freqId', 'numWords', 'resv2', 'ver', 'resv3'],
                            'numWords',
                            '<L',
                            ['dwrd']),

    (CLS_NAV, MSG_POSECEF): ('NAV_POSECEF',
                             '<LlllL',
                             ['iTOW', 'ecefX', 'ecefY', 'ecefZ', 'pAcc']),

    (CLS_NAV, MSG_VELECEF): ('NAV_VELECEF',
                             '<LlllL',
                             ['iTOW', 'ecefVX', 'ecefVY', 'ecefVZ', 'sAcc']),

    (CLS_NAV, MSG_TIMEGPS): ('NAV_TIMEGPS',
                             '<LlhbBL',
                             ['iTOW', 'fTOW', 'week', 'leapS', 'valid', 'tAcc'])
}

class Ublox:
    '''
    Ublox class, init with serial port object
    '''
    def __init__(self, port, baudrate=9600, timeout=0, bytesize=8, stopsize=1,
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

    def read_raw(self, rdline=False):
        '''
        read n bytes from serial port
        '''
        raw_data = b''; data = b''
        try:
            chunks = iter(lambda : self.ser.read(), b'\xb5')
            for chk in chunks:
                raw_data += chk
        except serial.SerialException as e:
            sys.stderr.write('Could not read from serial port {}:{}\n'.format(self.port, e))

        raw_len = len(raw_data)

        if raw_data == b'' :
            data = b''
        elif raw_data[0] == PREAMBLE2:
            # raw data field without sync char 1, add b'\xb5'
            # while in concat bytes.
            # (sync char 2, class, id, length, ck_a, ck_b) = 7
            # rxm_rawx still need (msg_len - raw_len + 7)
            msg_len, = unpack('<H', raw_data[3:5])
            data = raw_data if (msg_len == raw_len-7) else (raw_data + b'\xb5' + self.ser.read(msg_len-raw_len+6))
        else:
            print(raw_data[0], ' Something wrong with serial read')
        return data

    def decode_raw(self):
        '''
        decode ublox binary message
        '''
        data = self.read_raw()
        if len(data) > 1:
            cls_id, msg_id, dlen = unpack('<BBH', data[1:5])

            print(cls_id, msg_id, dlen)
            if self.check_sum(data):
                print('Check sum passed!')

                # decode binary data
                cnt = 0
                if msg_id == MSG_RAWX:
                    cnt, = unpack('<B', data[16:17])
                elif msg_id == MSG_SFRBX:
                    cnt, = unpack('<B', data[9:10])
                dec = UbloxDesp(data[5:-2], cnt, dmsg_type[(cls_id, msg_id)])
                name, fld = dec.decode()

                if msg_id == MSG_TIMEGPS:
                    print(name, fld.iTOW, fld.fTOW, fld.week, fld.leapS, bin(fld.valid), fld.tAcc)
                elif msg_id == MSG_VELECEF:
                    print(name, fld.iTOW, fld.ecefVX, fld.ecefVY, fld.ecefVZ, fld.sAcc)
                elif msg_id == MSG_POSECEF:
                    print(name, fld.iTOW, fld.ecefX, fld.ecefY, fld.ecefZ, fld.pAcc)
                elif msg_id == MSG_RAWX:
                    print(name, fld.leapS, fld.numMeas, fld.recStat)
                elif msg_id == MSG_SFRBX:
                    print(name, fld.gnssId, fld.svId, fld.numWords)
            else:
                print('Check sum failure!')
                return False
        else:
            return False

    # def

    def check_sum(self, dat):
        '''
        ublox check sum calculation
        '''
        cck_a = 0; cck_b = 0
        for i in dat[1:-2]:
            cck_a = (cck_a + i) & 0xFF
            cck_b = (cck_b + cck_a) & 0xFF

        ck_a, ck_b = unpack('<BB', dat[-2:])
        if ck_a == cck_a and ck_b == cck_b:
            return True
        else:
            return False

if __name__ == '__main__':
    dev = Ublox('com23', 115200)
    while True:
        dev.decode_raw()
    # ser = serial.Serial('com12', 115200)
    # ser.close()
    # ser.open()
    # while True:
    #     chunks = iter(lambda : ser.read(2), pack('<BB', PREAMBLE1, PREAMBLE2))
    #     b = b''
    #     for chk in chunks:
    #         b += chk
    #     print(b)
    #     b = b''


