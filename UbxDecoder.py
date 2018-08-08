"""
# Author: Chey
# -*- coding: utf-8 -*-
# @Time     : 6/12/2018, 9:31 PM
# @Author   : Chey
# @Email    : cheyu856@gmail.com
# @File     : UbxDecoder.py
# @Software : PyCharm
"""
import sys
import serial
import datetime
import argparse
import subprocess
import UbxRaw2Rinex as u2r
from struct import Struct, unpack, unpack_from, calcsize


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
CMD = "./cmd/FmiConv.exe -r ubx "

class NestedStruct:
    '''
    Descriptor representing a nested structure
    '''
    def __init__(self, name, struct_type, offset):
        self.name = name
        self.struct_type = struct_type
        self.offset = offset

    def __get__(self, instance, cls):
        if instance is None:
            return self
        else:
            data = instance._buffer[self.offset:
                   self.offset+self.struct_type.struct_size]
            result = self.struct_type(data)
            # Save resulting structure back on instance to avoid
            # further recomputation of this step
            setattr(instance, self.name, result)
            return result

class StructField:
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
            if isinstance(format, UbxStructureMeta):
                setattr(self, fieldname,
                        NestedStruct(fieldname, format, offset))
                offset += format.struct_size
            else:
                if format.startswith(('<', '>', '!', '@')):
                    byte_order = format[0]
                    format = format[1:]
                format = byte_order + format
                setattr(self, fieldname, StructField(format, offset))
                offset += calcsize(format)
        setattr(self, 'struct_size', offset)

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
    def from_serial(cls, ser):
        return cls(ser.read(cls.struct_size))

    @classmethod
    def from_buff(cls, buf):
        return cls(buf)

class Meas(Structure):
    '''
    Ublox rxm_rawx Measurements NumMeas fields
    '''
    _fields_ = [
        ('<d', 'prMes'), ('d', 'cpMes'), ('f', 'doMes'), ('B', 'gnssId'),
        ('B', 'svId'), ('B', 'resv2'), ('B', 'freqId'), ('H', 'locktime'),
        ('B', 'cno'), ('b', 'prStdev'), ('b', 'cpStdev'), ('b', 'doStdev'),
        ('b', 'trkStat'), ('B', 'resv3')
    ]

class Ephe(Structure):
    '''
    Ublox rxm_sfrbx Ephemeris NumWords fields
    '''
    _fields_ = [
        ('<L', 'dwrd')
    ]


class SizeRecords:
    def __init__(self, bytesize):
        self._buffer = memoryview(bytesize)

    @classmethod
    def form_file(cls, f, size_fmt, include_size=True):
        sz_nbytes = calcsize(size_fmt)
        sz_bytes = f.read(sz_nbytes)
        sz, = unpack(size_fmt, sz_bytes)
        buf = f.read(sz - include_size * sz_nbytes)
        return cls(buf)

    def iter_as(self, code):
        if isinstance(code, str):
            s = Struct(code)
            for off in range(0, len(self._buffer), s.size):
                yield s.unpack_from(self._buffer, off)
        elif isinstance(code, UbxStructureMeta):
            size = code.struct_size
            for off in range(0, len(self._buffer), size):
                data = self._buffer[off:off+size]
                yield code(data)

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

        # extract format from structure list into fmt
        fmt = []; self._fields_ = []
        for i in range(1, len(args[0][1])):
            fmt.append(args[0][1][i])
        fmt[0] = '<' + fmt[0]

        # args[0][2] is the structure fields with respect to fmt
        self.msg_format = fmt
        if args[0][2] is None:
            self.fields = []
        else:
            self.fields = args[0][2]

        # generate fields list member
        for flt, fld in zip(fmt, self.fields):
            self._fields_.append((flt, fld))

        # for multi measurements
        if len(args[0]) > 3:
            self.count_field = count
            type_name = args[0][3]

            cnt = 0
            if type_name == 'numWords':
                while cnt < self.count_field:
                    self._fields_.append((Ephe, 'words'+str(cnt)))
                    cnt += 1
            elif type_name == 'numMeas':
                while cnt < self.count_field:
                    self._fields_.append((Meas, 'meas'+str(cnt)))
                    cnt += 1
        else:
            self.count_field = None
            self.format2 = None
            self.fields2 = None

    def decode(self):
        '''
        unpack a UbloxMessage with specified fields
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
                             'numMeas'),

    (CLS_RXM, MSG_SFRBX) : ('RXM_SFRBX',
                            '<BBBBBBBB',
                            ['gnssId', 'svId', 'resv1', 'freqId', 'numWords', 'resv2', 'ver', 'resv3'],
                            'numWords'),

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

    def close(self):
        self.ser.close()

    def readbytes(self):
        return self.ser.read(1024)


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
        if raw_data == b'':
            data = b''
        elif raw_data[0] == PREAMBLE2:
            # raw data field without sync char 1, add b'\xb5'
            # while in concat bytes.
            # (sync char 2, class, id, length, ck_a, ck_b) = 7
            # rxm_rawx still need (msg_len - raw_len + 7)
            msg_len, = unpack('<H', raw_data[3:5])
            data = raw_data if (msg_len == raw_len-7) else (raw_data + b'\xb5' + self.ser.read(msg_len-raw_len+6))
        else:
            sys.stderr.write('Head {}, Something wrong with serial read\n'.format(raw_data[0]))
        return data

    def decode_raw(self):
        '''
        decode ublox binary message
        '''
        data = self.read_raw()
        if len(data) > 1:
            cls_id, msg_id, dlen = unpack('<BBH', data[1:5])

            print(f'message cls id {cls_id}, msg id {msg_id}, data length {dlen}')
            if self.check_sum(data):
                # decode binary data
                cnt = 0
                if msg_id == MSG_RAWX:
                    cnt, = unpack('<B', data[16:17])
                elif msg_id == MSG_SFRBX:
                    cnt, = unpack('<B', data[9:10])
                dec = UbloxDesp(data[5:-2], cnt, dmsg_type[(cls_id, msg_id)])
                # self.show_obs(dec.decode())
                name, fld = dec.decode()
                return name, fld
            else:
                sys.stderr.write('Check sum failure!\n')
                return None
        else:
            sys.stderr.write('Data length error!\n')
            return None

    def show_obs(self, func):
        '''
        Show ublox observations data
        '''
        name, fld = func
        if name == MSG_TIMEGPS:
            if fld.valid & 0x3:
                week = fld.week
                sow = fld.iTOW*1e-3 + fld.fTOW*1e-9
                epoch = u2r.gpstime_to_epoch(week, sow)
                print(epoch.strftime("Current GPS Time: %Y %m %d %H %M %S.%f\n").
                      replace(' 0', ' '))
            else:
                print('Current Epoch not valid!')
        elif name == 'NAV_POSECEF':
            print('Current Position ECEF(m): Px:{}, Py:{}, Pz:{}, pAcc:{}\n'.format(
                fld.ecefX*1e-2, fld.ecefY*1e-2, fld.ecefZ*1e-2, fld.pAcc*1e-2))
        elif name == 'NAV_VELECEF':
            print('Current Velocity ECEF(m/s): Vx:{}, Vy:{}, Vz:{}, sAcc:{}\n'.format(
                fld.ecefVX*1e-2, fld.ecefVY*1e-2, fld.ecefVZ*1e-2, fld.sAcc*1e-2))
        elif name == 'RXM_RAWX':
            epoch = u2r.gpstime_to_epoch(fld.week, fld.rcvTow)
            print(epoch.strftime("Current Epoch Time: %Y %m %d %H %M %S.%f\n").
                  replace(' 0', ' '))
            for i in range(0, fld.numMeas):
                meas = fld.__getattribute__('meas'+str(i))
                print('{} {}\n'
                    'PrMes: {}, cpMes: {}, doMes: {}, CN0: {}, locktime: {}, trackStat: {}\n'.
                    format(u2r.map_gnssid[meas.gnssId], meas.svId, meas.prMes, meas.cpMes, meas.doMes,
                           meas.cno, meas.locktime, hex(meas.trkStat)))
        elif name == 'RXM_SFRBX':
            print('{} {}\n'.format(u2r.map_gnssid[fld.gnssId], fld.svId))
            print(list(fld.__getattribute__('words'+str(i-8)).dwrd for i in range(8, 8+fld.numWords)), fld.numWords)

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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port', type=str, default=None, help="Serial com port")

    parser.add_argument('--output', '-o', metavar='<output rinex3 file>', type=str, default=None, dest='rnx',
                        help="Output RINEX3 file. If not set (default), RINEX3 will be written to the standard output")
    parser.add_argument('--min', '-m', type=float, default=None, dest='min',
                        help="Capture ublox raw data time duration")
    #
    # parser.add_argument('--marker-name', '-m', metavar='<marker name>', type=str, default="UNKN",
    #                     help="Specify the marker name (station id)")
    # parser.add_argument('--observer', '-n', metavar='<observer name>', type=str, default="UNKN",
    #                     help="Specify the observer name or e-mail")
    # parser.add_argument('--agency', '-a', metavar='<agency name>', type=str, default="UNKN",
    #                     help="Specify the agency name")
    # parser.add_argument('--receiver-number', metavar='<str>', type=str, default="UNKN",
    #                     help="Specify the receiver number")
    # parser.add_argument('--receiver-type', metavar='<str>', type=str, default="UNKN", help="Specify the receiver type")
    # parser.add_argument('--receiver-version', metavar='<str>', type=str, default="TIM 3.01",
    #                     help="Specify the receiver version")
    # parser.add_argument('--antenna-number', metavar='<str>', type=str, default="UNKN",
    #                     help="Specify the antenna number")
    # parser.add_argument('--skip-edit', dest='skip_edit', action='store_true',
    #                     help="Skip pseudorange data edit that checks that the range is within bounds")
    # parser.add_argument('--antenna-type', metavar='<str>', type=str, default="internal",
    #                     help="Specify the receiver type")
    #
    args = parser.parse_args()
    port = args.port
    # Handler for the output RINEX
    # try:
    #     out = open(args.rnx, "w") if args.rnx is not None else sys.stdout
    # except:
    #     sys.stderr.write('Write file failed!')
    #     sys.exit(1)
    t = args.min
    fn = '.\data\\'+args.rnx+".bin"

    tmp = open(fn, 'wb')
    # out_raw = open('./data/test_raw.txt', 'w')
    # out_eph = open('./data/test_eph.txt', 'w')

    # out_raw.write(u2r.rinex3_header(marker=args.marker_name, observer=args.observer, agency=args.agency, runby="FMI Tech",
    #                                      receiver=args.receiver_number, rxtype=args.receiver_type, version=args.receiver_version,
    #                                      antenna=args.antenna_number, anttype=args.antenna_type))
    #
    # out_eph.write(u2r.rinex3_nav_header(runby="FMI Tech"))
    #
    if t >= 60:
        h = int(t/60)
        m = int(t-h*60)
        s = int((t-h*60-m)*60)
    else:
        h = 0
        m = int(t)
        s = int((t-m)*60)
    print(f'Record time {h}hr {m}min {s}sec\nrecording...')
    start = datetime.datetime.now()
    dlt = datetime.timedelta(hours=h, minutes=m, seconds=s)
    dev = Ublox(port, 115200)
    while datetime.datetime.now() < start + dlt:
        tmp.write(dev.readbytes())

    dev.close()
    tmp.close()

    subprocess.call(CMD + fn)
    # while True:
    #     try:
    #         name, fld = dev.decode_raw()
    #         if fld is not None:
    #             if name == 'RXM_RAWX':
    #                 u2r.to_obs(out_raw, fld)
    #             elif name == 'RXM_SFRBX':
    #                 u2r.to_eph(out_eph, fld)
    #     except TypeError:
    #         continue
    # out_raw.close()
    # out_eph.close()
