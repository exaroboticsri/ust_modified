# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : Commons.py
# Project Name : MobileRobotTest
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2021.08.24 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import datetime
import os
import sys
import psutil
from typing import Union
import pkg_resources
import logging
from logging import handlers
import threading

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include
ROOT_PATH = os.path.dirname(FILE_PATH)
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
sys.path.extend(
    [INCLUDE_PATH, RESOURCES_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, RESOURCES_PATH

MAX_SERIAL_BUFFER = 1024  # 1kb
MAX_SIZE = 100 * 1024 * 1024
glob_logger = None


def check_argument_type(obj, arg):
    if type(obj) == arg:
        return True
    if obj is None:  # TODO:
        return True
    if arg == object:
        return True
    if arg in obj.__class__.__bases__:
        return True
    return False


def is_ros_installed() -> bool:
    ros_requirements = {'rclpy', 'geometry_msgs'}
    installed = {pkg.key for pkg in pkg_resources.working_set}
    missing = ros_requirements - installed

    if missing:
        return False
    else:
        return True


def is_system_win() -> bool:
    import platform
    if platform.system() == 'Windows':
        return True

    return False


def get_ip_addr_list():
    result = []
    netlst = psutil.net_if_addrs()
    for key in netlst.keys():
        if 'loopback' not in key.lower():
            adap = key
            mac, ipv4, ipv6 = '', '', ''
            sniclist = netlst[key]
            for snic in sniclist:
                family = int(snic[0])
                address = snic[1]
                if family == -1 or family == 18:
                    mac = address
                elif family == 2:
                    ipv4 = address
                elif family == 23 or family == 30:
                    ipv6 = address
            result.append({'name': adap, 'mac': mac, 'ipv4': ipv4, 'ipv6': ipv6})
    return result


class PySignal(object):
    _args = None
    _callback = None

    def __init__(self, *args):
        self._args = args

    def connect(self, callback):
        self._callback = callback

    def emit(self, *args):
        if len(args) != len(self._args):
            raise Exception('Callback::Argument Length Mismatch')
        arglen = len(args)
        if arglen > 0:
            validTypes = [check_argument_type(args[i], self._args[i]) for i in range(arglen)]
            if sum(validTypes) != arglen:
                raise Exception('Callback::Argument Type Mismatch (Definition: {}, Call: {}, Result: {})'.format(
                    self._args, args, validTypes))
        if self._callback is not None:
            self._callback(*args)


def convert_u32_to_bytearray_be(anVal: int) -> bytearray:
    if anVal > 4294967295:
        anVal = 4294967295

    if anVal < 0:
        anVal = 0

    buff = bytearray([])
    buff.append((anVal >> 24) & 0xFF)
    buff.append((anVal >> 16) & 0xFF)
    buff.append((anVal >> 8) & 0xFF)
    buff.append(anVal & 0xFF)
    return buff


def convert_s32_to_bytearray_be(anVal: int) -> bytearray:
    if anVal > 2147483647:
        anVal = 2147483647

    if anVal < -2147483648:
        anVal = -2147483648

    if anVal < 0:
        anVal = ~(anVal * -1) + 1  # 2's complement

    buff = bytearray([])
    buff.append((anVal >> 24) & 0xFF)
    buff.append((anVal >> 16) & 0xFF)
    buff.append((anVal >> 8) & 0xFF)
    buff.append(anVal & 0xFF)
    return buff


def convert_u16_to_bytearray_be(anVal: int) -> bytearray:
    if anVal > 65535:
        anVal = 65535

    if anVal < 0:
        anVal = 0

    buff = bytearray([])
    buff.append((anVal >> 8) & 0xFF)
    buff.append(anVal & 0xFF)
    return buff


def convert_s16_to_bytearray_be(anVal: int) -> bytearray:
    if anVal > 32767:
        anVal = 32767

    if anVal < -32768:
        anVal = -32768

    if anVal < 0:
        anVal = ~(anVal * -1) + 1  # 2's complement

    buff = bytearray([])
    buff.append((anVal >> 8) & 0xFF)
    buff.append(anVal & 0xFF)
    return buff


def convert_u32_to_bytearray_le(anVal: int) -> bytearray:
    if anVal > 4294967295:
        anVal = 4294967295

    if anVal < 0:
        anVal = 0

    buff = bytearray([])
    buff.append(anVal & 0xFF)
    buff.append((anVal >> 8) & 0xFF)
    buff.append((anVal >> 16) & 0xFF)
    buff.append((anVal >> 24) & 0xFF)
    return buff


def convert_s32_to_bytearray_le(anVal: int) -> bytearray:
    if anVal > 2147483647:
        anVal = 2147483647

    if anVal < -2147483648:
        anVal = -2147483648

    if anVal < 0:
        anVal = ~(anVal * -1) + 1  # 2's complement

    buff = bytearray([])
    buff.append(anVal & 0xFF)
    buff.append((anVal >> 8) & 0xFF)
    buff.append((anVal >> 16) & 0xFF)
    buff.append((anVal >> 24) & 0xFF)
    return buff


def convert_u16_to_bytearray_le(anVal: int) -> bytearray:
    if anVal > 65535:
        anVal = 65535

    if anVal < 0:
        anVal = 0

    buff = bytearray([])
    buff.append(anVal & 0xFF)
    buff.append((anVal >> 8) & 0xFF)
    return buff


def convert_s16_to_bytearray_le(anVal: int) -> bytearray:
    if anVal > 32767:
        anVal = 32767

    if anVal < -32768:
        anVal = -32768

    if anVal < 0:
        anVal = ~(anVal * -1) + 1  # 2's complement

    buff = bytearray([])
    buff.append(anVal & 0xFF)
    buff.append((anVal >> 8) & 0xFF)
    return buff


def convert_bytearray_to_int_be(abtaData: bytearray, abIsSigned=False) -> int:
    if isinstance(abtaData, int):
        return int(abtaData)

    num = 0
    for b in abtaData:
        num <<= 8
        num |= int(b) & 0XFF

    if abIsSigned:
        int_max = 2 ** (8 * len(abtaData))
        upper_limit = int((int_max / 2) - 1)

        if num > upper_limit:
            num = -(int_max - num)

    return num


def convert_bytearray_to_int_le(abtaData: bytearray, abIsSigned=False) -> int:
    if isinstance(abtaData, int):
        return int(abtaData)

    abtaData.reverse()
    return convert_bytearray_to_int_be(abtaData, abIsSigned)


def search_pattern(src, pattern) -> Union[None, int]:
    nRange = len(src) - len(pattern) + 1
    for i in range(nRange):
        if src[i] != pattern[0]:
            continue

        for j in range(len(pattern) - 1, 0, -1):
            if (src[i + j]) != pattern[j]:
                break

            if j == 1:
                return i

    return None


def convert_str_to_bytearray(strData: str) -> bytearray:
    sData = bytearray([0x00])
    if isinstance(strData, str):  # String to Bytes: Ascii
        tmp = bytearray()
        tmp.extend(map(ord, strData))
        sData = tmp

    return sData


def timestamp_to_string(timestamp: datetime.datetime):
    h = timestamp.hour
    m = timestamp.minute
    s = timestamp.second
    # ms = timestamp.microsecond // 1000
    us = timestamp.microsecond
    return '%02d:%02d:%02d.%06d' % (h, m, s, us)


def get_curr_time():
    return '<%s>' % timestamp_to_string(datetime.datetime.now())


def write_log(strMsg: str, obj: object = None, logfile: bool = True):
    global glob_logger
    if glob_logger is None:
        curpath = os.path.dirname(os.path.abspath(__file__))
        dirpath = os.path.dirname(curpath)
        logpath = os.path.join(dirpath, 'Log')
        if not os.path.isdir(logpath):
            os.mkdir(logpath)
        logfilepath = os.path.join(logpath, 'Console.log')
        glob_logger = logging.getLogger('console')
        fh = logging.handlers.RotatingFileHandler(logfilepath, maxBytes=MAX_SIZE, backupCount=10, encoding='utf-8')
        formatter = logging.Formatter('[%(asctime)s]%(message)s')
        fh.setFormatter(formatter)
        glob_logger.addHandler(fh)
        glob_logger.setLevel(logging.DEBUG)

    strTime = get_curr_time()
    if obj is not None:
        if isinstance(obj, threading.Thread):
            if obj.ident is not None:
                strObj = ' [%s][Thread ID:0x%x]' % (type(obj).__name__, obj.ident)
            else:
                strObj = ' [%s]' % type(obj).__name__
        else:
            strObj = ' [%s]' % type(obj).__name__
    else:
        strObj = ''
    print(strTime + strObj + ' ' + strMsg)
    if logfile:
        glob_logger.info(strObj + ' ' + strMsg)