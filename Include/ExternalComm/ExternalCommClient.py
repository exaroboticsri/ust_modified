# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : ExternalCommClient.py
# Project Name : ExaRobotCtrl
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2022.05.04 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import sys
from enum import IntEnum, auto, unique
from typing import Tuple, Union
from dataclasses import \
    dataclass  # for python < 3.7, dataclasses should be installed : python -m pip install dataclasses

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Library/Socket
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
LIBRARY_PATH = os.path.join(ROOT_PATH, "Library")
SOCKET_PATH = os.path.join(LIBRARY_PATH, "Socket")
MISC_PATH = os.path.join(ROOT_PATH, "Misc")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
sys.path.extend([FILE_PATH, ROOT_PATH, INCLUDE_PATH, RESOURCES_PATH, SOCKET_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, RESOURCES_PATH, SOCKET_PATH

from Commons import PySignal, MAX_SERIAL_BUFFER, search_pattern
from Commons import convert_s16_to_bytearray_be, convert_u16_to_bytearray_be, convert_bytearray_to_int_be
from SocketCommon import EmbdTcpClient
from CRC16 import CRC16


@unique
class EnumExternalCommCmd(IntEnum):
    NO_CMD = 0x00
    MOVE_POSE = 0x01
    EMG_STOP = 0x02
    MOVE_MOTOR_CENTER_VEL = 0x03
    MOVE_MOTOR_JOINT_VEL = 0x04
    GET_CURR_POS = 0x05


class CExternalCommClient:
    _client: EmbdTcpClient

    def __init__(self):
        super(CExternalCommClient, self).__init__()
        self._client = None
        self.sig_start = PySignal()
        self.sig_stop = PySignal()
        self.sig_error = PySignal()
        self.sig_recv_raw = PySignal(bytes)
        self.sig_send_raw = PySignal(bytes)
        self.btaRecvBuff = bytearray()
        self._crc = CRC16(0x8005, 0xFFFF, True, True, 0x0000)

    def start_client(self, anPort: int = 7506, strAddr: str = ""):
        self.stop_client()
        self._client = EmbdTcpClient()
        self._client.sig_connected.connect(lambda x: self.sig_start.emit())
        self._client.sig_disconnected.connect(lambda y: self.sig_stop.emit())
        self._client.sig_send_data.connect(self.on_client_send_data)
        self._client.sig_recv_data.connect(self.on_client_recv_data)
        bRet = self._client.connect_to_server(strAddr, anPort)
        if not bRet:
            self.stop_client()
            self.sig_error.emit()

        return bRet

    def stop_client(self):
        bRet = True
        if self._client is not None:
            bRet = self._client.close()

        del self._client
        self._client = None

        return bRet

    def on_client_send_data(self, abData: bytes):
        self.sig_send_raw.emit(abData)

    def on_client_recv_data(self, abData: bytes):
        self.sig_recv_raw.emit(abData)
        if len(self.btaRecvBuff) >= MAX_SERIAL_BUFFER:
            self.btaRecvBuff.clear()

        try:
            self.btaRecvBuff.extend(bytearray(abData))
            self.deserialize_packet()

        except Exception:
            self.btaRecvBuff.clear()

    def deserialize_packet(self):
        bta_stx = bytearray([0x02, 0x5B])  # STX
        n_next_packet_idx = 0

        n_idx = search_pattern(self.btaRecvBuff, bta_stx)
        if n_idx is not None:
            # minimum length of the whole packet
            # STX: 2 bytes, Packet Len: 1 byte, CMD: 1 byte, CRC16: 2 bytes, ETX: 2 bytes
            if len(self.btaRecvBuff) >= n_idx + 8:
                n_packet_len = self.btaRecvBuff[n_idx + 2] + 7  # only data and CMD are counted in packet length
                if len(self.btaRecvBuff) >= n_idx + n_packet_len:
                    if self.btaRecvBuff[n_idx + n_packet_len - 2] == 0x5D and self.btaRecvBuff[
                        n_idx + n_packet_len - 1] == 0x03:
                        tmp = self.btaRecvBuff[n_idx: n_idx + n_packet_len]  # start of payload
                        self.parse_packet_values(tmp)
                        n_next_packet_idx = n_idx + n_packet_len

        if len(self.btaRecvBuff) >= n_next_packet_idx:
            self.btaRecvBuff = self.btaRecvBuff[n_next_packet_idx:]

    def send_pose(self, axPos: float, ayPos: float, aangleDeg: float):
        data = bytearray()
        xPos = int(axPos * 1000)
        yPos = int(ayPos * 1000)
        aangleDeg = int(aangleDeg * 1000)

        data.extend(convert_s16_to_bytearray_be(xPos))
        data.extend(convert_s16_to_bytearray_be(yPos))
        data.extend(convert_s16_to_bytearray_be(aangleDeg))
        self.send_packet(int(EnumExternalCommCmd.MOVE_POSE), data)

    def send_emg_stop(self):
        self.send_packet(int(EnumExternalCommCmd.EMG_STOP))

    def parse_packet_values(self, abtData):
        b_cmd = int(abtData[3])
        if b_cmd == EnumExternalCommCmd.EMG_STOP:
            pass

        elif b_cmd == EnumExternalCommCmd.MOVE_POS:
            pass

    def send_raw(self, abtaData):
        self._client.sendData(abtaData)

    def make_packet(self, anCmd: int, abtaData: bytearray) -> bytearray:
        nPacketLen = 1 + len(abtaData) if abtaData is not None else 1
        buff = bytearray([0x02, 0x5B])  # STX
        buff.append(nPacketLen)
        buff.append(int(anCmd))

        if abtaData is not None:
            buff.extend(abtaData)

        temp = buff[2:]
        nCRC = self._crc.calculate(temp)
        print(nCRC)
        buff.extend(convert_u16_to_bytearray_be(nCRC))
        buff.extend([0x5D, 0x03])

        return buff

    def send_packet(self, anCmd: int, abtaPayLoad: bytearray = None) -> bool:
        bta_packet = self.make_packet(anCmd, abtaPayLoad)
        if self._client.is_connected():
            self._client.sendData(bta_packet)
            return True
        else:
            return False

    def release(self):
        self.stop_client()
