# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : ExternalComm.py
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
STT_PATH = os.path.join(INCLUDE_PATH, "AudioRecognition")
MDI_PATH = os.path.join(INCLUDE_PATH, "MdiBackground")
DOCK_PATH = os.path.join(INCLUDE_PATH, "DockWidgets")
MISC_PATH = os.path.join(INCLUDE_PATH, "Misc")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
sys.path.extend([FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, SOCKET_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, SOCKET_PATH

from Commons import PySignal, MAX_SERIAL_BUFFER, search_pattern
from Commons import convert_bytearray_to_int_be
from SocketCommon import EmbdTcpServer
from CRC16 import CRC16


@unique
class EnumExternalCommCmd(IntEnum):
    NO_CMD = 0x00
    MOVE_POSE = 0x01
    EMG_STOP = 0x02
    MOVE_MOTOR_CENTER_VEL = 0x03
    MOVE_MOTOR_JOINT_VEL = 0x04
    GET_CURR_POS = 0x05


class CExternalComm:
    _server: EmbdTcpServer

    def __init__(self):
        super(CExternalComm, self).__init__()
        self._server = None
        self.sig_start = PySignal()
        self.sig_stop = PySignal()
        self.sig_error = PySignal()
        self.sig_accept_client = PySignal(tuple)
        self.sig_disconnect_client = PySignal(tuple)
        self.sig_recv_raw = PySignal(bytes)
        self.btaRecvBuff = bytearray()

        self.sig_emg_stop = PySignal()
        self.sig_move_pose = PySignal(float, float, float)
        self._crc = CRC16(0x8005, 0xFFFF, True, True, 0x0000)

    def start_server(self, anPort: int = 7506, strAddr: str = ""):
        # self.stop_server()
        self._server = EmbdTcpServer()
        self._server.sig_start.connect(self.on_server_start)
        self._server.sig_send_data.connect(self.on_server_send_data)
        self._server.sig_recv_data.connect(self.on_server_recv_data)
        self._server.sig_accept_client.connect(self.on_server_accept_client)
        self._server.sig_disconnect_client.connect(self.on_server_disconnect_client)
        self._server.sig_stop.connect(self.on_server_stop)

        bRet = self._server.start_server(anPort, strAddr)
        if not bRet:
            self.sig_error.emit()
            self.stop_server()

        return bRet

    def stop_server(self):
        if self._server:
            self._server.stop()

    def disconnect_client(self, tplAddrPort: tuple):
        self._server.remove_client_by_address(tplAddrPort)

    def on_server_start(self):
        self.sig_start.emit()

    def on_server_stop(self):
        del self._server
        self._server = None
        self.sig_stop.emit()

    def on_server_accept_client(self, tplAddrPort: tuple):
        self.sig_accept_client.emit(tplAddrPort)

    def on_server_disconnect_client(self, tplAddrPort: tuple):
        self.sig_disconnect_client.emit(tplAddrPort)

    def on_server_send_data(self, abData: bytes):
        pass

    def on_server_recv_data(self, abData: bytes):
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
            # todo: add CRC
            # STX: 2 bytes, Packet Len: 1 byte, CMD: 1 byte, CRC16: 2bytes,  ETX: 2 bytes
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

    def parse_packet_values(self, abtData):
        b_cmd = int(abtData[3])
        n_recv_crc = convert_bytearray_to_int_be(abtData[-4:-2])
        print(n_recv_crc)


        if b_cmd == EnumExternalCommCmd.EMG_STOP:
            self.sig_emg_stop.emit()

        elif b_cmd == EnumExternalCommCmd.MOVE_POSE:
            x_pos = float(convert_bytearray_to_int_be(abtData[4:6], abIsSigned=True) / 1000.)
            y_pos = float(convert_bytearray_to_int_be(abtData[6:8], abIsSigned=True) / 1000.)
            aAngle = float(convert_bytearray_to_int_be(abtData[8:10], abIsSigned=True) / 1000.)
            self.sig_move_pose.emit(x_pos, y_pos, aAngle)

        else:
            pass

    def send_raw(self, abtaData):
        self._server.send_to_clients(abtaData)

    def make_packet(self, anCmd: int, abtaData: bytearray) -> bytearray:
        nPacketLen = 1 + len(abtaData) if abtaData is not None else 1
        buff = bytearray([0x02, 0x5B])  # STX
        buff.append(int(anCmd))
        buff.append(nPacketLen)

        if abtaData is not None:
            buff.extend(abtaData)

        buff.extend([0x5D, 0x03])

        return buff

    def send_packet(self, anCmd: int, abtaPayLoad: bytearray = None) -> bool:
        bta_packet = self.make_packet(anCmd, abtaPayLoad)

        if self._server.is_connected():
            self._server.send_to_clients(bta_packet)
            return True
        else:
            return False

    def release(self):
        self.stop_server()


if __name__ == '__main__':
    sample = CExternalComm()
