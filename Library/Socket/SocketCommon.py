# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : SocketCommon.py
# Project Name : ExaRobotCtrl
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2021.07.09 - First Commit
# >> 2022.05.04 - Add TCP Server
# -------------------------------------------------------------------------------------------------------------------- #
import os
import sys
import socket
import queue
import threading
import time
from typing import List

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Library/Socket
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
SOURCES_PATH = os.path.join(ROOT_PATH, "Sources")
MDI_PATH = os.path.join(ROOT_PATH, "MdiBackground")
DOCK_PATH = os.path.join(ROOT_PATH, "DockWidgets")
MISC_PATH = os.path.join(ROOT_PATH, "Misc")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
sys.path.extend([FILE_PATH, ROOT_PATH, SOURCES_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, SOURCES_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH

from Commons import *


class EmbdSocket:
    _socket: socket.socket = None

    def __init__(self, afTimeOut: float = 1.):
        super(EmbdSocket, self).__init__()
        self.sig_connected = PySignal(object)
        self.sig_connect_failed = PySignal(object)
        self.sig_disconnected = PySignal(object)
        self.sig_error = PySignal(object, str)

        self._buffsz = MAX_SERIAL_BUFFER
        self._threadSend = None
        self._threadRecv = None

        self._queueSend = queue.Queue()
        self._queueRecv = queue.Queue()
        self._logging: bool = False

        self.fTimeOut = afTimeOut
        self.bIsSocketInit = False
        self.bIsSocketTcp = False

    def init_socket_with_server(self, aSocket: socket.socket):
        self._socket = aSocket

    def init_socket(self, bIsTcp: bool = True) -> bool:
        if self._socket is not None:
            return False

        nSocketType = socket.SOCK_DGRAM  # UDP
        if bIsTcp:
            nSocketType = socket.SOCK_STREAM

        self._socket = socket.socket(socket.AF_INET, nSocketType)

        # reuse address to prevent "Address already in use" error
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.bIsSocketInit = True
        self.bIsSocketTcp = bIsTcp
        return True

    def shutdown_socket(self):
        self._socket.shutdown(socket.SHUT_RDWR)

    def connect_to_server(self, strAddr: str, nPort: int, afTimeOut: float = 1.):
        if not self.is_socket_init():
            raise RuntimeError

        try:
            self._socket.settimeout(afTimeOut)
            self._socket.connect((strAddr, nPort))

        except:
            return False

        self._socket.setblocking(True)
        return True

    def set_blocking(self, bIsBlocking):
        if not self.is_socket_init():
            raise RuntimeError

        self._socket.setblocking(bIsBlocking)

    def bind_address(self, anPort: int, strAddress: str):
        if not self.is_socket_init():
            raise RuntimeError

        # only TCP needs listening
        strHost = strAddress  # default is blank, which is INADDRY_ANY or '0.0.0.0'
        self._socket.bind((strHost, anPort))

    def listen_socket(self, anBackLog: int):
        if not self.is_socket_init() or not self.is_socket_tcp():
            raise RuntimeError

        # only TCP needs listening
        self._socket.listen(anBackLog)

    def accept_client(self):
        if not self.is_socket_init() or not self.is_socket_tcp():
            raise RuntimeError

        return self._socket.accept()

    def close_socket(self):
        if self._socket:
            # self.shutdown_socket()
            try:
                self._socket.close()
                del self._socket
                self._socket = None
                return True

            except OSError:
                self.shutdown_socket()
                self.close_socket()

        return False

    def is_socket_tcp(self):
        return self.bIsSocketTcp

    def is_socket_init(self):
        return self.bIsSocketInit

    def get_socket(self):
        return self._socket

    def get_buff_size(self):
        return self._buffsz


class TcpClientRcvThread(threading.Thread):
    keepAlive: bool = True

    def __init__(self, asSocket: EmbdSocket):
        super(TcpClientRcvThread, self).__init__()
        self.sig_recv = PySignal(bytes)
        self.sig_terminated = PySignal()
        self._socket = asSocket.get_socket()
        self._buffsize = asSocket.get_buff_size()
        self.setDaemon(True)

    def stop(self):
        self.keepAlive = False

    def run(self) -> None:
        write_log("Started", self)
        while self.keepAlive:
            try:
                recv_data = self._socket.recv(self._buffsize)
                if len(recv_data) == 0:
                    break
                else:
                    self.sig_recv.emit(recv_data)
            except:
                break

        write_log("Terminated", self)
        self.sig_terminated.emit()


class TcpClientSendThread(threading.Thread):
    keepAlive: bool = True

    def __init__(self, asSocket: EmbdSocket, q: queue.Queue):
        super(TcpClientSendThread, self).__init__()
        self.sig_send = PySignal(bytes)
        self.sig_terminated = PySignal(bool)
        self._socket = asSocket.get_socket()
        self._queue = q
        self._buffsize = asSocket.get_buff_size()
        self.setDaemon(True)

    def stop(self):
        self.keepAlive = False

    def run(self) -> None:
        bBreakFromException = False
        write_log("Started", self)
        while self.keepAlive:
            try:
                if not self._queue.empty():
                    send_data = self._queue.get()
                    self._socket.sendall(send_data)
                    self.sig_send.emit(send_data)
                else:
                    time.sleep(1e-3)

            except Exception:
                bBreakFromException = True
                break

        write_log("Terminated", self)
        self.sig_terminated.emit(bBreakFromException)


class EmbdTcpClient:
    _threadRecv: TcpClientRcvThread = None
    _threadSend: TcpClientSendThread = None
    _queueSend: queue.Queue = None
    _isConnected: bool = False

    def __init__(self):
        self._socket = EmbdSocket()
        self.sig_connected = PySignal(tuple)
        self.sig_disconnected = PySignal(tuple)
        self.sig_send_data = PySignal(bytes)
        self.sig_recv_data = PySignal(bytes)
        self.tupleAddPort = tuple()

    def connect_to_server(self, strAddress: str, nPort: int):
        self._socket.close_socket()
        self._socket.init_socket(True)
        try:
            bRet = self._socket.connect_to_server(strAddress, nPort)
            if bRet:
                self.start_listening()
        except:
            return False

        self.tupleAddPort = (strAddress, nPort)
        return bRet

    def init_with_server(self, aSocket: socket.socket, tplAddPort: str):
        self._socket.init_socket_with_server(aSocket)
        self.tupleAddPort = tplAddPort

    def start_listening(self):
        self._isConnected = True
        self.start_recv_thread()
        self.start_send_thread()
        self.sig_connected.emit(self.tupleAddPort)

    def start_recv_thread(self):
        if self._threadRecv is None:
            self._threadRecv = TcpClientRcvThread(self._socket)
            self._threadRecv.sig_recv.connect(self.on_recv_recv_thread)
            self._threadRecv.sig_terminated.connect(self.on_terminated_recv_thread)
            self._threadRecv.start()

    def stop_recv_thread(self):
        if self._threadRecv is not None:
            self._threadRecv.stop()

    def on_recv_recv_thread(self, abData: bytes):
        self.sig_recv_data.emit(abData)

    def on_terminated_recv_thread(self):
        del self._threadRecv
        self._threadRecv = None

        if self.is_connected():
            self.close()
            self._isConnected = False

    def sendData(self, data):
        try:
            sData = bytes()
            if isinstance(data, str):  # String to Bytes: Ascii
                tmp = bytearray()
                tmp.extend(map(ord, data))
                sData = bytes(tmp)
            elif isinstance(data, bytes) or isinstance(data, bytearray):
                sData = bytes(data)

            self._queueSend.put(sData)
            return True

        except Exception:
            return False

    def start_send_thread(self):
        if self._threadSend is None:
            self._queueSend = queue.Queue()
            self._threadSend = TcpClientSendThread(self._socket, self._queueSend)
            self._threadSend.sig_send.connect(self.on_send_send_thread)
            self._threadSend.sig_terminated.connect(self.on_terminated_send_thread)
            self._threadSend.start()

    def stop_send_thread(self):
        if self._threadSend is not None:
            self._threadSend.stop()

    def on_send_send_thread(self, abData: bytes):
        self.sig_send_data.emit(abData)

    def on_terminated_send_thread(self, bBreakFromException):
        del self._threadSend
        self._threadSend = None

        if bBreakFromException:
            self.close()
            self._isConnected = False

    def close(self):
        self.stop_recv_thread()
        self.stop_send_thread()

        if self._socket.close_socket():
            self.sig_disconnected.emit(self.tupleAddPort)

    def is_connected(self):
        return self._isConnected


class EmbdTcpServer:
    _listClients: List[EmbdTcpClient]
    _threadAcceptClient: threading.Thread = None
    _threadKillDeadClient: threading.Thread = None

    def __init__(self):
        super(EmbdTcpServer, self).__init__()
        self.sig_start = PySignal()
        self.sig_stop = PySignal()
        self.sig_send_data = PySignal(bytes)
        self.sig_recv_data = PySignal(bytes)
        self.sig_accept_client = PySignal(tuple)
        self.sig_disconnect_client = PySignal(tuple)
        self._socket = EmbdSocket()
        self._listClients = []

    def start_server(self, anPort: int, strIPAddr: str = '', anBackLog: int = 1, abIsNonBlocking: bool = False):
        self._socket.init_socket(bIsTcp=True)

        try:
            self._socket.bind_address(anPort, strIPAddr)
            self._socket.listen_socket(anBackLog)
            if abIsNonBlocking:
                self._socket.set_blocking(False)

            self.sig_start.emit()

        except RuntimeError:
            self.stop()
            return False

        except:
            self.stop()
            return False

        if not strIPAddr:
            strIPAddr = "localhost"

        write_log("Starting server: " + strIPAddr + ":%d" % anPort, self)
        self.start_client_accept_thread()
        self.start_client_killer_thread()

        return True

    def send_to_clients(self, data):
        for client in self._listClients:
            client.sendData(data)

    def add_client(self, aClient: EmbdTcpClient):
        self._listClients.append(aClient)

    def remove_client_by_address(self, tplAddrPort: tuple):
        for client in self._listClients:
            if client.tupleAddPort == tplAddrPort:
                self.remove_client(client)

    def remove_client(self, aClient: EmbdTcpClient):
        aClient.close()
        self._listClients.remove(aClient)

    def get_clients(self):
        return self._listClients

    def is_connected(self):
        if len(self._listClients) == 0:
            return False
        else:
            return True

    def accept_client(self):
        return self._socket.accept_client()

    def start_client_accept_thread(self):
        if self._threadAcceptClient is None:
            self._threadAcceptClient = ClientAcceptThread(self)
            self._threadAcceptClient.sig_terminated.connect(self.on_terminate_client_accept_thread)
            self._threadAcceptClient.start()

    def stop_client_accept_thread(self):
        if self._threadAcceptClient is not None:
            self._threadAcceptClient.stop()

    def on_terminate_client_accept_thread(self):
        del self._threadAcceptClient
        self._threadAcceptClient = None

    def start_client_killer_thread(self):
        if self._threadKillDeadClient is None:
            self._threadKillDeadClient = ClientKillerThread(self)
            self._threadKillDeadClient.sig_terminated.connect(self.on_terminate_client_killer_thread)
            self._threadKillDeadClient.start()

    def stop_client_killer_thread(self):
        if self._threadKillDeadClient is not None:
            self._threadKillDeadClient.stop()

    def on_terminate_client_killer_thread(self):
        del self._threadKillDeadClient
        self._threadKillDeadClient = None

    def on_client_recv(self, abData: bytes):
        self.sig_recv_data.emit(abData)

    def on_client_send(self, abData: bytes):
        self.sig_send_data.emit(abData)

    def on_accept_client(self, tplAddPort: tuple):
        self.sig_accept_client.emit(tplAddPort)

    def on_disconnect_client(self, tplAddPort: tuple):
        self.sig_disconnect_client.emit(tplAddPort)

    def stop(self):
        self.stop_client_accept_thread()
        self.stop_client_killer_thread()

        for client in self._listClients:
            try:
                client.close()
            except RuntimeError:
                pass
            except Exception:
                pass

        self._listClients.clear()
        self._socket.close_socket()
        write_log("Stopped server!", self)
        self.sig_stop.emit()


class ClientKillerThread(threading.Thread):
    _keepAlive: bool = True

    def __init__(self, server: EmbdTcpServer):
        super(ClientKillerThread, self).__init__()
        self._server = server
        self.sig_terminated = PySignal()
        self.setDaemon(True)

    def stop(self):
        self._keepAlive = False

    def run(self) -> None:
        while self._keepAlive:
            try:
                for client in self._server.get_clients():
                    if not client.is_connected():
                        self._server.remove_client(client)

            except RuntimeError:
                pass

            except Exception:
                pass

            time.sleep(1)

        self.sig_terminated.emit()


class ClientAcceptThread(threading.Thread):
    _keepAlive: bool = True

    def __init__(self, server: EmbdTcpServer):
        super(ClientAcceptThread, self).__init__()
        self._server = server
        self.setDaemon(True)
        self.sig_terminated = PySignal()

    def stop(self):
        self._keepAlive = False

    def run(self) -> None:
        while self._keepAlive:
            try:
                client_sck, client_address = self._server.accept_client()  # this should be blocking
                newClient = EmbdTcpClient()
                newClient.sig_connected.connect(self._server.on_accept_client)
                newClient.sig_disconnected.connect(self._server.on_disconnect_client)
                newClient.sig_send_data.connect(self._server.on_client_send)
                newClient.sig_recv_data.connect(self._server.on_client_recv)
                newClient.init_with_server(client_sck, client_address)
                newClient.start_listening()
                self._server.add_client(newClient)

            except RuntimeError:
                break
            except Exception:
                break

        self.sig_terminated.emit()


# HOST = "0.0.0.0"  # The server's hostname or IP address
HOST = ""  # The server's hostname or IP address
PORT = 9924  # The port used by the server

if __name__ == '__main__':
    pass
    sample = EmbdTcpServer()
    sample.start_server(PORT)

    while True:
        time.sleep(1e-3)
