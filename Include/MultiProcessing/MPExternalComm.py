# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MPExternalComm.py
# Project Name : ExaRobotCtrl
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2022.05.05 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import multiprocessing as mp
import os
import sys
from typing import Tuple
from multiprocessing import Queue
# from multiprocessing.connection import PipeConnection
from typing import Union
import time

import psutil

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Library/Socket
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
EXTERNAL_COMM_PATH = os.path.join(INCLUDE_PATH, "ExternalComm")
MDI_PATH = os.path.join(INCLUDE_PATH, "MdiBackground")
DOCK_PATH = os.path.join(INCLUDE_PATH, "DockWidgets")
MISC_PATH = os.path.join(INCLUDE_PATH, "Misc")
RESOURCES_PATH = os.path.join(INCLUDE_PATH, "Resources")
MULTIPROCESS_PATH = os.path.join(INCLUDE_PATH, "MultiProcessing")
LIBRARY_PATH = os.path.join(ROOT_PATH, "Library")
SERIAL_PATH = os.path.join(LIBRARY_PATH, "Serial")
sys.path.extend([FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH,
                 MULTIPROCESS_PATH, EXTERNAL_COMM_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH, MULTIPROCESS_PATH, EXTERNAL_COMM_PATH

from Commons import PySignal
from MultiProcessBase import CMultiProcessBase
from ExternalComm import CExternalComm

gExternalComm = None


# dict: CMD, VALUE
# pipeChild is an instance of PipeConnection on Windows, but this is different on Linux
def proc_external_comm(pipeChild, feedbackQueue: Queue, feedbackQueueBk: Union[Queue, None]):
    global gExternalComm

    n_pid = os.getpid()
    str_name = mp.current_process().name
    this_proc = psutil.Process(n_pid)

    while this_proc.is_running():
        try:
            if pipeChild.poll():
                dict_cmd = pipeChild.recv()
                try:
                    cmd = dict_cmd["CMD"]
                    val = dict_cmd["VAL"]
                except KeyError:
                    cmd = ""
                    val = ""

                if cmd == "CREATE":
                    gExternalComm = CExternalComm()
                    # common signals
                    gExternalComm.sig_start.connect(lambda: on_comm_status("START", feedbackQueue))
                    gExternalComm.sig_stop.connect(lambda: on_comm_status("STOP", feedbackQueue))
                    gExternalComm.sig_error.connect(lambda: on_comm_status("ERROR", feedbackQueue))
                    gExternalComm.sig_accept_client.connect(lambda x: on_client_connect(x, feedbackQueue))
                    gExternalComm.sig_disconnect_client.connect(lambda x: on_client_disconnect(x, feedbackQueue))
                    gExternalComm.sig_recv_raw.connect(lambda x: on_recv_raw(x, feedbackQueue))
                    gExternalComm.sig_emg_stop.connect(lambda: on_emg_stop(feedbackQueue))
                    gExternalComm.sig_move_pose.connect(lambda x, y, z: on_move_pose(x, y, z, feedbackQueue))

                elif cmd == "START":
                    addr, port = val
                    gExternalComm.start_server(int(port), str(addr))


                elif cmd == "STOP":
                    gExternalComm.stop_server()

                elif cmd == "SEND_RAW":
                    msg = str(val)
                    gExternalComm.send_raw(msg)

                elif cmd == "REMOVE_CLIENT":
                    msg = tuple(val)
                    gExternalComm.disconnect_client(msg)

                elif cmd == "RELEASE":
                    gExternalComm.release()
                else:
                    pass

                time.sleep(1e-3)
            else:
                time.sleep(1e-3)
        except:
            time.sleep(1e-3)

        time.sleep(1e-6)


## COMMON ##
def on_comm_status(astrStatus: str, q: mp.Queue):
    dict_msg = dict()
    dict_msg['SERVER_STATUS'] = astrStatus
    q.put(dict_msg)


def on_client_connect(tplAddrPort: tuple, q: mp.Queue):
    dict_msg = dict()
    dict_msg['CLIENT_CONNECT'] = tplAddrPort
    q.put(dict_msg)


def on_client_disconnect(tplAddrPort: tuple, q: mp.Queue):
    dict_msg = dict()
    dict_msg['CLIENT_DISCONNECT'] = tplAddrPort
    q.put(dict_msg)


def on_recv_raw(abData: bytes, q: mp.Queue):
    dict_msg = dict()
    dict_msg['RECV_RAW'] = abData
    q.put(dict_msg)


def on_emg_stop(q: mp.Queue):
    dict_msg = dict()
    dict_msg['EMG_STOP'] = True
    q.put(dict_msg)


def on_move_pose(x, y, z, q: mp.Queue):
    print(x, y, z)
    dict_msg = dict()
    dict_msg['MOVE_POSE'] = (x, y, z)
    q.put(dict_msg)
