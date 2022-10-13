# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MPMobileRobot.py
# Project Name : ExaRobotCtrl
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2022.02.17 - First Commit
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
MDI_PATH = os.path.join(INCLUDE_PATH, "MdiBackground")
DOCK_PATH = os.path.join(INCLUDE_PATH, "DockWidgets")
MISC_PATH = os.path.join(INCLUDE_PATH, "Misc")
RESOURCES_PATH = os.path.join(INCLUDE_PATH, "Resources")
MULTIPROCESS_PATH = os.path.join(INCLUDE_PATH, "MultiProcessing")
LIBRARY_PATH = os.path.join(ROOT_PATH, "Library")
SERIAL_PATH = os.path.join(LIBRARY_PATH, "Serial")
sys.path.extend([FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH,
                 MULTIPROCESS_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH, MULTIPROCESS_PATH

from Commons import PySignal
from TMRKobuki2 import CTMRKobuki2
from TMRStellaB2 import CTMRStellaB2
from TMRExaRobot import CTMRExaRobot
from MultiProcessBase import CMultiProcessBase

gMobileRobot = None


# dict: CMD, VALUE
# pipeChild is an instance of PipeConnection on Windows, but this is different on Linux
def proc_mobile_robot(pipeChild, feedbackQueue: Queue, feedbackQueueBk: Union[Queue, None]):
    global gMobileRobot

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
                    if val == "STELLAB2":
                        gMobileRobot = CTMRStellaB2()
                        gMobileRobot.sig_is_run_monitoring.connect(
                            lambda state: on_run_monitoring(state, feedbackQueue))

                    elif val == "KOBUKI2":
                        gMobileRobot = CTMRKobuki2()
                        gMobileRobot.sig_get_hw_ver.connect(lambda ver: on_get_hw_version(ver, feedbackQueue))

                    elif val == "EXAROBOT":
                        gMobileRobot = CTMRExaRobot()
                    else:
                        pass

                    # common signals
                    gMobileRobot.sig_comm_error.connect(lambda: on_comm_status("ERROR", feedbackQueue))
                    gMobileRobot.sig_connected.connect(lambda: on_comm_status("CONNECTED", feedbackQueue))
                    gMobileRobot.sig_disconnected.connect(lambda: on_comm_status("DISCONNECTED", feedbackQueue))
                    gMobileRobot.sig_get_enc.connect(lambda r, l: on_get_enc(r, l, feedbackQueue))
                    gMobileRobot.sig_get_vel.connect(lambda r, l: on_get_vel(r, l, feedbackQueue))
                    gMobileRobot.sig_get_pos.connect(lambda x, y, theta: on_get_pos(x, y, theta, feedbackQueue))
                    gMobileRobot.sig_get_wheel_pos.connect(
                        lambda r, l: on_get_wheel_pos(r, l, feedbackQueue))
                    gMobileRobot.sig_get_state.connect(lambda state: on_get_state(state, feedbackQueue))
                    gMobileRobot.sig_get_ver.connect(lambda ver: on_get_ver(ver, feedbackQueue))

                elif cmd == "CONNECT":
                    if gMobileRobot.IsSerial:
                        str_comport = val
                        gMobileRobot.connect(str_comport)

                elif cmd == "DISCONNECT":
                    if gMobileRobot.IsSerial:
                        gMobileRobot.disconnect()

                elif cmd == "MONITORING":
                    if isinstance(gMobileRobot, CTMRStellaB2):
                        if bool(val):
                            gMobileRobot.start_monitoring_thread()
                        else:
                            gMobileRobot.stop_monitoring_thread()

                    else:
                        pass

                elif cmd == "RESET_MOTOR":
                    if isinstance(gMobileRobot, CTMRStellaB2):
                        gMobileRobot.reset_motor()
                    elif isinstance(gMobileRobot, CTMRKobuki2):
                        gMobileRobot.play_sound_sequence(1)
                    else:
                        pass

                elif cmd == "INIT_MOTOR":
                    if isinstance(gMobileRobot, CTMRStellaB2):
                        gMobileRobot.set_factory_settings()
                    elif isinstance(gMobileRobot, CTMRKobuki2):
                        gMobileRobot.play_sound_sequence(1)
                    else:
                        pass

                elif cmd == "MOVE_JOINT_SPACE":
                    right, left = val
                    gMobileRobot.move_joint_space(right, left)

                elif cmd == "SET_VELOCITY_CONTROL":
                    vel, ang_vel = val
                    gMobileRobot.set_velocity_control(vel, ang_vel)

                elif cmd == "STOP":
                    if isinstance(gMobileRobot, CTMRStellaB2):
                        stop_type = val
                        gMobileRobot.move_stop(stop_type)
                    else:
                        gMobileRobot.move_stop()

                elif cmd == "RELEASE":
                    gMobileRobot.release()
                else:
                    pass

                time.sleep(1e-3)
            else:
                time.sleep(1e-3)
        except:
            time.sleep(1e-3)

        time.sleep(1e-6)


## COMMON ##
def on_get_ver(strVer: str, q: mp.Queue):
    dict_msg = dict()
    dict_msg['VERSION'] = strVer
    q.put(dict_msg)


def on_get_state(state: int, q: mp.Queue):
    dict_msg = dict()
    dict_msg['STATUS'] = state
    q.put(dict_msg)


def on_get_wheel_pos(right: float, left: float, q: mp.Queue):
    dict_msg = dict()
    dict_msg['WHEEL_POSITION'] = (right, left)
    q.put(dict_msg)


def on_get_pos(x: float, y: float, theta: float, q: mp.Queue):
    dict_msg = dict()
    dict_msg['POSITION'] = (x, y, theta)
    q.put(dict_msg)


def on_get_vel(right: float, left: float, q: mp.Queue):
    dict_msg = dict()
    dict_msg['VELOCITY'] = (right, left)
    q.put(dict_msg)


def on_get_enc(right: int, left: int, q: mp.Queue):
    dict_msg = dict()
    dict_msg['ENCODER'] = (right, left)
    q.put(dict_msg)


def on_comm_status(astrStatus: str, q: mp.Queue):
    dict_msg = dict()
    dict_msg['COMM_STATUS'] = astrStatus
    q.put(dict_msg)


## STELLA B2 ##
def on_run_monitoring(isMonitoring: bool, q: mp.Queue):
    dict_msg = dict()
    dict_msg['MONITORING'] = bool(isMonitoring)
    q.put(dict_msg)


## KOBUKI2 ##
def on_get_hw_version(strVer: str, q: mp.Queue):
    dict_msg = dict()
    dict_msg['HW_VERSION'] = strVer
    q.put(dict_msg)
