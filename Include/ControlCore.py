# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : ControlCore.py
# Project Name : MobileRobotTest
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2021.08.24 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import queue
import threading
import sys
import time
from typing import Tuple, List, Dict
import multiprocessing as mp

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include
ROOT_PATH = os.path.dirname(FILE_PATH)

INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
MDI_PATH = os.path.join(INCLUDE_PATH, "MdiBackground")
DOCK_PATH = os.path.join(INCLUDE_PATH, "DockWidgets")
PATH_PLANNER_PATH = os.path.join(INCLUDE_PATH, "PathPlanning")
MOBILE_ROBOT_PATH = os.path.join(INCLUDE_PATH, "MobileRobot")
MISC_PATH = os.path.join(INCLUDE_PATH, "Misc")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
MULTIPROC_PATH = os.path.join(INCLUDE_PATH, "MultiProcessing")
TTS_PATH = os.path.join(INCLUDE_PATH, 'AudioRecognition')
LIBRATY_PATH = os.path.join(ROOT_PATH, "Library")
CAMERA_DEV_PATH = os.path.join(LIBRATY_PATH, "Devices/Camera")
sys.path.extend(
    [FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, PATH_PLANNER_PATH, MOBILE_ROBOT_PATH,
     MULTIPROC_PATH, CAMERA_DEV_PATH, TTS_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, PATH_PLANNER_PATH, MOBILE_ROBOT_PATH, MULTIPROC_PATH, CAMERA_DEV_PATH, TTS_PATH

from Commons import *

from MultiProcessBase import CMultiProcessBase
import MPMobileRobot
import MPExternalComm
from TextToSpeech import CTextToSpeech
from SpeechToText import CSpeechToText

from CameraCommon import CCameraCommon

if is_ros_installed():
    import MPROSVelPub
    import MPROSVelSub
    import MPROSTfPub
    import MPROSNavigation


class EmbdControlCore(object):
    # queuePlotter: queue.Queue = None
    # threadingPlotter: ThreadPlotter = None
    _mpStellaB2: CMultiProcessBase = None
    _mpKobuki2: CMultiProcessBase = None
    _mpExaRobot: CMultiProcessBase = None
    _mpROSCmdVelPub: CMultiProcessBase = None
    _mpROSCmdVelSub: CMultiProcessBase = None
    _mpROSTfPub: CMultiProcessBase = None
    _mpROSNavigation: CMultiProcessBase = None
    _mpCamTest: CMultiProcessBase = None
    _mpExternalComm: CMultiProcessBase = None
    _ttsHandler: CTextToSpeech = None
    _sttHandler: CSpeechToText = None

    _dictProc: dict
    _camTest: CCameraCommon = None

    def __init__(self):
        write_log("Started Control Core", self)
        super(EmbdControlCore, self).__init__()
        # self.StartThreadPlanner()
        mp.set_start_method('spawn')
        # tts handler
        self._ttsHandler = CTextToSpeech()

        # stt handler
        self.sig_stt_msg = PySignal(str)
        self._sttHandler = CSpeechToText()
        self._sttHandler.sig_stt_msg.connect(lambda x: self.sig_stt_msg.emit(x))

        # stella_b2 signals
        self.sig_stellab2_connected = PySignal()
        self.sig_stellab2_disconnected = PySignal()
        self.sig_stellab2_comm_error = PySignal()
        self.sig_stellab2_comm_status = PySignal(str)
        self.sig_stellab2_is_monitoring = PySignal(bool)
        self.sig_stellab2_get_vel = PySignal(float, float)
        self.sig_stellab2_get_enc = PySignal(int, int)
        self.sig_stellab2_get_wheel_pos = PySignal(float)
        self.sig_stellab2_get_pos = PySignal(float, float, float)
        self.sig_stellab2_get_state = PySignal(int)
        self.sig_stellab2_get_ver = PySignal(str)

        # kobuki2 signals
        self.sig_kobuki2_connected = PySignal()
        self.sig_kobuki2_disconnected = PySignal()
        self.sig_kobuki2_comm_error = PySignal()
        self.sig_kobuki2_comm_status = PySignal(str)
        self.sig_kobuki2_get_vel = PySignal(float, float)
        self.sig_kobuki2_get_enc = PySignal(int, int)
        self.sig_kobuki2_get_wheel_pos = PySignal(float)
        self.sig_kobuki2_get_pos = PySignal(float, float, float)
        self.sig_kobuki2_get_state = PySignal(int)
        self.sig_kobuki2_get_ver = PySignal(str)
        self.sig_kobuki2_get_hw_ver = PySignal(str)

        # exarobot signals
        self.sig_exarobot_connected = PySignal()
        self.sig_exarobot_disconnected = PySignal()
        self.sig_exarobot_comm_error = PySignal()
        self.sig_exarobot_comm_status = PySignal(str)
        self.sig_exarobot_get_vel = PySignal(float, float)
        self.sig_exarobot_get_enc = PySignal(int, int)
        self.sig_exarobot_get_wheel_pos = PySignal(float)
        self.sig_exarobot_get_pos = PySignal(float, float, float)
        self.sig_exarobot_get_state = PySignal(int)

        # external comm signals
        self.sig_external_comm_server_status = PySignal(str)
        self.sig_external_comm_client_connected = PySignal(tuple)
        self.sig_external_comm_client_disconnected = PySignal(tuple)

        # camera
        self.sig_cam_recv_stream = PySignal(tuple)
        self.sig_cam_stopped_stream = PySignal()

        # process manager
        self.sig_proc_monitor_pid = PySignal(dict)
        self._dictProc = dict()

        # multiprocessing
        self.init_multi_processes()
        self.stellab2_create()
        self.kobuki2_create()
        self.exarobot_create()
        self.external_comm_create()

        self.ros_vel_pub_create(0.05, "cmd_vel")
        self.ros_vel_sub_create("cmd_vel")
        self.ros_tf_pub_create(0.05, 'odom')
        self.ros_nav_create()

    def release(self):
        self.stop_multi_processes()
        write_log("Release Control Core", self)
        # self.StopThreadPlanner()

    def init_multi_processes(self):
        try:
            # main process
            appProc = mp.current_process()
            pid = appProc.pid
            self._dictProc['MAIN'] = pid

        except:
            pass

        try:
            self._mpStellaB2 = CMultiProcessBase('Stella B2', MPMobileRobot.proc_mobile_robot, False, True)
            self._mpStellaB2.sig_queue_bcast.connect(self.on_stellab2_queue_bcast)
            self._mpStellaB2.sig_error.connect(self.on_mp_error)
            self._mpStellaB2.start()
            self._dictProc[self._mpStellaB2.NAME] = self._mpStellaB2.PID

            self._mpKobuki2 = CMultiProcessBase('Kobuki2', MPMobileRobot.proc_mobile_robot, False, True)
            self._mpKobuki2.sig_queue_bcast.connect(self.on_kobuki2_queue_bcast)
            self._mpKobuki2.sig_error.connect(self.on_mp_error)
            self._mpKobuki2.start()
            self._dictProc[self._mpKobuki2.NAME] = self._mpKobuki2.PID

            self._mpExaRobot = CMultiProcessBase('ExaRobot', MPMobileRobot.proc_mobile_robot, False, True)
            self._mpExaRobot.sig_queue_bcast.connect(self.on_exarobot_queue_bcast)
            self._mpExaRobot.sig_error.connect(self.on_mp_error)
            self._mpExaRobot.start()
            self._dictProc[self._mpExaRobot.NAME] = self._mpExaRobot.PID

            self._mpExternalComm = CMultiProcessBase('ExternalComm', MPExternalComm.proc_external_comm, False, True)
            self._mpExternalComm.sig_queue_bcast.connect(self.on_external_comm_queue_bcast)
            self._mpExternalComm.sig_error.connect(self.on_mp_error)
            self._mpExternalComm.start()
            self._dictProc[self._mpExternalComm.NAME] = self._mpExternalComm.PID

            if is_ros_installed():
                self._mpROSCmdVelPub = CMultiProcessBase('ROS Cmd Vel Pub', MPROSVelPub.proc_ros_vel_node, False, True)
                self._mpROSCmdVelPub.sig_queue_bcast.connect(self.on_ros_cmd_vel_pub)
                self._mpROSCmdVelPub.sig_error.connect(self.on_mp_error)
                self._mpROSCmdVelPub.start()
                self._dictProc[self._mpROSCmdVelPub.NAME] = self._mpROSCmdVelPub.PID

                self._mpROSCmdVelSub = CMultiProcessBase('ROS Cmd Vel Sub', MPROSVelSub.proc_ros_vel_node, False, True)
                self._mpROSCmdVelSub.sig_queue_bcast.connect(self.on_ros_cmd_vel_sub)
                self._mpROSCmdVelSub.sig_error.connect(self.on_mp_error)
                self._mpROSCmdVelSub.start()
                self._dictProc[self._mpROSCmdVelSub.NAME] = self._mpROSCmdVelSub.PID

                self._mpROSTfPub = CMultiProcessBase('ROS Tf Pub', MPROSTfPub.proc_ros_tf_node, False, True)
                self._mpROSTfPub.sig_error.connect(self.on_mp_error)
                self._mpROSTfPub.start()
                self._dictProc[self._mpROSTfPub.NAME] = self._mpROSTfPub.PID

                self._mpROSNavigation = CMultiProcessBase('ROS Nav', MPROSNavigation.proc_ros_navigation, False, True)
                self._mpROSNavigation.sig_error.connect(self.on_mp_error)
                self._mpROSNavigation.start()
                self._dictProc[self._mpROSNavigation.NAME] = self._mpROSNavigation.PID

        except:
            pass

    def get_proc_info(self):
        self.sig_proc_monitor_pid.emit(self._dictProc)

    def stop_multi_processes(self):
        try:
            self.stellab2_release()
            self._mpStellaB2.release()
            self.kobuki2_release()
            self._mpKobuki2.release()
            self.exarobot_release()
            self._mpExaRobot.release()
            self.external_comm_release()
            self._mpExternalComm.release()

            if is_ros_installed():
                self.ros_vel_pub_release()
                self._mpROSCmdVelPub.release()
                self.ros_vel_sub_release()
                self._mpROSCmdVelSub.release()
                self.ros_tf_pub_release()
                self._mpROSTfPub.release()
                self.ros_nav_terminate()
                self._mpROSNavigation.release()
        except:
            pass

    ### Kobuki2 ####
    def send_cmd_kobuki2(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        self._mpKobuki2.send_command(dictCmd)

    def kobuki2_create(self):
        self.send_cmd_kobuki2('CREATE', 'KOBUKI2')

    def kobuki2_release(self):
        self.send_cmd_kobuki2('RELEASE')

    def kobuki2_connect(self, strComPort: str):
        self.send_cmd_kobuki2('CONNECT', strComPort)
        self.ros_nav_launch()
        self.ros_tf_pub_start_timer()

    def kobuki2_disconnect(self):
        self.send_cmd_kobuki2('DISCONNECT')
        self.ros_tf_pub_stop_timer()
        self.ros_nav_terminate()

    def kobuki2_reset_motor(self):  # send sound sequence 1
        self.send_cmd_kobuki2('RESET_MOTOR')

    def kobuki2_factory_reset(self):  # send sound sequence 0
        self.send_cmd_kobuki2('INIT_MOTOR')

    # todo: make this extendable to other robots
    def kobuki2_move_joint_space(self, right_vel: int, left_vel: int):
        self.send_cmd_kobuki2("MOVE_JOINT_SPACE", (right_vel, left_vel))

    def kobuki2_set_velocity_control(self, vel: float, ang_vel: float):
        self.send_cmd_kobuki2("SET_VELOCITY_CONTROL", (vel, ang_vel))

    def kobuki2_stop_robot(self, stop_type: int):
        self.send_cmd_kobuki2("STOP", int(stop_type))

    def kobuki2_get_pos(self):
        self.send_cmd_kobuki2("POS")

    def on_kobuki2_queue_bcast(self, msg: dict):
        try:
            if "VERSION" in msg.keys():
                version = msg['VERSION']
                self.sig_kobuki2_get_ver.emit(version)

            elif "HW_VERSION" in msg.keys():
                version = msg['HW_VERSION']
                self.sig_kobuki2_get_hw_ver.emit(version)

            elif "COMM_STATUS" in msg.keys():
                comm_status = msg['COMM_STATUS']
                self.sig_kobuki2_comm_status.emit(comm_status)

            elif "ENCODER" in msg.keys():
                right, left = msg["ENCODER"]
                self.sig_kobuki2_get_enc.emit(int(right), int(left))

            elif "VELOCITY" in msg.keys():
                right, left = msg["VELOCITY"]
                self.sig_kobuki2_get_vel.emit(float(right), float(left))

            elif "WHEEL_POSITION" in msg.keys():
                right, left = msg["WHEEL_POSITION"]
                self.sig_kobuki2_get_wheel_pos.emit(float(right), float(left))

            elif "STATUS" in msg.keys():
                state = msg["STATUS"]
                self.sig_kobuki2_get_state.emit(int(state))

            elif "POSITION" in msg.keys():
                x, y, theta = msg["POSITION"]

                if is_ros_installed():
                    self.ros_tf_pub_update_odom(x, y, theta, 0., 0.)
            else:
                pass

        except:
            pass

    ### ROS CMD_VEL Subscriber ####
    def send_cmd_ros_vel_sub(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        if is_ros_installed():
            self._mpROSCmdVelSub.send_command(dictCmd)

    def ros_vel_sub_create(self, strTopicName: str):
        createParam = strTopicName
        self.send_cmd_ros_vel_sub("CREATE", createParam)

    def ros_vel_sub_destroy(self):
        self.send_cmd_ros_vel_sub("DESTROY")

    def ros_vel_sub_release(self):
        self.send_cmd_ros_vel_sub("RELEASE")

    def on_ros_cmd_vel_sub(self, msg: dict):
        if "VEL_CMD" in msg.keys():
            vel, ang_vel = msg["VEL_CMD"]
            self.stellab2_set_velocity_control(float(vel), float(ang_vel))
            self.kobuki2_set_velocity_control(float(vel), float(ang_vel))
        else:
            pass

    ### ROS CMD_VEL Publisher ####
    def send_cmd_ros_vel_pub(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        if is_ros_installed():
            self._mpROSCmdVelPub.send_command(dictCmd)

    # todo: fPeriod should be a paramater of start_timer
    def ros_vel_pub_create(self, fPeriod: float, strTopicName: str):
        createParam = (fPeriod, strTopicName)
        self.send_cmd_ros_vel_pub("CREATE", createParam)

    def ros_vel_pub_start_timer(self):
        self.send_cmd_ros_vel_pub("START_TIMER")

    def ros_vel_pub_stop_timer(self):
        self.send_cmd_ros_vel_pub("STOP_TIMER")

    def ros_vel_pub_clear(self):
        self.send_cmd_ros_vel_pub("CLEAR")

    def ros_vel_pub_add_vel(self, velCmds: Tuple[List[float], List[float]]):
        self.send_cmd_ros_vel_pub("ADD_VEL_CMD", velCmds)

    def ros_vel_pub_reset_vel(self):
        self.send_cmd_ros_vel_pub("RESET_VEL_CMD")

    def ros_vel_pub_reg_vel(self, velCmds: Tuple[List[float], List[float]]):
        self.send_cmd_ros_vel_pub("REG_VEL_CMD", velCmds)

    def ros_vel_pub_destroy(self):
        self.send_cmd_ros_vel_pub("DESTROY")

    def ros_vel_pub_release(self):
        self.send_cmd_ros_vel_pub("RELEASE")

    def on_ros_cmd_vel_pub(self, msg: str):
        if msg == "LAST_CMD":
            self.ros_vel_pub_stop_timer()
            self.ros_vel_pub_reset_vel()

    ### ExaRobot ####
    def send_cmd_exarobot(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        self._mpExaRobot.send_command(dictCmd)

    def exarobot_create(self):
        self.send_cmd_exarobot('CREATE', 'EXAROBOT')

    def exarobot_release(self):
        self.send_cmd_exarobot('RELEASE')

    def exarobot_connect(self, strComPort: str):
        self.send_cmd_exarobot('CONNECT', strComPort)
        # self.ros_vel_pub_create(0.05, "cmd_vel")
        # self.ros_vel_sub_create("cmd_vel")
        # self.ros_tf_pub_create(0.05, 'odom')
        # self.ros_tf_pub_start_timer()
        # self.ros_nav_launch()

    def exarobot_disconnect(self):
        self.send_cmd_exarobot('DISCONNECT')
        # self.ros_tf_pub_stop_timer()
        # self.ros_tf_pub_release()
        # self.ros_vel_pub_release()
        # self.ros_vel_sub_release()
        # self.ros_nav_terminate()

    def exarobot_reset_motor(self):  # send sound sequence 1
        self.send_cmd_exarobot('RESET_MOTOR')

    def exarobot_factory_reset(self):  # send sound sequence 0
        self.send_cmd_exarobot('INIT_MOTOR')

    # todo: make this extendable to other robots
    def exarobot_move_joint_space(self, right_vel: int, left_vel: int):
        self.send_cmd_exarobot("MOVE_JOINT_SPACE", (right_vel, left_vel))

    def exarobot_set_velocity_control(self, vel: float, ang_vel: float):
        self.send_cmd_exarobot("SET_VELOCITY_CONTROL", (vel, ang_vel))

    def exarobot_stop_robot(self, stop_type: int):
        self.send_cmd_exarobot("STOP", int(stop_type))

    def exarobot_get_pos(self):
        self.send_cmd_exarobot("POS")

    def on_exarobot_queue_bcast(self, msg: dict):
        try:
            if "COMM_STATUS" in msg.keys():
                comm_status = msg['COMM_STATUS']
                self.sig_exarobot_comm_status.emit(comm_status)

            elif "ENCODER" in msg.keys():
                right, left = msg["ENCODER"]
                self.sig_exarobot_get_enc.emit(int(right), int(left))

            elif "VELOCITY" in msg.keys():
                right, left = msg["VELOCITY"]
                self.sig_exarobot_get_vel.emit(float(right), float(left))

            elif "WHEEL_POSITION" in msg.keys():
                right, left = msg["WHEEL_POSITION"]
                self.sig_exarobot_get_wheel_pos.emit(float(right), float(left))

            elif "STATUS" in msg.keys():
                state = msg["STATUS"]
                self.sig_exarobot_get_state.emit(int(state))

            elif "POSITION" in msg.keys():
                x, y, theta = msg["POSITION"]
                # print('ControlCore', x,y,theta)

                if is_ros_installed():
                    self.ros_tf_pub_update_odom(x, y, theta, 0., 0.)
            else:
                pass

        except:
            pass

    ### STELLA B2 ####
    def send_cmd_stellab2(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        self._mpStellaB2.send_command(dictCmd)

    def stellab2_create(self):
        self.send_cmd_stellab2('CREATE', 'STELLAB2')

    def stellab2_release(self):
        self.send_cmd_stellab2('RELEASE')

    def stellab2_connect(self, strComPort: str):
        self.send_cmd_stellab2('CONNECT', strComPort)

    def stellab2_disconnect(self):
        self.send_cmd_stellab2('DISCONNECT')

    def stellab2_set_monitoring(self, state: bool):
        self.send_cmd_stellab2('MONITORING', bool(state))

    def stellab2_reset_motor(self):
        self.send_cmd_stellab2('RESET_MOTOR')

    def stellab2_factory_reset(self):
        self.send_cmd_stellab2('INIT_MOTOR')

    # todo: make this extendable to other robots
    def stellab2_move_joint_space(self, right_vel: int, left_vel: int):
        self.send_cmd_stellab2("MOVE_JOINT_SPACE", (right_vel, left_vel))

    def stellab2_set_velocity_control(self, vel: float, ang_vel: float):
        self.send_cmd_stellab2("SET_VELOCITY_CONTROL", (vel, ang_vel))

    def stellab2_stop_robot(self, stop_type: int):
        self.send_cmd_stellab2("STOP", int(stop_type))

    def on_stellab2_queue_bcast(self, msg: dict):
        try:
            if "VERSION" in msg.keys():
                version = msg['VERSION']
                self.sig_stellab2_get_ver.emit(version)

            elif "COMM_STATUS" in msg.keys():
                comm_status = msg['COMM_STATUS']
                self.sig_stellab2_comm_status.emit(comm_status)

            elif "MONITORING" in msg.keys():
                monitoring_state = msg["MONITORING"]
                self.sig_stellab2_is_monitoring.emit(monitoring_state)

            elif "ENCODER" in msg.keys():
                right, left = msg["ENCODER"]
                self.sig_stellab2_get_enc.emit(int(right), int(left))

            elif "VELOCITY" in msg.keys():
                right, left = msg["VELOCITY"]
                self.sig_stellab2_get_vel.emit(float(right), float(left))

            elif "WHEEL_POSITION" in msg.keys():
                right, left = msg["WHEEL_POSITION"]
                self.sig_stellab2_get_wheel_pos.emit(float(right), float(left))

            elif "STATUS" in msg.keys():
                state = msg["STATUS"]
                self.sig_stellab2_get_state.emit(int(state))

            else:
                pass

        except:
            pass

    def on_mp_error(self, msg: dict):
        pass

    ### External Communication ####
    def send_external_comm(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        self._mpExternalComm.send_command(dictCmd)

    def external_comm_create(self):
        self.send_external_comm('CREATE')

    def external_comm_start(self, bSckType: str, strAddr: str, nPort: int):
        # todo: add UDP later. socket will be determined by bScktype
        self.send_external_comm('START', (strAddr, nPort))

    def external_comm_stop(self):
        self.send_external_comm('STOP')

    def external_comm_send_raw(self, strMsg: str):
        self.send_external_comm('SEND_RAW', strMsg)

    def external_comm_disconnect_client(self, tplAddrPort: tuple):
        self.send_external_comm('REMOVE_CLIENT', tplAddrPort)

    def external_comm_release(self):
        self.send_external_comm('RELEASE')

    def on_external_comm_queue_bcast(self, msg: dict):
        try:
            if "SERVER_STATUS" in msg.keys():
                server_status = msg["SERVER_STATUS"]
                self.sig_external_comm_server_status.emit(server_status)

            elif "CLIENT_CONNECT" in msg.keys():
                tplAddrPort = msg["CLIENT_CONNECT"]
                self.sig_external_comm_client_connected.emit(tplAddrPort)

            elif "CLIENT_DISCONNECT" in msg.keys():
                tplAddrPort = msg["CLIENT_DISCONNECT"]
                self.sig_external_comm_client_disconnected.emit(tplAddrPort)

            elif "RECV_RAW" in msg.keys():
                strMsg = msg["RECV_RAW"]

            elif "EMG_STOP" in msg.keys():
                print("EMG STOP")

            elif "MOVE_POSE" in msg.keys():
                xPos, yPos, aAngle = msg["EMG_STOP"]
                print(xPos, yPos, aAngle)

            else:
                pass

        except:
            pass

    ### TTS ###
    def set_tts_msg(self, strMsg: str):
        self._ttsHandler.SetTTSMessage(strMsg)

    ### STT ##
    def start_stt_listen(self, strMic: str):
        self._sttHandler.start_listen(strMic)



    ### Camera ###
    def onCamStart(self, nId: int, nWidth: int, nHeight: int):
        if self._camTest is None:
            self._camTest = CCameraCommon(nId, nWidth, nHeight)
            self._camTest.sig_image_data.connect(self.on_recv_image)
            self._camTest.sig_stopped.connect(self.on_stopped_cam)
            self._camTest.start_stream()

    def stopCam(self):
        if self._camTest is not None:
            self._camTest.stop_stream()

    def on_recv_image(self, data: tuple):
        self.sig_cam_recv_stream.emit(data)

    def on_stopped_cam(self):
        if self._camTest is not None:
            del self._camTest
            self._camTest = None

        self.sig_cam_stopped_stream.emit()

    ### ROS TF PUB###
    def send_cmd_ros_tf_pub(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        if is_ros_installed():
            self._mpROSTfPub.send_command(dictCmd)

    def ros_tf_pub_create(self, fPeriod: float, strTopicName: str):
        createParam = (fPeriod, strTopicName)
        self.send_cmd_ros_tf_pub("CREATE", createParam)

    def ros_tf_pub_start_timer(self):
        self.send_cmd_ros_tf_pub("START_TIMER")

    def ros_tf_pub_stop_timer(self):
        self.send_cmd_ros_tf_pub("STOP_TIMER")

    def ros_tf_pub_release(self):
        self.send_cmd_ros_tf_pub("RELEASE")

    def ros_tf_pub_update_odom(self, fPos_X, fPos_Y, fPos_Theta, fVel_c, fOmega_c):
        odom = (fPos_X, fPos_Y, fPos_Theta, fVel_c, fOmega_c)
        self.send_cmd_ros_tf_pub("UPDATE_ODOM", odom)

    ### ROS Navigation ####
    def send_cmd_ros_nav(self, acmd: str, aparams: object = None):
        aCmd = str(acmd)
        dictCmd = dict()
        dictCmd["CMD"] = aCmd
        dictCmd["VAL"] = aparams
        if is_ros_installed():
            self._mpROSNavigation.send_command(dictCmd)

    def ros_nav_create(self):
        self.send_cmd_ros_nav('CREATE')

    def ros_nav_terminate(self):
        self.send_cmd_ros_nav('TERMINATE')

    def ros_nav_launch(self):
        self.send_cmd_ros_nav('LAUNCH')

    ### ROS Global Path Planner Simulation ####
    # def startPathPlanningSim(self, planner_type: str, start_pos: Tuple[int, int], goal_pos: Tuple[int, int]):
    #     type = planner_type
    #
    #     if type == "AStar":
    #         my_planner = AStar(start_pos, goal_pos, "euclidean")
    #         planner_name = "A*"
    #
    #     elif type == "Dijkstra":
    #         my_planner = Dijkstra(start_pos, goal_pos, 'None')
    #         planner_name = "Dijkstra's"
    #
    #     plannerDet = (my_planner, planner_name, start_pos, goal_pos)
    #     self.queuePlotter.put(plannerDet)

    # def StartThreadPlanner(self):
    #     if self.threadingPlotter is None:
    #         self.queuePlotter = queue.Queue()
    #         self.threadingPlotter = ThreadPlotter(self.queuePlotter)
    #         self.threadingPlotter.sig_terminated.connect(self.OnThreadPlannerTerminated)
    #         self.threadingPlotter.start()
    #
    # def StopThreadPlanner(self):
    #     if self.threadingPlotter is not None:
    #         self.threadingPlotter.stop()
    #
    # def OnThreadPlannerTerminated(self):
    #     if self.threadingPlotter is not None:
    #         del self.threadingPlotter
    #         self.threadingPlotter = None
