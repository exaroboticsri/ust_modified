# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MPROSVelPub.py
# Project Name : ExaRobotCtrl
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2022.02.28 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import multiprocessing as mp
import os
import sys
import threading
from typing import Tuple
from multiprocessing import Queue
import rclpy
from rclpy.executors import SingleThreadedExecutor

# from multiprocessing.connection import PipeConnection
from typing import Union
import time
import psutil

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Library/Socket
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
ROS_PATH = os.path.join(INCLUDE_PATH, "ROSIntegration")
MDI_PATH = os.path.join(INCLUDE_PATH, "MdiBackground")
DOCK_PATH = os.path.join(INCLUDE_PATH, "DockWidgets")
MISC_PATH = os.path.join(INCLUDE_PATH, "Misc")
RESOURCES_PATH = os.path.join(INCLUDE_PATH, "Resources")
MULTIPROCESS_PATH = os.path.join(INCLUDE_PATH, "MultiProcessing")
LIBRARY_PATH = os.path.join(ROOT_PATH, "Library")
SERIAL_PATH = os.path.join(LIBRARY_PATH, "Serial")
sys.path.extend([FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH,
                 MULTIPROCESS_PATH, ROS_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH, MULTIPROCESS_PATH, ROS_PATH

from Commons import PySignal, is_ros_installed
from MultiProcessBase import CMultiProcessBase
from ROSNodeVelPub import CROSNodeVelPub
from ROSNodeBase import CROSNodeTestPub

gROSVelNode = None
# dict: CMD, VALUE
# pipeChild is an instance of PipeConnection on Windows, but this is different on Linux
def proc_ros_vel_node(pipeChild, feedbackQueue: Queue, feedbackQueueBk: Union[Queue, None]):
    global gROSVelNode

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
                    f_dtime, str_topic_name = val
                    rclpy.init(args=None)
                    gROSVelNode = CROSNodeVelPub(f_dtime, str_topic_name)
                    gROSVelNode.sig_vel_last.connect(lambda: on_vel_last_cmd(feedbackQueue))
                    thisExecutor = SingleThreadedExecutor()
                    thisExecutor.add_node(gROSVelNode)
                    thisExecutorThread = threading.Thread(target=thisExecutor.spin, daemon=True)
                    thisExecutorThread.start()

                elif cmd == "START_TIMER":
                    gROSVelNode.start_timer()

                elif cmd == "STOP_TIMER":
                    gROSVelNode.stop_timer()

                elif cmd == "CLEAR":
                    gROSVelNode.clear_vel_cmd()

                elif cmd == "ADD_VEL_CMD":
                    gROSVelNode.add_vel_cmd(val)

                elif cmd == "REG_VEL_CMD":
                    gROSVelNode.reg_vel_cmd(val)

                elif cmd == "RESET_VEL_CMD":
                    gROSVelNode.reset_vel_cmd()

                elif cmd == "DESTROY":
                    gROSVelNode.destroy_node()

                elif cmd == "RELEASE":
                    gROSVelNode.destroy_node()
                    rclpy.shutdown()
                else:
                    pass

                time.sleep(1e-6)

            else:
                time.sleep(1e-6)
        except:
            time.sleep(1e-6)
            pass

        time.sleep(1e-6)
        pass


def on_vel_last_cmd(q: Queue):
    q.put("LAST_CMD")
