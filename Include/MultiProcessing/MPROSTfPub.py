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
from rclpy.executors import SingleThreadedExecutor
# from multiprocessing.connection import PipeConnection
from typing import Union
import time

import psutil
import rclpy

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

from Commons import PySignal
from ROSNodeTfPub import CROSNodeTfPub


gROSTfNode = None
# pipeChild is an instance of PipeConnection on Windows, but this is different on Linux
def proc_ros_tf_node(pipeChild, feedbackQueue: Queue, feedbackQueueBk: Union[Queue, None]):
    global gROSTfNode

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
                    print('MP TF PUB CREATE')
                    f_dtime, str_topic_name = val
                    rclpy.init(args=None)
                    gROSTfNode = CROSNodeTfPub(f_dtime, str_topic_name)
                    thisExecutor = SingleThreadedExecutor()
                    thisExecutor.add_node(gROSTfNode)
                    thisExecutorThread = threading.Thread(target=thisExecutor.spin, daemon=True)
                    thisExecutorThread.start()

                elif cmd == "START_TIMER":
                    print('MP TF PUB START TIMER')
                    gROSTfNode.start_timer()

                elif cmd == "STOP_TIMER":
                    print('MP TF PUB STOP TIMER')
                    gROSTfNode.stop_timer()

                elif cmd == "UPDATE_ODOM":
                    x, y, theta, vel_c, omega_c = val
                    # print('MPROSTfPub', x, y, theta)
                    gROSTfNode.update_odom_and_tf(x, y, theta, vel_c, omega_c)

                elif cmd == "RELEASE":
                    gROSTfNode.destroy_node()
                    rclpy.shutdown()
                else:
                    pass

                time.sleep(1e-3)

            else:
                time.sleep(1e-3)
        except:
            time.sleep(1e-6)
            pass

        time.sleep(1e-6)
        pass
