import os
import sys
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from typing import List, Tuple
import time
import sys

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Library/Socket
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
MDI_PATH = os.path.join(ROOT_PATH, "MdiBackground")
DOCK_PATH = os.path.join(ROOT_PATH, "DockWidgets")
MISC_PATH = os.path.join(ROOT_PATH, "Misc")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
sys.path.extend([FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH
from Commons import PySignal
from ROSNodeBase import CROSNodeBase


class CROSNodeVelSub(CROSNodeBase):

    def __init__(self, astrTopicName: str = "cmd_vel"):
        super(CROSNodeVelSub, self).__init__('CMD_VEL_SUB', astrTopicName)
        self.sig_ros_pub_cmd_vel = PySignal(tuple)

        # create subscriber
        self._topic = self.create_subscription(Twist, self.strTopicName, self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg: Twist):
        vel = msg.linear.x
        ang_vel = msg.angular.z
        self.sig_ros_pub_cmd_vel.emit((vel, ang_vel))
