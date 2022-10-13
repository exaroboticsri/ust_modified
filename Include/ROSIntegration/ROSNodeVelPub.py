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


class CROSNodeVelPub(CROSNodeBase):
    _twist: Twist
    _velCmd: List[Tuple[float, float]]
    nDataCnt: int

    def __init__(self, afPeriod: float, astrTopicName: str = "cmd_vel"):
        super(CROSNodeVelPub, self).__init__('CMD_VEL_PUB', astrTopicName)
        self._qos = QoSProfile(depth=10)
        self._qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._qos.history = QoSHistoryPolicy.KEEP_LAST
        self._qos.durability = QoSDurabilityPolicy.VOLATILE

        # create publisher
        self._topic = self.create_publisher(Twist, self.strTopicName, self._qos)
        self.fPeriod = afPeriod  # should be in seconds
        self._twist = Twist()
        self.velCmd = []
        self.sig_vel_last = PySignal()
        self.nDataCnt = 0
        self.nDataMax = 0

    def reset_datamax(self):
        # we assume that the lenght of velocity commands are equal. They should be.
        self.nDataMax = len(self.velCmd[0])

    def start_timer(self):
        self._timer = self.create_timer(self.fPeriod, self.timer_callback)

    def stop_timer(self):
        return self.destroy_timer(self._timer)

    def reset_vel_cmd(self):
        self.nDataCnt = 0

    def clear_vel_cmd(self):
        self.velCmd.clear()
        self.reset_datamax()

    def reg_vel_cmd(self, avelCmd: List[Tuple[float, float]]):
        self.velCmd = avelCmd
        self.reset_datamax()

    def add_vel_cmd(self, avelCmd: List[Tuple[float, float]]):
        self.velCmd.extend(avelCmd)
        self.reset_datamax()

    def timer_callback(self):
        try:
            self._twist.linear.x = float(self.velCmd[0][self.nDataCnt])
            self._twist.angular.z = float(self.velCmd[1][self.nDataCnt])
            self.nDataCnt += 1

            if self.nDataCnt >= self.nDataMax:
                self.sig_vel_last.emit()
            else:
                self._topic.publish(self._twist)

        except IndexError:
            # reset when the timer is not destroyed outside
            self.reset_vel_cmd()
