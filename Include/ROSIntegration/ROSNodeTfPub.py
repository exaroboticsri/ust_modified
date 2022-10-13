import os
import sys
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformStamped, TransformBroadcaster

from typing import List, Tuple
import time
import sys
import math

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

class CROSNodeTfPub(CROSNodeBase):
    _odom: Odometry

    def __init__(self, afPeriod: float, astrTopicName: str = "odom"):
        super(CROSNodeTfPub, self).__init__('ODOM_PUB', astrTopicName)
        self._qos = QoSProfile(depth=10)
        self._qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._qos.history = QoSHistoryPolicy.KEEP_LAST
        self._qos.durability = QoSDurabilityPolicy.VOLATILE

        # create publisher
        self._topic = self.create_publisher(Odometry, self.strTopicName, self._qos)

        self.fPeriod = afPeriod  # should be in seconds
        self._odom = Odometry()
        self._odom.header.frame_id = 'odom'
        self._odom.child_frame_id = 'base_link'

        self._TfBroadcaster = TransformBroadcaster(self, 10)
        self._odom_tf = TransformStamped()
        self._odom_tf.header.frame_id = 'odom'
        self._odom_tf.child_frame_id = 'base_link'
        print("CROSNodeTfPub_Init ")

    def start_timer(self):
        self._timer = self.create_timer(self.fPeriod, self.timer_callback)
        print("start_timer ROS Tf Pub Cycle:{}sec".format(self.fPeriod))

    def stop_timer(self):
        self.destroy_timer(self._timer)
        print("stop_timer ROS Tf Pub ")

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """

        odometry = Odometry()
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        odometry.pose.pose.orientation.w = q[0]
        odometry.pose.pose.orientation.x = q[1]
        odometry.pose.pose.orientation.y = q[2]
        odometry.pose.pose.orientation.z = q[3]
        return odometry.pose.pose.orientation
        # return q

    def update_odom_and_tf(self, fPos_x, fPos_y, fPos_theta, fVel_c, fOmega_c):
        # print('ROSNodeTfPub', x, y, theta)
        current_time = self.get_clock().now().to_msg()

        self._odom.pose.pose.position.x = fPos_x
        self._odom.pose.pose.position.y = fPos_y
        self._odom.pose.pose.position.z = 0.0

        self._odom.pose.pose.orientation = self.quaternion_from_euler(0, 0, fPos_theta)
        self._odom.twist.twist.linear.x = fVel_c
        self._odom.twist.twist.linear.y = 0.0
        self._odom.twist.twist.linear.z = 0.0
        self._odom.twist.twist.angular.x = 0.0
        self._odom.twist.twist.angular.y = 0.0
        self._odom.twist.twist.angular.z = fOmega_c
        self._odom.header.stamp = current_time

        self._odom_tf.header.stamp = current_time
        self._odom_tf.transform.translation.x = self._odom.pose.pose.position.x
        self._odom_tf.transform.translation.y = self._odom.pose.pose.position.y
        self._odom_tf.transform.translation.z = self._odom.pose.pose.position.z
        self._odom_tf.transform.rotation = self._odom.pose.pose.orientation

    def timer_callback(self):
        try:
            self._topic.publish(self._odom)
            self._TfBroadcaster.sendTransform(self._odom_tf)
        except IndexError:
            # reset when the timer is not destroyed outside
            pass

