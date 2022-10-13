import os
import sys
import time
import rclpy
from concurrent.futures import ThreadPoolExecutor
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.executors import Executor
from rclpy.timer import Timer
from typing import Union

from multiprocessing import Process

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


class CROSNodeBase(Node):
    strName: str
    strTopicName: str
    fPeriod: float
    _qos: QoSProfile
    _topic: Union[Publisher, Subscription]
    _timer: Timer

    def __init__(self, astrNodeName: str, astrTopicName: str):
        super(CROSNodeBase, self).__init__(astrNodeName)
        self.strName = astrNodeName
        self.strTopicName = astrTopicName

    def timer_callback(self):
        pass


from std_msgs.msg import String


class CROSNodeTestPub(CROSNodeBase):
    def __init__(self, afPeriod: float, astrTopicName: str = "test_node"):
        super(CROSNodeTestPub, self).__init__("MY_TEST_PUB", astrTopicName)
        self.sig_end_execution = PySignal()

        self._topic = self.create_publisher(String, self.strTopicName, 10)
        self.fPeriod = afPeriod
        # self.create_timer_re()
        # self.start_timer()
        self.nCnt = 0

    def stop_timer(self):
        self.destroy_timer(self._timer)

    def start_timer(self):
        self._timer = self.create_timer(self.fPeriod, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.nCnt
        self._topic.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.nCnt += 1

        if self.nCnt == 10:
            self.sig_end_execution.emit()


class CROSNodeTestExecutor(Executor):

    def __init__(self):
        super(CROSNodeTestExecutor, self).__init__()
        self.high_priority_nodes = set()
        self.hp_executor = ThreadPoolExecutor(max_workers=os.cpu_count() or 4)
        self.lp_executor = ThreadPoolExecutor(max_workers=1)

    def add_high_priority_node(self, node):
        self.high_priority_nodes.add(node)
        # add_node inherited
        self.add_node(node)

    def spin_once(self, timeout_sec=None):
        """
        Execute a single callback, then return.
        This is the only function which must be overridden by a custom executor. Its job is to
        start executing one callback, then return. It uses the method `wait_for_ready_callbacks`
        to get work to execute.
        :param timeout_sec: Seconds to wait. Block forever if None. Don't wait if <= 0
        :type timeout_sec: float or None
        """
        # wait_for_ready_callbacks yields callbacks that are ready to be executed
        try:
            handler, group, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
        except StopIteration:
            pass
        else:
            if node in self.high_priority_nodes:
                self.hp_executor.submit(handler)
            else:
                self.lp_executor.submit(handler)