# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : CameraCommon.py
# Project Name : ExaRobotCtrl
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2022.03.03 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import sys
import cv2

import time
import queue
import threading
from typing import List, Union

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
LIBRARY_PATH = os.path.join(ROOT_PATH, "Library")
SERIAL_PATH = os.path.join(LIBRARY_PATH, "Serial")
sys.path.extend([INCLUDE_PATH, RESOURCES_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH

from Commons import PySignal, is_system_win

COMMON_RESOLUTION_D = {"Default": (None, None), "800x600": (800, 600), "1024x768": (1024, 768), "1152x864": (1152, 864),
                       "1200x960": (1200, 960), "1280x600": (1280, 600), "1280x720": (1280, 720),
                       "1280x768": (1280, 768), "1280x800": (1280, 800), "1280x960": (1280, 960),
                       "1280x1024": (1280, 1024), "1360x768": (1360, 768), "1366x768": (1366, 768),
                       "1400x1050": (1400, 1050), "1440x900": (1440, 900), "1600x900": (1600, 900),
                       "1600x1200": (1600, 1200), "1680x1050": (1680, 1050), "1792x1344": (1792, 1344),
                       "1856x1392": (1856, 1392), "1920x1080": (1920, 1080), "1920x1200": (1920, 1200)}

class CCameraCommon:
    nWidth: int
    nHeight: int
    nCamID: int

    threadCamFeed: threading.Thread = None

    def __init__(self, anCamID: int, anWidth: int = 640, anHeight=480):
        self.nWidth = anWidth
        self.nHeight = anHeight
        self.nCamID = anCamID
        self.sig_image_data = PySignal(tuple)
        self.sig_stopped = PySignal()

    def grab(self):
        pass

    def start_stream(self):
        self.start_threading_camfeed()

    def stop_stream(self):
        self.stop_threading_camfeed()

    def pause_stream(self):
        pass

    def start_threading_camfeed(self):
        if self.threadCamFeed is None:
            self.threadCamFeed = ThreadingCamFeed(self)
            self.threadCamFeed.sig_image_data.connect(self.onRecvImageData)
            self.threadCamFeed.sig_terminated.connect(self.onTerminated)
            self.threadCamFeed.start()

    def stop_threading_camfeed(self):
        if self.threadCamFeed is not None:
            self.threadCamFeed.stop()

    def onRecvImageData(self, image: tuple):
        self.sig_image_data.emit(image)

    def onTerminated(self):
        del self.threadCamFeed
        self.threadCamFeed = None

        self.sig_stopped.emit()

    @property
    def CAM_ID(self) -> int:
        return self.nCamID

    @CAM_ID.setter
    def CAM_ID(self, anCamID: int):
        self.nCamID = anCamID


class ThreadingCamFeed(threading.Thread):
    keepAlive: bool = True

    def __init__(self, c: CCameraCommon):
        super(ThreadingCamFeed, self).__init__()
        self.setDaemon(True)
        self.sig_image_data = PySignal(tuple)
        self.sig_terminated = PySignal()
        self._cam = c

    def run(self) -> None:
        vid_capture = cv2.VideoCapture(self._cam.CAM_ID)

        if self._cam.nWidth is not None and self._cam.nHeight is not None:
            vid_capture.set(3, self._cam.nWidth)
            vid_capture.set(4, self._cam.nHeight)

        while self.keepAlive:
            ret, frame = vid_capture.read()
            if ret:
                image_to_send = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image_to_send = cv2.flip(image_to_send, 1)
                self.sig_image_data.emit((image_to_send,))

            time.sleep(0.1)

        vid_capture.release()
        self.sig_terminated.emit()

    def stop(self):
        self.keepAlive = False


if __name__ == "__main__":
    ex_cam = CCameraCommon(1, 1920, 1080)
    ex_cam.start_threading_camfeed()
    time.sleep(10)
