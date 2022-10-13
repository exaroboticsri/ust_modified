# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : TextToSpeech.py
# Project Name : ExaRobotCtrl
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2022.04.26 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import queue
import sys
import datetime
import threading
import time
import pyttsx3

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include/SubWindow
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
SCRIPTS_PATH = os.path.join(ROOT_PATH, "Include")
SUBWIN_PATH = os.path.join(SCRIPTS_PATH, "SubWindow")
WIDGETS_PATH = os.path.join(SUBWIN_PATH, "SerialTerm")
MISC_PATH = os.path.join(SCRIPTS_PATH, "Misc")
sys.path.extend([SCRIPTS_PATH, RESOURCES_PATH, WIDGETS_PATH, MISC_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, SCRIPTS_PATH, RESOURCES_PATH, WIDGETS_PATH, MISC_PATH

from UiCommon import *
from Commons import *


class ThreadTextToSpeech(threading.Thread):
    keepAlive: bool = True

    def __init__(self, engine: pyttsx3.Engine, q: queue.Queue):
        super(ThreadTextToSpeech, self).__init__()
        self._queue = q
        self._engine = engine
        self.setDaemon(True)
        self.sig_is_done = PySignal()

    def run(self) -> None:
        while self.keepAlive:
            if not self._queue.empty():
                msg = self._queue.get()
                self._engine.say(msg)
            else:
                self._engine.iterate()
                time.sleep(1e-6)

        self._engine.endLoop()
        self.sig_is_done.emit()

    def stop(self):
        self.keepAlive = False


class CTextToSpeech:
    _ttsEngine: pyttsx3.Engine
    _queue: queue.Queue
    _threadTTS: ThreadTextToSpeech = None

    def __init__(self):
        super(CTextToSpeech, self).__init__()
        self._ttsEngine = pyttsx3.init()
        self._ttsEngine.startLoop(False)
        self._ttsEngine.setProperty("rate", 130)
        self._queue = queue.Queue()
        self.startThreadTTS()

    def SetTTSMessage(self, strMsg: str):
        self._queue.put(strMsg)

    def startThreadTTS(self):
        if self._threadTTS is None:
            self._threadTTS = ThreadTextToSpeech(self._ttsEngine, self._queue)
            self._threadTTS.sig_is_done.connect(self.doneThreadTTS)
            self._threadTTS.start()
            # self._threadTTS.join()

    def stopThreadTTS(self):
        if self._threadTTS is not None:
            self._threadTTS.stop()

    def doneThreadTTS(self):
        self._ttsEngine = pyttsx3.init()
        self._ttsEngine.startLoop(False)

        del self._threadTTS
        self._threadTTS = None