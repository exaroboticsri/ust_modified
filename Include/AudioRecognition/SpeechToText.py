# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : SpeechToText.py
# Project Name : AudioRecognition
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
import speech_recognition as sr

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


class CSpeechToText:
    _sttEngine: sr.Recognizer
    _sttMic: sr.Microphone
    _threadSTT: threading.Thread = None

    def __init__(self):
        super(CSpeechToText, self).__init__()
        self.sig_stt_msg = PySignal(str)
        self.initAudio()
        self.start_thread_stt()

    def initAudio(self):
        self._sttEngine = sr.Recognizer()

    def get_mic_list(self) -> list:
        return sr.Microphone.list_microphone_names()

    def select_mic(self, nMicIndex: int):
        self._sttMic = sr.Microphone(nMicIndex)

    def listen_mic(self) -> str:
        with self._sttMic as source:
            st = time.time()
            self._sttEngine.adjust_for_ambient_noise(source, duration=0.5)
            audio = self._sttEngine.listen(source, phrase_time_limit=5)
            print(time.time() - st)

        try:
            sst = self._sttEngine.recognize_google(audio, language='ko-KR')

        except sr.RequestError:
            # API was unreachable or unresponsive
            sst = "API unavailable"

        except sr.UnknownValueError:
            # speech was unintelligible
            sst = "Unable to recognize speech"

        return sst

    def start_listen(self, strMic: str):
        try:
            nIndex = self.get_mic_list().index(strMic)
        except:
            nIndex = 0

        self.select_mic(nIndex)
        self.trigger_thread_stt()

    def start_thread_stt(self):
        if self._threadSTT is None:
            self._threadSTT = ThreadSpeechToText(self)
            self._threadSTT.sig_is_terminated.connect(self.on_thread_stt_terminated)
            self._threadSTT.sig_is_done.connect(self.on_thread_stt_done)
            self._threadSTT.start()

    def stop_thread_stt(self):
        if self._threadSTT is not None:
            self._threadSTT.stop()

    def trigger_thread_stt(self):
        if self._threadSTT is not None:
            self._threadSTT.trigger()

    def on_thread_stt_done(self, strMsg: str):
        self.sig_stt_msg.emit(strMsg)

    def on_thread_stt_terminated(self):
        del self._threadSTT
        self._threadSTT = None


class ThreadSpeechToText(threading.Thread):
    keepAlive: bool = True

    def __init__(self, handle: CSpeechToText):
        super(ThreadSpeechToText, self).__init__()
        self._engine = handle
        self.setDaemon(True)
        self.sig_is_done = PySignal(str)
        self.sig_is_terminated = PySignal()
        self._waitEv = threading.Event()

    def trigger(self):
        print("HERE")
        self._waitEv.set()

    def run(self) -> None:
        while self.keepAlive:
            self._waitEv.wait()
            sst = self._engine.listen_mic()
            self.sig_is_done.emit(sst)
            self._waitEv.clear()

        self.sig_is_terminated.emit()

    def stop(self):
        self.keepAlive = False
        self.trigger()


if __name__ == '__main__':
    sample = CSpeechToText()
