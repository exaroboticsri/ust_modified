# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MainWindow.py
# Project Name : PySerialTerm
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2022.04.26 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import sys
import datetime
import time
import queue
import pyttsx3

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include/SubWindow
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
STT_PATH = os.path.join(INCLUDE_PATH, "AudioRecognition")
SUBWIN_PATH = os.path.join(INCLUDE_PATH, "SubWindow")
WIDGETS_PATH = os.path.join(SUBWIN_PATH, "SerialTerm")
MISC_PATH = os.path.join(INCLUDE_PATH, "Misc")
sys.path.extend([INCLUDE_PATH, RESOURCES_PATH, WIDGETS_PATH, MISC_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, RESOURCES_PATH, WIDGETS_PATH, MISC_PATH

from UiCommon import *
from Commons import *
from SpeechToText import CSpeechToText


class SubWindowTTS(QtWidgets.QMdiSubWindow):
    sig_close = QtCore.pyqtSignal()
    sig_text_to_convert = QtCore.pyqtSignal(str)
    sig_listen_sst = QtCore.pyqtSignal(str)
    btnPlayTTS: QtWidgets.QPushButton
    txtTextToConvert: QtWidgets.QLineEdit
    _gBoxTTS: QtWidgets.QGroupBox
    _gBoxSTT: QtWidgets.QGroupBox
    _threadLogWriter: threading.Thread = None

    def closeEvent(self, closeEvent: QtGui.QCloseEvent) -> None:
        self.sig_close.emit()

    def __init__(self, parent=None):
        super(SubWindowTTS, self).__init__(parent)
        self.setWindowIcon(EmbdQtIcon("tts.png"))
        self.setWindowTitle('TTS Test')
        self._queueMsg = queue.Queue()
        self.initUI()
        self.setMinimumWidth(267)
        # self.setMaximumHeight(80)
        # self.resize(267, 80)
        self.initSlots()
        self.on_clicked_btn_refresh()
        self.start_thread_logwriter()

    def initUI(self):
        self.initTTS()
        self.initSTT()
        centralWidget = QtWidgets.QWidget(self)
        MainLayout = QtWidgets.QVBoxLayout(centralWidget)
        self.setWidget(centralWidget)
        MainLayout.addWidget(self._gBoxTTS)
        MainLayout.addWidget(self._gBoxSTT)

    def initTTS(self):
        self._gBoxTTS = QtWidgets.QGroupBox('TextToSpeech (TTS)')
        HlayoutTTS = QtWidgets.QHBoxLayout(self._gBoxTTS)
        self.txtTextToConvert = QtWidgets.QLineEdit()
        self.txtTextToConvert.setFixedHeight(30)
        self.btnPlayTTS = QtWidgets.QPushButton('Play')
        self.btnPlayTTS.setIcon(EmbdQtIcon("play.png"))
        self.btnPlayTTS.setIconSize(QtCore.QSize(15, 15))
        self.btnPlayTTS.setFixedWidth(130)
        self.btnPlayTTS.setFixedHeight(30)
        self.btnClearTTS = QtWidgets.QPushButton()
        self.btnClearTTS.setIcon(EmbdQtIcon("clear2.png"))
        self.btnClearTTS.setFixedWidth(25)
        self.btnClearTTS.setFixedHeight(30)
        HlayoutTTS.addWidget(self.txtTextToConvert, 1)
        HlayoutTTS.addWidget(self.btnPlayTTS)
        HlayoutTTS.addWidget(self.btnClearTTS)
        HlayoutTTS.setSpacing(5)

    def initSTT(self):
        self._gBoxSTT = QtWidgets.QGroupBox('SpeechToText (STT)')
        HLayoutSTT = QtWidgets.QHBoxLayout(self._gBoxSTT)
        self.txtSSTHistory = QtWidgets.QTextEdit()
        self.txtSSTHistory.setReadOnly(True)
        self.btnListenSST = QtWidgets.QPushButton('Listen')

        VLayoutCtrl = QtWidgets.QVBoxLayout()
        HLayoutListen = QtWidgets.QHBoxLayout()
        self.btnListenSST.setIcon(EmbdQtIcon("mic.png"))
        self.btnListenSST.setIconSize(QtCore.QSize(20, 20))
        self.btnListenSST.setFixedWidth(130)
        self.btnListenSST.setFixedHeight(30)
        self.btnClearSST = QtWidgets.QPushButton()
        self.btnClearSST.setIcon(EmbdQtIcon("clear2.png"))
        self.btnClearSST.setFixedWidth(25)
        self.btnClearSST.setFixedHeight(30)
        HLayoutListen.addWidget(self.btnListenSST)
        HLayoutListen.addWidget(self.btnClearSST)
        HLayoutListen.setSpacing(5)

        self.cmbMicList = QtWidgets.QComboBox()
        self.cmbMicList.setFixedWidth(130)

        HLayoutRefresh = QtWidgets.QHBoxLayout()
        self.btnRefresh = QtWidgets.QPushButton()
        self.btnRefresh.setIcon(EmbdQtIcon("reset.png"))
        self.btnRefresh.setFixedWidth(25)
        HLayoutRefresh.addWidget(self.cmbMicList)
        HLayoutRefresh.addWidget(self.btnRefresh)
        VLayoutCtrl.addLayout(HLayoutListen)
        VLayoutCtrl.addLayout(HLayoutRefresh)
        VLayoutCtrl.addStretch(1)

        HLayoutSTT.addWidget(self.txtSSTHistory, 1)
        HLayoutSTT.addLayout(VLayoutCtrl)

    def initSlots(self):
        self.btnPlayTTS.clicked.connect(self.on_clicked_btn_play)
        self.btnRefresh.clicked.connect(self.on_clicked_btn_refresh)
        self.btnListenSST.clicked.connect(self.on_clicked_listen_sst)
        self.btnClearTTS.clicked.connect(lambda: self.on_clicked_clear(self.txtTextToConvert))
        self.btnClearSST.clicked.connect(lambda: self.on_clicked_clear(self.txtSSTHistory))

    def on_clicked_btn_refresh(self):
        self.cmbMicList.clear()
        self.cmbMicList.addItems(CSpeechToText().get_mic_list())

        for i, mic in enumerate(CSpeechToText().get_mic_list()):
            self.cmbMicList.setItemData(i, mic, QtCore.Qt.ToolTipRole)

    def on_clicked_btn_play(self):
        str_to_convert = str(self.txtTextToConvert.text())
        self.sig_text_to_convert.emit(str_to_convert)

    def put_sst_queue(self, strMsg: str):
        self._queueMsg.put(strMsg)

    def add_sst_message(self, strMsg: str, is_newline: bool = True):
        rows = self.txtSSTHistory.document().blockCount()
        if rows >= 100:
            self.txtTcpMonitor.clear()

        cursor = QtGui.QTextCursor(self.txtSSTHistory.textCursor())
        cursor.movePosition(QtGui.QTextCursor.End)
        fmt = QtGui.QTextCharFormat()
        cursor.setCharFormat(fmt)

        if is_newline:
            strMsg = strMsg + '\n'

        cursor.insertText(strMsg)
        vsbar = self.txtSSTHistory.verticalScrollBar()
        vsbar.setValue(vsbar.maximum())

    def start_thread_logwriter(self):
        if self._threadLogWriter is None:
            self._threadLogWriter = ThreadLogWriter(self, self._queueMsg)
            self._threadLogWriter.sig_terminated.connect(self.on_terminate_thread_logwriter)
            self._threadLogWriter.start()

    def stop_thread_logwriter(self):
        if self._threadLogWriter is not None:
            self._threadLogWriter.stop()

    def on_terminate_thread_logwriter(self):
        del self._threadLogWriter
        self._threadLogWriter = None

    def on_clicked_clear(self, txtBox: QtWidgets.QWidget):
        txtBox.clear()

    def on_clicked_listen_sst(self):
        strMic = self.cmbMicList.currentText()
        self.sig_listen_sst.emit(strMic)


class ThreadLogWriter(QtCore.QThread):
    keepAlive = True
    sig_terminated = QtCore.pyqtSignal()
    sig_update_log = QtCore.pyqtSignal(tuple)

    def __init__(self, handle: SubWindowTTS, queue: queue.Queue):
        super(ThreadLogWriter, self).__init__()
        self._queue = queue
        self._handle = handle

    def run(self) -> None:
        write_log('Started', self)

        while self.keepAlive:
            try:
                qMsg = self._queue.get(block=True)
                if qMsg == 'STOP':
                    break
                else:
                    self._handle.add_sst_message(qMsg)

            except:
                pass

        self.sig_terminated.emit()
        write_log('Terminated', self)

    def stop(self):
        self._queue.put('STOP')


if __name__ == '__main__':
    app = QtCore.QCoreApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)

    MainUI = SubWindowTTS()
    MainUI.show()

    app.exec_()
