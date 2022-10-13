# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MainWindow.py
# Project Name : PySerialTerm
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2022.02.19 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import sys
import datetime

__appname__ = "PySerialTerm"
__author__ = u"Raim.Delgado"
__build__ = "2022.02.19"
__status__ = "(Alpha)"
__version__ = "0.0.1" + " " + __status__
__maintainer__ = "Raim.Delgado"
__email__ = "raim223@seoultech.ac.kr"

import time

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
from SerialSendWidget import CWidgetSerialSend
from SerialConfiglWidget import CWidgetSerialConfig
from SerialLogWidget import CWidgetSerialLog
from EmbdLed import EmbdLed


class SubWindowSerialTerm(QtWidgets.QMdiSubWindow):
    _widgetSerialConfig: CWidgetSerialConfig
    _widgetSerialSend: CWidgetSerialSend
    _widgetSerialLogTx: CWidgetSerialLog
    _widgetSerialLogRx: CWidgetSerialLog
    sig_close = QtCore.pyqtSignal()

    def closeEvent(self, closeEvent: QtGui.QCloseEvent) -> None:
        self.sig_close.emit()

    def __init__(self, parent=None):
        super(SubWindowSerialTerm, self).__init__(parent)
        self.setWindowIcon(EmbdQtIcon("serial_port_icon.svg"))
        self.setWindowTitle('Serial Terminal')
        self.initUI()
        self.setMinimumWidth(800)
        self.setMinimumHeight(630)
        self.resize(800, 630)
        self.initSlots()
        self.on_serial_disconnected()

    def initUI(self):
        self.initSerialWidget()
        self.initSerialLogger()

        self.initStatusBar()
        centralWidget = QtWidgets.QWidget(self)
        MainLayout = QtWidgets.QVBoxLayout(centralWidget)
        self.setWidget(centralWidget)

        VLayoutLeft = QtWidgets.QVBoxLayout()
        VLayoutRight = QtWidgets.QVBoxLayout()
        gBoxSerialConfig = QtWidgets.QGroupBox('Serial Configuration')
        VLayoutConfig = QtWidgets.QVBoxLayout(gBoxSerialConfig)
        VLayoutConfig.addWidget(self._widgetSerialConfig)
        VLayoutLeft.addWidget(gBoxSerialConfig)
        VLayoutLeft.addWidget(self._widgetSerialSend, 1)

        VLayoutRight.addWidget(self._widgetSerialLogTx)
        VLayoutRight.addWidget(self._widgetSerialLogRx)
        VLayoutRight.setContentsMargins(0, 0, 0, 0)

        HLayoutLeftRight = QtWidgets.QHBoxLayout()
        HLayoutLeftRight.addLayout(VLayoutLeft)
        HLayoutLeftRight.addLayout(VLayoutRight, 1)

        MainLayout.addLayout(HLayoutLeftRight, 1)
        MainLayout.addWidget(self.statusBar)

    def initSlots(self):
        pass
        # self._core.serialPort.sig_connected.connect(self.on_serial_connected)
        # self._core.serialPort.sig_disconnected.connect(self.on_serial_disconnected)
        # self._core.serialPort.sig_send_data.connect(self.on_serial_send)
        # self._core.serialPort.sig_recv_data.connect(self.on_serial_recv)

    def initSerialWidget(self):
        self._widgetSerialConfig = CWidgetSerialConfig(self)
        # self._widgetSerialConfig.sig_serial_connect.connect(self._core.connect_serial_comm)
        # self._widgetSerialConfig.sig_serial_disconnect.connect(self._core.disconnect_serial_comm)

        self._widgetSerialSend = CWidgetSerialSend(self)
        self._widgetSerialSend.sig_send_serial_packet.connect(self.on_click_btn_serial_packet_send)

    def initSerialLogger(self):
        self._widgetSerialLogTx = CWidgetSerialLog('TX')
        self._widgetSerialLogRx = CWidgetSerialLog('RX')

    def release(self):
        self.close()

    def initStatusBar(self):
        self.statusBar = QtWidgets.QStatusBar(self)
        self.statusBar.setObjectName("statusBar")

        WidgetLED = QtWidgets.QWidget()
        HLayoutLED = QtWidgets.QHBoxLayout(WidgetLED)

        lblStatus = QtWidgets.QLabel()
        lblStatus.setText('Status:')
        self.ledStatus = EmbdLed()

        self.ledStatus.setLedSize(15, 15)
        lblTx = QtWidgets.QLabel()
        lblTx.setText('TX:')
        self.ledTx = EmbdLed()
        self.ledTx.setLedSize(15, 15)
        lblRx = QtWidgets.QLabel()
        lblRx.setText('RX:')
        self.ledRx = EmbdLed()
        self.ledRx.setLedSize(15, 15)

        # HLayoutLED.addLayout(HLayoutStat)
        HLayoutLED.addWidget(lblStatus)
        HLayoutLED.addWidget(self.ledStatus)
        HLayoutLED.addWidget(lblTx)
        HLayoutLED.addWidget(self.ledTx)
        HLayoutLED.addWidget(lblRx)
        HLayoutLED.addWidget(self.ledRx)
        HLayoutLED.addStretch(1)
        HLayoutLED.setSpacing(0)

        self.statusBar.addWidget(WidgetLED)

        self.lblCurrentDatetime = QtWidgets.QLabel()
        self.lblCurrentDatetime.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.statusBar.addPermanentWidget(self.lblCurrentDatetime)

    def onThreadMonitorUpdateTime(self, cur_time: datetime.datetime):
        self.lblCurrentDatetime.setText(cur_time.strftime('%Y-%m-%d %H:%M:%S'))

    def on_serial_connected(self):
        self.ledStatus.TurnOnG()
        self._widgetSerialConfig.on_serial_connected_state(True)
        self._widgetSerialSend.setEnabled(True)

    def on_serial_disconnected(self):
        self.ledStatus.TurnOff()
        self.ledTx.TurnOff()
        self.ledRx.TurnOff()
        self._widgetSerialSend.setEnabled(False)
        self._widgetSerialConfig.on_serial_connected_state(False)

    def on_serial_recv(self, data: bytes):
        cur_time = datetime.datetime.now()
        str_time = cur_time.strftime('%H:%M:%S')
        self.ledRx.TurnOnR()
        btaData = bytearray([])
        btaData.extend(bytearray(data))
        self._widgetSerialLogRx.add_table_contents(str_time, btaData)
        time.sleep(0.1)
        self.ledRx.TurnOff()

    def on_serial_send(self, data: bytes):
        cur_time = datetime.datetime.now()
        str_time = cur_time.strftime('%H:%M:%S')
        self.ledTx.TurnOnG()
        btaData = bytearray([])
        btaData.extend(bytearray(data))
        self._widgetSerialLogTx.add_table_contents(str_time, btaData)
        time.sleep(0.1)
        self.ledTx.TurnOff()

    def on_click_btn_serial_packet_send(self, msg: bytearray):
        pass
        # self._core.serialPort.sendData(msg)


if __name__ == '__main__':
    app = QtCore.QCoreApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)

    MainUI = SubWindowSerialTerm()
    MainUI.show()

    app.exec_()
