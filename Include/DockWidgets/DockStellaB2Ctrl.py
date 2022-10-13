# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : DockStellaB2Ctrl.py
# Project Name : ExaRobotCtrl
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2022.02.03 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import sys
from typing import Tuple

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include/DOckWidgets
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
LIBRARY_PATH = os.path.join(ROOT_PATH, "Library")
SERIAL_PATH = os.path.join(LIBRARY_PATH, "Serial")
MDI_PATH = os.path.join(ROOT_PATH, "MdiBackground")
MISC_PATH = os.path.join(ROOT_PATH, "Misc")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
sys.path.extend([INCLUDE_PATH, MDI_PATH, RESOURCES_PATH, SERIAL_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH

from EmbdLed import EmbdLed
from UiCommon import *
from Commons import *
from SerialPort import get_serial_port_name_list


class QTableWidgetFeedback(QtWidgets.QTableWidget):
    listTableHHeader: list
    listTableVHeader: list

    def __init__(self, parent=None):
        super(QTableWidgetFeedback, self).__init__(parent)
        self.listTableHHeader = ["Right Wheel", "Left Wheel"]
        self.listTableVHeader = ["Velocity", "Encoder", "State", "Version"]
        self.initUi()

    def initUi(self):
        self.setColumnCount(len(self.listTableHHeader))
        self.setHorizontalHeaderLabels(self.listTableHHeader)
        self.setRowCount(len(self.listTableVHeader))
        self.setVerticalHeaderLabels(self.listTableVHeader)
        self.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self.setSpan(3, 0, 1, 2)
        self.setSpan(2, 0, 1, 2)

        self.set_table_items((0.00, 0.00), (0, 0), 0, "Unknown")

    def set_table_items(self, atupleVel: Tuple[float, float], atupleEnc: Tuple[int, int], anState: int, astrVer: str):
        rightVel, leftVel = atupleVel
        rightEnc, leftEnc = atupleEnc
        rightVel = EmbdQtTableItem(rightVel, True)
        leftVel = EmbdQtTableItem(leftVel, True)
        rightEnc = EmbdQtTableItem(rightEnc, True)
        leftEnc = EmbdQtTableItem(leftEnc, True)
        state = EmbdQtTableItem(anState, True)
        version = EmbdQtTableItem(astrVer, True)

        self.setItem(0, 0, rightVel)
        self.setItem(0, 1, leftVel)
        self.setItem(1, 0, rightEnc)
        self.setItem(1, 1, leftEnc)
        self.setItem(2, 0, state)
        self.setItem(3, 0, version)
        self.resizeRowsToContents()
        self.viewport().update()  # refresh table

    def set_encoder_value(self, anRightEnc: int, anLeftEnc: int):
        rightEnc = EmbdQtTableItem(anRightEnc, True)
        leftEnc = EmbdQtTableItem(anLeftEnc, True)
        self.setItem(1, 0, rightEnc)
        self.setItem(1, 1, leftEnc)
        self.resizeRowsToContents()
        self.viewport().update()  # refresh table

    def set_velocity_value(self, afRightVel: float, afLeftVel: float):
        rightVel = EmbdQtTableItem(afRightVel, True)
        leftVel = EmbdQtTableItem(afLeftVel, True)
        self.setItem(0, 0, rightVel)
        self.setItem(0, 1, leftVel)
        self.resizeRowsToContents()
        self.viewport().update()  # refresh table

    def set_state(self, anState: int):
        state = EmbdQtTableItem(anState, True)
        self.setItem(2, 0, state)
        self.resizeRowsToContents()
        self.viewport().update()  # refresh table

    def set_version(self, astrVer: str):
        version = EmbdQtTableItem(astrVer, True)
        self.setItem(3, 0, version)
        self.resizeRowsToContents()
        self.viewport().update()  # refresh table


class DockStellaB2Ctrl(QtWidgets.QDockWidget):
    _widgetMenuBar: QtWidgets.QWidget = None
    _widgetCtrl: QtWidgets.QWidget = None
    _widgetCtrlFb: QtWidgets.QWidget = None
    _widgetFeedback: QTableWidgetFeedback = None
    sig_close = QtCore.pyqtSignal()

    def closeEvent(self, closeEvent: QtGui.QCloseEvent) -> None:
        self.sig_close.emit()

    def __init__(self, parent=None):
        super(DockStellaB2Ctrl, self).__init__(parent)
        self.sig_serial_connect = PySignal(str)
        self.sig_serial_disconnect = PySignal()
        self.sig_set_monitoring = PySignal(bool)
        self.sig_stop_type = PySignal(int)
        self.sig_move_joint_space = PySignal(int, int)
        self.sig_init = PySignal()
        self.sig_reset = PySignal()

        self.setWindowTitle('Stella B2 Control Panel')
        self.initUi()
        self.on_serial_disconnect()

    def initUi(self):
        self.initMenuBar()
        self.initCtrlWidget()
        self.initFeedbackTable()
        centralWidget = QtWidgets.QWidget(self)
        self.setWidget(centralWidget)
        MainLayout = QtWidgets.QVBoxLayout(centralWidget)
        MainLayout.addWidget(self._widgetMenuBar)

        self._widgetCtrlFb = QtWidgets.QWidget()
        HLayoutCtrlFb = QtWidgets.QHBoxLayout(self._widgetCtrlFb)
        HLayoutCtrlFb.addWidget(self._widgetCtrl)
        gbFeedback = QtWidgets.QGroupBox('Feedback')
        VLayoutFeedback = QtWidgets.QVBoxLayout(gbFeedback)
        VLayoutFeedback.addWidget(self._widgetFeedback)
        HLayoutCtrlFb.addWidget(gbFeedback, 1)

        MainLayout.addWidget(self._widgetCtrlFb)
        MainLayout.addStretch(1)

    def initCtrlWidget(self):
        self._widgetCtrl = QtWidgets.QWidget()
        VLayoutCtrl = QtWidgets.QVBoxLayout(self._widgetCtrl)

        HLayoutInitReset = QtWidgets.QHBoxLayout()
        self.btnInit = QtWidgets.QPushButton('INIT')
        self.btnInit.clicked.connect(self.on_click_btn_init)
        self.btnReset = QtWidgets.QPushButton('RESET')
        self.btnReset.clicked.connect(self.on_click_btn_reset)
        HLayoutInitReset.addWidget(self.btnInit)
        HLayoutInitReset.addWidget(self.btnReset)

        gbJointSpace = QtWidgets.QGroupBox('Joint Space Control (rads)')
        VLayoutJointSpace = QtWidgets.QVBoxLayout(gbJointSpace)
        lblRightWheel = QtWidgets.QLabel('RIGHT WHEEL')
        lblRightWheel.setFixedWidth(80)
        self.spinRightWheel = QtWidgets.QSpinBox()
        self.spinRightWheel.setMinimum(-270)
        self.spinRightWheel.setMaximum(270)
        self.sliderRightWheel = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sliderRightWheel.setMinimum(-270)
        self.sliderRightWheel.setMaximum(270)
        self.spinRightWheel.valueChanged.connect(self.sliderRightWheel.setValue)
        self.sliderRightWheel.valueChanged.connect(self.spinRightWheel.setValue)
        HLayoutRightWheel = QtWidgets.QHBoxLayout()
        HLayoutRightWheel.addWidget(lblRightWheel)
        HLayoutRightWheel.addWidget(self.spinRightWheel)
        HLayoutRightWheel.addWidget(self.sliderRightWheel, 1)

        lblLeftWheel = QtWidgets.QLabel('LEFT WHEEL')
        lblLeftWheel.setFixedWidth(80)
        self.spinLeftWheel = QtWidgets.QSpinBox()
        self.spinLeftWheel.setMinimum(-270)
        self.spinLeftWheel.setMaximum(270)
        self.sliderLeftWheel = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.spinLeftWheel.valueChanged.connect(self.sliderLeftWheel.setValue)
        self.sliderLeftWheel.valueChanged.connect(self.spinLeftWheel.setValue)
        self.sliderLeftWheel.setMinimum(-270)
        self.sliderLeftWheel.setMaximum(270)
        HLayoutLeftWheel = QtWidgets.QHBoxLayout()
        HLayoutLeftWheel.addWidget(lblLeftWheel)
        HLayoutLeftWheel.addWidget(self.spinLeftWheel)
        HLayoutLeftWheel.addWidget(self.sliderLeftWheel, 1)
        self.btnMoveJointSpace = QtWidgets.QPushButton('MOVE JOINT SPACE')
        self.btnMoveJointSpace.clicked.connect(self.on_click_btn_move_joint_space)

        VLayoutJointSpace.addLayout(HLayoutRightWheel)
        VLayoutJointSpace.addLayout(HLayoutLeftWheel)
        VLayoutJointSpace.addWidget(self.btnMoveJointSpace)

        gbStopType = QtWidgets.QGroupBox('STOP Type')
        HLayoutStopType = QtWidgets.QHBoxLayout(gbStopType)

        self.btnStopFree = QtWidgets.QPushButton('STOP-FREE')
        self.btnStopFree.stop_type = 1
        self.btnStopFree.clicked.connect(self.on_click_btn_stop)
        self.btnStopHold = QtWidgets.QPushButton('STOP-HOLD')
        self.btnStopHold.stop_type = 2
        self.btnStopHold.clicked.connect(self.on_click_btn_stop)
        self.btnStopHoldDecc = QtWidgets.QPushButton('STOP-HOLD(Dec)')
        self.btnStopHoldDecc.stop_type = 3
        self.btnStopHoldDecc.clicked.connect(self.on_click_btn_stop)
        HLayoutStopType.addWidget(self.btnStopFree)
        HLayoutStopType.addWidget(self.btnStopHold)
        HLayoutStopType.addWidget(self.btnStopHoldDecc)

        VLayoutCtrl.addLayout(HLayoutInitReset)
        VLayoutCtrl.addWidget(gbJointSpace)
        VLayoutCtrl.addWidget(gbStopType)

    def initFeedbackTable(self):
        self._widgetFeedback = QTableWidgetFeedback(self)

    def initMenuBar(self):
        mb_icon_size = 20

        self._widgetMenuBar = QtWidgets.QWidget()
        HLayoutMenuBar = QtWidgets.QHBoxLayout(self._widgetMenuBar)
        self.ledConnect = EmbdLed('Connection')
        HLayoutMenuBar.setContentsMargins(0, 0, 0, 0)
        HLayoutMenuBar.addWidget(self.ledConnect)
        self.btnConnect = QtWidgets.QPushButton()
        self.btnConnect.setIcon(EmbdQtIcon("connect.png"))
        self.btnConnect.setIconSize(QtCore.QSize(mb_icon_size, mb_icon_size))
        self.btnConnect.setFixedSize(QtCore.QSize(mb_icon_size + 2, mb_icon_size + 2))
        self.btnConnect.setFlat(True)
        self.btnConnect.clicked.connect(self.on_click_btn_serial_connect)
        HLayoutMenuBar.addWidget(self.btnConnect)
        self.btnDisconnect = QtWidgets.QPushButton()
        self.btnDisconnect.setIcon(EmbdQtIcon("disconnect.png"))
        self.btnDisconnect.setIconSize(QtCore.QSize(mb_icon_size, mb_icon_size))
        self.btnDisconnect.setFixedSize(QtCore.QSize(mb_icon_size + 2, mb_icon_size + 2))
        self.btnDisconnect.setFlat(True)
        self.btnDisconnect.clicked.connect(self.on_click_btn_serial_disconnect)
        HLayoutMenuBar.addWidget(self.btnDisconnect)
        self.cmbSerialPort = QtWidgets.QComboBox()
        HLayoutMenuBar.addWidget(self.cmbSerialPort)
        self.btnResetSerialPort = QtWidgets.QPushButton()
        self.btnResetSerialPort.setIcon(EmbdQtIcon("reset.png"))
        self.btnResetSerialPort.setFlat(True)
        self.btnResetSerialPort.setIconSize(QtCore.QSize(mb_icon_size, mb_icon_size))
        self.btnResetSerialPort.setFixedSize(QtCore.QSize(mb_icon_size + 2, mb_icon_size + 2))
        self.btnResetSerialPort.clicked.connect(self.on_click_btn_refresh)
        HLayoutMenuBar.addWidget(self.btnResetSerialPort)
        self.cbMonitoring = QtWidgets.QCheckBox('Monitoring')
        self.cbMonitoring.stateChanged.connect(self.on_state_changed_monitoring)
        # self.cbMonitoring.setChecked(True)
        HLayoutMenuBar.addWidget(self.cbMonitoring)
        HLayoutMenuBar.addStretch(1)

        self.on_click_btn_refresh()  # reset serial port combo-box

    def on_click_btn_move_joint_space(self):
        right_vel = int(self.spinRightWheel.value())
        left_vel = int(self.spinLeftWheel.value())
        self.sig_move_joint_space.emit(right_vel, left_vel)

    def on_click_btn_init(self):
        self.sig_init.emit()

    def on_click_btn_reset(self):
        self.sig_reset.emit()

    def on_click_btn_stop(self):
        stop_type = self.sender().stop_type
        self.sig_stop_type.emit(stop_type)

    def on_state_changed_monitoring(self, state):
        if state == QtCore.Qt.Checked:
            sig_bool = True
        else:
            sig_bool = False

        self.sig_set_monitoring.emit(sig_bool)

    def on_click_btn_refresh(self):
        self.cmbSerialPort.clear()
        self.cmbSerialPort.addItems(com_port for com_port in get_serial_port_name_list())

    def on_click_btn_serial_connect(self):
        port = str(self.cmbSerialPort.currentText())
        self.sig_serial_connect.emit(port)

    def on_click_btn_serial_disconnect(self):
        self.sig_serial_disconnect.emit()

    def on_serial_connect(self):
        self.ledConnect.TurnOnG()
        self.btnConnect.setEnabled(False)
        self.btnResetSerialPort.setEnabled(False)
        self.cmbSerialPort.setEnabled(False)
        self.cbMonitoring.setEnabled(True)
        self.btnDisconnect.setEnabled(True)
        self._widgetCtrl.setEnabled(True)

    def on_serial_error(self):
        self.ledConnect.TurnOnR()

    def on_serial_disconnect(self):
        self.ledConnect.TurnOff()
        self.btnConnect.setEnabled(True)
        self.btnResetSerialPort.setEnabled(True)
        self.cmbSerialPort.setEnabled(True)
        self.cbMonitoring.setEnabled(False)
        self.btnDisconnect.setEnabled(False)
        self._widgetCtrl.setEnabled(False)

    def on_changed_monitoring_state(self, state: bool):
        self.cbMonitoring.blockSignals(True)
        self.cbMonitoring.setChecked(state)
        self.cbMonitoring.blockSignals(False)

    def set_fb_velocity(self, afRightVel: float, afLeftVel: float):
        self._widgetFeedback.set_velocity_value(afRightVel, afLeftVel)

    def set_fb_encoder(self, anRightEnc: int, anLeftEnc: int):
        self._widgetFeedback.set_encoder_value(anRightEnc, anLeftEnc)

    def set_fb_state(self, anState: int):
        self._widgetFeedback.set_state(anState)

    def set_fb_version(self, astrVer: str):
        self._widgetFeedback.set_version(astrVer)
