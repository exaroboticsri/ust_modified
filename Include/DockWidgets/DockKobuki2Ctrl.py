# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : DockKobuki2Ctrl.py
# Project Name : ExaRobotCtrl
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2022.02.09 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import sys
from typing import Tuple
from numpy import rad2deg, deg2rad

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include/DOckWidgets
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
MOBILE_ROBOT_PATH = os.path.join(INCLUDE_PATH, "MobileRobot")
LIBRARY_PATH = os.path.join(ROOT_PATH, "Library")
SERIAL_PATH = os.path.join(LIBRARY_PATH, "Serial")
MDI_PATH = os.path.join(ROOT_PATH, "MdiBackground")
MISC_PATH = os.path.join(ROOT_PATH, "Misc")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
sys.path.extend([INCLUDE_PATH, MDI_PATH, RESOURCES_PATH, SERIAL_PATH, MOBILE_ROBOT_PATH, ])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH, MOBILE_ROBOT_PATH

from EmbdLed import EmbdLed
from UiCommon import *
from Commons import *
from SerialPort import get_serial_port_name_list
from TMRKobuki2 import CTMRKobuki2


class QTableWidgetFeedback(QtWidgets.QTableWidget):
    listTableHHeader: list
    listTableVHeader: list

    def __init__(self, parent=None):
        super(QTableWidgetFeedback, self).__init__(parent)
        self.listTableHHeader = ["Right Wheel", "Left Wheel"]
        self.listTableVHeader = ["Velocity", "Encoder", "State", "FW Version", "HW Version"]
        self.initUi()

    def initUi(self):
        self.setColumnCount(len(self.listTableHHeader))
        self.setHorizontalHeaderLabels(self.listTableHHeader)
        self.setRowCount(len(self.listTableVHeader))
        self.setVerticalHeaderLabels(self.listTableVHeader)
        self.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self.setSpan(2, 0, 1, 2)
        self.setSpan(3, 0, 1, 2)
        self.setSpan(4, 0, 1, 2)


        self.set_table_items((0.00, 0.00), (0, 0), 0, "Unknown", "Unknown")

    def set_table_items(self, atupleVel: Tuple[float, float], atupleEnc: Tuple[int, int], anState: int, astrFWVer: str, astrHWVer: str):
        rightVel, leftVel = atupleVel
        rightEnc, leftEnc = atupleEnc
        rightVel = EmbdQtTableItem(rightVel, True)
        leftVel = EmbdQtTableItem(leftVel, True)
        rightEnc = EmbdQtTableItem(rightEnc, True)
        leftEnc = EmbdQtTableItem(leftEnc, True)
        state = EmbdQtTableItem(anState, True)
        fw_version = EmbdQtTableItem(astrFWVer, True)
        hw_version = EmbdQtTableItem(astrHWVer, True)

        self.setItem(0, 0, rightVel)
        self.setItem(0, 1, leftVel)
        self.setItem(1, 0, rightEnc)
        self.setItem(1, 1, leftEnc)
        self.setItem(2, 0, state)
        self.setItem(3, 0, fw_version)
        self.setItem(4, 0, hw_version)
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

    def set_fw_version(self, astrVer: str):
        version = EmbdQtTableItem(astrVer, True)
        self.setItem(3, 0, version)
        self.resizeRowsToContents()
        self.viewport().update()  # refresh table

    def set_hw_version(self, astrVer: str):
        version = EmbdQtTableItem(astrVer, True)
        self.setItem(4, 0, version)
        self.resizeRowsToContents()
        self.viewport().update()  # refresh table


class DockKobuki2Ctrl(QtWidgets.QDockWidget):
    _widgetMenuBar: QtWidgets.QWidget = None
    _widgetCtrl: QtWidgets.QWidget = None
    _widgetCtrlFb: QtWidgets.QWidget = None
    _widgetFeedback: QTableWidgetFeedback = None
    sig_close = QtCore.pyqtSignal()

    def closeEvent(self, closeEvent: QtGui.QCloseEvent) -> None:
        self.sig_close.emit()

    def __init__(self, parent=None):
        super(DockKobuki2Ctrl, self).__init__(parent)
        self.sig_serial_connect = PySignal(str)
        self.sig_serial_disconnect = PySignal()
        self.sig_move_robot = PySignal(float, float)
        self.sig_init = PySignal()
        self.sig_reset = PySignal()

        self.setWindowTitle('Kobuki 2 Control Panel')
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
        self.btnInit = QtWidgets.QPushButton('ON')
        self.btnInit.clicked.connect(self.on_click_btn_init)
        self.btnReset = QtWidgets.QPushButton('OFF')
        self.btnReset.clicked.connect(self.on_click_btn_reset)
        HLayoutInitReset.addWidget(self.btnInit)
        HLayoutInitReset.addWidget(self.btnReset)

        gbJointSpace = QtWidgets.QGroupBox('Joint Space Control (deg/s)')
        VLayoutJointSpace = QtWidgets.QVBoxLayout(gbJointSpace)
        lblRightWheel = QtWidgets.QLabel('RIGHT WHEEL')
        lblRightWheel.setFixedWidth(80)
        self.spinRightWheel = QtWidgets.QSpinBox()
        self.spinRightWheel.setMinimum(-1507)  # todo: do not hardcode
        self.spinRightWheel.setMaximum(1507)  # todo: do not hardcode
        self.sliderRightWheel = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sliderRightWheel.setMinimum(-1507)  # todo: do not hardcode
        self.sliderRightWheel.setMaximum(1507)  # todo: do not hardcode
        HLayoutRightWheel = QtWidgets.QHBoxLayout()
        HLayoutRightWheel.addWidget(lblRightWheel)
        HLayoutRightWheel.addWidget(self.spinRightWheel)
        HLayoutRightWheel.addWidget(self.sliderRightWheel, 1)
        lblLeftWheel = QtWidgets.QLabel('LEFT WHEEL')
        lblLeftWheel.setFixedWidth(80)
        self.spinLeftWheel = QtWidgets.QSpinBox()
        self.spinLeftWheel.setMinimum(-1507)  # todo: do not hardcode
        self.spinLeftWheel.setMaximum(1507)  # todo: do not hardcode
        self.sliderLeftWheel = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sliderLeftWheel.setMinimum(-1507)  # todo: do not hardcode
        self.sliderLeftWheel.setMaximum(1507)  # todo: do not hardcode
        HLayoutLeftWheel = QtWidgets.QHBoxLayout()
        HLayoutLeftWheel.addWidget(lblLeftWheel)
        HLayoutLeftWheel.addWidget(self.spinLeftWheel)
        HLayoutLeftWheel.addWidget(self.sliderLeftWheel, 1)
        VLayoutJointSpace.addLayout(HLayoutRightWheel)
        VLayoutJointSpace.addLayout(HLayoutLeftWheel)

        gbCenterCtrl = QtWidgets.QGroupBox('Center Control')
        VLayoutCenterCtrl = QtWidgets.QVBoxLayout(gbCenterCtrl)
        lblCenterVel = QtWidgets.QLabel('VEL (mm/s)')
        lblCenterVel.setFixedWidth(80)
        self.spinCenterVel = QtWidgets.QSpinBox()
        self.spinCenterVel.setMinimum(-700)  # todo: do not hardcode
        self.spinCenterVel.setMaximum(700)  # todo: do not hardcode
        self.sliderCenterVel = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sliderCenterVel.setMinimum(-700)  # todo: do not hardcode
        self.sliderCenterVel.setMaximum(700)  # todo: do not hardcode
        HLayoutCenterVel = QtWidgets.QHBoxLayout()
        HLayoutCenterVel.addWidget(lblCenterVel)
        HLayoutCenterVel.addWidget(self.spinCenterVel)
        HLayoutCenterVel.addWidget(self.sliderCenterVel, 1)
        lblYawRate = QtWidgets.QLabel('YAW (deg/s)')
        lblYawRate.setFixedWidth(80)
        self.spinYawRate = QtWidgets.QSpinBox()
        self.spinYawRate.setMinimum(-110)  # todo: do not hardcode
        self.spinYawRate.setMaximum(110)  # todo: do not hardcode
        self.sliderYawRate = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.sliderYawRate.setMinimum(-110)  # todo: do not hardcode
        self.sliderYawRate.setMaximum(110)  # todo: do not hardcode
        HLayoutYawRate = QtWidgets.QHBoxLayout()
        HLayoutYawRate.addWidget(lblYawRate)
        HLayoutYawRate.addWidget(self.spinYawRate)
        HLayoutYawRate.addWidget(self.sliderYawRate, 1)
        # self.btnMoveJointSpace = QtWidgets.QPushButton('MOVE JOINT SPACE')
        # self.btnMoveJointSpace.clicked.connect(self.on_click_btn_move_joint_space)
        VLayoutCenterCtrl.addLayout(HLayoutCenterVel)
        VLayoutCenterCtrl.addLayout(HLayoutYawRate)
        # VLayoutCenterCtrl.addWidget(self.btnMoveJointSpace)

        gbMove = QtWidgets.QGroupBox('MOVE')
        HLayoutStopType = QtWidgets.QHBoxLayout(gbMove)
        self.btnMove = QtWidgets.QPushButton('MOVE ROBOT')
        self.btnMove.clicked.connect(self.on_click_btn_move)
        self.btnStop = QtWidgets.QPushButton('STOP ROBOT')
        self.btnStop.clicked.connect(self.on_click_btn_stop)
        HLayoutStopType.addWidget(self.btnMove)
        HLayoutStopType.addWidget(self.btnStop)
        VLayoutCtrl.addLayout(HLayoutInitReset)
        VLayoutCtrl.addWidget(gbJointSpace)
        VLayoutCtrl.addWidget(gbCenterCtrl)
        VLayoutCtrl.addWidget(gbMove)

        # slots
        self.spinYawRate.valueChanged.connect(self.sliderYawRate.setValue)
        self.sliderYawRate.valueChanged.connect(self.on_value_changed_yaw_rate)
        # self.sliderYawRate.valueChanged.connect(self.spinYawRate.setValue)

        self.spinCenterVel.valueChanged.connect(self.sliderCenterVel.setValue)
        # self.sliderCenterVel.valueChanged.connect(self.spinCenterVel.setValue)
        self.sliderCenterVel.valueChanged.connect(self.on_value_changed_center_vel)

        self.spinRightWheel.valueChanged.connect(self.sliderRightWheel.setValue)
        self.sliderRightWheel.valueChanged.connect(self.on_value_changed_right_wheel)

        self.sliderLeftWheel.valueChanged.connect(self.on_value_changed_left_wheel)
        self.spinLeftWheel.valueChanged.connect(self.sliderLeftWheel.setValue)

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
        HLayoutMenuBar.addStretch(1)

        self.on_click_btn_refresh()  # reset serial port combo-box

    def on_click_btn_move(self):
        center_vel = self.spinCenterVel.value() / 1000.
        yaw_rate = deg2rad(self.spinYawRate.value())
        self.sig_move_robot.emit(center_vel, yaw_rate)

    def on_click_btn_init(self):
        self.sig_init.emit()

    def on_click_btn_reset(self):
        self.sig_reset.emit()

    def on_click_btn_stop(self):
        self.sig_move_robot.emit(0., 0.)

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
        self.btnDisconnect.setEnabled(True)
        self._widgetCtrl.setEnabled(True)

    def on_serial_error(self):
        self.ledConnect.TurnOnR()

    def on_serial_disconnect(self):
        self.ledConnect.TurnOff()
        self.btnConnect.setEnabled(True)
        self.btnResetSerialPort.setEnabled(True)
        self.cmbSerialPort.setEnabled(True)
        self.btnDisconnect.setEnabled(False)
        self._widgetCtrl.setEnabled(False)

    def on_value_changed_right_wheel(self, val: int):
        self.spinRightWheel.setValue(val)
        omega_r = deg2rad(val)
        omega_l = deg2rad(self.spinLeftWheel.value())
        vel_c, omega_c = CTMRKobuki2().calculate_center_vel((omega_r, omega_l))

        self.sliderCenterVel.blockSignals(True)
        self.spinCenterVel.blockSignals(True)
        self.sliderYawRate.blockSignals(True)
        self.spinYawRate.blockSignals(True)
        self.sliderCenterVel.setValue(int(vel_c * 1000))
        self.spinCenterVel.setValue(int(vel_c * 1000))
        self.sliderYawRate.setValue(int(rad2deg(omega_c)))
        self.spinYawRate.setValue(int(rad2deg(omega_c)))
        self.sliderCenterVel.blockSignals(False)
        self.spinCenterVel.blockSignals(False)
        self.sliderYawRate.blockSignals(False)
        self.spinYawRate.blockSignals(False)

    def on_value_changed_left_wheel(self, val: int):
        self.spinLeftWheel.setValue(val)
        omega_r = deg2rad(self.spinRightWheel.value())
        omega_l = deg2rad(val)
        vel_c, omega_c = CTMRKobuki2().calculate_center_vel((omega_r, omega_l))

        self.sliderCenterVel.blockSignals(True)
        self.spinCenterVel.blockSignals(True)
        self.sliderYawRate.blockSignals(True)
        self.spinYawRate.blockSignals(True)
        self.sliderCenterVel.setValue(int(vel_c * 1000))
        self.spinCenterVel.setValue(int(vel_c * 1000))
        self.sliderYawRate.setValue(int(rad2deg(omega_c)))
        self.spinYawRate.setValue(int(rad2deg(omega_c)))
        self.sliderCenterVel.blockSignals(False)
        self.spinCenterVel.blockSignals(False)
        self.sliderYawRate.blockSignals(False)
        self.spinYawRate.blockSignals(False)

    def on_value_changed_center_vel(self, val: int):
        self.spinCenterVel.setValue(val)
        omega_c = deg2rad(self.spinYawRate.value())
        omega_r, omega_l = CTMRKobuki2().calculate_joint_space_vel(val / 1000., omega_c)

        omega_r = int(rad2deg(omega_r))
        omega_l = int(rad2deg(omega_l))

        self.spinRightWheel.blockSignals(True)
        self.sliderRightWheel.blockSignals(True)
        self.spinLeftWheel.blockSignals(True)
        self.sliderLeftWheel.blockSignals(True)
        self.spinRightWheel.setValue(omega_r)
        self.sliderRightWheel.setValue(omega_r)
        self.spinLeftWheel.setValue(omega_l)
        self.sliderLeftWheel.setValue(omega_l)
        self.spinRightWheel.blockSignals(False)
        self.sliderRightWheel.blockSignals(False)
        self.spinLeftWheel.blockSignals(False)
        self.sliderLeftWheel.blockSignals(False)

    def on_value_changed_yaw_rate(self, val: int):
        self.spinYawRate.setValue(val)
        vel_c = self.spinCenterVel.value() / 1000.
        omega_r, omega_l = CTMRKobuki2().calculate_joint_space_vel(vel_c, deg2rad(val))

        omega_r = int(rad2deg(omega_r))
        omega_l = int(rad2deg(omega_l))

        self.spinRightWheel.blockSignals(True)
        self.sliderRightWheel.blockSignals(True)
        self.spinLeftWheel.blockSignals(True)
        self.sliderLeftWheel.blockSignals(True)
        self.spinRightWheel.setValue(omega_r)
        self.sliderRightWheel.setValue(omega_r)
        self.spinLeftWheel.setValue(omega_l)
        self.sliderLeftWheel.setValue(omega_l)
        self.spinRightWheel.blockSignals(False)
        self.sliderRightWheel.blockSignals(False)
        self.spinLeftWheel.blockSignals(False)
        self.sliderLeftWheel.blockSignals(False)

    def on_changed_monitoring_state(self, state: bool):
        pass

    def set_fb_velocity(self, afRightVel: float, afLeftVel: float):
        self._widgetFeedback.set_velocity_value(rad2deg(afRightVel), rad2deg(afLeftVel))

    def set_fb_encoder(self, anRightEnc: int, anLeftEnc: int):
        self._widgetFeedback.set_encoder_value(anRightEnc, anLeftEnc)

    def set_fb_state(self, anState: int):
        self._widgetFeedback.set_state(anState)

    def set_fb_fw_version(self, astrVer: str):
        self._widgetFeedback.set_fw_version(astrVer)

    def set_fb_hw_version(self, astrVer: str):
        self._widgetFeedback.set_hw_version(astrVer)


if __name__ == '__main__':
    # omega_r, omega_l = CTMRKobuki2().calculate_joint_space_vel(0.7, 0)

    app = QtCore.QCoreApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)

    MainUI = DockKobuki2Ctrl()
    MainUI.show()

    app.exec_()
# MainUI.release()
