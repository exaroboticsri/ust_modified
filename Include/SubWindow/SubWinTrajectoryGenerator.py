# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : SubWinTrajecotryGenerator.py
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
import matplotlib

matplotlib.rcParams.update({'font.size': 6})
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

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

from UiCommon import *
from Commons import *

from TMRKobuki2 import CTMRKobuki2
from TMRStellaB2 import CTMRStellaB2
from BezierCurveConv import CBezierCurveTraj


class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=2, height=2, dpi=100):
        self.parent = parent
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        super(MplCanvas, self).__init__(self.fig)
        self.axes = self.fig.add_subplot(111)
        self.fig.subplots_adjust(wspace=0, hspace=0)
    # self.fig.tight_layout()


class SubWindowTrajGen(QtWidgets.QMdiSubWindow):
    _widgetPhyLimits: QtWidgets.QWidget = None
    _widgetPosCtrl: QtWidgets.QWidget = None
    _widgetROSCtrl: QtWidgets.QWidget = None
    listRobots: list = None
    sig_close = QtCore.pyqtSignal()

    def closeEvent(self, closeEvent: QtGui.QCloseEvent) -> None:
        self.sig_close.emit()

    def __init__(self, parent=None):
        super(SubWindowTrajGen, self).__init__(parent)
        self.listRobots = ["Kobuki2", "StellaB2", "Custom"]
        self.setWindowIcon(EmbdQtIcon("path.png"))
        self.setWindowTitle('Bezier Curve and Convolution Trajectory Generator')
        self.setMinimumWidth(850)
        self.setMinimumHeight(600)
        self.resize(850, 600)

        self.sig_ros_start_timer = PySignal()
        self.sig_ros_stop_timer = PySignal()
        self.sig_ros_add_vel_cmd = PySignal(tuple)
        self.sig_ros_reg_vel_cmd = PySignal(tuple)
        self.sig_ros_clear_vel_cmd = PySignal()

        self.initUi()
        # self.show()

    def initUi(self):
        self.initPhyLimits()
        self.initPosCtrl()
        self.initROSCtrl()
        self.initFigureCanvas()

        centralWidget = QtWidgets.QWidget()
        self.setWidget(centralWidget)
        HLayoutMain = QtWidgets.QHBoxLayout(centralWidget)

        self.cmbRobot = QtWidgets.QComboBox()
        self.cmbRobot.addItems(self.listRobots)
        self.cmbRobot.setCurrentIndex(0)
        self.on_text_change_cmb_robot(self.cmbRobot.currentText())
        self.cmbRobot.currentTextChanged.connect(self.on_text_change_cmb_robot)
        lblRobots = QtWidgets.QLabel()
        lblRobots.setText('Robots')
        FLayoutRobot = QtWidgets.QFormLayout()
        FLayoutRobot.addRow(lblRobots, self.cmbRobot)

        VLayoutLeft = QtWidgets.QVBoxLayout()
        VLayoutLeft.addLayout(FLayoutRobot)
        VLayoutLeft.addWidget(self._widgetPhyLimits)
        VLayoutLeft.addWidget(self._widgetPosCtrl)
        VLayoutLeft.addWidget(self._widgetROSCtrl)
        VLayoutLeft.addStretch(1)

        HLayoutMain.addLayout(VLayoutLeft)
        HLayoutMain.addWidget(self._widgetFigs, 1)

        self.listVelC = [0.]
        self.listOmegaC = [0.]
        self.listThetaC = [0.]

    def initPhyLimits(self):
        self._widgetPhyLimits = QtWidgets.QWidget(self)
        VPhyLimits = QtWidgets.QVBoxLayout(self._widgetPhyLimits)
        gbLimits = QtWidgets.QGroupBox("Physical Limits")
        VLayoutLimits = QtWidgets.QVBoxLayout(gbLimits)
        FLayoutLimits = QtWidgets.QFormLayout()
        lblMaxVel = QtWidgets.QLabel('Max Vel (m/s)')
        self.dspinMaxVel = QtWidgets.QDoubleSpinBox()
        self.dspinMaxVel.setRange(0., 5.)
        self.dspinMaxVel.setSingleStep(0.01)
        lblMaxAcc = QtWidgets.QLabel('Max Acc (m/s²)')
        self.dspinMaxAcc = QtWidgets.QDoubleSpinBox()
        self.dspinMaxAcc.setRange(0., 5.)
        self.dspinMaxAcc.setSingleStep(0.01)
        lblMaxJerk = QtWidgets.QLabel('Max Jerk (m/s³)')
        self.dspinMaxJerk = QtWidgets.QDoubleSpinBox()
        self.dspinMaxJerk.setRange(0., 5.)
        self.dspinMaxJerk.setSingleStep(0.01)
        lblSamplingTime = QtWidgets.QLabel('Sampling Time (ms)')
        self.spinSamplingTime = QtWidgets.QSpinBox()
        self.spinSamplingTime.setRange(0, 5000)
        self.spinSamplingTime.setValue(10)
        FLayoutLimits.addRow(lblMaxVel, self.dspinMaxVel)
        FLayoutLimits.addRow(lblMaxAcc, self.dspinMaxAcc)
        FLayoutLimits.addRow(lblMaxJerk, self.dspinMaxJerk)
        FLayoutLimits.addRow(lblSamplingTime, self.spinSamplingTime)
        VLayoutLimits.addLayout(FLayoutLimits)
        VPhyLimits.addWidget(gbLimits)

    def initPosCtrl(self):
        self._widgetPosCtrl = QtWidgets.QWidget(self)
        VPosCtrl = QtWidgets.QVBoxLayout(self._widgetPosCtrl)
        gbPos = QtWidgets.QGroupBox("Position Control")
        HPos = QtWidgets.QHBoxLayout(gbPos)

        FLayoutInit = QtWidgets.QFormLayout()
        lblInitX = QtWidgets.QLabel()
        lblInitX.setText('X_init (m)')
        self.dspinInitX = QtWidgets.QDoubleSpinBox()
        self.dspinInitX.setRange(-10., 10.)
        self.dspinInitX.setSingleStep(0.01)
        lblInitY = QtWidgets.QLabel()
        lblInitY.setText('Y_init (m)')
        self.dspinInitY = QtWidgets.QDoubleSpinBox()
        self.dspinInitY.setRange(-10., 10.)
        self.dspinInitY.setSingleStep(0.01)
        lblInitTheta = QtWidgets.QLabel()
        lblInitTheta.setText('θ_init (deg)')
        self.dspinInitTheta = QtWidgets.QDoubleSpinBox()
        self.dspinInitTheta.setRange(-10., 10.)
        self.dspinInitTheta.setSingleStep(0.01)
        lblInitVel = QtWidgets.QLabel()
        lblInitVel.setText('Vel_init (m/s)')
        self.dspinInitVel = QtWidgets.QDoubleSpinBox()
        self.dspinInitVel.setRange(-10., 10.)
        self.dspinInitVel.setSingleStep(0.01)
        FLayoutInit.addRow(lblInitX, self.dspinInitX)
        FLayoutInit.addRow(lblInitY, self.dspinInitY)
        FLayoutInit.addRow(lblInitTheta, self.dspinInitTheta)
        FLayoutInit.addRow(lblInitVel, self.dspinInitVel)

        FLayoutTarg = QtWidgets.QFormLayout()
        lblTargX = QtWidgets.QLabel()
        lblTargX.setText('X_targ (m)')
        self.dspinTargX = QtWidgets.QDoubleSpinBox()
        self.dspinTargX.setRange(-10., 10.)
        self.dspinTargX.setSingleStep(0.01)
        lblTargY = QtWidgets.QLabel()
        lblTargY.setText('Y_targ (m)')
        self.dspinTargY = QtWidgets.QDoubleSpinBox()
        self.dspinTargY.setRange(-10., 10.)
        self.dspinTargY.setSingleStep(0.01)
        lblTargTheta = QtWidgets.QLabel()
        lblTargTheta.setText('Θ_targ (deg)')
        self.dspinTargTheta = QtWidgets.QDoubleSpinBox()
        self.dspinTargTheta.setRange(-10., 10.)
        self.dspinTargTheta.setSingleStep(0.01)
        lblTargVel = QtWidgets.QLabel()
        lblTargVel.setText('Vel_targ (m/s)')
        self.dspinTargVel = QtWidgets.QDoubleSpinBox()
        self.dspinTargVel.setRange(-10., 10.)
        self.dspinTargVel.setSingleStep(0.01)
        FLayoutTarg.addRow(lblTargX, self.dspinTargX)
        FLayoutTarg.addRow(lblTargY, self.dspinTargY)
        FLayoutTarg.addRow(lblTargTheta, self.dspinTargTheta)
        FLayoutTarg.addRow(lblTargVel, self.dspinTargVel)
        HPos.addLayout(FLayoutInit)
        HPos.addLayout(FLayoutTarg)

        HLayoutBtn = QtWidgets.QHBoxLayout()
        self.btnCalculate = QtWidgets.QPushButton('CALCULATE')
        self.btnCalculate.clicked.connect(self.on_btn_clicked_calculate)
        self.btnSimulate = QtWidgets.QPushButton('SIMULATE')
        self.btnSimulate.clicked.connect(self.on_btn_clicked_simulate)
        self.btnClear = QtWidgets.QPushButton('CLEAR')
        self.btnClear.clicked.connect(self.on_btn_clicked_clear)
        self.dspinTargVel.setValue(0.)
        self.dspinTargVel.setEnabled(False)
        self.dspinInitVel.setValue(0.)
        self.dspinInitVel.setEnabled(False)
        HLayoutBtn.addWidget(self.btnCalculate)
        HLayoutBtn.addWidget(self.btnSimulate)
        HLayoutBtn.addWidget(self.btnClear)

        VPosCtrl.addWidget(gbPos)
        VPosCtrl.addLayout(HLayoutBtn)

    def initFigureCanvas(self):
        self._widgetFigs = QtWidgets.QWidget()
        HLayoutFigs = QtWidgets.QHBoxLayout(self._widgetFigs)
        self.figCartesian = MplCanvas()
        self.figCartesian.fig.suptitle('Cartesian Plane')
        HLayoutFigs.addWidget(self.figCartesian)
        self.figVel = MplCanvas()
        self.figVel.fig.suptitle('Linear Velocity (m/s)')
        self.figOmega = MplCanvas()
        self.figOmega.fig.suptitle('Angular Velocity (rad/s)')
        VLayoutRightFigs = QtWidgets.QVBoxLayout()
        VLayoutRightFigs.addWidget(self.figVel)
        VLayoutRightFigs.addWidget(self.figOmega)
        HLayoutFigs.addLayout(VLayoutRightFigs)

    def initROSCtrl(self):
        self._widgetROSCtrl = QtWidgets.QWidget()
        VLayoutROS = QtWidgets.QVBoxLayout(self._widgetROSCtrl)

        gBoxROS = QtWidgets.QGroupBox('ROS Control')
        VLayoutGBoxRos = QtWidgets.QVBoxLayout(gBoxROS)

        HlayoutTimer = QtWidgets.QHBoxLayout()
        self.btnRosStartTimer = QtWidgets.QPushButton('START TIMER')
        self.btnRosStartTimer.clicked.connect(lambda: self.sig_ros_start_timer.emit())
        self.btnRosStopTimer = QtWidgets.QPushButton('STOP TIMER')
        self.btnRosStopTimer.clicked.connect(lambda: self.sig_ros_stop_timer.emit())
        HlayoutTimer.addWidget(self.btnRosStartTimer)
        HlayoutTimer.addWidget(self.btnRosStopTimer)

        HlayoutCmdVel = QtWidgets.QHBoxLayout()
        self.btnRosClearVelCmd = QtWidgets.QPushButton('CLEAR CMD_VEL')
        self.btnRosClearVelCmd.clicked.connect(lambda: self.sig_ros_clear_vel_cmd.emit())
        self.btnRosAddVelCmd = QtWidgets.QPushButton("ADD CMD_VEL")
        self.btnRosAddVelCmd.clicked.connect(self.on_btn_clicked_ros_add_vel_cmd)
        self.btnRosRegVelCmd = QtWidgets.QPushButton("REG CMD_VEL")
        self.btnRosRegVelCmd.clicked.connect(self.on_btn_clicked_ros_reg_vel_cmd)
        HlayoutCmdVel.addWidget(self.btnRosRegVelCmd)
        HlayoutCmdVel.addWidget(self.btnRosAddVelCmd)
        HlayoutCmdVel.addWidget(self.btnRosClearVelCmd)
        VLayoutGBoxRos.addLayout(HlayoutCmdVel)
        VLayoutGBoxRos.addLayout(HlayoutTimer)
        VLayoutROS.addWidget(gBoxROS)

    def on_btn_clicked_ros_add_vel_cmd(self):
        if bool(self.listVelC) and bool(self.listOmegaC):
            self.sig_ros_add_vel_cmd.emit((self.listVelC, self.listOmegaC))

    def on_btn_clicked_ros_reg_vel_cmd(self):
        if bool(self.listVelC) and bool(self.listOmegaC):
            self.sig_ros_reg_vel_cmd.emit((self.listVelC, self.listOmegaC))

    def on_btn_clicked_simulate(self):
        x_i, y_i = self.dspinInitX.value(), self.dspinInitY.value()
        dT = self.spinSamplingTime.value() / 1000.

        # note that the method simulate_robot_drive is inherited from CTMRDiff
        # any children of CTMRDiff will produce the same results
        x_c, y_c = CTMRKobuki2().simulate_robot_drive(x_i, y_i, self.listVelC, self.listThetaC, dT)

        if len(self.figCartesian.axes.lines) >= 2:
            self.figCartesian.axes.lines[1].remove()

        if len(self.figCartesian.axes.lines) >= 1:
            self.figCartesian.axes.plot(x_c, y_c, 'r', label="simulated")
            self.figCartesian.axes.legend(loc="upper left")
            self.figCartesian.draw()

    def on_btn_clicked_clear(self):
        self.figCartesian.axes.cla()
        self.figCartesian.draw()
        self.figVel.axes.cla()
        self.figVel.draw()
        self.figOmega.axes.cla()
        self.figOmega.draw()

    def on_btn_clicked_calculate(self):
        max_vel = self.dspinMaxVel.value()
        max_acc = self.dspinMaxAcc.value()
        max_jerk = self.dspinMaxJerk.value()
        dT = self.spinSamplingTime.value() / 1000.
        initPos = self.dspinInitX.value(), self.dspinInitY.value(), self.dspinInitTheta.value()
        targPos = self.dspinTargX.value(), self.dspinTargY.value(), self.dspinTargTheta.value()
        v_i = self.dspinInitVel.value()
        v_f = self.dspinTargVel.value()

        c_bezier = CBezierCurveTraj(max_vel, max_acc, max_jerk)
        try:
            vel_c, omega_c, XYTheta = c_bezier.generate_bezier_trajectory(initPos, targPos, v_i, v_f, dT)
            x_c = [x for x, y, angles in XYTheta]
            y_c = [y for x, y, angles in XYTheta]
            theta_c = [angles for x, y, angles in XYTheta]

            self.listVelC = vel_c
            self.listOmegaC = omega_c
            self.listThetaC = theta_c[1:]

            timestamp = 0.
            listTime = []
            for i in range(len(vel_c)):
                timestamp += dT
                listTime.append(timestamp)

            self.on_btn_clicked_clear()
            self.figCartesian.axes.plot(x_c, y_c, 'b', label="reference")
            self.figCartesian.axes.legend(loc="upper left")
            self.figCartesian.draw()
            self.figVel.axes.plot(listTime, vel_c, 'b')
            self.figVel.draw()
            self.figOmega.axes.plot(listTime, omega_c, 'b')
            # ax2 = self.figOmega.axes.twinx()
            # ax2.clear()
            # ax2.plot(listTime, self.listThetaC, 'cyan')
            self.figOmega.draw()

        except ValueError as err:
            msgbox = QtWidgets.QMessageBox(self)
            msgbox.setWindowTitle(self.windowTitle())
            msgbox.setIcon(QtWidgets.QMessageBox.Critical)
            msgbox.setText(str(err))
            msgbox.exec_()

    def on_text_change_cmb_robot(self, strText: str):
        if strText == "Custom":
            self.dspinMaxAcc.setValue(0.)
            self.dspinMaxVel.setValue(0.)
            self.dspinMaxJerk.setValue(0.)
            self.dspinMaxJerk.setValue(0)
            self.spinSamplingTime.setMinimum(0)
            self.spinSamplingTime.setValue(10)
        else:
            vel, acc, jerk, yaw = 0., 0., 0., 0.
            sampling_time = 0
            if strText == "StellaB2":
                sampling_time = 20
                vel, acc, jerk, yaw = CTMRStellaB2().get_phy_limits_t()

            elif strText == "Kobuki2":
                sampling_time = 50
                vel, acc, jerk, yaw = CTMRKobuki2().get_phy_limits_t()

            self.spinSamplingTime.setMinimum(sampling_time)
            self.spinSamplingTime.setValue(sampling_time)
            self.dspinMaxVel.setValue(vel)
            self.dspinMaxAcc.setValue(acc)
            self.dspinMaxJerk.setValue(jerk)


if __name__ == '__main__':
    app = QtCore.QCoreApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)

    MainUI = SubWindowTrajGen()
    # MainUI.show()

    app.exec_()
# MainUI.release()
