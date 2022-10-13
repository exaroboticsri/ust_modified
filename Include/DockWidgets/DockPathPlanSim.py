# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : DockPathPlanSim.py
# Project Name : MobileRobotTest
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2021.08.24 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import sys

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include/DOckWidgets
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
MDI_PATH = os.path.join(ROOT_PATH, "MdiBackground")
MISC_PATH = os.path.join(ROOT_PATH, "Misc")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
sys.path.extend([INCLUDE_PATH, MDI_PATH, RESOURCES_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, RESOURCES_PATH

from UiCommon import *
from Commons import *


class DockPathPlanSim(QtWidgets.QDockWidget):
    sig_planner_pos = QtCore.pyqtSignal(str, tuple, tuple)  # type, start(x,y) and goal(x,y)
    def_fontSize: int = 10
    sig_close = QtCore.pyqtSignal()

    def closeEvent(self, closeEvent: QtGui.QCloseEvent) -> None:
        self.sig_close.emit()

    def __init__(self, parent=None):
        super(DockPathPlanSim, self).__init__(parent)
        self.setWindowTitle('ROS Path Planning Simulation')
        self.initUi()
        self.initSlots()
        self.setDefaults()

    def initUi(self):
        self.initPathPlannerSelector()
        centralWidget = QtWidgets.QWidget(self)
        self.setWidget(centralWidget)
        MainLayout = QtWidgets.QVBoxLayout(centralWidget)
        MainLayout.setSpacing(6)
        MainLayout.setContentsMargins(0, 0, 0, 0)
        MainLayout.addWidget(self._gbxPathPlannerSelector)
        MainLayout.addStretch(1)

    def initPathPlannerSelector(self):
        self._gbxPathPlannerSelector = QtWidgets.QGroupBox()
        VLayoutPathPlanner = QtWidgets.QVBoxLayout(self._gbxPathPlannerSelector)
        self.cmbPathPlannerSelector = QtWidgets.QComboBox()
        VLayoutPathPlanner.addWidget(self.cmbPathPlannerSelector)

        HLayoutStart = QtWidgets.QHBoxLayout()
        lblXStart = QtWidgets.QLabel('X_START')
        self.spinXStart = QtWidgets.QSpinBox()
        self.spinXStart.setMaximum(51)
        self.spinXStart.setMinimum(0)
        self.spinXStart.setValue(5)
        lblYStart = QtWidgets.QLabel('Y_START')
        self.spinYStart = QtWidgets.QSpinBox()
        self.spinYStart.setMaximum(51)
        self.spinYStart.setMinimum(0)
        self.spinYStart.setValue(5)
        HLayoutStart.addWidget(lblXStart)
        HLayoutStart.addWidget(self.spinXStart, 1)
        HLayoutStart.addWidget(lblYStart)
        HLayoutStart.addWidget(self.spinYStart, 1)

        HLayoutGoal = QtWidgets.QHBoxLayout()
        lblXGoal = QtWidgets.QLabel('X_GOAL')
        self.spinXGoal = QtWidgets.QSpinBox()
        self.spinXGoal.setMaximum(51)
        self.spinXGoal.setMinimum(0)
        self.spinXGoal.setValue(45)
        lblYGoal = QtWidgets.QLabel('Y_GOAL')
        self.spinYGoal = QtWidgets.QSpinBox()
        self.spinYGoal.setMaximum(51)
        self.spinYGoal.setMinimum(0)
        self.spinYGoal.setValue(25)
        HLayoutGoal.addWidget(lblXGoal)
        HLayoutGoal.addWidget(self.spinXGoal, 1)
        HLayoutGoal.addWidget(lblYGoal)
        HLayoutGoal.addWidget(self.spinYGoal, 1)

        self.btnStartPlanning = QtWidgets.QPushButton()
        self.btnStartPlanning.setText('START PLANNER')

        VLayoutPathPlanner.addLayout(HLayoutStart)
        VLayoutPathPlanner.addLayout(HLayoutGoal)
        VLayoutPathPlanner.addWidget(self.btnStartPlanning)

    def initSlots(self):
        self.btnStartPlanning.clicked.connect(self.onClickBtnStartPlanning)

    def setDefaults(self):
        listPlanner = ["AStar", "Dijkstra"]
        self.cmbPathPlannerSelector.addItems(listPlanner)

    def onClickBtnStartPlanning(self):
        type = str(self.cmbPathPlannerSelector.currentText())
        x_start = self.spinXStart.value()
        y_start = self.spinYStart.value()
        x_goal = self.spinXGoal.value()
        y_goal = self.spinYGoal.value()
        self.sig_planner_pos.emit(type, (x_start, y_start), (x_goal, y_goal))
