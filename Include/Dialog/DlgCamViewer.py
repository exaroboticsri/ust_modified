# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : DlgCamViewer.py
# Project Name : ExaRobotCtrl
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2022.02.09 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import sys
import cv2

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
from CameraCommon import COMMON_RESOLUTION_D


class DlgCamViewer(QtWidgets.QDialog):
    _widgetCtrl: QtWidgets.QWidget = None
    _pixmapImage: QtWidgets.QLabel = None
    sig_close = QtCore.pyqtSignal()
    sig_start = QtCore.pyqtSignal(int, int, int)
    sig_stop = QtCore.pyqtSignal()
    sig_pause = QtCore.pyqtSignal()

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        self.sig_close.emit()
        super().closeEvent(a0)

    def __init__(self, parent=None):
        super(DlgCamViewer, self).__init__(parent)
        self.setWindowTitle('Camera Viewer')
        self.initUi()

    def initUi(self):
        self.initCtrl()
        self._pixmapImage = QtWidgets.QLabel()
        VLayoutMain = QtWidgets.QVBoxLayout(self)

        HLayoutImage = QtWidgets.QHBoxLayout()
        HLayoutImage.addStretch(1)
        HLayoutImage.addWidget(self._pixmapImage)
        HLayoutImage.addStretch(1)
        VLayoutMain.addLayout(HLayoutImage, 1)
        VLayoutMain.addWidget(self._widgetCtrl)

    def initCtrl(self):
        self._widgetCtrl = QtWidgets.QWidget()
        HLayoutCtrl = QtWidgets.QHBoxLayout(self._widgetCtrl)
        lblCameraID = QtWidgets.QLabel()
        lblCameraID.setText('CAM ID:')
        self.cmbCamID = QtWidgets.QComboBox()
        self.cmbCamID.addItems([str(i) for i in range(10)])
        lblCameraRes = QtWidgets.QLabel()
        lblCameraRes.setText('RESOLUTION:')
        self.cmbCamRes = QtWidgets.QComboBox()
        self.cmbCamRes.addItems([res for res in COMMON_RESOLUTION_D.keys()])
        self.btnStart = QtWidgets.QPushButton('START')
        self.btnStart.clicked.connect(self.on_clicked_btn_start)
        self.btnStop = QtWidgets.QPushButton('STOP')
        self.btnStop.clicked.connect(lambda: self.sig_stop.emit())
        self.btnPause = QtWidgets.QPushButton('PAUSE')
        self.btnPause.clicked.connect(lambda: self.sig_pause.emit())

        HLayoutCtrl.addWidget(lblCameraID)
        HLayoutCtrl.addWidget(self.cmbCamID, 1)
        HLayoutCtrl.addWidget(lblCameraRes)
        HLayoutCtrl.addWidget(self.cmbCamRes, 1)
        HLayoutCtrl.addWidget(self.btnStart)
        HLayoutCtrl.addWidget(self.btnStop)
        HLayoutCtrl.addWidget(self.btnPause)

    def on_clicked_btn_start(self):
        cam_id = int(self.cmbCamID.currentText())
        resolution = str(self.cmbCamRes.currentText())
        width, height = COMMON_RESOLUTION_D[resolution]
        self.sig_start.emit(cam_id, width, height)

    def on_get_image(self, data: tuple):
        image, = data
        qImage = QtGui.QImage(image.data, image.shape[1], image.shape[0], QtGui.QImage.Format_RGB888)
        self._pixmapImage.setPixmap(QtGui.QPixmap.fromImage(qImage))

    def on_stopped_cam(self):
        self._pixmapImage.clear()


if __name__ == '__main__':
    app = QtCore.QCoreApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)

    MainUI = DlgCamViewer()
    MainUI.show()

    app.exec_()
