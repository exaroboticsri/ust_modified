# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : main.py
# Project Name : MobileRobotTest
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2021.08.24 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import sys

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include
ROOT_PATH = FILE_PATH
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
sys.path.extend([FILE_PATH, ROOT_PATH, INCLUDE_PATH])
sys.path = list(set(sys.path))

del FILE_PATH, ROOT_PATH, INCLUDE_PATH

from MainWindow import MainWindow
from ControlCore import EmbdControlCore
from UiCommon import *


def main():
    controlApp = EmbdControlCore()
    app = QtCore.QCoreApplication.instance()

    if app is None:
        app = QtWidgets.QApplication(sys.argv)

    app.setStyle('fusion')
    MainUI = MainWindow(controlApp)
    MainUI.show()

    app.exec_()
    MainUI.release()


if __name__ == '__main__':
    main()
