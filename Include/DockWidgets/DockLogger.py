# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : DockLogger.py
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


class DockLogger(QtWidgets.QDockWidget):
    def_fontSize: int = 10
    _writeThread = None
    sig_close = QtCore.pyqtSignal()

    def closeEvent(self, closeEvent: QtGui.QCloseEvent) -> None:
        self.sig_close.emit()

    def __init__(self, my_title: str = "", parent=None):
        super(DockLogger, self).__init__(parent)
        self.setWindowTitle(my_title)

        self.initUi()

    def initUi(self):
        self.initMenuBar()
        centralWidget = QtWidgets.QWidget(self)
        self.setWidget(centralWidget)

        self.editLogger = QtWidgets.QTextEdit()
        self.editLogger.setReadOnly(True)
        self.editLogger.setLineWrapColumnOrWidth(-1)
        self.editLogger.setLineWrapMode(QtWidgets.QTextEdit.FixedPixelWidth)


        MainLayout = QtWidgets.QVBoxLayout(centralWidget)
        MainLayout.setContentsMargins(0, 0, 0, 2)
        MainLayout.setSpacing(2)
        MainLayout.addWidget(self.menuBar)
        MainLayout.addWidget(self.editLogger, 1)

    def initMenuBar(self):
        self.menuBar = QtWidgets.QWidget(self)
        HLayoutMenuBar = QtWidgets.QHBoxLayout(self.menuBar)
        HLayoutMenuBar.setContentsMargins(5, 0, 0, 0)
        HLayoutMenuBar.setSpacing(4)

        self.cbAutoClear = QtWidgets.QCheckBox('Auto Clear')
        self.spbAutoClear = QtWidgets.QSpinBox()
        self.spbAutoClear.setMinimum(1)
        self.spbAutoClear.setMaximum(5000)
        self.spbAutoClear.setValue(1000)
        self.cbFollow = QtWidgets.QCheckBox('Follow')

        self.btnClear = QtWidgets.QPushButton()
        self.btnClear.setMaximumWidth(22)
        self.btnClear.setIcon(EmbdQtIcon('clear.png'))
        self.btnClear.setIconSize(QtCore.QSize(20, 20))
        self.btnClear.setFlat(True)

        self.btnSave = QtWidgets.QPushButton()
        self.btnSave.setMaximumWidth(22)
        self.btnSave.setIcon(EmbdQtIcon('save.png'))
        self.btnSave.setIconSize(QtCore.QSize(20, 20))
        self.btnSave.setFlat(True)

        self.btnFontUp = QtWidgets.QPushButton()
        self.btnFontUp.setMaximumWidth(22)
        self.btnFontUp.setIcon(EmbdQtIcon('font_plus.png'))
        self.btnFontUp.setIconSize(QtCore.QSize(20, 20))
        self.btnFontUp.setFlat(True)

        self.btnFontDown = QtWidgets.QPushButton()
        self.btnFontDown.setMaximumWidth(22)
        self.btnFontDown.setIcon(EmbdQtIcon('font_minus.png'))
        self.btnFontDown.setIconSize(QtCore.QSize(20, 20))
        self.btnFontDown.setFlat(True)

        HLayoutMenuBar.addWidget(self.cbAutoClear)
        HLayoutMenuBar.addWidget(self.spbAutoClear)
        HLayoutMenuBar.addWidget(self.cbFollow)
        HLayoutMenuBar.addWidget(self.btnClear)
        HLayoutMenuBar.addWidget(self.btnSave)
        HLayoutMenuBar.addWidget(self.btnFontUp)
        HLayoutMenuBar.addWidget(self.btnFontDown)
        HLayoutMenuBar.addStretch(1)
