# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MdiBackground.py
# Project Name : MobileRobotTest
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2021.08.24 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import sys

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include/MdiBackground
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
sys.path.extend(
    [INCLUDE_PATH, RESOURCES_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, RESOURCES_PATH

from UiCommon import *


class EmbdMdiBground(QtWidgets.QMdiArea):
    _sigReSizeEvent = QtCore.pyqtSignal(QtCore.QSize)

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:
        self._sigReSizeEvent.emit(event.size())
        event.accept()

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        # return super().mousePressEvent(event)
        self.setFocus()

    def __init__(self, parent=None):
        QtWidgets.QMdiArea.__init__(self, parent)
        self.strInfoEnvTxt = ""

        self.m_pixmapLeftLogo = EmbdQtPixmap("seoultech_emblem.gif").scaled(150, 150, QtCore.Qt.KeepAspectRatio)
        self.m_pixmapRightLogo = EmbdQtPixmap("seoultech_wide2.jpg").scaled(400, 150, QtCore.Qt.KeepAspectRatio)
        self.m_nMargin = 3

        minWidth = self.m_pixmapLeftLogo.width() + self.m_pixmapRightLogo.width() + 4 * self.m_nMargin
        self.setMinimumWidth(minWidth)
        minHeight = max(self.m_pixmapLeftLogo.height(), self.m_pixmapRightLogo.height()) + 2 * self.m_nMargin
        self.setMinimumHeight(minHeight)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self.viewport())
        painter.fillRect(event.rect(), QtGui.QBrush(QtCore.Qt.white))

        #  This widget size
        nMyWidth, nMyHeight = self.width(), self.height()

        # Left Logo
        nPixmapWidth, nPixmapHeight = self.m_pixmapLeftLogo.width(), self.m_pixmapLeftLogo.height()
        nXPos, nYPos = 0, nMyHeight - nPixmapHeight
        nXPos, nYPos = nXPos + self.m_nMargin, nYPos - self.m_nMargin
        painter.drawPixmap(nXPos, nYPos, nPixmapWidth, nPixmapHeight, self.m_pixmapLeftLogo)

        # Right Logo
        nPixmapWidthB, nPixmapHeightB = self.m_pixmapRightLogo.width(), self.m_pixmapRightLogo.height()
        nXPosB, nYPosB = nMyWidth - nPixmapWidthB, nMyHeight - nPixmapHeightB
        nXPosB, nYPosB = nXPosB - self.m_nMargin, nYPosB - self.m_nMargin
        painter.drawPixmap(nXPosB, nYPosB, nPixmapWidthB, nPixmapHeightB, self.m_pixmapRightLogo)

        # Text painter should be last
        painter.setPen(QtGui.QColor('Black'))
        painter.setFont(QtGui.QFont('Decorative', 12))

        textRect = QtCore.QRect(self.m_nMargin, self.m_nMargin, nMyWidth, 500)
        painter.drawText(textRect, QtCore.Qt.AlignLeft, self.strInfoEnvTxt)

    def setInfoEnv(self, strAppName: str, strAppVer: str, strAppBuild: str, strAuthor: str):
        import platform
        import datetime
        strPrintText = strAppName + "\n"
        strPrintText += "Version: " + strAppVer + "\n"
        strPrintText += "Build: " + strAppBuild + "\n"
        strPrintText += "Developer: " + strAuthor + "\n\n"

        strPrintText += "Python %s (%s)\n" % (platform.python_version(), platform.architecture()[0])
        strPrintText += "Processor: " + str(platform.processor()) + "\n"
        strPrintText += "Machine: " + str(platform.machine()) + "\n"
        strPrintText += "OS: " + str(platform.platform()) + "\n\n"

        now_year = str(datetime.datetime.now().year)
        strPrintText += "Copyright © %s, Embedded System Laboratory - SeoulTech. All Rights Reserved." % now_year
        self.strInfoEnvTxt = strPrintText