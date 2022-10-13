# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : EmbdLed.py
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


class EmbdLed(QtWidgets.QPushButton):
	default_size = 20

	def __init__(self, title: str = "", parent=None):
		super(EmbdLed, self).__init__(text=title, parent=parent)
		self.iconBlack: QtGui.QIcon = EmbdQtIcon("led_gray.png")
		self.iconGreen: QtGui.QIcon = EmbdQtIcon("led_green.png")
		self.iconYellow: QtGui.QIcon = EmbdQtIcon("led_yellow.png")
		self.iconRed: QtGui.QIcon = EmbdQtIcon("led_red.png")
		self.setTextSize(10)
		self.setFlat(True)
		self.TurnOff()
		self.setLedSize(10 ,10)

	def setTextSize(self, font_size: int):
		btnFont = QtGui.QFont()
		btnFont.setFamily("Decorative")
		btnFont.setPointSize(font_size)
		self.setFont(btnFont)

	def setLedSize(self, width: int, height: int):
		self.setIconSize(QtCore.QSize(width, height))

	def TurnOff(self):
		self.setIcon(self.iconBlack)

	def TurnOnG(self):
		self.setIcon(self.iconGreen)

	def TurnOnY(self):
		self.setIcon(self.iconYellow)

	def TurnOnR(self):
		self.setIcon(self.iconRed)
