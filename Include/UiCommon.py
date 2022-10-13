# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : UiCommon.py
# Project Name : ExaRobotCtrl
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2021.08.24 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
from PyQt5 import QtCore, QtGui, QtWidgets
import os
import sys

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include
ROOT_PATH = os.path.dirname(FILE_PATH)
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
sys.path.extend([INCLUDE_PATH, RESOURCES_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, RESOURCES_PATH

from Commons import is_system_win


### Wrapper for QtIcon to either embed the image or use absolute path ###
def EmbdQtIcon(icon_name: str, embedded=True) -> QtGui.QIcon:
	if embedded:
		import resources
		icon_prefix = ":/icons/"
	else:
		current_path = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/UI
		root_path = os.path.dirname(current_path)
		resources_path = os.path.join(root_path, "Resources")
		icon_prefix = str(resources_path)
		if is_system_win():
			icon_prefix += "\\"
		else:
			icon_prefix += "/"

	icon_path = icon_prefix + icon_name
	return QtGui.QIcon(icon_path)


### Wrapper for QtPixmap to either embed the image or use absolute path ###
def EmbdQtPixmap(icon_name: str, embedded=True) -> QtGui.QPixmap:
	if embedded:
		import resources
		icon_prefix = ":/icons/"
	else:
		current_path = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/UI
		root_path = os.path.dirname(current_path)
		resources_path = os.path.join(root_path, "Resources")
		icon_prefix = str(resources_path)
		if is_system_win():
			icon_prefix += "\\"
		else:
			icon_prefix += "/"

	icon_path = icon_prefix + icon_name
	return QtGui.QPixmap(icon_path)


def EmbdQtTableItem(aTableItem: object, abIsReadOnly=False, abIsCenter: bool = True) -> QtWidgets.QTableWidgetItem:
	temp = QtWidgets.QTableWidgetItem(str(aTableItem))
	if abIsReadOnly:
		temp.setFlags(temp.flags() ^ QtCore.Qt.ItemIsEditable)

	if abIsCenter:
		temp.setTextAlignment(QtCore.Qt.AlignCenter)

	return temp