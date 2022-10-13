# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : SerialLogWidget.py
# Project Name : PySerialTerm
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2022.02.20 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import sys
import threading
import queue
import time
from typing import List, Union

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Scripts
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
SCRIPTS_PATH = os.path.join(ROOT_PATH, "Scripts")
WIDGETS_PATH = os.path.join(SCRIPTS_PATH, "Widgets")
LIBRARY_PATH = os.path.join(ROOT_PATH, "Library")
SERIAL_PATH = os.path.join(LIBRARY_PATH, "Serial")
sys.path.extend([SCRIPTS_PATH, RESOURCES_PATH, WIDGETS_PATH, LIBRARY_PATH, SERIAL_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, SCRIPTS_PATH, RESOURCES_PATH, WIDGETS_PATH, LIBRARY_PATH, SERIAL_PATH

from UiCommon import *
from Commons import *


class LogTable(QtWidgets.QTableWidget):
	listHorHeader: List[str]

	def __init__(self, parent=None):
		super(LogTable, self).__init__(parent)
		self.listHorHeader = ["TIME", "RAW(Hex)", "ASCII(Char)"]
		self.setColumnCount(len(self.listHorHeader))
		self.setHorizontalHeaderLabels(self.listHorHeader)
		self.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
		self.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Interactive)
		self.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
		self.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.Stretch)
		self.setSelectionBehavior(QtWidgets.QTableView.SelectRows)
		self.verticalHeader().hide()

	def clearTable(self):
		self.setRowCount(0)

	def addRow(self, strTime: str, btData: bytearray):
		self.insertRow(self.rowCount())
		strText = str()
		strAscii = str()
		for data in btData:
			strText += "%0.2X" % data + "|"
			strAscii += bytes.fromhex("%0.2X" % data).decode("ISO-8859-1")

		self.setItem(self.rowCount()-1, 0, EmbdQtTableItem(strTime, abIsReadOnly=True, abIsCenter=False))
		self.setItem(self.rowCount()-1, 1, EmbdQtTableItem(strText, abIsReadOnly=True, abIsCenter=False))
		self.setItem(self.rowCount()-1, 2, EmbdQtTableItem(strAscii, abIsReadOnly=True, abIsCenter=False))
		self.viewport().update()  # refresh table contents


class CWidgetSerialLog(QtWidgets.QWidget):
	_threadAdder: Union[None, threading.Thread] = None
	_queueAdder: queue.Queue

	def __init__(self, astrName="", parent=None):
		super(CWidgetSerialLog, self).__init__(parent)
		self.strName = astrName
		self._queueAdder = queue.Queue()
		self.initUi()
		self.initThreadAdder()

	def initUi(self):
		MainLayout = QtWidgets.QVBoxLayout(self)
		HLayoutTitle = QtWidgets.QHBoxLayout()
		labelName = QtWidgets.QLabel()
		labelName.setText(self.strName.upper() + " LOG")
		self.btnClear = QtWidgets.QPushButton('CLEAR')
		self.btnClear.clicked.connect(self.on_click_btn_clear)
		HLayoutTitle.addWidget(labelName)
		HLayoutTitle.addWidget(self.btnClear)
		HLayoutTitle.addStretch(1)
		self.widgetLogTable = LogTable()

		MainLayout.addLayout(HLayoutTitle)
		MainLayout.addWidget(self.widgetLogTable, 1)
		MainLayout.setContentsMargins(0, 0, 0, 0)

	def initThreadAdder(self):
		if self._threadAdder is None:
			self.clear_queue()
			self._threadAdder = ThreadAddTableContents(self, self._queueAdder)
			self._threadAdder.sig_terminated.connect(self.on_terminate_thread_adder)
			self._threadAdder.start()

	def clear_queue(self):
		try:
			while not self._queueAdder.empty():
				self._queueAdder.get()
		except Exception:
			pass

	def stopThreadAdder(self):
		if self._threadAdder is not None:
			self._threadAdder.stop()

	def on_terminate_thread_adder(self):
		del self._threadAdder
		self._threadAdder = None

	def on_click_btn_clear(self):
		self.widgetLogTable.clearTable()

	def add_table_contents(self, strTime: str, btaData: bytearray):
		self._queueAdder.put((strTime, btaData))


class ThreadAddTableContents(threading.Thread):
	_keepAlive = True

	def __init__(self, s: CWidgetSerialLog, q: queue.Queue):
		super(ThreadAddTableContents, self).__init__()
		self.setDaemon(True)
		self.handler = s
		self.queue = q
		self.sig_terminated = PySignal()

	def run(self) -> None:
		while self._keepAlive:
			try:
				if not self.queue.empty():
					strTime, btaData = self.queue.get()
					self.handler.widgetLogTable.addRow(strTime, btaData)
				else:
					time.sleep(1e-6)
			except Exception:
				pass

		self.sig_terminated.emit()

	def stop(self):
		self._keepAlive = False


if __name__ == '__main__':
	app = QtCore.QCoreApplication.instance()
	if app is None:
		app = QtWidgets.QApplication(sys.argv)

	app.setStyle('fusion')
	MainUI = CWidgetSerialLog('TX')
	MainUI.show()

	app.exec_()
