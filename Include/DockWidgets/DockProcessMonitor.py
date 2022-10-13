# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : DockStellaB2Ctrl.py
# Project Name : ExaRobotCtrl
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2022.02.03 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import queue
import sys
import datetime
import threading
import time
from typing import Tuple
import psutil

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Include/DOckWidgets
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
LIBRARY_PATH = os.path.join(ROOT_PATH, "Library")
SERIAL_PATH = os.path.join(LIBRARY_PATH, "Serial")
MDI_PATH = os.path.join(ROOT_PATH, "MdiBackground")
MISC_PATH = os.path.join(ROOT_PATH, "Misc")
RESOURCES_PATH = os.path.join(ROOT_PATH, "Resources")
sys.path.extend([INCLUDE_PATH, MDI_PATH, RESOURCES_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH

from UiCommon import *
from Commons import *


class DockProcessMonitor(QtWidgets.QDockWidget):
    sig_close = QtCore.pyqtSignal()
    sig_btn_refresh = QtCore.pyqtSignal()
    tableProcInfo: QtWidgets.QTableWidget
    _timer: QtCore.QTimer
    _threadUpdateTable: threading.Thread = None
    _queueTable: queue.Queue

    def closeEvent(self, closeEvent: QtGui.QCloseEvent) -> None:
        self.sig_close.emit()

    def __init__(self, parent=None):
        super(DockProcessMonitor, self).__init__(parent)
        self.setWindowTitle('Process Monitor')
        self._queueTable = queue.Queue()
        self.start_thread_update_table()
        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(lambda: self.sig_btn_refresh.emit())
        self.initUI()
        self.test = 0

    def initUI(self):
        centralWidget = QtWidgets.QWidget()
        MainLayout = QtWidgets.QVBoxLayout(centralWidget)
        self.setWidget(centralWidget)

        self.initTable()
        HLayoutBtn = QtWidgets.QHBoxLayout()
        self.btnRefresh = QtWidgets.QPushButton('REFRESH')
        self.btnRefresh.setIcon(EmbdQtIcon("reset.png"))
        self.btnRefresh.clicked.connect(lambda: self.sig_btn_refresh.emit())
        self.spinAutoRefresh = QtWidgets.QSpinBox()
        self.spinAutoRefresh.setValue(500)
        self.spinAutoRefresh.setMaximum(2000)
        self.spinAutoRefresh.setMinimum(500)
        self.spinAutoRefresh.setSingleStep(50)
        lblMS = QtWidgets.QLabel()
        lblMS.setText('[ms]')
        self.cbAutoRefresh = QtWidgets.QCheckBox('AUTO')
        self.cbAutoRefresh.stateChanged.connect(self.on_change_state)
        self.cbAutoRefresh.setChecked(False)

        HLayoutBtn.addWidget(self.btnRefresh)
        HLayoutBtn.addWidget(self.cbAutoRefresh)
        HLayoutBtn.addWidget(self.spinAutoRefresh)
        HLayoutBtn.addWidget(lblMS)
        HLayoutBtn.addStretch(1)

        MainLayout.addWidget(self.tableProcInfo)
        MainLayout.addLayout(HLayoutBtn)
        MainLayout.addStretch(1)

    def initTable(self):
        if is_system_win():
            listTableHHheader = ["PID", "STATUS", "CPU %", "NICE", "# of Threads", "CTX", "CREATE TIME"]
        else:
            listTableHHheader = ["PID", "STATUS", "CPU %", "CPU #", "NICE", "# of Threads", "CTX-V", "CTX-I",
                                 "CREATE TIME"]

        self.tableProcInfo = QtWidgets.QTableWidget()
        self.tableProcInfo.setColumnCount(len(listTableHHheader))
        self.tableProcInfo.setHorizontalHeaderLabels(listTableHHheader)
        self.tableProcInfo.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)

    def on_change_state(self, state: int):
        if state == QtCore.Qt.Checked:
            period = int(self.spinAutoRefresh.value())
            self._timer.start(period)
        else:
            self._timer.stop()
            with self._queueTable.mutex:
                self._queueTable.queue.clear()

    def on_update_dict(self, dict_proc: dict):
        self._queueTable.put(dict_proc)

    def start_thread_update_table(self):
        if self._threadUpdateTable is None:
            self._threadUpdateTable = ThreadUpdateTable(self, self._queueTable)
            self._threadUpdateTable.sig_terminated.connect(self.on_terminated_thread_update_table)
            self._threadUpdateTable.sig_update_list.connect(self.on_update_list_update_table)
            self._threadUpdateTable.start()

    def on_update_list_update_table(self, dictContents: dict):
        list_proc_name = dictContents.keys()
        self.tableProcInfo.setRowCount(len(list_proc_name))
        self.tableProcInfo.setVerticalHeaderLabels(list_proc_name)

        for i, proc_name in enumerate(list_proc_name):
            dictResult = dictContents[proc_name]
            self.tableProcInfo.setItem(i, 0, EmbdQtTableItem(dictResult["PID"], True))
            self.tableProcInfo.setItem(i, 1, EmbdQtTableItem(dictResult["STATUS"], True))
            self.tableProcInfo.setItem(i, 2, EmbdQtTableItem(dictResult["CPU_USAGE"], True))

            if is_system_win():
                self.tableProcInfo.setItem(i, 3, EmbdQtTableItem(dictResult["NICE"], True))
                self.tableProcInfo.setItem(i, 4, EmbdQtTableItem(dictResult["THREADS"], True))
                self.tableProcInfo.setItem(i, 5, EmbdQtTableItem(dictResult["CTX"], True))
                self.tableProcInfo.setItem(i, 6, EmbdQtTableItem(dictResult["CREATE_TM"], True))
            else:
                self.tableProcInfo.setItem(i, 3, EmbdQtTableItem(dictResult["CPU_NO"], True))
                self.tableProcInfo.setItem(i, 4, EmbdQtTableItem(dictResult["NICE"], True))
                self.tableProcInfo.setItem(i, 5, EmbdQtTableItem(dictResult["THREADS"], True))
                self.tableProcInfo.setItem(i, 6, EmbdQtTableItem(dictResult["CTX"], True))
                self.tableProcInfo.setItem(i, 7, EmbdQtTableItem(dictResult["CTX_INV"], True))
                self.tableProcInfo.setItem(i, 8, EmbdQtTableItem(dictResult["CREATE_TM"], True))

        self.tableProcInfo.viewport().update()
        self.tableProcInfo.resizeRowsToContents()

    def on_terminated_thread_update_table(self):
        del self._threadUpdateTable
        self._threadUpdateTable = None

    def stop_thread_update_table(self):
        if self._threadUpdateTable is not None:
            self._threadUpdateTable.stop()


class ThreadUpdateTable(QtCore.QThread):
    _keepAlive: bool = True
    sig_terminated = QtCore.pyqtSignal()
    sig_update_list = QtCore.pyqtSignal(dict)

    def __init__(self, aDock: DockProcessMonitor, q: queue.Queue):
        super(ThreadUpdateTable, self).__init__()
        self._handle = aDock
        self._queue = q

    def run(self) -> None:
        while self._keepAlive:
            try:
                if not self._queue.empty():
                    dict_proc = self._queue.get()
                    dict_update = dict()
                    list_proc_name = dict_proc.keys()

                    for proc_name in list_proc_name:
                        pid_no = dict_proc[proc_name]
                        dict_result = dict()

                        if pid_no is not None:
                            this_proc = psutil.Process(pid_no)
                            pid_no = this_proc.pid
                            status = this_proc.status()
                            cpu_usage = this_proc.cpu_percent(0.1)
                            niceness = this_proc.nice()
                            threads = this_proc.num_threads()
                            ctx_sw = this_proc.num_ctx_switches().voluntary
                            create_time = datetime.datetime.fromtimestamp(this_proc.create_time()).strftime(
                                "%Y-%m-%d %H:%M:%S")
                            dict_result["PID"] = pid_no
                            dict_result["STATUS"] = status
                            dict_result["CPU_USAGE"] = cpu_usage
                            dict_result["NICE"] = niceness
                            dict_result["THREADS"] = threads
                            dict_result["CTX"] = ctx_sw
                            dict_result["CREATE_TM"] = create_time
                            if not is_system_win():
                                cpu_no = this_proc.cpu_num()
                                ctx_sw_inv = this_proc.num_ctx_switches().involuntary
                                dict_result["CPU_NO"] = cpu_no
                                dict_result["CTX_INV"] = ctx_sw_inv

                            dict_update[proc_name] = dict_result

                    self.sig_update_list.emit(dict_update)
                    time.sleep(1e-3)
                else:
                    time.sleep(1e-6)
            except Exception:
                pass  # todo: add exception handler

        self.sig_terminated.emit()

    def stop(self):
        self._keepAlive = False


if __name__ == '__main__':
    app = QtCore.QCoreApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)

    MainUI = DockProcessMonitor()
    MainUI.show()

    app.exec_()
