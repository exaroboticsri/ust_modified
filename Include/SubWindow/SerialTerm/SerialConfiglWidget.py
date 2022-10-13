# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : SerialConfiglWidget.py
# Project Name : PySerialTerm
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2022.02.19 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import sys

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
from SerialPort import get_serial_port_name_list, get_serial_port_description, get_serial_baud_list


class CWidgetSerialConfig(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(CWidgetSerialConfig, self).__init__(parent)
        self.sig_serial_connect = PySignal(str, int)
        self.sig_serial_disconnect = PySignal()
        self.initUi()
        self.on_click_btn_refresh()

    def initUi(self):
        MainLayout = QtWidgets.QVBoxLayout(self)

        # Serial Info
        HLayoutPort = QtWidgets.QHBoxLayout()
        lblPort = QtWidgets.QLabel()
        lblPort.setText('PORT')
        self.cmbSerialPort = QtWidgets.QComboBox()
        self.cmbSerialPort.currentTextChanged.connect(self.on_text_change_cmb_port)
        lblBaud = QtWidgets.QLabel()
        lblBaud.setText('BAUD')
        self.cmbSerialBaud = QtWidgets.QComboBox()
        self.cmbSerialBaud.addItems(baud for baud in get_serial_baud_list())
        self.cmbSerialBaud.setCurrentIndex(8)  # 115200
        self.btnRefresh = QtWidgets.QPushButton()
        self.btnRefresh.clicked.connect(self.on_click_btn_refresh)
        self.btnRefresh.setIcon(EmbdQtIcon("reset.png"))
        # self.btnRefresh.setFlat(True)
        # self.btnResetSerialPort.setIconSize(QtCore.QSize(mb_icon_size, mb_icon_size))
        # self.btnResetSerialPort.setFixedSize(QtCore.QSize(mb_icon_size + 2, mb_icon_size + 2))
        HLayoutPort.addWidget(lblPort)
        HLayoutPort.addWidget(self.cmbSerialPort, 1)
        HLayoutPort.addWidget(lblBaud)
        HLayoutPort.addWidget(self.cmbSerialBaud, 1)
        HLayoutPort.addWidget(self.btnRefresh)

        # Serial Control
        HLayoutCtrl = QtWidgets.QHBoxLayout()
        self.btnConnect = QtWidgets.QPushButton('CONNECT')
        self.btnConnect.clicked.connect(self.on_click_btn_connect)
        self.btnDisConnect = QtWidgets.QPushButton('DISCONNECT')
        self.btnDisConnect.clicked.connect(self.on_click_btn_disconnect)
        HLayoutCtrl.addWidget(self.btnConnect)
        HLayoutCtrl.addWidget(self.btnDisConnect)

        # Serial Description
        self.txtSerialDesc = QtWidgets.QLineEdit()
        self.txtSerialDesc.setReadOnly(True)

        MainLayout.addLayout(HLayoutPort)
        MainLayout.addLayout(HLayoutCtrl)
        MainLayout.addWidget(self.txtSerialDesc)
        MainLayout.setContentsMargins(0,0,0,0)

    def on_click_btn_disconnect(self):
        self.sig_serial_disconnect.emit()

    def on_click_btn_connect(self):
        port = self.cmbSerialPort.currentText()
        baud = int(self.cmbSerialBaud.currentText())
        self.sig_serial_connect.emit(port, baud)

    def on_text_change_cmb_port(self, device: str):
        desc = get_serial_port_description(device)
        self.txtSerialDesc.setText(desc)

    def on_click_btn_refresh(self):
        self.cmbSerialPort.clear()
        self.cmbSerialPort.addItems(com_port for com_port in get_serial_port_name_list())

    def on_serial_connected_state(self, state: bool):
        self.btnDisConnect.setEnabled(state)
        self.btnConnect.setEnabled(not state)
        self.btnRefresh.setEnabled(not state)
        self.cmbSerialPort.setEnabled(not state)
        self.cmbSerialBaud.setEnabled(not state)

    def release(self):
        self.close()
