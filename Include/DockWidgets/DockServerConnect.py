# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : DockServerConnect.py
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


class DockServerConnect(QtWidgets.QDockWidget):
    def_fontSize: int = 10
    _writeThread = None
    sig_close = QtCore.pyqtSignal()

    def closeEvent(self, closeEvent: QtGui.QCloseEvent) -> None:
        self.sig_close.emit()

    sig_start_server = QtCore.pyqtSignal(str, str, int)  # ip address, port
    sig_disconnect_client = QtCore.pyqtSignal(tuple)
    sig_stop_server = QtCore.pyqtSignal()
    sig_send_msg = QtCore.pyqtSignal(str)
    _sckType: str

    def __init__(self, parent=None):
        super(DockServerConnect, self).__init__(parent)
        self.setWindowTitle('Server Connection')
        self.initUi()
        self.initSlots()

    def initUi(self):
        self.initAddressPort()
        self.initSocketType()
        self.initButtons()
        self.initConnectionList()
        self.initTestSender()
        self.initClientList()

        centralWidget = QtWidgets.QWidget(self)
        self.setWidget(centralWidget)

        MainLayout = QtWidgets.QVBoxLayout(centralWidget)
        MainLayout.setSpacing(6)
        MainLayout.setContentsMargins(0, 0, 0, 0)
        MainLayout.addWidget(self.wdgAddressPort)
        MainLayout.addWidget(self.wdgSocketType)
        MainLayout.addWidget(self.wdgButtons)
        MainLayout.addWidget(self.gbxConnection, 1)
        MainLayout.addWidget(self.wdgTestSender)
        MainLayout.addWidget(self.gbxClientList)

        self.onServerStart(False)

    def initAddressPort(self):
        self.wdgAddressPort = QtWidgets.QWidget()
        HLayoutAddressPort = QtWidgets.QHBoxLayout(self.wdgAddressPort)
        HLayoutAddressPort.setContentsMargins(5, 0, 0, 0)

        lblAddress = QtWidgets.QLabel()
        lblAddress.setText("Address")

        ipRange = "(?:[0-1]?[0-9]?[0-9]|2[0-4][0-9]|25[0-5])"  # Part of the regular expression
        ipRegex = QtCore.QRegExp("^" + ipRange + "\\." + ipRange + "\\." + ipRange + "\\." + ipRange + "$")
        ipValidator = QtGui.QRegExpValidator(ipRegex, self)
        self.txtAddress = QtWidgets.QLineEdit()
        self.txtAddress.setPlaceholderText("0.0.0.0")
        self.txtAddress.setValidator(ipValidator)

        lblPort = QtWidgets.QLabel()
        lblPort.setText("Port")

        self.spbPort = QtWidgets.QSpinBox()
        self.spbPort.setMinimum(0)
        self.spbPort.setMaximum(65535)
        self.spbPort.setValue(7506)
        HLayoutAddressPort.addWidget(lblAddress)
        HLayoutAddressPort.addWidget(self.txtAddress, 1)
        HLayoutAddressPort.addWidget(lblPort)
        HLayoutAddressPort.addWidget(self.spbPort)

    def initSocketType(self):
        self.wdgSocketType = QtWidgets.QWidget()
        HLayoutSocketType = QtWidgets.QHBoxLayout(self.wdgSocketType)
        HLayoutSocketType.setContentsMargins(5, 0, 0, 0)
        self.rbtnTcp = QtWidgets.QRadioButton('TCP/IP')
        self.rbtnTcp.sckType = "TCP"
        self._sckType = "TCP"
        self.rbtnTcp.setChecked(True)
        self.rbtnUdp = QtWidgets.QRadioButton('UDP')
        self.rbtnUdp.sckType = "UDP"
        self.rbtnUdp.setEnabled(False)
        HLayoutSocketType.addWidget(self.rbtnTcp)
        HLayoutSocketType.addWidget(self.rbtnUdp)

    def initButtons(self):
        self.wdgButtons = QtWidgets.QWidget()
        HLayoutButtons = QtWidgets.QHBoxLayout(self.wdgButtons)
        HLayoutButtons.setContentsMargins(5, 0, 0, 0)

        self.btnStart = QtWidgets.QPushButton('START')
        self.btnStart.setIcon(EmbdQtIcon('ethernet_open.png'))
        self.btnStop = QtWidgets.QPushButton('STOP')
        self.btnStop.setIcon(EmbdQtIcon('ethernet_close.png'))

        HLayoutButtons.addWidget(self.btnStart)
        HLayoutButtons.addWidget(self.btnStop)

    def initConnectionList(self):
        self.gbxConnection = QtWidgets.QGroupBox('Server Adaptor List')
        VLayoutConnection = QtWidgets.QVBoxLayout(self.gbxConnection)

        HLayoutConnect = QtWidgets.QHBoxLayout()
        self.btnRefresh = QtWidgets.QPushButton('Refresh')
        self.btnRefresh.setIcon(EmbdQtIcon('reset.png'))
        HLayoutConnect.addWidget(self.btnRefresh)
        HLayoutConnect.addStretch(1)

        hHeader = ["Adaptor Name", "IPv4 Address"]
        self.tblAdaptorList = QtWidgets.QTableWidget()
        self.tblAdaptorList.setColumnCount(len(hHeader))
        self.tblAdaptorList.setHorizontalHeaderLabels(hHeader)
        self.tblAdaptorList.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        self.tblAdaptorList.setSelectionBehavior(QtWidgets.QTableView.SelectRows)
        self.tblAdaptorList.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.tblAdaptorList.doubleClicked.connect(self.onDoubleClickedAdaptor)
        VLayoutConnection.addLayout(HLayoutConnect)
        VLayoutConnection.addWidget(self.tblAdaptorList, 1)

    def initClientList(self):
        self.gbxClientList = QtWidgets.QGroupBox('Client List')
        VLayoutConnection = QtWidgets.QVBoxLayout(self.gbxClientList)
        hHeader = ["IPv4 Address", "Port"]
        self.tblClientList = QtWidgets.QTableWidget()
        self.tblClientList.setColumnCount(len(hHeader))
        self.tblClientList.setHorizontalHeaderLabels(hHeader)
        self.tblClientList.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        self.tblClientList.setSelectionBehavior(QtWidgets.QTableView.SelectRows)
        self.tblClientList.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        HLayoutDisconnectClient = QtWidgets.QHBoxLayout()
        self.btnDisconnectClient = QtWidgets.QPushButton('DISCONNECT')
        HLayoutDisconnectClient.addStretch()
        HLayoutDisconnectClient.addWidget(self.btnDisconnectClient)
        VLayoutConnection.addWidget(self.tblClientList, 1)
        VLayoutConnection.addLayout(HLayoutDisconnectClient)

    def onDoubleClickedAdaptor(self):
        self.txtAddress.setText(str(self.tblAdaptorList.item(self.tblAdaptorList.currentRow(), 1).text()))

    def initTestSender(self):
        self.wdgTestSender = QtWidgets.QWidget()
        HLayoutTestSender = QtWidgets.QHBoxLayout(self.wdgTestSender)
        self.txtSendMsg = QtWidgets.QLineEdit()
        self.btnSendMsg = QtWidgets.QPushButton('SEND')
        self.btnSendMsg.setIcon(EmbdQtIcon('send.png'))
        HLayoutTestSender.addWidget(self.txtSendMsg, 1)
        HLayoutTestSender.addWidget(self.btnSendMsg)

    def initSlots(self):
        self.btnRefresh.clicked.connect(self.onClickRefresh)
        self.btnStart.clicked.connect(self.onClickStart)
        self.btnStop.clicked.connect(self.onClickStop)
        self.btnSendMsg.clicked.connect(self.onClickSend)
        self.rbtnTcp.clicked.connect(self.onClickSckType)
        self.rbtnUdp.clicked.connect(self.onClickSckType)
        self.btnDisconnectClient.clicked.connect(self.onClickDisconnectClient)

    def onClickDisconnectClient(self):
        clientAddr = str(self.tblAdaptorList.item(self.tblClientList.currentRow(), 0).text())
        clientPort = int(self.tblAdaptorList.item(self.tblClientList.currentRow(), 1).text())
        self.sig_disconnect_client.emit((clientAddr, clientPort))

    def onClickStart(self):
        ipAddress = self.txtAddress.text() if self.txtAddress.text() != "" else "0.0.0.0"
        portNo = self.spbPort.value()
        self.sig_start_server.emit(self._sckType, ipAddress, portNo)

    def onClickStop(self):
        self.sig_stop_server.emit()

    def onClickSend(self):
        txtToSend = self.txtSendMsg.text()
        self.sig_send_msg.emit(txtToSend)

    def onClickRefresh(self):
        iplist = get_ip_addr_list()
        self.tblAdaptorList.setRowCount(len(iplist))

        for row, ip in enumerate(iplist):
            name = QtWidgets.QTableWidgetItem('%s' % ip['name'])
            name.setFlags(QtCore.Qt.ItemFlags(int(name.flags()) ^ QtCore.Qt.ItemIsEditable))
            ip = QtWidgets.QTableWidgetItem('%s' % ip['ipv4'])
            ip.setFlags(QtCore.Qt.ItemFlags(int(ip.flags()) ^ QtCore.Qt.ItemIsEditable))
            self.tblAdaptorList.setItem(row, 0, name)
            self.tblAdaptorList.setItem(row, 1, ip)

    def onClickSckType(self):
        rBtn: QtWidgets.QRadioButton = self.sender()
        if rBtn.isChecked():
            self._sckType = rBtn.sckType

    def showEvent(self, a0: QtGui.QShowEvent) -> None:
        self.onClickRefresh()

    def addClient(self, tpAddPort: tuple):
        address, port = tpAddPort
        self.tblClientList.insertRow(self.tblClientList.rowCount())

        self.tblClientList.setItem(self.tblClientList.rowCount() - 1, 0,
                                   EmbdQtTableItem(str(address), abIsReadOnly=True))
        self.tblClientList.setItem(self.tblClientList.rowCount() - 1, 1,
                                   EmbdQtTableItem(str(port), abIsReadOnly=True))

        self.tblClientList.viewport().update()

    def removeClient(self, tpAddPort: tuple):
        address, port = tpAddPort

        for i in range(self.tblClientList.rowCount()):
            find_add = str(self.tblClientList.item(i, 0).text())
            find_port = int(self.tblClientList.item(i, 1).text())
            if find_port == port and find_add == address:
                self.tblClientList.removeRow(i)

    def onServerStart(self, is_start: bool):
        self.txtAddress.setEnabled(not is_start)
        self.spbPort.setEnabled(not is_start)
        self.btnStart.setEnabled(not is_start)
        self.tblAdaptorList.setEnabled(not is_start)
        self.btnRefresh.setEnabled(not is_start)

        self.btnSendMsg.setEnabled(is_start)
        self.txtSendMsg.setEnabled(is_start)
        self.btnDisconnectClient.setEnabled(is_start)
        self.btnStop.setEnabled(is_start)


if __name__ == '__main__':
    app = QtCore.QCoreApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)

    MainUI = DockServerConnect()
    MainUI.show()

    app.exec_()
# MainUI.release()
