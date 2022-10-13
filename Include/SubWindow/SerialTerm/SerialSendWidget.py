# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : SerialSendWidget.py
# Project Name : PySerialTerm
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2022.02.19 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import os
import sys
from typing import List

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


class NumericDelegate(QtWidgets.QStyledItemDelegate):
    def createEditor(self, parent, option, index):
        editor = super(NumericDelegate, self).createEditor(parent, option, index)
        if isinstance(editor, QtWidgets.QLineEdit):
            reg_ex = QtCore.QRegExp("[0-9A-Fa-f]{1,2}")
            validator = QtGui.QRegExpValidator(reg_ex, editor)
            editor.setValidator(validator)
        return editor


class CharacterDelegate(QtWidgets.QStyledItemDelegate):
    def createEditor(self, parent, option, index):
        editor = super(CharacterDelegate, self).createEditor(parent, option, index)
        if isinstance(editor, QtWidgets.QLineEdit):
            editor.setMaxLength(1)
        return editor


class PacketTable(QtWidgets.QTableWidget):
    listHorHeader: List[str]

    def __init__(self, parent=None):
        super(PacketTable, self).__init__(parent)
        self.listHorHeader = ["No.", "RAW(Hex)", "ASCII(Char)"]
        self.setColumnCount(len(self.listHorHeader))
        self.setHorizontalHeaderLabels(self.listHorHeader)
        self.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        self.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.Stretch)
        self.setSelectionBehavior(QtWidgets.QTableView.SelectRows)
        self.verticalHeader().hide()
        num_delegate = NumericDelegate(self)
        self.setItemDelegateForColumn(1, num_delegate)
        char_delegate = CharacterDelegate(self)
        self.setItemDelegateForColumn(2, char_delegate)
        self.cellChanged.connect(self.on_cell_changed)


    def on_cell_changed(self, row: int, col: int):
        self.blockSignals(True)
        str_item = self.item(row, col).text()
        if col == 1:
            if len(str_item) < 2:
                str_item = "0" + str_item

            self.setItem(row, col, QtTableItem(str_item.upper()))  # align center
            str_ascii = bytes.fromhex(str_item).decode("ISO-8859-1")

            self.setItem(row, 2, QtTableItem(str_ascii))

        elif col == 2:
            self.setItem(row, col, QtTableItem(str_item))
            str_raw = format(ord(str_item), "x")
            self.setItem(row, 1, QtTableItem(str_raw.upper()))

        self.blockSignals(False)

    def clearTable(self):
        self.setRowCount(0)

    def adjustRowNo(self):
        for i in range(self.rowCount()):
            self.setItem(i, 0, QtTableItem(i, True))

    def addRow(self):
        if self.rowCount() != 0:
            indices = self.selectedIndexes()
            indices = [index.row() for index in indices]
            if indices:
                index = max(indices)
                row_no = index + 1
            else:
                row_no = self.rowCount()
        else:
            row_no = self.rowCount()


        self.insertRow(row_no)
        self.setRowHeight(row_no, 1)
        self.setItem(row_no, 1, QtTableItem("00"))
        self.adjustRowNo()
        self.viewport().update()  # refresh table contents

    def delRow(self):
        if self.rowCount() != 0:
            indices = self.selectionModel().selectedRows()
            if not indices:
                self.removeRow(self.rowCount() - 1)
            else:
                for index in reversed(sorted(indices)):
                    self.removeRow(index.row())

            self.adjustRowNo()
            self.viewport().update()  # refresh table contents

    def shiftDown(self):
        self.blockSignals(True)
        curr_row = self.currentRow()

        if curr_row < self.rowCount() - 1:
            for i in range(1, 3):
                itemStore = self.takeItem(curr_row + 1, i)
                self.setItem(curr_row + 1, i, self.takeItem(curr_row, i))
                self.setItem(curr_row, i, itemStore)
            self.setCurrentCell(curr_row + 1, 0)

        self.blockSignals(False)

    def shiftUp(self):
        self.blockSignals(True)
        curr_row = self.currentRow()

        if curr_row > 0:
            for i in range(1, 3):
                itemStore = self.takeItem(curr_row - 1, i)
                self.setItem(curr_row - 1, i, self.takeItem(curr_row, i))
                self.setItem(curr_row, i, itemStore)
            self.setCurrentCell(curr_row - 1, 0)

        self.blockSignals(False)


class CWidgetSerialSend(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(CWidgetSerialSend, self).__init__(parent)
        self.sig_send_serial_packet = PySignal(bytearray)
        self.initUi()
        self.initSlots()

    def initUi(self):
        self.initPacketCtrl()
        self.initStringCtrl()
        self.initPacketStorage()
        MainLayout = QtWidgets.QVBoxLayout(self)

        MainLayout.addWidget(self.widgetPacket, 1)
        MainLayout.addWidget(self.widgetCtrlStr)
        MainLayout.addWidget(self.widgetCtrlStorage)

        MainLayout.setContentsMargins(0, 0, 0, 0)

    def initTableWidget(self):
        self.tablePacket = PacketTable()

    def initSlots(self):
        self.btnClear.clicked.connect(self.on_click_btn_clear)
        self.btnAdd.clicked.connect(self.on_click_btn_add)
        self.btnSub.clicked.connect(self.on_click_btn_del)
        self.btnShiftUp.clicked.connect(self.on_click_btn_shift_up)
        self.btnShiftDwn.clicked.connect(self.on_click_btn_shift_down)
        self.btnSend.clicked.connect(self.on_click_btn_send)
        self.btnSendString.clicked.connect(self.on_click_btn_send_str)
        self.btnStore.clicked.connect(self.on_click_btn_store)
        self.btnSendStorage.clicked.connect(self.on_click_btn_send_storage)
        self.btnClearStorage.clicked.connect(self.on_click_btn_clear_storage)
        self.btnDelStorage.clicked.connect(self.on_click_btn_del_storage)

    def initPacketCtrl(self):
        self.initTableWidget()
        btnHeight = 25
        btnFontSz = 16
        btnFont = QtGui.QFont()
        btnFont.setPointSize(btnFontSz)

        self.widgetCtrlPacket = QtWidgets.QWidget()
        VLayoutCtrl = QtWidgets.QVBoxLayout(self.widgetCtrlPacket)
        self.btnClear = QtWidgets.QPushButton('CLEAR')
        self.btnClear.setFixedHeight(btnHeight)
        HLayoutAddSub = QtWidgets.QHBoxLayout()
        self.btnAdd = QtWidgets.QPushButton('+')
        self.btnAdd.setFixedHeight(btnHeight)
        self.btnAdd.setFont(btnFont)
        self.btnSub = QtWidgets.QPushButton('-')
        self.btnSub.setFixedHeight(btnHeight)
        self.btnSub.setFont(btnFont)
        self.btnShiftUp = QtWidgets.QPushButton('▲')
        self.btnShiftUp.setFixedHeight(btnHeight)
        # self.btnShiftUp.setFont(btnFont)
        self.btnShiftDwn = QtWidgets.QPushButton('▼')
        self.btnShiftDwn.setFixedHeight(btnHeight)
        # self.btnShiftDwn.setFont(btnFont)
        GLayoutPos = QtWidgets.QGridLayout()
        GLayoutPos.addWidget(self.btnAdd, 0, 0)
        GLayoutPos.addWidget(self.btnSub, 0, 1)
        GLayoutPos.addWidget(self.btnShiftUp, 1, 0)
        GLayoutPos.addWidget(self.btnShiftDwn, 1, 1)
        self.btnSend = QtWidgets.QPushButton('SEND')
        self.btnSend.setFixedHeight(btnHeight)

        HLayoutFmt = QtWidgets.QHBoxLayout()
        self.rbtnForward = QtWidgets.QRadioButton('Forward')
        self.rbtnForward.setChecked(True)
        self.rbtnReverse = QtWidgets.QRadioButton('Reverse')
        HLayoutFmt.addWidget(self.rbtnForward)
        HLayoutFmt.addWidget(self.rbtnReverse)
        self.btnStore = QtWidgets.QPushButton('STORE')
        self.btnStore.setFixedHeight(btnHeight)
        VLayoutCtrl.setContentsMargins(5, 0, 5, 0)
        VLayoutCtrl.addWidget(self.btnClear)
        VLayoutCtrl.addLayout(HLayoutAddSub)
        VLayoutCtrl.addLayout(GLayoutPos)
        VLayoutCtrl.addWidget(self.btnSend)
        VLayoutCtrl.addWidget(self.rbtnForward)
        VLayoutCtrl.addWidget(self.rbtnReverse)
        # VLayoutCtrl.addLayout(HLayoutFmt)
        VLayoutCtrl.addWidget(self.btnStore)
        VLayoutCtrl.addStretch(1)

        self.widgetCtrlPacket.setMaximumWidth(80)

        # Send Packet
        self.widgetPacket = QtWidgets.QWidget()
        VLayoutPacket = QtWidgets.QVBoxLayout(self.widgetPacket)
        lblPacket = QtWidgets.QLabel()
        lblPacket.setText('[Send Packet]')
        HLayoutPacketCtrl = QtWidgets.QHBoxLayout()
        HLayoutPacketCtrl.addWidget(self.tablePacket, 1)
        HLayoutPacketCtrl.addWidget(self.widgetCtrlPacket)
        VLayoutPacket.addWidget(lblPacket)
        VLayoutPacket.addLayout(HLayoutPacketCtrl)
        VLayoutPacket.setContentsMargins(0, 0, 0, 0)

    def initStringCtrl(self):
        btnHeight = 25
        self.widgetCtrlStr = QtWidgets.QWidget()
        VLayoutCtrlStr = QtWidgets.QVBoxLayout(self.widgetCtrlStr)
        lblCtrlStr = QtWidgets.QLabel()
        lblCtrlStr.setText('[Send String]')
        HLayoutCtrlStr = QtWidgets.QHBoxLayout()
        self.txtStringInput = QtWidgets.QLineEdit()
        self.txtStringInput.setFixedHeight(btnHeight)
        self.btnSendString = QtWidgets.QPushButton('SEND')
        self.btnSendString.setFixedHeight(btnHeight)
        self.btnSendString.setFixedWidth(70)
        HLayoutCtrlStr.addWidget(self.txtStringInput, 1)
        HLayoutCtrlStr.addWidget(self.btnSendString)
        HLayoutCtrlStr.setContentsMargins(0, 0, 5, 0)
        HLayoutCtrlStr.setSpacing(10)
        VLayoutCtrlStr.addWidget(lblCtrlStr)
        VLayoutCtrlStr.addLayout(HLayoutCtrlStr)
        VLayoutCtrlStr.setContentsMargins(0, 0, 0, 0)

    def initPacketStorage(self):
        btnHeight = 25
        btnWidth = 70
        self.widgetCtrlStorage = QtWidgets.QWidget()
        VLayoutCtrlStorage = QtWidgets.QVBoxLayout(self.widgetCtrlStorage)
        lblCtrlStr = QtWidgets.QLabel()
        lblCtrlStr.setText('[Send Stored Packet]')
        HLayoutCtrlStorage = QtWidgets.QHBoxLayout()
        self.lstPacketHistory = QtWidgets.QListWidget()
        self.lstPacketHistory.setSortingEnabled(True)
        VLayoutBtn = QtWidgets.QVBoxLayout()
        self.btnClearStorage = QtWidgets.QPushButton('CLEAR')
        self.btnClearStorage.setFixedHeight(btnHeight)
        self.btnClearStorage.setFixedWidth(70)
        self.btnDelStorage = QtWidgets.QPushButton('DELETE')
        self.btnDelStorage.setFixedHeight(btnHeight)
        self.btnDelStorage.setFixedWidth(70)
        self.btnSendStorage = QtWidgets.QPushButton('SEND')
        self.btnSendStorage.setFixedHeight(btnHeight)
        self.btnSendStorage.setFixedWidth(70)
        VLayoutBtn.addWidget(self.btnClearStorage)
        VLayoutBtn.addWidget(self.btnDelStorage)
        VLayoutBtn.addWidget(self.btnSendStorage)
        VLayoutBtn.addStretch(1)

        HLayoutCtrlStorage.addWidget(self.lstPacketHistory, 1)
        HLayoutCtrlStorage.addLayout(VLayoutBtn)
        HLayoutCtrlStorage.setContentsMargins(0, 0, 5, 0)
        HLayoutCtrlStorage.setSpacing(10)

        VLayoutCtrlStorage.addWidget(lblCtrlStr)
        VLayoutCtrlStorage.addLayout(HLayoutCtrlStorage)
        VLayoutCtrlStorage.setContentsMargins(0, 0, 0, 0)

    def on_click_btn_store(self):
        bt_temp = bytearray([])
        for i in range(self.tablePacket.rowCount()):
            temp = int(self.tablePacket.item(i, 1).text(), 16)
            bt_temp.append(temp)

        if self.rbtnReverse.isChecked():
            bt_temp.reverse()

        if len(bt_temp) > 0:
            strStore = "(" + str(len(bt_temp)) + ")-" + "0x"
            strStore += ''.join('{:02x}'.format(byte) for byte in bt_temp)
            self.lstPacketHistory.addItem(QtWidgets.QListWidgetItem(strStore))

    def on_click_btn_clear_storage(self):
        self.lstPacketHistory.clear()

    def on_click_btn_del_storage(self):
        self.lstPacketHistory.takeItem(self.lstPacketHistory.row(self.lstPacketHistory.currentItem()))

    def on_click_btn_send_storage(self):
        if self.lstPacketHistory.count() > 0 or self.lstPacketHistory is not None:
            strSelected = self.lstPacketHistory.currentItem().text().split('-0x')[1]
            bta_temp = bytearray.fromhex(strSelected)
            if self.rbtnReverse.isChecked():
                bta_temp.reverse()

            self.sig_send_serial_packet.emit(bta_temp)

    def on_click_btn_clear(self):
        self.tablePacket.clearTable()

    def on_click_btn_add(self):
        self.tablePacket.addRow()

    def on_click_btn_del(self):
        self.tablePacket.delRow()

    def on_click_btn_shift_up(self):
        self.tablePacket.shiftUp()

    def on_click_btn_shift_down(self):
        self.tablePacket.shiftDown()

    def on_click_btn_send_str(self):
        str_item = self.txtStringInput.text()
        bt_temp = bytearray([])
        for c in str_item:
            bt_temp.append(ord(c))

        if self.rbtnReverse.isChecked():
            bt_temp.reverse()

        self.sig_send_serial_packet.emit(bt_temp)

    def on_click_btn_send(self):
        bt_temp = bytearray([])
        for i in range(self.tablePacket.rowCount()):
            temp = int(self.tablePacket.item(i, 1).text(), 16)
            bt_temp.append(temp)

        if self.rbtnReverse.isChecked():
            bt_temp.reverse()

        if len(bt_temp) > 0:
            self.sig_send_serial_packet.emit(bt_temp)

    def release(self):
        self.close()


if __name__ == '__main__':
    app = QtCore.QCoreApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)

    app.setStyle('fusion')
    MainUI = CWidgetSerialSend()
    MainUI.show()

    app.exec_()
    MainUI.release()
