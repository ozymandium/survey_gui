#!/usr/bin/env python
import sys, os
import fileinput
from yaml import load
from time import sleep
from collections import deque
from pprint import pprint as pp
import pdb
from copy import deepcopy as dcp
from PySide import QtGui, QtCore
from PySide.QtGui import (QWidget, QTabWidget, QItemSelectionModel, 
                          QMessageBox, QTableView, QSortFilterProxyModel,
                          QAbstractItemView, QItemSelection)
from ui_main_window import Ui_MainWindow
from tablemodel import TableModel


class MainWindow(QtGui.QMainWindow):
    """survey mainwindow class"""
    requestPosition = QtCore.Signal()

    viz_ip = '127.0.0.1'
    viz_port = 9001

    class VizThread(QtCore.QThread):
        """Thread for the viz MainWindow"""
        def run():
            """reimplement run()"""
            socket = QtNetwork.QTcpSocket()
            socket.connectToHost(MainWindow.viz_ip, MainWindow.viz_port)
            self.exec_()

    def __init__(self, ui, config):
        QtGui.QMainWindow.__init__(self)
        
        MainWindow.viz_ip = config['ip']
        MainWindow.viz_port = config['port']
        self.thread = MainWindow.VizThread()

        self.ui = ui
        self.ui.setupUi(self)

        # Table
        self.table_model = TableModel()
        self.ui.tableView.setModel(self.table_model)
        self.ui.tableView.horizontalHeader().setStretchLastSection(True)


        self.moos_data = (None, None, None) # latest
        self.survey_points = deque() # each element: [n, (x,y,z), 'descr']

        self.variance_threshold = config["variance_threshold"]

        # UI Signals/Slots
        self.ui.recordButton.released.connect(self.onRecordRequested)

    @QtCore.Slot(tuple)
    def receivePosition(pos):
        """connected to moos widget's position sender"""
        self.moos_data = pos

    @QtCore.Slot()
    def onRecordRequested(self):
        """When the record button is pressed, listens to moos until good 
        position acquired for write"""
        _sum = _2sum = (0, 0, 0)
        n = 0
        keep_going = True
        while keep_going:
            # Get values
            MainWindow.requestPosition()
            sleep(0.1) # let moos reply
            pos = self.moos_data[0:2]

            # calculate the variance
            _mean = (0, 0, 0)
            _var = (0, 0, 0)
            n += 1
            for i in range(3):
                _sum[i] += pos[i]
                _2sum[i] += pos[i]**2
                _mean[i] = _sum[i]/n
                _var[i] = _2sum[i]/n - _mean[i]**2

            if sum(_var) < self.variance_threshold:
                x = _mean[0]
                y = _mean[1]
                z = _mean[2]
                description = self.ui.descriptionLineEdit.text()
                self.addEntry(x=x, y=y, z=z, description=description)

                print('Survey Point Added: ( %f , %f , %f )  %s' % \
                    _mean[0:2], self.ui.text())
                keep_going = False


    def addEntry(self, x=None, y=None, z=None, description=None):
        if x is None and y is None and z is None and description is None:
            raise Exception('I have no dialog box set up for manual entry')

        point = {"x":x, "y":y, "z":z, "description":description}
        points = self.table_model.points[:]

        # TODO check for duplicates by attempting to remove
        # Assume new point
        self.table_model.insertRows(0) # at position zero

        ix = self.table_model.index(0, 0, QtCore.QModelIndex())
        self.table_model.setData(ix, point["x"], Qt.EditRole)

        ix = self.table_model.index(0, 1, QtCore.QModelIndex())
        self.table_model.setData(ix, point["y"], Qt.EditRole)

        ix = self.table_model.index(0, 2, QtCore.QModelIndex())
        self.table_model.setData(ix, point["z"], Qt.EditRole)

        ix = self.table_model.index(0, 3, QtCore.QModelIndex())
        self.table_model.setData(ix, point["description"], Qt.EditRole)

        # may need some resizing
        self.currentWidget.resizeRowToContents(ix.row())



if __name__ == '__main__':
    raise RuntimeError('Do not run this file directly')