#!/usr/bin/env python
import sys, os
import fileinput
from time import sleep
from collections import deque
from pprint import pprint as pp
import pdb
from copy import deepcopy as dcp

from PySide import QtGui, QtCore
from PySide.QtGui import (QWidget, QTabWidget, QItemSelectionModel, 
                          QMessageBox, QTableView, QSortFilterProxyModel,
                          QAbstractItemView, QItemSelection, QFileDialog,
                          QDialog)
from ui_main_window import Ui_MainWindow

from table_model import TableModel
from moos import MoosWidget

import sip
sip.setapi('QString', 2)

if not config['use_moos']:
    import rospy, roslib
    roslib.load_manifest('survey_gui')

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

        def setupMOOS():
            MainWindow.viz_ip = config['ip']
            MainWindow.viz_port = config['port']
            self.thread = MainWindow.VizThread()

            self.moos_widget = MoosWidget(config['moos'])
            self.moos_data =  # latest
            self.moos_data_fresh = False

            self.requestPosition.connect(self.moos_widget.onPositionRequested)
            self.moos_widget.sendPosition.connect(self.receivePosition)        

        QtGui.QMainWindow.__init__(self)

        if config['use_moos']:
            setupMOOS()

        self.ui = ui
        self.ui.setupUi(self)

        # Table
        self.table_model = TableModel()
        self.ui.tableView.setModel(self.table_model)
        self.ui.tableView.horizontalHeader().setStretchLastSection(True)

        # output
        self.output_file_dialog = QFileDialog()
        self.output_file = None

        self.manual_dialog = QtGui.QDialog()

        self.variance_threshold = config["variance_threshold"]

        # Signals/Slots
        self.ui.recordButton.released.connect(self.onRecordRequested)
        self.ui.actionManual_Entry.triggered.connect(self.addManualPoint)
        self.ui.actionWrite.triggered.connect(self.writeToFile)

    @QtCore.Slot(tuple)
    def receivePosition(self, pos):
        """connected to moos widget's position sender"""
        self.moos_data = pos
        self.moos_data_fresh = True

    @QtCore.Slot()
    def onRecordRequested(self):
        """When the record button is pressed, listens to moos until good 
        position acquired for write"""
        _sum = _2sum = (0, 0, 0)
        n = 0
        keep_going = True
        #FIXME this may just display the first value - variance is 0-initialized
        while keep_going:
            # Get values
            self.moos_data_fresh = False
            MainWindow.requestPosition()
            while not self.moos_data_fresh:
                sleep(0.01) # let moos reply
            pos = self.moos_data[0:2]
            dev = self.moos_data[3:5]

            # calculate the variance
            _mean = (0, 0, 0)
            _var = (0, 0, 0)
            n += 1
            for i in range(3):
                _sum[i] += pos[i]
                _2sum[i] += pos[i]**2
                _mean[i] = _sum[i]/n
                _var[i] = _2sum[i]/n - _mean[i]**2

            self.showVariance(_var)

            if sum(_var) < self.variance_threshold:
                x = _mean[0]
                y = _mean[1]
                z = _mean[2]
                description = self.ui.descriptionLineEdit.text()
                self.addEntry(x=x, y=y, z=z, description=description)

                print('Survey Point Added: ( %f , %f , %f )  %s' % \
                    _mean[0:2], self.ui.text())
                keep_going = False

    @QtCore.Slot()
    def addManualPoint(self):
        x, x_ok = QtGui.QInputDialog.getDouble(self.manual_dialog, "Input X Coordinate",
                    "(ECEF) X:", 0.0, -1e9, 1e9, 1)
        if not x_ok: return
        y, y_ok = QtGui.QInputDialog.getDouble(self.manual_dialog, "Input Y Coordinate",
                    "(ECEF) Y:", 0.0, -1e9, 1e9, 1)
        if not y_ok: return
        z, z_ok = QtGui.QInputDialog.getDouble(self.manual_dialog, "Input Z Coordinate",
                    "(ECEF) Z:", 0.0, -1e9, 1e9, 1)
        if not z_ok: return
        desc, desc_ok = QtGui.QInputDialog.getText(self.manual_dialog, "Input Point Description",
                    "Description:", QtGui.QLineEdit.Normal)
        if not desc_ok: return

        self.addEntry(x=z, y=y, z=z, description=desc)

    @QtCore.Slot()
    def writeToFile(self):
        self.output_file = open( str(self.output_file_dialog.getOpenFileName()[0]), 'w' )

        # Header
        self.output_file.write('## GAVLab Survey - Recorded %s' % time.asctime(time.localtime(time.time())))

        for line in self.getAllData():
            line = (str(line[0]), str(line[1]), str(line[2]), line[3])
            for item in line:
                self.output_file.write('%s\t\t\t' % item)
            self.output_file.write('\n')

        self.output_file.close()
        print('Output Log File Written!')
        

    def showVariance(self, var=(0, 0, 0)):
        """update the variance LCD's while waiting for point to go low"""
        self.ui.xVarianceLcd.display(var[0])
        self.ui.yVarianceLcd.display(var[1])
        self.ui.zVarianceLcd.display(var[2])

    def addEntry(self, x=None, y=None, z=None, description=None):
        if x is None and y is None and z is None and description is None:
            raise Exception('I have no dialog box set up for manual entry')

        point = {"x":x, "y":y, "z":z, "description":description}
        points = self.table_model.points[:]

        # TODO check for duplicates by attempting to remove
        # Assume new point
        self.table_model.insertRows(0) # at position zero

        ix = self.table_model.index(0, 0, QtCore.QModelIndex())
        self.table_model.setData(ix, point["x"], QtCore.Qt.EditRole)

        ix = self.table_model.index(0, 1, QtCore.QModelIndex())
        self.table_model.setData(ix, point["y"], QtCore.Qt.EditRole)

        ix = self.table_model.index(0, 2, QtCore.QModelIndex())
        self.table_model.setData(ix, point["z"], QtCore.Qt.EditRole)

        ix = self.table_model.index(0, 3, QtCore.QModelIndex())
        self.table_model.setData(ix, point["description"], QtCore.Qt.EditRole)

        # may need some resizing
        # self.currentWidget.resizeRowToContents(ix.row())  ##!! AttributeError: 'MainWindow' object has no attribute 'currentWidget'

    def getAllData(self):
        data = []
        for pt in range(len(self.table_model.points)):
            data.append([])
            for w in range(4):
                ix = self.table_model.index(pt, w, QtCore.QModelIndex())
                data[pt].append(self.table_model.data(ix))
        return data

    # def




if __name__ == '__main__':
    raise RuntimeError('Do not run this file directly')