#!/usr/bin/env python
import sys, os
import fileinput
from time import *
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

# import sip
# sip.setapi('QString', 2)

# TODO  make length of position data a communicated/coordinated value

class MainWindow(QtGui.QMainWindow, QtCore.QObject):
    """survey mainwindow class"""
    requestPosition = QtCore.Signal()

    viz_ip = '127.0.0.1'
    viz_port = 9001

    class QtComm(QtCore.QObject):
        requestPosition = QtCore.Signal()

    class VizThread(QtCore.QThread):
        """Thread for the viz MainWindow"""
        def run():
            """reimplement run()"""
            socket = QtNetwork.QTcpSocket()
            socket.connectToHost(MainWindow.viz_ip, MainWindow.viz_port)
            self.exec_()

    def __init__(self, app, ui, config, parent=None):
        print('\n---------- GAVLab RTK Survey ----------')
        super(MainWindow, self).__init__(parent)
        self.app = app # reference to the Qt Application
        
        def setupMOOS():
            # manual threading is only necessary if reading messages from moos
            # roslaunch automatically configures threads - ros is superior to moos
            self.comm_arch = 'moos'
            MainWindow.viz_ip = config['ip']
            MainWindow.viz_port = config['port']
            self.thread = MainWindow.VizThread()

            self.moos_widget = MoosWidget(config['moos'])

            self.requestPosition.connect(self.moos_widget.onPositionRequested)
            self.moos_widget.sendPosition.connect(self.receivePosition)
            print('MOOS setup')

        def setupROS():
            self.comm_arch = 'ros'
            # ...
            print('ROS setup')

        # Determine Comm arch
        if config['ros']:
            import rospy, roslib
            roslib.load_manifest('survey_gui')
            setupROS()
        elif config['moos']:
            from moos_comm import MoosWidget, MOOSPositionWarning, MOOSConnectionWarning
            setupMOOS()
        else:
            raise Exception('Need a config file with comm architecture')

        self.qt_comm = MainWindow.QtComm()
        
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

        self.pos_data =  [None]*6 # latest
        self.pos_data_good = False
        self.variance_threshold = config["variance_threshold"]

        # Signals/Slots
        self.ui.actionExit.triggered.connect(self.app.closeAllWindows)
        self.ui.recordButton.released.connect(self.onRecordRequested)
        self.ui.actionManual_Entry.triggered.connect(self.addManualPoint)
        self.ui.actionWrite.triggered.connect(self.writeToFile)

    ############################################################################
    #####  Signals & Slots Stuff  ##############################################
    ############################################################################

    @QtCore.Slot(tuple)
    def receivePosition(self, pos):
        """connected to moos widget's position sender"""
        ## TODO make indexing better here (check length of pos with length of pos_data)
        n = 0
        while True:
            print('MainWindow: receivePosition: iterating through...')
            try: 
                i = pos.next()
            except StopIteration:
                print('MainWindow: receivePosition: reached end of pos')
                break

            if not i:
                raise MOOSPositionWarning('MainWindow: receivePosition: got a None value')
                break
            
            self.pos_data[n] = pos.next()
            print('MainWindow: receivePosition position successfully received')
            n += 1
        
        if all(self.pos_data):
            self.pos_data_good = True

    @QtCore.Slot()
    def onRecordRequested(self):
        """When the record button is pressed, listens to moos until good 
        position acquired for write"""
        print('Record requested')
        _sum = _2sum = [0.0, 0.0, 0.0]
        n = 0
        keep_averaging = True
        while keep_averaging:
            # Get values
            self.pos_data_good = False
            self.requestPosition.emit()
            sleep(0.1)
            if not self.pos_data_good:
                print('\nMainWindow: onRecordRequested: pos_data_good False')
                pass

            # calculate the variance
            _mean = [0.0, 0.0, 0.0]
            _var = [0.0, 0.0, 0.0]
            n += 1
            for i in range(3):
                _sum[i] += self.pos_data[i]
                _2sum[i] += self.pos_data[i]**2
                _mean[i] = _sum[i]/n
                _var[i] = _2sum[i]/n - _mean[i]**2

            self.showVariance(var=_var)

            if sum(_var) < self.variance_threshold:
                x = _mean[0]
                y = _mean[1]
                z = _mean[2]
                description = self.ui.descriptionLineEdit.text()
                self.addEntry(x=x, y=y, z=z, description=description)

                print('Survey Point Added: ( %f , %f , %f )  %s' % (x, y, z, description))
                keep_averaging = False

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
        self.output_file.write('## GAVLab Survey - Recorded %s' % asctime(localtime(time())))

        for line in self.getAllData():
            line = (str(line[0]), str(line[1]), str(line[2]), line[3])
            for item in line:
                self.output_file.write('%s\t\t\t' % item)
            self.output_file.write('\n')

        self.output_file.close()
        print('Output Log File Written!')

    # @QtCore.Slot()
    # def onExitRequested(self):

    ############################################################################
    #####  Helper functions ####################################################
    ############################################################################

    def showVariance(self, var=(0, 0, 0)):
        """update the variance LCD's while waiting for point to go low"""
        # print('MainWindow: showVariance')
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


if __name__ == '__main__':
    raise RuntimeError('Do not run this file directly')