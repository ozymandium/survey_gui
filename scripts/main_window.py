#!/usr/bin/env python
import sys, os
import fileinput
from time import *
from pprint import pprint as pp
import pdb
from copy import deepcopy as dcp

from numpy import array, inf, sqrt

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

    class VizThread(QtCore.QThread):
        """Thread for the viz MainWindow"""
        def run():
            """reimplement run()"""
            socket = QtNetwork.QTcpSocket()
            socket.connectToHost(MainWindow.viz_ip, MainWindow.viz_port)
            self.exec_()

    def __init__(self, app, ui, config, parent=None):
        super(MainWindow, self).__init__(parent)
        print('\n---------- GAVLab RTK Survey ----------')

        def setupMOOS():
            # manual threading is only necessary if reading messages from moos
            # roslaunch automatically configures threads - ros is superior to moos
            self.comm_arch = 'moos'

            try: 
                MainWindow.viz_ip = config['ip']
                MainWindow.viz_port = config['port']
            except: pass
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
        
        self.app = app # reference to the Qt Application
        self.ui = ui
        self.ui.setupUi(self)
        self.config = config

        # Table
        self.table_model = TableModel()
        self.ui.tableView.setModel(self.table_model)
        self.ui.tableView.horizontalHeader().setStretchLastSection(True)

        # output
        self.output_file_dialog = QFileDialog()
        self.output_file = None
        self.header = '## GAVLab Survey - Recorded %s\n' + \
                      '## All data in ECEF (m): X Y Z Description\n'

        self.manual_dialog = QtGui.QDialog()

        self.pos_data =  array([None]*3) # latest
        self.pos_data_good = False
        self.std_dev_threshold = config["std_dev_threshold"]
        if not self.std_dev_threshold: # blank string --> accept first position received and record
            self.std_dev_threshold = inf

        # Signals/Slots
        self.ui.actionExit.triggered.connect(self.app.closeAllWindows)
        self.ui.recordButton.clicked.connect(self.onRecordRequested)
        self.ui.actionManual_Entry.triggered.connect(self.addManualPoint)
        self.ui.actionWrite.triggered.connect(self.writeToFile)

        # Number cruncing states (iterative)
        self._sum = array([0.0]*3)
        self._2sum = array([0.0]*3)

    ############################################################################
    #####  Signals & Slots Stuff  ##############################################
    ############################################################################

    @QtCore.Slot(tuple)
    def receivePosition(self, pos):
        """connected to moos widget's position sender
            Assume self.pos_data_good = False at invocation"""
        ## TODO make indexing better here (check length of pos with length of pos_data)
        n = 0
        while True:
            try: 
                i = pos.next()
                self.pos_data[n] = i
            except StopIteration: break

            n += 1

        if n == len(self.config['moos']['desired_variables']): # TODO refer to subscribed vars of either comm set
            self.pos_data_good = True
        else:
            raise MOOSPositionWarning('Not enough values received')
            
    @QtCore.Slot()
    def onRecordRequested(self):
        """When the record button is pressed, listens to moos until good 
        position acquired for write"""
        self._sum = array([0.0]*3)
        self._2sum = array([0.0]*3)
        n = int(0)
        keep_averaging = True
        print('\nRecord requested')
        while keep_averaging:
            # Get values
            self.pos_data_good = False
            try: # put most time consuming procs in a catcher
                self.requestPosition.emit()
                sleep(1)
            except KeyboardInterrupt: # user has requested stop
                print('** Aborting Record')
                self.ui.recordButton.setChecked(False)
                break

            if not self.pos_data_good:
                print('\tWaiting for good data ...')
                pass
            else:
                print('\tGood data set recieved:  (%.2f, %.2f, %.2f) ' % \
                    (self.pos_data[0], self.pos_data[1], self.pos_data[2]))

                n += 1
                n, _mean, _stddev_mag = self.iterateRecord(n)   
                print('\tStd Dev magnitude = %f' % _stddev_mag)

                # Check the variance magnitude
                if n == 1: 
                    continue
                if _stddev_mag < self.std_dev_threshold:
                    x = _mean[0]
                    y = _mean[1]
                    z = _mean[2]
                    description = str(self.ui.descriptionLineEdit.text())
                    self.addEntry(x=str(x), y=str(y), z=str(z), description=description)

                    # exit stuff
                    print('\t---> Survey Point Added: ( %.2f , %.2f , %.2f )  %s\n' % (x, y, z, description))
                    keep_averaging = False
                    self.ui.recordButton.setChecked(False) # done recording
                    self.ui.descriptionLineEdit.setText('')
                else:
                    print('\tWaiting for variance to drop...\n')

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

        self.addEntry(x=x, y=y, z=z, description=desc)

    @QtCore.Slot()
    def writeToFile(self):
        self.output_file = open( str(self.output_file_dialog.getOpenFileName()[0]), 'w' )

        # Header
        self.output_file.write(self.header % asctime(localtime(time())))

        for line in self.getAllData():
            line = (str(line[0]), str(line[1]), str(line[2]), line[3])
            for item in line:
                self.output_file.write('%s\t\t' % item)
            self.output_file.write('\n')

        self.output_file.close()
        print('Output Log File Written!')

    # @QtCore.Slot()
    # def onExitRequested(self):

    ############################################################################
    #####  Helper functions ####################################################
    ############################################################################

    def iterateRecord(self, n):
        # calculate the variance
        self._sum += self.pos_data
        self._2sum += self.pos_data**2
        _mean = self._sum/n
        _var = self._2sum/n - _mean**2
        _sdev = sqrt(abs(_var))

        self.updateLCD(var=_var)
        _stddev_mag = sqrt(sum(_sdev**2))

        return n, _mean, _stddev_mag

    def updateLCD(self, var=(0, 0, 0), stddev=(0, 0, 0)):
        """update the variance LCD's while waiting for point to go low"""
        # print('MainWindow: showVariance')
        self.ui.xVarianceLcd.display(var[0])
        self.ui.yVarianceLcd.display(var[1])
        self.ui.zVarianceLcd.display(var[2])

        self.ui.xStdDevLcd.display(stddev[0])
        self.ui.yStdDevLcd.display(stddev[1])
        self.ui.zStdDevLcd.display(stddev[2])

    def addEntry(self, x=None, y=None, z=None, description=None):
        if x is None and y is None and z is None and description is None:
            raise Exception('something has to go into the entry')

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