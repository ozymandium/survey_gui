#!/usr/bin/env python
import sys
import os
import fileinput
from yaml import load
from time import sleep
from collections import deque
from pprint import pprint as pp
import pdb
from copy import deepcopy as dcp
from PySide import QtCore, QtGui, QtNetwork
from ui_main_window import Ui_MainWindow


class MainWindow(QtGui.QMainWindow):
    """survey mainwindow class"""
    requestPosition = QtCore.QSignal()

    class VizThread(QtCore.QThread):
        """Thread for the LFviz MainWindow"""
        this_ip = '127.0.0.1'
        this_port = 9001

        def run():
            """reimplement run()"""
            socket = QtNetwork.QTcpSocket()
            socket.connectToHost(VizThread.this_ip, VizThread.this_port)
            self.exec_()

    def __init__(self, ui, config):
        QtGui.QMainWindow.__init__(self)
        self.thread = VizThread()
        self.ui = ui
        self.ui.setupUi(self)

        self.moos_data = (None)*6 # latest
        self.survey_points = deque()

        self.variance_threshold = config["variance_threshold"]

    @QtCore.Slot(tuple)
    def receive_position_(pos):
        """connected to moos widget's send_position_"""
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
            MainWindow.requestPosition.emit()
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
                self.survey_points.append()
                print('Survey Point Added: ( %f , %f , %f )' %0)
                keep_going = False

        
    def writeToFile(self):


if __name__ == '__main__':
    raise RuntimeError('Do not run this file directly')