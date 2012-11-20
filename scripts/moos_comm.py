#!/usr/bin/env python
"""
Implementation of the pyMOOS MOOSCommClient
"""
import sys
from pprint import pprint as pp
from time import sleep
from time import time as time_now
import pdb
import warnings

from pymoos.MOOSCommClient import MOOSCommClient

from PySide import QtGui, QtCore


class MOOSConnectionWarning(Warning):
    """Raise when MOOS Connection is not available or fruitful"""
    def __init__(self, msg):
        self.msg = msg


class MOOSPositionWarning(Warning):
    """Raised when position received from MOOS is not valid"""
    def __init__(self, msg):
        self.msg = msg


class MoosWidget(QtGui.QWidget):
    """
        Qt implementation of the MOOSCommClient
    """
    sendPosition = QtCore.Signal(tuple)

    moosdb_ip = '127.0.0.1'
    moosdb_port = 9000

    class MoosThread(QtCore.QThread):
        """Thread for the MOOSCommClient"""
        def run():
            """reimplement run()"""
            socket = QtNetwork.QTcpSocket()
            socket.connectToHost(MoosWidget.moosdb_ip, MoosWidget.moosdb_port)
            self.exec_()

    def __init__(self, config, parent=None):
        QtGui.QWidget.__init__(self, parent)

        try:
            MoosWidget.moosdb_ip = config['ip']
            MoosWidget.moosdb_port = config['port']
        except: pass
        self.thread = MoosWidget.MoosThread()

        self.moos_client = MOOSCommClient()
        self.moos_client.SetOnConnectCallBack(self.onConnect)
        self.moos_client.SetOnMailCallBack(self.onMail)
        self.moos_client.Run(self.moosdb_ip, self.moosdb_port, \
                             'survey', 50)

        self.time_buffer = config["time_buffer"]
        self.desired_variables = config['desired_variables']
        self.sensor = config['sensor']
        print('MoosWidget will subscribe to Sensor: %s' % self.sensor)
        print('MoosWidget will subscribe to Variables:'); 
        for dv in self.desired_variables:
            print('\t%s'%dv) 

        self.partial_positions = {}
        self.current_position = {}
        self.current_position_time = None
        for i in self.desired_variables:
            self.current_position[i] = None
        self.num_var = len(self.desired_variables)

        # Check Connection
        for x in range(30):
            sleep(0.1)
            if self.moos_client.IsConnected():
                print("Connected to MOOSDB")
                break
        if not self.moos_client.IsConnected():
            print("MOOSCommClient Error:: Failed to Connect to MOOSDB")
            sys.exit(-1)

    def onConnect(self):
        """MOOS callback - required in every MOOS app's class definition"""
        # print('MoosWidget: onConnect: waiting for mail..')
        for var in self.desired_variables:
            self.moos_client.Register(var)
        return

    def onMail(self):
        """MOOS callback - required in every MOOS app's class definition"""
        # print('\n--- In MoosWidget.onMail :: Retrieving inbox contents ---')
        # print('MoosWidget: onMail: Fetching recent mail')
        for message in self.moos_client.FetchRecentMail():
            self.unpackMsg(message)
        return True

    def unpackMsg(self, msg):
        """parse moos messages. put into dictionary
        handles conversion of any strings
        """
        # print('\nIn unpack_msg: \t%s' % msg.GetKey())
        time = round(msg.GetTime(), 3)
        name = msg.GetKey() # 'z_____' String
        
        if msg.GetSource() != self.sensor:
            Warning('QtMOOS Receiving messages from undesired sensor')
            return
        
        if msg.IsDouble():
            var_type = 'double'
            valu = msg.GetDouble()
        elif msg.IsString():
            Warning('Strings not supported')
            return
        
        self.handleMsg(time, name, valu)

    def handleMsg(self, time, name, valu):
        """ update the current position when possible """
        # print('\nIn handle_msg: \t%s' % msg['name'])
        if time not in self.partial_positions:
            self.partial_positions[time] = {}
    
        self.partial_positions[time][name] = valu

        ts = sorted(self.partial_positions.iterkeys()) # from low to high
        for t in ts:
            cull = False
            if len(self.partial_positions[t]) == self.num_var: # update
                self.current_position = self.partial_positions[t]
                self.current_position_time = time_now()
                cull = True

            if time - t > self.time_buffer:
                cull = True

            if cull:
                del self.partial_positions[t]
                break # only one can be completed @ a time

    @QtCore.Slot()
    def onPositionRequested(self):
        """survey instance wants a position, trigger send position emit"""
        if not (all(self.current_position) and self.current_position_time):
            MOOSConnectionWarning('MoosWidget: onPositionRequested: Nones in current position')
            pass
        elif time_now() - self.current_position_time > self.time_buffer:
            MOOSPositionWarning('MoosWidget: onPositionRequested: Time since last update too old - Disconnected?')
            pass
        else:
            print('\nMoosWidget: onPositionRequested: current_position is --'); pp(self.current_position)
            out = (self.current_position[p] for p in self.desired_variables)
            self.sendPosition.emit(out)
        # print('MoosWidget: Emitted Slot sendPosition')


################################################################################
if __name__ == '__main__':
    raise Exception('Do not run this file directly.')
