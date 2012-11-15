#!/usr/bin/env python
"""
Implementation of the pyMOOS MOOSCommClient
"""
import sys
from pymoos.MOOSCommClient import MOOSCommClient
from pprint import pprint as pp
from PySide import QtGui
from PySide import QtCore
from time import sleep
import pdb


class MoosWidget(QtGui.QWidget):
    """
    Qt implementation of the MOOSCommClient
    """
    sendPosition = QtCore.Signal(tuple)

    class MoosThread(QtCore.QThread):
        """Thread for the MOOSCommClient"""
        this_ip = '127.0.0.1'
        this_port = 9000

        def run():
            """reimplement run()"""
            socket = QtNetwork.QTcpSocket()
            socket.connectToHost(MoosThread.this_ip, MoosThread.this_port)
            self.exec_()

    def __init__(self, config, parent=None):
        QtGui.QWidget.__init__(self, parent)
        self.thread = MoosThread()
        # TODO implement configuring port/ip
        # self.thread.this_ip = config["ip"]
        # self.thread.this_port = config["port"]
        
        self.moos_client = MOOSCommClient()
        self.moos_client.SetOnConnectCallBack(self.onConnect)
        self.moos_client.SetOnMailCallBack(self.onMail)
        self.moos_client.Run(self.thread.this_ip, self.thread.this_port, \
                             'survey', 50)

        self.time_buffer = config["time_buffer"]
        self.desired_variables = config['desired_variables']
        self.partial_positions = {}
        self.current_position = {}
        for i in self.desired_variables:
            self.current_position[i] = None
        self.num_var = len(self.desired_variables)

        # Check Connection
        for x in range(30):
            sleep(0.1)
            if self.moos_client.IsConnected():
                print("\nConnected to MOOSDB")
                break
        if not self.moos_client.IsConnected():
            print("MOOSCommClient Error:: Failed to Connect to MOOSDB")
            sys.exit(-1)

    def onConnect(self):
        """MOOS callback - required in every MOOS app's class definition"""
        print('In MoosWidget.onConnect :: waiting for mail..')
        for var in self.desired_variables:
            self.moos_client.Register(var)
        return

    def onMail(self):
        """MOOS callback - required in every MOOS app's class definition"""
        # print('\n--- In MoosWidget.onMail :: Retrieving inbox contents ---')
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
                raise Warning('Receiving messages from undesired sensor')
                return
            
            if msg.IsDouble():
                var_type = 'double'
                valu = msg.GetDouble()
            elif msg.IsString():
                raise Warning('Strings not supported')
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
                cull = True

            if time - t > self.time_buffer:
                cull = True

            if cull:
                del self.partial_positions[t]

    @QtCore.Slot()
    def onPositionRequested():
        """survey instance wants a position, trigger send position emit"""
        out = (self.current_position[p] for p in self.desired_variables)
        MoosWidget.sendPosition.emit(out)

################################################################################
if __name__ == '__main__':
    raise Exception('Do not run this file directly.')
