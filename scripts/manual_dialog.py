#!/usr/bin/env python
from PySide import QtCore, QtGui

class ManualDialog(QtGui.QDialog):
    def __init__(self, parent=None):
        super(ManualDialog, self).__init__(parent)
        self.dialog = QtGui.QDialog()

        # Values to obtain
        self.x = None
        self.y = None
        self.z = None
        self.description = None

        xlabel = QtGui.QLabel("X: ")
        xLineEdit = QtGui.QLineEdit()
        
        ylabel = QtGui.QLabel("Y: ")
        yLineEdit = QtGui.QLineEdit()
        
        zlabel = QtGui.QLabel("Z: ")
        zLineEdit = QtGui.QLineEdit()
        
        desclabel = QtGui.QLabel("Description: ")
        descLineEdit = QtGui.QLineEdit()

    # def getX(self):
        # self.


