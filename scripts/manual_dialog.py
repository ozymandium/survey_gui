#!/usr/bin/env python
from PySide import QtCore, QtGui

class ManualDialog(QtGui.QDialog):
    def __init__(self, parent=None):
        super(ManualDialog, self).__init__(parent)
        self.dialog = QDialog()
        