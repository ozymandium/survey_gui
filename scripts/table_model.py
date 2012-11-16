#!/usr/bin/env python
from PySide import QtCore
from collections import deque

class TableModel(QtCore.QAbstractTableModel):

    def __init__(self, points=None, parent=None):
        super(TableModel, self).__init__(parent)

        if points is None:
            self.points = []
        else: 
            self.points = points

    def rowCount(self, index=QtCore.QModelIndex()):
        return len(self.points)

    def columnCount(self, index=QtCore.QModelIndex()):
        return 4

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if not index.isValid():
            return None

        if not 0 <=index.row() < len(self.points):
            return None

        if role == QtCore.Qt.DisplayRole:
            x = self.points[index.row()]["x"]
            y = self.points[index.row()]["y"]
            z = self.points[index.row()]["z"]
            description = self.points[index.row()]["description"]

            if index.column() == 0:
                return x
            elif index.column() == 1:
                return y
            elif index.column() == 2:
                return z
            elif index.column() == 3:
                return description

        return None

    def headerData(self, section, orientation, role=QtCore.Qt.DisplayRole):
        if role != QtCore.Qt.DisplayRole:
            return None

        if orientation == QtCore.Qt.Horizontal:
            if section == 0:
                return "X"
            elif section == 1:
                return "Y"
            elif section == 2:
                return "Z"
            elif section == 3:
                return "Description"

        return None

    def insertRows(self, position, rows=1, index=QtCore.QModelIndex()):
        self.beginInsertRows(QtCore.QModelIndex(), position, position+rows-1)

        for row in range(rows):
            self.points.insert(position+row, {"x":"", "y":"", "z":"", "description":""})

        self.endInsertRows()
        return True

    def removeRows(self, position, rows=1, index=QtCore.QModelIndex()):
        self.beginRemoveRows(QtCore.QModelIndex(), position, position+rows-1)

        del self.points[position:position+rows]

        self.endRemoveRows()
        return True

    def setData(self, index, value, role=QtCore.Qt.EditRole):
        if role != QtCore.Qt.EditRole:
            return False

        if index.isValid() and 0 <= index.row() < len(self.points):
            point = self.points[index.row()]
            if index.column() == 0:
                point["x"] = value
            elif index.column() == 1:
                point["y"] = value
            elif index.column() == 2:
                point["z"] = value
            elif index.column() == 3:
                point["description"] = value
            else:
                return False

            self.dataChanged.emit(index, index)

        return False

    def flags(self, index):
        if not index.isValid():
            return QtCore.Qt.ItemIsEnabled
        return QtCore.Qt.ItemFlags(QAbstractTableModel.flags(self, index) |
                            QtCore.Qt.ItemIsEditable)