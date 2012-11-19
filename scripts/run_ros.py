#!/usr/bin/env python
"""
Copyright 2012 by Robert Cofield, for GAVLab. All rights reserved.

Master runner file.
Execute from within the scripts/ folder
"""
from survey import MainWindow
from yaml import load
from PySide import QtGui
import sys, os
from ui_main_window import Ui_MainWindow

import rospy, roslib
roslib.load_manifest('survey_gui')

### Settings ###
viz_config = load(file('/home/rgcofield/devel/survey_ws/moos_rtk_survey/cfg/survey.yaml'))


def main():
    #Setup QtApp
    app = QtGui.QApplication(sys.argv[0])
    ui_mainwindow = Ui_MainWindow()

    # Ros


    main_window = MainWindow(ui_mainwindow, viz_config)
    main_window.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    rospy.init_node('survey')
    main()