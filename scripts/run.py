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

### Settings ###
# pkg_dir = os.path.abspath('moos_rtk_survey')
# viz_config = load(file(pkg_dir+'/cfg/survey.yaml', 'r'))
# moos_config = load(file(pkg_dir+'/cfg/moos.yaml', 'r'))
viz_config = load(file('/home/rgcofield/devel/survey_ws/moos_rtk_survey/cfg/survey.yaml'))
# moos_config = load(file('/home/rgcofield/devel/survey_ws/moos_rtk_survey/cfg/moos.yaml'))


def main():
    #Setup QtApp
    app = QtGui.QApplication(sys.argv[0])
    ui_mainwindow = Ui_MainWindow()

    main_window = MainWindow(ui_mainwindow, viz_config)
    main_window.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()