#!/usr/bin/env python


import sys
import os

import rospy

from functools import partial;
from threading import Timer;

import python_qt_binding;
from python_qt_binding.QtGui import QTextEdit, QErrorMessage, QMainWindow, QColor, QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout, QDialog, QLabel
from python_qt_binding.QtGui import QButtonGroup, QRadioButton, QFrame, QInputDialog, QDoubleSpinBox, QMessageBox, QApplication
from python_qt_binding.QtCore import pyqtSignal

class MarkupManagementUI(QDialog):
    
    def __init__(self):
        super(MarkupManagementUI,self).__init__();
        
        # Fill our space with the UI:
        guiPath = os.path.join(os.path.dirname(__file__), '../qtFiles/markupManagement/markupManagement/markupmanagementdialog.ui');
        self.ui = python_qt_binding.loadUi(guiPath, self);
        
        
        
# ------------------- Testing -----------------------

if __name__ == '__main__':
    
    app = QApplication(sys.argv);
    ui = MarkupManagementUI();
    app.exec_();
    ui.show();
    sys.exit();
        
        