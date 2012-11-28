#!/usr/bin/env python


import sys
import os

import rospy

from functools import partial;
from threading import Timer;
from functools import partial;

import python_qt_binding;
from python_qt_binding import QtGui
from python_qt_binding.QtGui import QTextEdit, QErrorMessage, QMainWindow, QColor, QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout, QDialog, QLabel
from python_qt_binding.QtGui import QButtonGroup, QRadioButton, QFrame, QInputDialog, QDoubleSpinBox, QMessageBox, QApplication
from python_qt_binding.QtCore import pyqtSignal, pyqtSlot

from pythonScriptDialog import DialogService;


from speakeasy_ui import TextPanel;
from markupManagement import Markup, MarkupManagement;

#from accessibleSlider import 

class MarkupManagementUI(QDialog):
    
    markupRequestSig = pyqtSignal(Markup.baseType());
    
    def __init__(self, textPanel=None):
        super(MarkupManagementUI,self).__init__();

        self.dialogService = DialogService();
        # Fill our space with the UI:
        self.setupUI();
        
        if textPanel is None:
            self.textPanel = TextPanel(self.ui.testTextEdit);
        else:
            self.textPanel = textPanel;        
        self.connectWidgets();
        self.markupManager = MarkupManagement();
        
    def setupUI(self):
        guiPath = os.path.join(os.path.dirname(__file__), '../qtFiles/markupManagement/markupManagement/markupmanagementdialog.ui');
        self.ui = python_qt_binding.loadUi(guiPath, self);
        self.deleteRadioBt  = self.ui.deleteRadioButton;
        self.silenceRadioBt = self.ui.silenceRadioButton;
        self.rateRadioBt    = self.ui.rateRadioButton;
        self.pitchRadioBt   = self.ui.pitchRadioButton;
        self.volumeRadioBt  = self.ui.volumeRadioButton;
        self.emphasisRadioBt= self.ui.emphasisRadioButton;
        self.valueSlider    = self.ui.markupValSlider;
        self.emphasisBox    = self.ui.emphasisValuesGroupBox;
        self.emphasisBox.hide();
        
        
    def connectWidgets(self):
        self.deleteRadioBt.clicked.connect(partial(self.speechModTypeAction, 'delete'));
        self.silenceRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.SILENCE));
        self.rateRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.RATE));
        self.pitchRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.PITCH));
        self.volumeRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.VOLUME));
        self.emphasisRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.EMPHASIS));
        
        self.markupRequestSig.connect(self.addOrRemoveMarkup);
    
    def speechModTypeAction(self, markupType):
        #self.addOrRemoveMarkup(markupType);
        self.markupRequestSig.emit(markupType);
           
    @pyqtSlot(Markup.baseType())
    def addOrRemoveMarkup(self, markupType):
        (txtStr, cursorPos, selStart, selEnd) = self.getTextAndSelection();
        
        # Are we to delete enclosing markup?
        if markupType == 'delete':
            newStr = self.markupManager.removeMarkup(txtStr, cursorPos);
            self.textPanel.setText(newStr);
            return;
        
        if selStart is None or selEnd is None:
            self.dialogService.showErrorMsg('Select a piece of text that you wish to modulate.');
            return;
        selLen = selEnd - selStart;
        value  = self.valueSlider.value();
        newStr = self.markupManager.addMarkup(txtStr, markupType, selStart, length=selLen, value=value);
        self.textPanel.setText(newStr);
        
      
    def getTextAndSelection(self):
        '''
        Returns a four-tuple string, cursor position, start index, and end index. The string is a copy of
        the current self.textPanel object. The two index numbers are the start (inclusive),
        and end (exclusive) of the current text selection. If no text is selected, 
        return (text, None, None).
        @return: triplet string, and start/end of text selection.
        @rtype: (String,int,int,int) or (String,int,None,None)
        '''
        
        txtCursor = self.textPanel.textCursor();
        curPos    = txtCursor.position();
        selStart  = txtCursor.selectionStart();
        selEnd    = txtCursor.selectionEnd();
        if selStart == selEnd:
            selStart = selEnd = None;
        txt       = self.textPanel.getText();
        return (txt,curPos,selStart,selEnd);       
          
# ------------------- Testing -----------------------

class TextPanel(object):
    
    def __init__(self, qTextPanel):
        self.textPanel = qTextPanel;
    
    def setText(self, theStr):
        self.textPanel.clear();
        self.textPanel.setText(theStr);
    
    def getText(self):
        # Get text field text as plain text, and convert
        # unicode to ascii, ignoring errors:
        #return self.toPlainText().encode('ascii', 'ignore');
        return self.textPanel.toPlainText().encode('utf-8');
    
    def textCursor(self):
        return self.textPanel.textCursor();
    
    def isEmpty(self):
        return len(self.textPanel.toPlainText()) == 0;
          
if __name__ == '__main__':
    
    app = QApplication(sys.argv);
    ui = MarkupManagementUI();
    ui.show();
    app.exec_();
    sys.exit();
        
        