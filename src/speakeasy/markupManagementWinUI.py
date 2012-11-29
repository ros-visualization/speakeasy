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
from python_qt_binding.QtGui import QButtonGroup, QRadioButton, QIntValidator, QApplication
from python_qt_binding.QtCore import pyqtSignal, pyqtSlot

from pythonScriptDialog import DialogService;


from speakeasy_ui import TextPanel;
from markupManagement import Markup, MarkupManagement;

# Maximum silence duration one can insert:
MAX_SILENCE = 10000;

class SliderMode:
    PERCENTAGES = 0
    TIME = 1

class MarkupManagementUI(QDialog):
    
    markupRequestSig = pyqtSignal(Markup.baseType(), str);
    
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
        self.emphasisNoneRadioBt= self.ui.emphasisNoneValRadioButton;
        self.emphasisModerateRadioBt= self.ui.emphasisModerateValRadioButton;
        self.emphasisStrongRadioBt= self.ui.emphasisStrongValRadioButton;
        self.valueSlider    = self.ui.markupValSlider;
        self.valueReadout   = self.ui.valueReadoutLineEdit;
        self.valueReadout.setValidator(QIntValidator());
        self.valueReadout.setText(str(self.valueSlider.value()));
        self.sliderMinLabel = self.ui.sliderMinLabel;
        self.sliderMaxLabel = self.ui.sliderMaxLabel;
        self.sliderExplanationLabel = self.ui.sliderExplanationLabel;
        
        # Saved slider percentage value: 
        self.prevPercentVal = 0;
        # Saved slider msecs value:
        self.prevTimeVal = 0;
        # Remember whether most recently shown slider was percentages or time:
        self.prevSliderMode = SliderMode.PERCENTAGES;
        # Initialize UI related states that are changed in response to clicks:
        self.currMarkupType = Markup.PITCH;
        self.currentEmph = None
        self.setUIToPercentages();
        
    def connectWidgets(self):
        self.deleteRadioBt.clicked.connect(partial(self.speechModTypeAction, 'delete', emph='none'));
        self.silenceRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.SILENCE, emph='none'));
        self.rateRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.RATE, emph='none'));
        self.pitchRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.PITCH, emph='none'));
        self.volumeRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.VOLUME, emph='none'));
        self.emphasisNoneRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.EMPHASIS, emph='none'));
        self.emphasisModerateRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.EMPHASIS, emph='moderate'));
        self.emphasisStrongRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.EMPHASIS, emph='strong'));
        
        self.valueSlider.valueChanged.connect(self.valueSliderChangedAction);
        self.valueSlider.sliderReleased.connect(self.valueSliderManualSlideFinishedAction)
        self.valueReadout.editingFinished.connect(self.valueReadoutEditingFinishedAction);
        
        self.markupRequestSig.connect(self.addOrRemoveMarkup);
    
    def speechModTypeAction(self, markupType, emph='none'):
        self.markupRequestSig.emit(markupType, emph);
           
    def valueSliderChangedAction(self, newVal):
        self.valueReadout.setText(str(newVal));
    
    def valueSliderManualSlideFinishedAction(self):
        # Readout box is already updated. Just need to remember
        # the new value:
        newSliderVal = self.valueSlider.value();
        if self.prevSliderMode == SliderMode.PERCENTAGES:
            self.prevPercentVal = newSliderVal;
        elif self.prevSliderMode == SliderMode.TIME:
            self.prevTimeVal = newSliderVal;
        else:
            raise ValueError('self.prevSliderMode not initialized.');
        
    def valueReadoutEditingFinishedAction(self):
        # Readout entry is already validated to be an integer
        # of the proper range:
        readoutValue = int(self.valueReadout.text());
        self.valueSlider.setValue(readoutValue);
        if self.prevSliderMode == SliderMode.PERCENTAGES:
            self.prevPercentVal = readoutValue;
        elif self.prevSliderMode == SliderMode.TIME:
            self.prevTimeVal = readoutValue;
        
    
    @pyqtSlot(Markup.baseType(), str)
    def addOrRemoveMarkup(self, markupType, emph):
        self.currMarkupType = markupType;
        self.currentEmph    = emph;
        if markupType == Markup.EMPHASIS:
            self.setUIToEmphasis();
        elif markupType == Markup.PITCH or\
             markupType == Markup.RATE or\
             markupType == Markup.VOLUME:
            self.setUIToPercentages();
        elif markupType == Markup.SILENCE:
            self.setUIToTime();
    
    def executeMarkupAction(self):
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
        if markupType != Markup.EMPHASIS:
            value  = self.valueSlider.value();
        else:
            value = MarkupManagement.emphasisCodes[emph];
        newStr = self.markupManager.addMarkup(txtStr, markupType, selStart, length=selLen, value=value);
        self.textPanel.setText(newStr);
        
    def setUIToEmphasis(self):
        self.hideSlider();
    
    def setUIToTime(self):
        self.valueSlider.setValue(self.prevTimeVal);
#        self.sliderMaxLabel.setText(str(MAX_SILENCE));
#        self.sliderMinLabel.setText = str(0);
        self.valueSlider.setMinimum(0);
        self.valueSlider.setMaximum(MAX_SILENCE);
        self.sliderExplanationLabel.setText('Time(msec)')
        self.valueReadout.validator().setRange(0,MAX_SILENCE);
        self.showSlider(SliderMode.TIME);
    
    def setUIToPercentages(self):
        self.valueSlider.setValue(self.prevPercentVal);
#        self.sliderMaxLabel.setText('100%');
#        self.sliderMinLabel.setText('-100%');
        self.sliderExplanationLabel.setText('Magnitude(%)')
        self.valueReadout.validator().setRange(-100,100);
        self.showSlider(SliderMode.PERCENTAGES);
        
    def hideSlider(self):
        self.valueSlider.hide();
        self.sliderMinLabel.hide();
        self.sliderMaxLabel.hide();
        self.sliderExplanationLabel.hide();
        self.valueReadout.hide();

    def showSlider(self, sliderMode):
#        if sliderMode == SliderMode.PERCENTAGES:
#            self.setUIToPercentages();
        self.sliderMinLabel.show();
        self.sliderMaxLabel.show();
        self.valueSlider.show();
        self.sliderExplanationLabel.show();
        self.prevSliderMode = sliderMode;
        self.valueReadout.show();
                  
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
        
        