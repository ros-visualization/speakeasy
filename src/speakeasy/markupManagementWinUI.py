#!/usr/bin/env python

#TODO:
#  o Delete button
#  o Insert button
#  o Undo?

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

class SliderID:
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
        self.deleteButton  = self.ui.deleteButton;
        self.insertVariationButton = self.ui.insertVariationButton;
        self.silenceRadioBt = self.ui.silenceRadioButton;
        self.rateRadioBt    = self.ui.rateRadioButton;
        self.pitchRadioBt   = self.ui.pitchRadioButton;
        self.volumeRadioBt  = self.ui.volumeRadioButton;
        self.emphasisNoneRadioBt= self.ui.emphasisNoneValRadioButton;
        self.emphasisModerateRadioBt= self.ui.emphasisModerateValRadioButton;
        self.emphasisStrongRadioBt= self.ui.emphasisStrongValRadioButton;
        self.valuePercSlider    = self.ui.percentageSlider;
        self.percSliderMin = self.valuePercSlider.minimum();
        self.percSliderMax = self.valuePercSlider.maximum();
        self.valueTimeSlider    = self.ui.timeSlider;
        self.timeSliderMin = self.valueTimeSlider.minimum();
        self.timeSliderMax = self.valueTimeSlider.maximum();
        self.valueReadout   = self.ui.valueReadoutLineEdit;
        self.valueReadout.setValidator(QIntValidator());
        self.valueReadout.setText(str(self.percentageSlider.value()));
        self.percMinLabel = self.ui.percMinLabel;
        self.percMaxLabel = self.ui.percMaxLabel;
        self.timeMinLabel = self.ui.timeMinLabel;
        self.timeMaxLabel = self.ui.timeMaxLabel;
        self.axisLabel = self.ui.axisLabel;
        
        # Initialize UI related states that are changed in response to clicks:
        self.currMarkupType = Markup.PITCH;
        self.currentEmph = None
        self.setUIToPercentages();
        
    def connectWidgets(self):
        self.deleteButton.clicked.connect(partial(self.speechModTypeAction, 'delete', emph='none'));
        self.silenceRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.SILENCE, emph='none'));
        self.rateRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.RATE, emph='none'));
        self.pitchRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.PITCH, emph='none'));
        self.volumeRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.VOLUME, emph='none'));
        self.emphasisNoneRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.EMPHASIS, emph='none'));
        self.emphasisModerateRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.EMPHASIS, emph='moderate'));
        self.emphasisStrongRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.EMPHASIS, emph='strong'));
        
        self.percentageSlider.valueChanged.connect(self.valueSliderChangedAction);
        self.percentageSlider.sliderReleased.connect(partial(self.valueSliderManualSlideFinishedAction, SliderID.PERCENTAGES));
        self.timeSlider.valueChanged.connect(self.valueSliderChangedAction);
        self.timeSlider.sliderReleased.connect(partial(self.valueSliderManualSlideFinishedAction, SliderID.TIME));
        self.valueReadout.editingFinished.connect(self.valueReadoutEditingFinishedAction);
        
        self.markupRequestSig.connect(self.addOrRemoveMarkup);
        self.insertVariationButton.clicked.connect(self.executeMarkupAction);
    
    def speechModTypeAction(self, markupType, emph='none'):
        self.markupRequestSig.emit(markupType, emph);
           
    def valueSliderChangedAction(self, newVal):
        self.valueReadout.setText(str(newVal));
    
    def valueSliderManualSlideFinishedAction(self, sliderID):
        pass
        
    def valueReadoutEditingFinishedAction(self):
        # Get value from the readout box. The validator
        # already ensured that the value is an int:
        newVal = int(self.valueReadout.text());
        if self.valuePercSlider.isVisible():
            if newVal > self.percSliderMax:
                newVal = self.percSliderMax;
                self.valueReadout.setText(str(newVal));
            elif newVal < self.percSliderMin:
                newVal = self.percSliderMin;
                self.valueReadout.setText(str(newVal));
            self.valuePercSlider.setValue(newVal);

        elif self.timeSlider.isVisible():
            if newVal > self.timeSliderMax:
                newVal = self.timeSliderMax;
                self.valueReadout.setText(str(newVal));
            elif newVal < self.timeSliderMin:
                newVal = self.timeSliderMin;
                self.valueReadout.setText(str(newVal));
            self.timeSlider.setValue(newVal);
    
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
        elif markupType == 'delete':
            (txtStr, cursorPos, selStart, selEnd) = self.getTextAndSelection();
            # Are we to delete enclosing markup?
            if markupType == 'delete':
                newStr = self.markupManager.removeMarkup(txtStr, cursorPos);
                self.textPanel.setText(newStr);
                return;
    
    def executeMarkupAction(self, markupType):
        (txtStr, cursorPos, selStart, selEnd) = self.getTextAndSelection();
#        # Are we to delete enclosing markup?
#        if markupType == 'delete':
#            newStr = self.markupManager.removeMarkup(txtStr, cursorPos);
#            self.textPanel.setText(newStr);
#            return;
        
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
        self.hideSliders();
    
    def setUIToTime(self):
        self.valueReadout.validator().setRange(0,MAX_SILENCE);
        self.valueReadout.setText(str(self.timeSlider.value()));
        self.showSlider(SliderID.TIME);
        self.hideSliders(SliderID.PERCENTAGES);
    
    def setUIToPercentages(self):
        self.valueReadout.validator().setRange(-100,100);
        self.valueReadout.setText(str(self.valuePercSlider.value()));
        self.showSlider(SliderID.PERCENTAGES);
        self.hideSliders(SliderID.TIME);
        
    def hideSliders(self, sliderID=None):
        if sliderID == SliderID.PERCENTAGES or sliderID is None:
            self.percentageSlider.hide();
            self.percMinLabel.hide();
            self.percMaxLabel.hide();
            
        if sliderID == SliderID.TIME or sliderID is None:
            self.timeSlider.hide();
            self.timeMinLabel.hide();
            self.timeMaxLabel.hide();
        if sliderID is None:
            self.axisLabel.hide();
            self.valueReadout.hide();

    def showSlider(self, sliderID):
        if sliderID == SliderID.PERCENTAGES:
            self.valuePercSlider.show();
            self.percMinLabel.show();
            self.percMaxLabel.show();
        elif sliderID == SliderID.TIME:
            self.timeSlider.show();
            self.timeMinLabel.show();
            self.timeMaxLabel.show();
        self.axisLabel.show();
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
        
        