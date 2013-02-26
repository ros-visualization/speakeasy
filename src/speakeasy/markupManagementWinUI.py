#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE

#TODO:
#  o Undo?
#  o Click in markup; select a different markup type in control; move slider ==> number changes
#  o [R-68[P69test]]: cursor on 68, and try to delete: keeps the -68 in the text. 
# Henry instructions:
#  o Select only text, not the markup (?)
#  o Occasionally, (after drag/drop?) text typed into the Text Panel does not advance the cursor.
#    Each letter replaces the previous one. Workaround: click anywhere outside the text panel, then
#    click within the panel again. 

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
from python_qt_binding.QtCore import pyqtSignal, pyqtSlot, QObject

from qt_dialog_service.qt_dialog_service import DialogService; 


try:
    from speakeasy_ui import TextPanel;
except ImportError:
    pass
    
from markupManagement import Markup, MarkupManagement;

# Maximum silence duration one can insert:
MAX_SILENCE = 10000;

DO_DELETE = True;
DONT_DELETE = False;

class SliderID:
    PERCENTAGES = 0
    TIME = 1

class MarkupManagementUI(QDialog):
    
    # Signal used to indicate that a new radio button was selected:
    markupRequestSig = pyqtSignal(Markup.baseType(), str);
    # Signal to indicate that the Delete Variation, or the InsertVariation button was pushed:
    insDelSignal = pyqtSignal(Markup.baseType(), str);

    # --------------------------   Public Methods   --------------------------    
    
    def __init__(self, textPanel=None, parent=None):
        super(MarkupManagementUI,self).__init__(parent=parent);

        self.dialogService = DialogService();
        # Fill our space with the UI:
        self.setupUI();
        
        if textPanel is None:
            self.textPanel = TextPanel(self.ui.testTextEdit);
            self.testing = True;
        else:
            self.textPanel = textPanel;
            # Hide the testing text panel
            self.testTextEdit.hide();
            self.testing = False;
                     
        self.connectWidgets();
        self.markupManager = MarkupManagement();

    def getCurrSliderValue(self):
        '''
        Return the slider value of the currently active
        slider: percentages for Volume/Pitch/Rate, or milliseconds for Silence.
        @return: slider value
        @rtype: int
        '''
        return self.valueReadout.value();
    
    def getCurrEmphValue(self):
        '''
        Return currently selected emphasis value radiobutton as an integer.
        Return None if no emphasis radio button is selected.
        @return: 0 for 'none', 1 for 'moderate', 2 for 'strong'
        @int
        '''
        if self.emphasisNoneRadioBt.isChecked():
            return MarkupManagement.emphasisStrs[0];
        elif self.emphasisModerateRadioBt.isChecked():
            return MarkupManagement.emphasisStrs[1];
        elif self.emphasisStrongRadioBt.isChecked():
            return MarkupManagement.emphasisStrs[2];
        return None
    
    def setUIForMarkup(self, markupType, val=None):
        '''
        Adjust the markup control panel's UI to match a particular
        markup type, and emphasis value. For example, an input of
        Markup.PITCH will hide the silence time slider, and instead
        expose the percentage slider. If sliderVal is None, the slider's
        value will remain unchanged. All radio buttons will be unchecked,
        unless emph is provided. In that case one of the three emphasis radio
        buttons will be checked. Silence/Volume/Pitch/Rate will still be unchecked.
        @param markupType: the markup for which the UI is to be adjusted.
        @type markupType: Markup
        @param val: optionally, the value to which the visible slider is to be set.
        @type val: {int | None}
        '''
        if markupType == Markup.EMPHASIS:
            self.setUIToEmphasis();
        elif markupType == Markup.PITCH or\
             markupType == Markup.RATE or\
             markupType == Markup.VOLUME:
            self.setUIToPercentages();
        elif markupType == Markup.SILENCE:
            self.setUIToTime();
        if markupType == Markup.SILENCE:
            self.silenceRadioBt.setChecked(True);
        elif markupType == Markup.RATE:
            self.rateRadioBt.setChecked(True);
        elif markupType == Markup.PITCH:
            self.pitchRadioBt.setChecked(True);
        elif markupType == Markup.VOLUME:
            self.volumeRadioBt.setChecked(True);
        
        # Depending on the markup type, the value passed in is
        # different: If markupTYpe is Emphasis, the val is an
        # emphasis code, in all other cases the value is a slider
        # value:
        if val is None:
            return;
        # If bad value type, forget it:
        if type(val) != type(10):
            return;
        if markupType == Markup.EMPHASIS:
            if val == 0:
                self.emphasisNoneRadioBt.setChecked(True);
            elif val == 1:
                self.emphasisModerateRadioBt.setChecked(True);
            elif val == 2:
                self.emphasisStrongRadioBt.setChecked(True);
            # Any other value is illegal. Either way, we're done:
            return;
                
        # Markup is one with a slider value:
        visibleSlider = self.visibleSlider();              
        if visibleSlider is not None:
            sliderVal = min(visibleSlider.maximum(), val);
            sliderVal = max(visibleSlider.minimum(), val);
            visibleSlider.setValue(sliderVal);
            # Echo the new slider value on the digital readout:
            self.valueReadout.setText(str(sliderVal));
                
    def cursorInMarkup(self):
        '''
        Return a Markup value if cursor of text panel is currently within
        a markup. Else return None
        @return: a Markup value if the text cursor sits anywhere within a marked-up text fragment. Else None.
        @rtype {int | None}
        '''
        return self.markupManager.getMarkupType(self.textPanel.getText(), self.textPanel.getCursorPos());
        
        
        
    # --------------------------   Private Methods   --------------------------
        
    def setupUI(self):
        
        guiPath = os.path.join(os.path.dirname(__file__), '../qtFiles/markupManagement/markupManagement/markupmanagementdialog.ui');
        self.ui = python_qt_binding.loadUi(guiPath, self);
        

        # Define slightly shorter names to the UI elements:
        self.deleteButton  = self.ui.deleteButton;
        self.deleteButton.setAutoDefault(False);
        self.insertVariationButton = self.ui.insertVariationButton;
        self.insertVariationButton.setAutoDefault(False);
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
        
        # Radio button connections:
        self.silenceRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.SILENCE, emph='none'));
        self.rateRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.RATE, emph='none'));
        self.pitchRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.PITCH, emph='none'));
        self.volumeRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.VOLUME, emph='none'));
        
        self.emphasisNoneRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.EMPHASIS, emph='none'));
        self.emphasisModerateRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.EMPHASIS, emph='moderate'));
        self.emphasisStrongRadioBt.clicked.connect(partial(self.speechModTypeAction, Markup.EMPHASIS, emph='strong'));
        
        # Slider connections:
        self.percentageSlider.valueChanged.connect(self.valueSliderChangedAction);
        #self.percentageSlider.sliderReleased.connect(partial(self.valueSliderManualSlideFinishedAction, SliderID.PERCENTAGES));
        self.timeSlider.valueChanged.connect(self.valueSliderChangedAction);
        #self.timeSlider.sliderReleased.connect(partial(self.valueSliderManualSlideFinishedAction, SliderID.TIME));
        
        # Value readout and input:
        self.valueReadout.editingFinished.connect(self.valueReadoutEditingFinishedAction);
        
        # Buttons connections:
        self.insertVariationButton.clicked.connect(partial(self.insOrDelButtonPushedAction, DONT_DELETE));
        self.deleteButton.clicked.connect(partial(self.insOrDelButtonPushedAction, DO_DELETE));
        
        # Signal connections:
        self.markupRequestSig.connect(self.adjustUIToRadioButtonSelection);
        self.insDelSignal.connect(self.executeMarkupAction);
        
        if not self.testing:
            self.textPanel.mouseClickSignal.connect(self.readyMarkupValChangeUI);

    @pyqtSlot()
    def readyMarkupValChangeUI(self):
        '''
        Handler for cursor changes in the text area. Check whether the cursor is
        within text that is surrounded by markup. If so, find which markup that is,
        and modify the UI to be appropriate for that markup type (expose the correct
        slider, etc.).
        '''
        theStr = self.textPanel.getText();
        cursorPos = self.textPanel.getCursorPos();
        enclosingMarkupType = self.markupManager.getMarkupType(theStr, cursorPos)
        if enclosingMarkupType is None:
            return
        markupValue = self.markupManager.getValue(theStr, cursorPos);
        self.setUIForMarkup(enclosingMarkupType, markupValue);
        # Restore the cursor position to where it was (it gets set to 0 by the actions above):
        self.textPanel.textCursor().setPosition(cursorPos); # **** Necessary ???
    
    def insOrDelButtonPushedAction(self, shouldDelete):
        '''
        Handles variation insert and delete buttons.
        @param markupType: Type of variation selected (the radio buttons)
        @type markupType: Markup
        @param emph: emphasis value (relevant only if Emphasis radio button active).
        @type emph: String
        '''
        if shouldDelete == DO_DELETE:
            self.insDelSignal.emit('delete', self.currentEmph);
        else:
            self.insDelSignal.emit(self.currMarkupType, self.currentEmph);
    
    def speechModTypeAction(self, markupType, emph='none'):
        '''
        Handles all radio button clicks. Raises a signal so that action is
        taken outside the GUI loop.
        @param markupType: the radio button type that was activated.
        @type markupType: Markup
        @param emph: {'none' | 'moderate' | 'strong'}
        @type emph: string
        '''
        self.markupRequestSig.emit(markupType, emph);
           
    def valueSliderChangedAction(self, newVal):
        '''
        Synchronize the digital input-output slider value box with a newly 
        set slider value.
        @param newVal: the new slider value
        @type newVal: int
        '''
        self.valueReadout.setText(str(newVal));
        self.updateMarkupValueInTextPanel(newVal);
        
    def updateMarkupValueInTextPanel(self, newVal):
        # If text panel cursor is currently within a markup,
        # modify that markup's value, unless it's an emphasis,
        # which has no continuous value:
        theStr = self.textPanel.getText();
        curPos = self.textPanel.getCursorPos();
        cursor = self.textPanel.textCursor();
        markupType = self.markupManager.getMarkupType(theStr, curPos);
        if markupType is None:
            return;
        # If mark type in the text is different than the mark type whose
        # radio button is active in the markup control panel, then do nothing:
        if self.getSelectedMarkupRadioBtn() != markupType:
            return;
        
        newStr = self.markupManager.changeValue(theStr, curPos, newVal);
        self.textPanel.setText(newStr);
        # Set the cursor to where it was before replacing the string
        # in the text panel:
        cursor.setPosition(curPos)
        self.textPanel.setTextCursor(cursor);
             
    def valueReadoutEditingFinishedAction(self):
        '''
        User typed a value into the value I/O box. Synchronize
        the slider that is currently visible.
        '''
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
    def adjustUIToRadioButtonSelection(self, markupType, emph):
        '''
        Signal handler for markupRequestSig. This signal is sent when
        a radio button is clicked in the UI. The UI is modified to
        reflect the next possible actions.
        @param markupType: the radio button that was clicked.
        @type markupType: Markup
        @param emph: relevant if Markup == Markup.EMPHASIS: the value of the emphasis value ('none', 'moderate', 'strong').
        @type emph: string
        '''
        self.currMarkupType = markupType;
        self.currentEmph    = emph;
        if markupType == Markup.EMPHASIS:
            self.setUIToEmphasis();
            emphasisValueInt = MarkupManagement.emphasisCodes[emph];
            self.updateMarkupValueInTextPanel(emphasisValueInt);
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
        
    
    @pyqtSlot(Markup.baseType(), str)
    def executeMarkupAction(self, markupType, emph):
        '''
        Handles signal insDelSignal is emitted, which happens when the Delete or
        Insert Variation button is clicked. Operates
        on the text panel to surround a selected text piece with a variation markup, or
        deletes a variation markup the surrounds the current cursor position.
        @param markupType: the active radio button's identifier, or 'delete' 
        @type markupType: {Markup | 'delete'}
        '''
        # Get a copy of the text panel's current content, the cursor position,
        # and selection boundaries, if a selection is present:
        (txtStr, cursorPos, selStart, selEnd) = self.getTextAndSelection();
        
        # Are we to delete enclosing markup?
        if markupType == 'delete':
            newStr = self.markupManager.removeMarkup(txtStr, cursorPos);
            self.textPanel.setText(newStr);
            return;

        # All actions other than markup silence, and deletion require a selection:         
        if markupType != Markup.SILENCE:
            if selStart is None or selEnd is None:
                self.dialogService.showErrorMsg('Select a piece of text that you wish to modulate.');
                return;
            else:
                selLen = selEnd - selStart;
        if markupType == Markup.SILENCE:
            value = self.valueTimeSlider.value();
            # Silence is inserted between words or at the start of a string.
            # So the cursor position is the marker, not a selection:
            selStart = cursorPos;
            selLen = 0;
        elif markupType != Markup.EMPHASIS:
            value  = self.valuePercSlider.value();
        else:
            value = MarkupManagement.emphasisCodes[emph];
        try:
            newStr = self.markupManager.addMarkup(txtStr, markupType, selStart, length=selLen, value=value);
        except ValueError as e:
            self.dialogService.showErrorMsg(`e`);
            return;
        self.textPanel.setText(newStr);
        
    def setUIToEmphasis(self):
        '''
        Modify UI to show only emphasis choices: 
        '''
        self.hideSliders();
    
    def setUIToTime(self):
        '''
        Modify UI to show the time duration slider for defining silence duration:
        '''
        self.valueReadout.validator().setRange(0,MAX_SILENCE);
        self.valueReadout.setText(str(self.timeSlider.value()));
        self.showSlider(SliderID.TIME);
        self.hideSliders(SliderID.PERCENTAGES);
    
    def setUIToPercentages(self):
        '''
        Modify UI to show the percentage slider relevant to 
        volume, rate, and pitch markups.
        '''
        self.valueReadout.validator().setRange(-100,100);
        self.valueReadout.setText(str(self.valuePercSlider.value()));
        self.showSlider(SliderID.PERCENTAGES);
        self.hideSliders(SliderID.TIME);
        
    def hideSliders(self, sliderID=None):
        '''
        Hide one or both percentage or time sliders.
        @param sliderID: which slider to hide. None if hide both
        @type sliderID: {SliderID | None}
        '''
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
        '''
        Reveal the specified slider
        @param sliderID: the slider to reveal (never both)
        @type sliderID: SliderID
        '''
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
                
    def visibleSlider(self):
        '''
        Return the slider object that is currently visible to the user.
        @return: slider object (percentage or time slider). If both sliders
        are currently hidden, return None.
        @rtype: {QSlider | None}
        '''
        
        if self.valuePercSlider.isVisible():
            return self.valuePercSlider;
        elif self.valueTimeSlider.isVisible():
            return self.valueTimeSlider;
        else:
            return None
          
    def getSelectedMarkupRadioBtn(self):
        '''
        Return the Markup type whose radio button is selected in the
        markup control panel. If none selected, return None
        @return: the Markup type whose radio button is selected
        @rtype: {Markup | None}
        '''
        if self.silenceRadioBt.isChecked():
            return Markup.SILENCE;
        elif self.rateRadioBt.isChecked(): 
            return Markup.RATE;
        elif self.pitchRadioBt.isChecked():
            return Markup.PITCH;
        elif self.volumeRadioBt.isChecked():
            return Markup.VOLUME;
        elif self.emphasisNoneRadioBt.isChecked() or self.emphasisModerateRadioBt.isChecked() or self.emphasisStrongRadioBt.isChecked():
            return Markup.EMPHASIS; 
        return None;
    
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

    def shutdown(self):
        sys.exit();


#******    class ValueReadoutValidator(QRegExpValidator):
         
        
# ------------------- Testing -----------------------

class TextPanel(object):

    mouseClickSignal = pyqtSignal();
    
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

    def getCursorPos(self):
        return self.textCursor().position;

    def isEmpty(self):
        return len(self.textPanel.toPlainText()) == 0;
          
if __name__ == '__main__':
    
    app = QApplication(sys.argv);
    ui = MarkupManagementUI();
    ui.show();
    app.exec_();
    sys.exit();
        
        