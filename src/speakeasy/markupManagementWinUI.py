#!/usr/bin/env python

#TODO:
#  o Undo?
#  o Value changes

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
        else:
            self.textPanel = textPanel;
            # Hide the testing text panel
            self.testTextEdit.hide();
                     
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
    
    def setUIForMarkup(self, markupType, sliderVal=None, emph=None):
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
        @param sliderVal: optionally, the value to which the visible slider is to be set.
        @type sliderVal: int
        @param emph: optionally, which emphasis checkbox is to be checked.
        @type emph: int [0..2] for 'none', 'moderate', 'strong'
        '''
        if markupType == Markup.EMPHASIS:
            self.setUIToEmphasis();
        elif markupType == Markup.PITCH or\
             markupType == Markup.RATE or\
             markupType == Markup.VOLUME:
            self.setUIToPercentages();
        elif markupType == Markup.SILENCE:
            self.setUIToTime();
        self.silenceRadioBt.setChecked(False);
        self.rateRadioBt.setChecked(False);
        self.pitchRadioBt.setChecked(False);
        self.volumeRadioBt.setChecked(False);
        if sliderVal is not None:
            if type(sliderVal) != type(10):
                return;
            visibleSlider = self.visibleSlider();              
            if visibleSlider is not None:
                sliderVal = min(visibleSlider.maximum(), sliderVal);
                sliderVal = max(visibleSlider.minimum(), sliderVal);
                visibleSlider.setValue(sliderVal);
                # Echo the new slider value on the digital readout:
                self.valueReadout.setText(str(sliderVal));
                

    # --------------------------   Private Methods   --------------------------
        
    def setupUI(self):
        
        guiPath = os.path.join(os.path.dirname(__file__), '../qtFiles/markupManagement/markupManagement/markupmanagementdialog.ui');
        self.ui = python_qt_binding.loadUi(guiPath, self);
        

        # Define slightly shorter names to the UI elements:
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
    
#    def valueSliderManualSlideFinishedAction(self, sliderID):
#        pass
        
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
        except ValueError, e:
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
        
        if self.valuePercSlider.isvisible():
            return self.valuePercSlider;
        elif self.valueTimeSlider.isvisible():
            return self.valueTimeSlider;
        else:
            return None
          
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
        
        