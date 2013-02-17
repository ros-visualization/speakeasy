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

import sys
import os

import rospy

from functools import partial;
from threading import Timer;

from python_qt_binding.QtGui import QTextEdit, QErrorMessage, QMainWindow, QColor, QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout, QDialog, QLabel
from python_qt_binding.QtGui import QButtonGroup, QRadioButton, QFrame, QInputDialog, QDoubleSpinBox, QMessageBox, QFocusEvent
from python_qt_binding.QtCore import pyqtSignal, pyqtSlot

from markupManagementWinUI import MarkupManagementUI;

#TODO: Play repeatedly.
#TODO: From speakeasy_node's capabilities message, find all voices and their tts engines. Represent them in the radio buttons.

NUM_SPEECH_PROGRAM_BUTTONS = 24;

class Option:
    YES = True;
    NO  = False;

class Orientation:
    HORIZONTAL = 0;
    VERTICAL   = 1;

class Alignment:
    LEFT = 0;
    CENTER = 1;
    RIGHT = 2;
    TOP = 3;
    BOTTOM = 4;
    
class LocationSpec():
    LEFT = 0;
    RIGHT = 1;
    TOP = 2;
    BOTTOM = 3;

class CheckboxGroupBehavior:
    RADIO_BUTTONS = 0;
    CHECKBOXES    = 1;

class PlayLocation:
    ROBOT   = 'PLAY_AT_ROBOT';
    LOCALLY = 'PLAY_LOCALLY';

# Whether the initial UI is started with the
# Robot as play destination, or with Local as 
# destination. This module variable may be
# used by clients to prepare the environment
# ahead of building the GUI (e.g. initialize
# a ROS node, or local sound engine.):

DEFAULT_PLAY_LOCATION = PlayLocation.ROBOT;


#--------------------------------  TextPanel Class ---------------------------
class TextPanel(QTextEdit):
    '''
    Text input field whose number of lines may be specified approximately.
    '''
    
    mouseClickSignal = pyqtSignal();

    #----------------------------------
    # Initializer 
    #--------------

    def __init__(self, numLines=None):
        super(TextPanel, self).__init__(None);
        
        if numLines is not None:
            # Want the text area to be numLines high.
            # Get the app's font, get its height, and use it
            # to specify the text field's maximum height:
            currentFontHeight = self.fontMetrics().height();
            # add 5 pixels for each line:
            self.setMinimumHeight((currentFontHeight * numLines) + (5 * numLines));
            self.setMinimumWidth(600); #px

    #----------------------------------
    # getText
    #--------------
    
    def getText(self):
        # Get text field text as plain text, and convert
        # unicode to ascii, ignoring errors:
        #return self.toPlainText().encode('ascii', 'ignore');
        return self.toPlainText().encode('utf-8');
    
    #----------------------------------
    # getCursorPos
    #--------------
    
    def getCursorPos(self):
        return self.textCursor().position();
    
    #----------------------------------
    # isEmpty
    #--------------

    def isEmpty(self):
        return len(self.toPlainText()) == 0;


    #----------------------------------
    # isEmpty
    #--------------

    @pyqtSlot()
    def mouseReleaseEvent(self,event):
        '''
        Called when mouse is clicked within the text panel.
        Raise a new signal that enclosing applications can
        subscribe to, and have the internal signal cascading
        system percolate the signal to the QTextPanel handler.
        @param event: details of the mouse event
        @type event: QMouseSignal
        '''
        self.mouseClickSignal.emit();

# ----------------------------------------------- Class CommChannel ------------------------------------


#class CommChannel(QObject):
#    '''
#    Combines signals into one place.
#    '''
#    hideButtonSignal  	= Signal(QPushButton); # hide the given button
#    showButtonSignal  	= Signal(QPushButton); # show the hidden button
#    
#    def __init__(self):
#        super(CommChannel, self).__init__();
    
# ----------------------------------------------- Class DialogService ------------------------------------

class DialogService(object):

    class ButtonSaveResult:
        UPDATE_CURRENT = 0;
        NEW_SET = 1;
        CANCEL = 2;
        

    #----------------------------------
    # Initializer
    #--------------

    def __init__(self, parent=None):
        
        # All-purpose error popup message:
        # Used by self.showErrorMsgByErrorCode(<errorCode>), 
        # or self.showErrorMsg(<string>). Returns a
        # QErrorMessage without parent, but with QWindowFlags set
	    # properly to be a dialog popup box:
        self.errorMsgPopup = QErrorMessage.qtHandler();
       	# Re-parent the popup, retaining the window flags set
        # by the qtHandler:
        self.errorMsgPopup.setParent(parent, self.errorMsgPopup.windowFlags());
        self.errorMsgPopup.setStyleSheet(SpeakEasyGUI.stylesheetAppBG);
        
        self.infoMsg = QMessageBox(parent=parent);
        self.infoMsg.setStyleSheet(SpeakEasyGUI.stylesheetAppBG);
    
    #----------------------------------
    # showErrorMsg
    #--------------
    QErrorMessage
    def showErrorMsg(self,errMsg):
        '''
        Given a string, pop up an error dialog.
        @param errMsg: The message
        @type errMsg: string
        '''
        self.errorMsgPopup.showMessage(errMsg);
    
    #----------------------------------
    # showInfoMsg 
    #--------------

    def showInfoMessage(self, text):
        self.infoMsg.setText(text);
        self.infoMsg.exec_();        

    #----------------------------------
    # newButtonSetOrUpdateCurrent
    #--------------
    
    def newButtonSetOrUpdateCurrent(self):
        '''
        Asks user whether saving of button set is to be 
        to a new file, or an update to the current file.
        Cancel is offered as well.
        @return: ButtonSaveResult.NEW_SET, ButtonSaveResult.UPDATE_CURRENT, or ButtonSaveResult.CANCEL
        @rtype: DialogService.ButtonSaveResult
        '''
        msgBox = QMessageBox()
        msgBox.setStyleSheet(SpeakEasyGUI.stylesheetAppBG);
        msgBox.setText('Create new button set, or update current set?')

        updateCurrButton   = QPushButton('Update current')
        msgBox.addButton(updateCurrButton, QMessageBox.NoRole)
        
        newSetButton   = QPushButton('Create new set')
        msgBox.addButton(newSetButton, QMessageBox.YesRole)
        
        cancelButton = QPushButton('Cancel');
        msgBox.addButton(cancelButton, QMessageBox.RejectRole)

        value = msgBox.exec_();
        return value;



#----------------------------------------------------  SpeakEasyGUI Class ---------------------------

class SpeakEasyGUI(QMainWindow):
    '''
    One instance of this class builds the entire sound play UI.
    Instance variable that hold widgets of interest to controllers:
    
	    - C{speechInputFld}
	    - C{onceOrRepeatDict}
	    - C{voicesRadioButtonsDict}
	    - C{recorderDict}
	    - C{programButtonDict}
	    - C{soundButtonDict}

    '''

    # ---------------------- Durations --------------------
    
    # Number of seconds a program button must be pressed to
    # go into program mode:
    PROGRAM_BUTTON_HOLD_TIME = 3.0;

    # Amount of time the program button stays in alternate look
    # to indicate that it is changing to programming mode: 

    PROGRAM_BUTTON_LOOK_CHANGE_DURATION = 0.2; # seconds

    # ---------------------- Button and Font Sizes -------------------- 

    # Minimum height of buttons:
    BUTTON_MIN_HEIGHT = 30; # pixels
    # Pushbutton minimum font size:
    BUTTON_LABEL_FONT_SIZE = 16; # pixels
    
    # Radio button minimum font size:
    RADIO_BUTTON_LABEL_FONT_SIZE = 16; # pixels
    
    EDIT_FIELD_TEXT_SIZE = 18; # pixels
    
    NUM_OF_PROGRAM_BUTTON_COLUMNS = 4;
    NUM_OF_SOUND_BUTTON_COLUMNS   = 4;
    
    # ---------------------- Names for Voices ----------------------
    
    # Official names of voices as recognized by the
    # underlying text-to-speech engines:
    voices = {'VOICE_1': 'voice_kal_diphone',  # Festival voice
              'VOICE_2': 'David',              # Cepstral voices
              'VOICE_3': 'Amy',
              'VOICE_4': 'Shouty',
              'VOICE_5': 'Whispery',
              'VOICE_6': 'Lawrence',
              'VOICE_7': 'William'
              };
        
    # ---------------------- Names for Widgets ----------------------
    
    interactionWidgets = {
                          'PLAY_ONCE': 'Play once',
                          'PLAY_REPEATEDLY': 'Play repeatedly',
                          'PLAY_REPEATEDLY_PERIOD': 'Pause between plays',
                          'VOICE_1': 'Machine',
                          'VOICE_2': 'David',
                          'VOICE_3': 'Amy',
                          'VOICE_4': 'Shout',
                          'VOICE_5': 'Whisper',
                          'VOICE_6': 'Lawrence',
                          'VOICE_7': 'William',
                          'PLAY_TEXT': 'Play Text',
                          'STOP': 'Stop',
                          'STOP_ALL' : 'Stop All',
                          
                          'SPEECH_1' : 'Speech 1',
                          'SPEECH_2' : 'Speech 2',
                          'SPEECH_3' : 'Speech 3',
                          'SPEECH_4' : 'Speech 4',
                          'SPEECH_5' : 'Speech 5',
                          'SPEECH_6' : 'Speech 6',
                          'SPEECH_7' : 'Speech 7',
                          'SPEECH_8' : 'Speech 8',
                          'SPEECH_9' : 'Speech 9',
                          'SPEECH_10' : 'Speech 10',
                          'SPEECH_11' : 'Speech 11',
                          'SPEECH_12' : 'Speech 12',
                          'SPEECH_13' : 'Speech 13',
                          'SPEECH_14' : 'Speech 14',
                          'SPEECH_15' : 'Speech 15',
                          'SPEECH_16' : 'Speech 16',
                          'SPEECH_17' : 'Speech 17',
                          'SPEECH_18' : 'Speech 18',
                          'SPEECH_19' : 'Speech 19',
                          'SPEECH_20' : 'Speech 20',
                          'SPEECH_21' : 'Speech 21',
                          'SPEECH_22' : 'Speech 22',
                          'SPEECH_23' : 'Speech 23',
                          'SPEECH_24' : 'Speech 24',
                          
                          'SOUND_1' : 'Rooster',
                          'SOUND_2' : 'Drill',
                          'SOUND_3' : 'Bull call',
                          'SOUND_4' : 'Clown horn',
                          'SOUND_5' : 'Cash register',
                          'SOUND_6' : 'Glass breaking',
                          'SOUND_7' : 'Cell door',
                          'SOUND_8' : 'Cow',
                          'SOUND_9' : 'Birds',
                          'SOUND_10' : 'Big truck',
                          'SOUND_11' : 'Seagulls',
                          'SOUND_12' : 'Lift',
                          
                          'NEW_SPEECH_SET' : 'Save speech set',
                          'PICK_SPEECH_SET' : 'Pick different speech set',
                          
                          'PLAY_LOCALLY' : 'Play locally',
                          'PLAY_AT_ROBOT' : 'Play at robot',
                          'SPEECH_MODULATION' : 'Speech modulation',
                          'PASTE' : 'Paste', 
                          'CLEAR' : 'Clear', 
                          } 
        
    # ---------------------- Application Background styling --------------------     

    # Application background:
    veryLightBlue = QColor(230, 255,255);
    stylesheetAppBG = 'QDialog {background-color: %s}' % veryLightBlue.name();
    
    defaultStylesheet = stylesheetAppBG;
    defaultStylesheetName = "Default";
    
    # ---------------------- Edit field styling -----------------

    editFieldBGColor =  QColor(12,21,109);
    editFieldTextColor = QColor(244,244,246);

    inputFldStylesheet =\
        'TextPanel {background-color: ' + editFieldBGColor.name() +\
        '; color: ' + editFieldTextColor.name() +\
        '; font-size: ' + str(EDIT_FIELD_TEXT_SIZE) + 'pt' +\
        '}';
    
    # ---------------------- Pushbutton styling -------------------- 
    
    # Button color definitions:
    recorderButtonBGColor = QColor(176,220,245); # Light blue
    recorderButtonDisabledBGColor = QColor(187,200,208); # Gray-blue
    recorderButtonTextColor = QColor(0,0,0);     # Black
    programButtonBGColor = QColor(117,150,169);  # Middle blue
    programButtonTextColor = QColor(251,247,247);# Off-white
    soundButtonBGColor = QColor(110,134,211);      # Dark blue
    soundButtonTextColor = QColor(251,247,247);  # Off-white

    # Button stylesheets:    
    recorderButtonStylesheet =\
        'QPushButton {background-color: ' + recorderButtonBGColor.name() +\
        '; color: ' + recorderButtonTextColor.name() +\
        '; font-size: ' + str(BUTTON_LABEL_FONT_SIZE) + 'px' +\
        '}';
        
    recorderButtonDisabledStylesheet =\
        'QPushButton {background-color: ' + recorderButtonDisabledBGColor.name() +\
        '; color: ' + recorderButtonTextColor.name() +\
        '; font-size: ' + str(BUTTON_LABEL_FONT_SIZE) + 'px' +\
        '}';
        
    programButtonStylesheet =\
        'QPushButton {background-color: ' + programButtonBGColor.name() +\
        '; color: ' + programButtonTextColor.name() +\
        '; font-size: ' + str(BUTTON_LABEL_FONT_SIZE) + 'px' +\
        '}';

    # Stylesheet for when program button look is temporarily
    # changed to indicate transition to programming mode:                                                          
    programButtonModeTransitionStylesheet =\
        'QPushButton {background-color: ' + editFieldBGColor.name() +\
        '; color: ' + programButtonTextColor.name() +\
        '; font-size: ' + str(BUTTON_LABEL_FONT_SIZE) + 'px' +\
        '}';
                                                          
                                                          
    soundButtonStylesheet =\
        'QPushButton {background-color: ' + soundButtonBGColor.name() +\
        '; color: ' + soundButtonTextColor.name() +\
        '; font-size: ' + str(BUTTON_LABEL_FONT_SIZE) + 'px' +\
        '}';

    # ---------------------- Radiobutton and Play Repeat Delay Spinbox styling -----------------
    
    # Radiobutton color definitions:
    playOnceRepeatButtonBGColor = QColor(121,229,230); # Very light cyan
    voicesButtonBGColor = QColor(97,164,165); # Darker cyan

    
    playOnceRepeatButtonStylesheet =\
     'font-size: ' + str(RADIO_BUTTON_LABEL_FONT_SIZE) + 'px' +\
     '; color: ' + soundButtonTextColor.name() +\
     '; background-color: ' + voicesButtonBGColor.name();
     
    playRepeatSpinboxStylesheet =\
     'font-size: ' + str(RADIO_BUTTON_LABEL_FONT_SIZE) + 'px' +\
     '; color: ' + soundButtonTextColor.name() +\
     '; background-color: ' + voicesButtonBGColor.name();

#    voiceButtonStylesheet =\
#     'font-size: ' + str(RADIO_BUTTON_LABEL_FONT_SIZE) + 'px' +\
#     '; background-color: ' + voicesButtonBGColor.name();

    voiceButtonStylesheet =\
     'font-size: ' + str(RADIO_BUTTON_LABEL_FONT_SIZE) + 'px' +\
     '; color: ' + soundButtonTextColor.name() +\
     '; background-color: ' + voicesButtonBGColor.name();

    # ---------------------- Signals -----------------
    
    hideButtonSignal  	= pyqtSignal(QPushButton); # hide the given button
    showButtonSignal  	= pyqtSignal(QPushButton); # show the hidden button

    #----------------------------------
    # Initializer 
    #--------------
        
    def __init__(self, parent=None, mirrored=True, stand_alone=False, sound_effect_labels=None):
        
        super(SpeakEasyGUI, self).__init__(parent);
        
        self.stand_alone = stand_alone;
        self.sound_effect_labels = sound_effect_labels;
        if (sound_effect_labels is None):
            raise ValueError("Must pass in non-null array of sound effect button labels.");
        
        #self.setMaximumWidth(1360);
        #self.setMaximumHeight(760);
        
            # Vertical box to hold all top level widget groups
        appLayout = QVBoxLayout();
        appWidget = QDialog();      
        appWidget.setStyleSheet(SpeakEasyGUI.defaultStylesheet);
        # Activate the window resize handle in the lower right corner
        # of the app window:
        appWidget.setSizeGripEnabled(True);
        
        self.setCentralWidget(appWidget);

        self.addTitle(appLayout);        
        self.addTxtInputFld(appLayout);
        self.buildTapeRecorderButtons(appLayout);
        self.addOnceOrRepeat_And_VoiceRadioButtons(appLayout);
        
        self.buildHorizontalDivider(appLayout);
        self.buildProgramButtons(appLayout);
        self.buildSoundButtons(appLayout);
        self.buildButtonSetControls(appLayout);
        self.buildOptionsRadioButtons(appLayout);
        
        appWidget.setLayout(appLayout);
    
        #*******self.connectEvents();    
        self.show();

    #----------------------------------
    # programButtonIterator 
    #--------------

    def programButtonIterator(self, gridLayout=None):
        
        if gridLayout is None:
            gridLayout = self.programButtonGridLayout;
        
        programButtonArray = [];
        # Collect the buttons into a flat array:
        for row in range(gridLayout.rowCount()):
            for col in range(SpeakEasyGUI.NUM_OF_PROGRAM_BUTTON_COLUMNS):
                layoutItem = gridLayout.itemAtPosition(row, col);
                if layoutItem is not None:
                    programButtonArray.append(layoutItem.widget());
        return iter(programButtonArray);

    #----------------------------------
    # replaceProgramButtons
    #--------------

    def replaceProgramButtons(self, buttonProgramsArray):
        
        programButtonObjIt = self.programButtonIterator();
        # Remove the existing program buttons from the application's
        # layout, and mark them for deletion:
        try:
            while True:
                programButtonObj = programButtonObjIt.next();
                self.programButtonGridLayout.removeWidget(programButtonObj);
                programButtonObj.deleteLater();
        except StopIteration:
            pass
        
        # Make an array of button labels from the ButtonProgram
        # instances in buttonProgramsArray:
        buttonLabelArr = [];
        for buttonProgram in buttonProgramsArray:
            buttonLabelArr.append(buttonProgram.getLabel());
            
        # No more program buttons present. Make 
        # new ones, and add them to the grid layout:
        (newProgramButtonGridLayout, self.programButtonDict) =\
            SpeakEasyGUI.buildButtonGrid(buttonLabelArr,
                                         SpeakEasyGUI.NUM_OF_PROGRAM_BUTTON_COLUMNS);
        for buttonObj in self.programButtonDict.values():
            buttonObj.setStyleSheet(SpeakEasyGUI.programButtonStylesheet);
            buttonObj.setMinimumHeight(SpeakEasyGUI.BUTTON_MIN_HEIGHT);

        # Transfer the buttons from this new gridlayout object to the
        # original layout that is embedded in the application Dialog:
        newButtonsIt = self.programButtonIterator(gridLayout=newProgramButtonGridLayout);
        try:
            while True:
                self.programButtonGridLayout.addWidget(newButtonsIt.next());
        except StopIteration:
            pass;
    
    #----------------------------------
    # connectEvents
    #--------------
    
    def connectEvents(self):
        mousePressEvent.connect(self.textAreaMouseClick);
        
    def textAreaMouseClick(self, event):
        print('Text area clicked.')
    
    
    #----------------------------------
    # connectSignalsToWidgets 
    #--------------
    
#    def connectSignalsToWidgets(self):
#        self.commChannel = CommChannel();
#        self.commChannel.hideButtonSignal.connect(SpeakEasyGUI.hideButtonHandler);
#        self.commChannel.showButtonSignal.connect(SpeakEasyGUI.showButtonHandler);


    #----------------------------------
    # addTitle 
    #--------------

    def addTitle(self,layout):
        title = QLabel("<H1>SpeakEasy</H1>");
        hbox = QHBoxLayout();
        hbox.addStretch(1);
        hbox.addWidget(title);
        hbox.addStretch(1);
        
        layout.addLayout(hbox);
        
        
    #----------------------------------
    # addTxtInputFld 
    #--------------
        
    def addTxtInputFld(self, layout):
        '''
        Creates text input field label and text field
        in a horizontal box layout. Adds that hbox layout
        to the passed-in layout.
        
        Sets instance variables:
            1. C{self.speechInputFld}
         
        @param layout: Layout object to which the label/txt-field C{hbox} is to be added.
        @type  layout: QLayout
        '''
        speechControlsLayout = QHBoxLayout();
        speechControlsLayout.addStretch(1);
        speechInputFldLabel = QLabel("<b>What to say:</b>")
        speechControlsLayout.addWidget(speechInputFldLabel);
        
        self.speechInputFld = TextPanel(numLines=5);
        self.speechInputFld.setStyleSheet(SpeakEasyGUI.inputFldStylesheet);
        self.speechInputFld.setFontPointSize(SpeakEasyGUI.EDIT_FIELD_TEXT_SIZE);
        
        speechControlsLayout.addWidget(self.speechInputFld);
        
        layout.addLayout(speechControlsLayout);
        
        # Create and hide the dialog for adding Cepstral voice modulation 
        # markup to text in the text input field:
        self.speechControls = MarkupManagementUI(textPanel=self.speechInputFld, parent=self);

    #----------------------------------
    # addVoiceRadioButtons
    #--------------
            
    def addOnceOrRepeat_And_VoiceRadioButtons(self, layout):
        '''
        Creates radio buttons for selecting whether a
        sound is to play once, or repeatedly until stopped.
        Also adds radio buttons for selecting voices.
        Places all in a horizontal box layout. Adds 
        that hbox layout to the passed-in layout.
        
        Sets instance variables:
            1. C{self.onceOrRepeatDict}
            2. C{self.voicesRadioButtonsDict}
         
        @param layout: Layout object to which the label/txt-field C{hbox} is to be added.
        @type  layout: QLayout
         '''
        
        hbox = QHBoxLayout();

        (self.onceOrRepeatGroup, onceOrRepeatButtonLayout, self.onceOrRepeatDict) =\
            self.buildRadioButtons([SpeakEasyGUI.interactionWidgets['PLAY_ONCE'], 
                                    SpeakEasyGUI.interactionWidgets['PLAY_REPEATEDLY']
                                    ],
                                   Orientation.HORIZONTAL,
                                   Alignment.LEFT,
                                   activeButtons=[SpeakEasyGUI.interactionWidgets['PLAY_ONCE']],
                                   behavior=CheckboxGroupBehavior.RADIO_BUTTONS);

        self.replayPeriodSpinBox = QDoubleSpinBox(self);
        self.replayPeriodSpinBox.setRange(0.0, 99.9); # seconds
        self.replayPeriodSpinBox.setSingleStep(0.5);
        self.replayPeriodSpinBox.setDecimals(1);
        onceOrRepeatButtonLayout.addWidget(self.replayPeriodSpinBox);
        secondsLabel = QLabel("secs delay");
        onceOrRepeatButtonLayout.addWidget(secondsLabel);
        
        # Create an array of voice radio button labels:
        voiceRadioButtonLabels = [];
        for voiceKey in SpeakEasyGUI.voices.keys():
            voiceRadioButtonLabels.append(SpeakEasyGUI.interactionWidgets[voiceKey]);
            
        (self.voicesGroup, voicesButtonLayout, self.voicesRadioButtonsDict) =\
            self.buildRadioButtons(voiceRadioButtonLabels,
                                   Orientation.HORIZONTAL,
                                   Alignment.RIGHT,
                                   activeButtons=[SpeakEasyGUI.interactionWidgets['VOICE_1']],
                                   behavior=CheckboxGroupBehavior.RADIO_BUTTONS);
                                   
        # Style all the radio buttons:
        for playFreqButton in self.onceOrRepeatDict.values():
            playFreqButton.setStyleSheet(SpeakEasyGUI.playOnceRepeatButtonStylesheet); 
        for playFreqButton in self.voicesRadioButtonsDict.values():
            playFreqButton.setStyleSheet(SpeakEasyGUI.voiceButtonStylesheet);
        #...and the replay delay spinbox:
        self.replayPeriodSpinBox.setStyleSheet(SpeakEasyGUI.playRepeatSpinboxStylesheet);
        #****** replayPeriodSpinBox styling 
                                   
        hbox.addLayout(onceOrRepeatButtonLayout);
        hbox.addStretch(1);
        hbox.addLayout(voicesButtonLayout);
        layout.addLayout(hbox);
        
    #----------------------------------
    # buildTapeRecorderButtons 
    #--------------
        
    def buildTapeRecorderButtons(self, layout):
        '''
        Creates tape recorder buttons (Play Text, Stop,etc.).
        Places all in a row, though the layout is a 
        QGridLayout. Adds QGridLayout to the passed-in 
        layout.
        
        Sets instance variables:
            1. C{self.recorderDict}
         
        @param layout: Layout object to which the label/txt-field C{QGridlayout} is to be added.
        @type  layout: QLayout
        '''
        
        (buttonGridLayout, self.recorderButtonDict) =\
            SpeakEasyGUI.buildButtonGrid([SpeakEasyGUI.interactionWidgets['PLAY_TEXT'],
                                          SpeakEasyGUI.interactionWidgets['STOP'],
                                          #SpeakEasyGUI.interactionWidgets['STOP_ALL'] # Stop button already stops all.
                                          ],
                                         2); # Two columns
        for buttonObj in self.recorderButtonDict.values():
            buttonObj.setStyleSheet(SpeakEasyGUI.recorderButtonStylesheet);
            buttonObj.setMinimumHeight(SpeakEasyGUI.BUTTON_MIN_HEIGHT);
        layout.addLayout(buttonGridLayout);                                 
         
    #----------------------------------
    # buildProgramButtons
    #--------------
        
    def buildProgramButtons(self, layout):
        '''
        Creates grid of buttons for saving sounds.
        Adds the resulting QGridLayout to the passed-in 
        layout.
        
        Sets instance variables:
            1. C{self.programButtonDict}
         
        @param layout: Layout object to which the label/txt-field C{QGridlayout} is to be added.
        @type  layout: QLayout
        '''

        buttonLabelArr = [];
        for i in range(NUM_SPEECH_PROGRAM_BUTTONS):
            key = "SPEECH_" + str(i+1);
            buttonLabelArr.append(SpeakEasyGUI.interactionWidgets[key]);
                    
        (self.programButtonGridLayout, self.programButtonDict) =\
            SpeakEasyGUI.buildButtonGrid(buttonLabelArr,
                                         SpeakEasyGUI.NUM_OF_PROGRAM_BUTTON_COLUMNS);
        for buttonObj in self.programButtonDict.values():
            buttonObj.setStyleSheet(SpeakEasyGUI.programButtonStylesheet);
            buttonObj.setMinimumHeight(SpeakEasyGUI.BUTTON_MIN_HEIGHT);
            
        layout.addLayout(self.programButtonGridLayout);                                 
        
    #----------------------------------
    # buildSoundButtons
    #--------------
        
    def buildSoundButtons(self, layout):
        '''
        Creates grid of buttons for playing canned sounds.
        Adds the resulting QGridLayout to the passed-in 
        layout.
        
        Sets instance variables:
            1. C{self.soundButtonDict}
         
        @param layout: Layout object to which the label/txt-field C{QGridlayout} is to be added.
        @type  layout: QLayout
        '''
        
        for i in range(len(self.sound_effect_labels)):
            key = "SOUND_" + str(i);
            SpeakEasyGUI.interactionWidgets[key] = self.sound_effect_labels[i];
        
        (buttonGridLayout, self.soundButtonDict) =\
            SpeakEasyGUI.buildButtonGrid(self.sound_effect_labels, SpeakEasyGUI.NUM_OF_SOUND_BUTTON_COLUMNS);
        for buttonObj in self.soundButtonDict.values():
            buttonObj.setStyleSheet(SpeakEasyGUI.soundButtonStylesheet);
            buttonObj.setMinimumHeight(SpeakEasyGUI.BUTTON_MIN_HEIGHT);
                        
        layout.addLayout(buttonGridLayout);                                 

    #----------------------------------
    # buildButtonSetControls
    #--------------

    def buildButtonSetControls(self, layout):
        
        buttonLabelArray = [self.interactionWidgets['NEW_SPEECH_SET'], self.interactionWidgets['PICK_SPEECH_SET']]; 
        # Two columns of buttons:
        (buttonGridLayout, self.speechSetButtonDict) = SpeakEasyGUI.buildButtonGrid(buttonLabelArray, 2);
        for buttonObj in self.speechSetButtonDict.values():
            buttonObj.setStyleSheet(SpeakEasyGUI.programButtonStylesheet);
            buttonObj.setMinimumHeight(SpeakEasyGUI.BUTTON_MIN_HEIGHT);
        layout.addLayout(buttonGridLayout);
        
    #----------------------------------
    # buildOptionsRadioButtons 
    #--------------
        
    def buildOptionsRadioButtons(self, layout):
        hbox = QHBoxLayout();
        (self.playLocalityGroup, playLocalityButtonLayout, self.playLocalityRadioButtonsDict) =\
            self.buildRadioButtons([SpeakEasyGUI.interactionWidgets['PLAY_LOCALLY'],
                                    SpeakEasyGUI.interactionWidgets['PLAY_AT_ROBOT']
                                    ],
                                   Orientation.HORIZONTAL,
                                   Alignment.LEFT,
                                   activeButtons=[SpeakEasyGUI.interactionWidgets[DEFAULT_PLAY_LOCATION]],
                                   behavior=CheckboxGroupBehavior.RADIO_BUTTONS);
                                   #behavior=CheckboxGroupBehavior.CHECKBOXES);
                                   
        # Style all the radio buttons:
        for playLocalityButton in self.playLocalityRadioButtonsDict.values():
            playLocalityButton.setStyleSheet(SpeakEasyGUI.voiceButtonStylesheet); 
        hbox.addLayout(playLocalityButtonLayout);
        hbox.addStretch(1);
        self.buildConvenienceButtons(hbox);
        layout.addLayout(hbox);

    #----------------------------------
    # buildConvenienceButtons
    #--------------
        
    def buildConvenienceButtons(self, layout):
        '''
        Creates buttons meant for accessibility convenience.
        Example: Paste.
        Places all in a row, though the layout is a 
        QGridLayout. Adds QGridLayout to the passed-in 
        layout.
        
        Sets instance variables:
            1. C{self.convenienceButtonDict}
         
        @param layout: Layout object to which the label/txt-field C{QGridlayout} is to be added.
        @type  layout: QLayout
        '''
        
        (buttonGridLayout, self.convenienceButtonDict) =\
            SpeakEasyGUI.buildButtonGrid([SpeakEasyGUI.interactionWidgets['SPEECH_MODULATION'],
                                          SpeakEasyGUI.interactionWidgets['PASTE'],
                                          SpeakEasyGUI.interactionWidgets['CLEAR'],
                                          ],
                                         2); # Two columns
        for buttonObj in self.convenienceButtonDict.values():
            buttonObj.setStyleSheet(SpeakEasyGUI.recorderButtonStylesheet);
            buttonObj.setMinimumHeight(SpeakEasyGUI.BUTTON_MIN_HEIGHT);
        layout.addLayout(buttonGridLayout);                                 
        
    #----------------------------------
    # buildHorizontalDivider 
    #--------------
        
    def buildHorizontalDivider(self, layout):
        frame = QFrame();
        frame.setFrameShape(QFrame.HLine);
        #*******frame.setFrameStyle(QFrame.Shadow.Sunken.value());
        frame.setLineWidth(3);
        frame.setMidLineWidth(3);
        layout.addWidget(frame);
        
        
    #----------------------------------
    # buildRadioButtons
    #--------------
        
    # Note: Whenever a button is switched on or off it emits the 
    # PySide.QtGui.QAbstractButton.toggled() signal. Connect to this 
    # signal if you want to trigger an action each time the button changes state.         
    
    def buildRadioButtons(self, 
                          labelTextArray,
                          orientation,
                          alignment,
                          activeButtons=None, 
                          behavior=CheckboxGroupBehavior.RADIO_BUTTONS):
        '''
        @param labelTextArray: Names of buttons
        @type labelTextArray: [string]
        @param orientation: Whether to arrange the buttons vertically or horizontally.
        @type orientation: Orientation
        @param alignment: whether buttons should be aligned Left/Center/Right for horizontal,
                          Top/Center/Bottom for vertical:/c
        @type  alignment: Alignment
        @param activeButtons: Name of the buttons that is to be checked initially. Or None.
        @type activeButtons: [string]
        @param behavior: Indicates whether the button group is to behave like Radio Buttons, or like Checkboxes.
        @type behavior: CheckboxGroupBehavior
        @return 
            1. The button group that contains the related buttons. Caller: ensure that 
               this object does not go out of scope. 
            2. The button layout, which callers will need to add to
               their own layouts.
            3. and a dictionary mapping button names to button objects that
               were created within this method. This dict is needed by the
               controller.
        @rtype (QButtonGroup, QLayout, dict<string,QRadioButton>).
        '''
        
        # Button control management group. It has no visible
        # representation. It allows the radio button bahavior, for instance:
        buttonGroup = QButtonGroup();
        
        if behavior == CheckboxGroupBehavior.CHECKBOXES:
            buttonGroup.setExclusive(False);
        else:
            buttonGroup.setExclusive(True);
        
        if orientation == Orientation.VERTICAL:
            buttonCompLayout = QVBoxLayout();
            # Arrange for the alignment (Part 1):
            if (alignment == Alignment.CENTER) or\
               (alignment == Alignment.BOTTOM):
                buttonCompLayout.addStretch(1);
        else:
            buttonCompLayout = QHBoxLayout();
            # Arrange for the alignment (Part 1):
            if (alignment == Alignment.CENTER) or\
               (alignment == Alignment.RIGHT):
                buttonCompLayout.addStretch(1);
            
        # Build the buttons:
        buttonDict  = {};
        
        for label in labelTextArray:
            button = QRadioButton(label);
            buttonDict[label] = button;
            # Add button to the button management group:
            buttonGroup.addButton(button);
            # Add the button to the visual group:
            buttonCompLayout.addWidget(button);
            if label in activeButtons:
                button.setChecked(True);

        if orientation == Orientation.VERTICAL:
            buttonCompLayout = QVBoxLayout();
            # Arrange for the alignment (Part 2):
            if (alignment == Alignment.CENTER) or\
               (alignment == Alignment.TOP):
                buttonCompLayout.addStretch(1);
        else:
            # Arrange for the alignment (Part 2):
            if (alignment == Alignment.CENTER) or\
               (alignment == Alignment.LEFT):
                buttonCompLayout.addStretch(1);
                        
        return (buttonGroup, buttonCompLayout, buttonDict);
    
    #----------------------------------
    # buildButtonGrid
    #--------------

    @staticmethod
    def buildButtonGrid(labelTextArray, 
                        numColumns):
        '''
        Creates a grid of QPushButton widgets. They will be 
        
        @param labelTextArray: Button labels.
        @type labelTextArray: [string]
        @param numColumns: The desired width of the button grid.
        @type numColumns: int
        @return: 1. a grid layout with the button objects inside.
                 2. a dictionary mapping button labels to button objects.
        @rtype: QGridLayout 
        '''

        buttonLayout = QGridLayout();
        
        # Compute number of button rows:
        numRows = len(labelTextArray) / numColumns;
        # If this number of rows leaves a few buttons left over (< columnNum),
        # then add a row for those:
        if (len(labelTextArray) % numColumns) > 0:
            numRows += 1;
        
        buttonDict = {}
        
        row = 0;
        col = 0;
        for buttonLabel in labelTextArray:
            button = QPushButton(buttonLabel);
            buttonDict[buttonLabel] = button;
            buttonLayout.addWidget(button, row, col);
            col += 1;
            if (col > (numColumns - 1)):
                col = 0;
                row += 1;
                
        return (buttonLayout, buttonDict);
        
    #----------------------------------
    # getNewButtonLabel 
    #--------------

    def getNewButtonLabel(self):
        '''
        Requests a new button label from the user.
        Returns None if user canceled out, or a string
        with the new button label. 
        @return: None if user canceled, else string from input field.
        @rtype: {None | string}
        
        '''
        prompt = "New label for this button:";
        dialogBox = QInputDialog(parent=self);
        dialogBox.setStyleSheet(SpeakEasyGUI.stylesheetAppBG);
        dialogBox.setLabelText(prompt);
        dialogBox.setCancelButtonText("Keep existing label");
        userChoice = dialogBox.exec_();

        if userChoice == QDialog.Accepted:
            return dialogBox.textValue();
        else:
            return None;
        
    #----------------------------------
    # playOnceChecked 
    #--------------

    def playOnceChecked(self):
        return self.onceOrRepeatDict[SpeakEasyGUI.interactionWidgets['PLAY_ONCE']].isChecked();

    #----------------------------------
    # setPlayOnceChecked 
    #--------------
    
    def setPlayOnceChecked(self):
        self.onceOrRepeatDict[SpeakEasyGUI.interactionWidgets['PLAY_ONCE']].setChecked(True);
    
    #----------------------------------
    # playRepeatedlyChecked 
    #--------------

    def playRepeatedlyChecked(self):
        return self.onceOrRepeatDict[SpeakEasyGUI.interactionWidgets['PLAY_REPEATEDLY']].isChecked();

    #----------------------------------
    # setPlayRepeatedlyChecked 
    #--------------
    
    def setPlayRepeatedlyChecked(self):
        self.onceOrRepeatDict[SpeakEasyGUI.interactionWidgets['PLAY_REPEATEDLY']].setChecked(True);
        
    #----------------------------------
    # playRepeatPeriod 
    #--------------

    def getPlayRepeatPeriod(self):
        return self.replayPeriodSpinBox.value();
    
    #----------------------------------
    # activeVoice
    #--------------

    def activeVoice(self):
        '''
        Return the official name of the voice that is currently
        checked in the UI. This is the name that will be recognized
        by the underlying text-to-speech engine(s).
        @return: Name of voice as per the SpeakEasyGUI.voices dict.
        @rtype: string
        '''
        for voiceKey in SpeakEasyGUI.voices.keys():
            if self.voicesRadioButtonsDict[SpeakEasyGUI.interactionWidgets[voiceKey]].isChecked():
                return SpeakEasyGUI.voices[voiceKey];

    #----------------------------------
    # whereToPlay 
    #--------------
    
    def whereToPlay(self):
        '''
        Returns which of the play location options is selected: Locally or Robot.
        @return: Selection of where sound and text-to-speech output is to occur.
        @rtype: PlayLocation
        '''
        if self.playLocalityRadioButtonsDict[SpeakEasyGUI.interactionWidgets['PLAY_LOCALLY']].isChecked():
            return PlayLocation.LOCALLY;
        else:
            return PlayLocation.ROBOT;
    
    #----------------------------------
    # setWhereToPlay 
    #--------------
    
    def setWhereToPlay(self, playLocation):
        '''
        Set the option radio button that determines where sound is produced,
        locally, or at the robot. No action is taken. This method merely sets
        the appropriate radio button.
        @param playLocation: PlayLocation.LOCALLY, or PlayLocation.ROBOT
        @type playLocation: PlayLocation
        '''
        
        if playLocation == PlayLocation.LOCALLY:
            radioButton = self.playLocalityRadioButtonsDict[SpeakEasyGUI.interactionWidgets['PLAY_LOCALLY']];
        else:
            radioButton = self.playLocalityRadioButtonsDict[SpeakEasyGUI.interactionWidgets['PLAY_AT_ROBOT']];
        radioButton.setChecked(True);
            
    #----------------------------------
    # setButtonLabel 
    #--------------
    
    def setButtonLabel(self, buttonObj, label):
        buttonObj.setText(label);

    #----------------------------------
    # blinkButton
    #--------------

    def blinkButton(self, buttonObj, turnOn):
        '''
        Used to make a program button blink in some way to indicate
        that it is changing into programming mode. Since this method
        is triggered by a timer thread, it cannot make any GUI changes.
        Instead, it sends a signal to have the GUI thread place the
        button into an alternative look. It then schedules a call
        to itself for a short time later. At that point it sends a
        signal to the GUI thread to return the button to its usual
        look:
        @param buttonObj: The button to be blinked
        @type  buttonObj: QPushButton
        @param turnOn:    Indicates whether to turn the button back into 
                          its normal state, or whether to make it take on
                          its alternate look. 
        @type  turnOn:    bool
        '''
        if turnOn:
            self.showButtonSignal.emit(buttonObj);
        else:
            self.hideButtonSignal.emit(buttonObj);
            self.buttonBlinkTimer = Timer(SpeakEasyGUI.PROGRAM_BUTTON_LOOK_CHANGE_DURATION, partial(self.blinkButton, buttonObj, True));
            self.buttonBlinkTimer.start();
    
    # ---------------------   Manage Sound Effect Buttons -------------------
          

def alternateLookHandler(buttonObj):
    #buttonObj.hide();
    buttonObj.setStyleSheet(SpeakEasyGUI.programButtonModeTransitionStylesheet);
    
def standardLookHandler(buttonObj):
    #buttonObj.show();
    buttonObj.setStyleSheet(SpeakEasyGUI.programButtonStylesheet);

if __name__ == "__main__":

    from PyQt4.QtGui import QStyleFactory, QApplication;
    
    # Create a Qt application
    
#    style = QStyleFactory.create("Plastique");
#    style = QStyleFactory.create("Motif");
#    style = QStyleFactory.create("GTK+");
#    style = QStyleFactory.create("Windows");

    style = QStyleFactory.create("Cleanlooks");
    QApplication.setStyle(style);
    app = QApplication(sys.argv);
        
    soundPlayGui = SpeakEasyGUI();
    soundPlayGui.hideButtonSignal.connect(alternateLookHandler);
    soundPlayGui.showButtonSignal.connect(standardLookHandler);
    
    # Enter Qt application main loop
    sys.exit(app.exec_());
        
        
