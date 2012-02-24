#!/usr/bin/python

import roslib; roslib.load_manifest('speakeasy');
import rospy

import sys
import os
import time
from functools import partial;
from threading import Timer;

from PySide.QtCore import * #@UnusedWildImport
from PySide.QtGui import * #@UnusedWildImport

from speakeasy_ui import SpeakEasyGUI;
from speakeasy_ui import DialogService;

from speakeasy.libspeakeasy import SoundClient;
from speakeasy.msg import SpeakEasyRequest;

from speakeasy_ui import alternateLookHandler;
from speakeasy_ui import standardLookHandler;


# ----------------------------------------------- Class Program ------------------------------------

class ButtonProgram(object):
    
    #----------------------------------
    # Initializer 
    #--------------
    
    def __init__(self, buttonObj, textToSave, voice, ttsEngine, playOnce=True):
        '''
        Create an object that holds the playback parameters for a 
        programmed button.
        @param buttonObj: The QPushButton instance of the button being programmed.
        @type  buttonObj: QPushButton
        @param textToSave: Text to play back with this button.
        @type  textToSave: string
        @param voice: The voice to use for the utterance
        @type  voice: string
        @param ttsEngine: The text-to-speech engine to use. (e.g. "festival", or "cepstral"
        @type string
        @param playOnce: Whether to play the utterance just once, or several times.
        @type  bool
        '''
        self.buttonLabel = buttonObj.text();
        self.textToSay   = textToSave;
        self.activeVoice = voice;
        self.ttsEngine   = ttsEngine;
        self.playOnce    = playOnce;
        
    #----------------------------------
    # getText 
    #--------------
        
        
    def getText(self):
        return self.textToSay;

# ----------------------------------------------- Class SpeakEasyController ------------------------------------

class SpeakEasyController(object):
    '''
    Control logic behind the speakeasy GUI. Relies on 
    libspeakeasy.py. Primitives in that library are as follows.

	- C{voiceSound(str)}: create voice C{Sound} instance
	- C{waveSound(abspath)}: create wave C{Sound} instance
	- C{builtInSound(soundID)}: create built-in C{Sound} instance. See below.
    
	- C{say(txt, voice=<voice-id-str>)}: say txt string with given voice. Default is male
	- C{repeat(txt)}: Say str over and over till stopSay(txt), or stopAll();
	- C{stopSaying(txt)}: stop repeating txt (which was started with C{ repeat(txt)}
	- C{playWave(abspath)}: play a .wav file, given its absolute path.
	- C{startWave(abspath)} play a .wav file repeatedly, given its absolute path.
	- C{stopWave(abspath)} stop repeatedly playing .wav file
	- C{play(soundID)} play one of the built-in sounds (see below for C{soundID}.)
	- C{start(soundID)} play built-in sound repeatedly
	- C{stop(soundID)} stop playing built-in sound repeatedly
	- C{stopAll()} stop all voice, .wav, and built-in sounds
	
	Available voices:
	Festival: 1.  Usually voice_kal_diphone (male) on Ubuntu installations
	Cepstral: Depends on your installation.       
            
    Available built-in, hardwired sounds (legacy from original sound_play_node.py):
	- C{msg.SpeakEasyRequest.BACKINGUP}
	- C{msg.SpeakEasyRequest.NEEDS_UNPLUGGING}
	- C{msg.SpeakEasyRequest.NEEDS_PLUGGING}
	- C{msg.SpeakEasyRequest.NEEDS_UNPLUGGING_BADLY}
	- C{msg.SpeakEasyRequest.NEEDS_PLUGGING_BADLY}
    '''
    
    soundPaths = {
                  'SOUND_1' : 'rooster.wav',
                  'SOUND_2':  'drill.wav',
                  'SOUND_3' : 'bullCall.wav',
                  'SOUND_4':  'clown_horn.wav',
                  'SOUND_5':  'cashreg.wav',
                  'SOUND_6':  'plateglass.wav',
                  'SOUND_7':  'old_cell.wav',
                  'SOUND_8':  'moo2.wav',
                  'SOUND_9':  'abirdm.wav',
                  'SOUND_10': 'bigtruck.wav', 
                  'SOUND_11': 'seagulls_shore.wav', 
                  'SOUND_12': 'hydrau_lift.wav' 
                  }
    
    def __init__(self, dirpath):
        
        self.gui = SpeakEasyGUI();
        # Handler that makes program button temporarily
        # look different to indicate entry into program mode:
        self.gui.hideButtonSignal.connect(alternateLookHandler);
        # Handler that makes program button look normal:
        self.gui.showButtonSignal.connect(standardLookHandler);
        
        self.dialogService = DialogService(self.gui);
        self.soundClient = SoundClient();
        self.dirpath = dirpath;
        
        # Allow multiple GUIs to run simultaneously. Therefore
        # the anonymous=True:
        rospy.init_node('speakeasy_gui', anonymous=True);
        # No program button is being held down:
        self.programButtonPushedTime = None;
        # No speech buttons programmed yet:
        self.programs = {};
        self.connectWidgetsToActions()

    #----------------------------------
    # connectWidgetsToActions 
    #--------------
    
    def connectWidgetsToActions(self):
        self.gui.speechInputFld
        for recorderButton in self.gui.recorderButtonDict.values():
            recorderButton.clicked.connect(partial(self.actionRecorderButtons, recorderButton));
        for programButton in self.gui.programButtonDict.values():
            programButton.pressed.connect(partial(self.actionProgramButtons, programButton));
            programButton.released.connect(partial(self.actionProgramButtonRelease, programButton));
        for soundButton in self.gui.soundButtonDict.values():
            soundButton.clicked.connect(partial(self.actionSoundButtons, soundButton));
    
    #----------------------------------
    # sayText 
    #--------------
    
    def sayText(self, text, voice, ttsEngine="festival", sayOnce=True):
        '''
        Send message to SpeakEasy service to say text, with the
        given voice, using the given text-to-speech engine.
        @param text: Text to be uttered by the tts engine
        @type  string
        @param voice: Name of speaking voice to be used.
        @type string
        @param ttsEngine: Name of tts engine to use (e.g. "festival" (the default), "cepstral"
        @type string
        @param sayOnce: Whether repeat the utterance over and over, or just say it once.
        @type bool
        '''
        # Repeat over and over? Or say once?
        if sayOnce:
            self.soundClient.say(text, voice=voice, ttsEngine=ttsEngine);
        else:
            self.soundClient.repeat(text, voice=voice, ttsEngine=ttsEngine);
        return;
    
    #----------------------------------
    # actionRecorderButtons
    #--------------
            
    def actionRecorderButtons(self, buttonObj):
        '''
        Handler for one of the recorder buttons pushed.
        this handler, because it is fast. However, 
        @param buttonObj: The button object that was pushed.
        @type  buttonObj: QPushButton
        '''
        
        # Play button pushed?
        buttonKey = self.gui.interactionWidgets['PLAY_TEXT'];
        if buttonObj == self.gui.recorderButtonDict[buttonKey]:
            # If nothing in text input field, error msg, and done:
            if self.gui.speechInputFld.isEmpty():
                self.dialogService.showErrorMsg("Nothing to play; enter text in the text field.");
                return;
            
            # Got text in input fld. Which of the voices is checked?
            voice = self.gui.activeVoice();
            if (voice == "Male"):
                voice = "voice_kal_diphone"
                ttsEngine = "festival"
            # TODO: **************** Read from radio buttons!
            elif voice == "David":
                ttsEngine = "cepstral"
            else:
                raise ValueError("Unknown voice: " + str(voice));
            # ****** end TODO
            self.sayText(self.gui.speechInputFld.getText(), voice, ttsEngine, self.gui.playOnceChecked());
            return;
        
        # Stop button pushed?
        buttonKey = self.gui.interactionWidgets['STOP'];
        if buttonObj == self.gui.recorderButtonDict[buttonKey]:
            # If nothing in text input field, then we are not playing text:
            if self.gui.speechInputFld.isEmpty():
                self.dialogService.showErrorMsg("No text is playing.");
                return;
            self.soundClient.stopSaying(self.gui.speechInputFld.getText());
            return;
        
        # Stop All button pushed?
        buttonKey = self.gui.interactionWidgets['STOP_ALL'];
        self.soundClient.stopAll();


    #----------------------------------
    # actionProgramButtons 
    #--------------

    def actionProgramButtons(self, buttonObj):      
        # Record press-down time:
        self.programButtonPushedTime = time.time(); # fractional seconds till beginning of epoch 
        # Schedule the button to blink when the programming mode hold delay is over:
        self.buttonBlinkTimer = Timer(SpeakEasyGUI.PROGRAM_BUTTON_HOLD_TIME, partial(self.gui.blinkButton, buttonObj, False));
        self.buttonBlinkTimer.start();

    #----------------------------------
    # actionProgramButtonRelease 
    #--------------
        
    def actionProgramButtonRelease(self, buttonObj):
        timeNow = time.time(); # fractional seconds till beginning of epoch
        self.buttonBlinkTimer.cancel();
        # Sometimes the down press seems to get missed, and then
        # self.programButtonPushedTime is None. Likely that happens
        # when buttons are clicked quickly:
        if self.programButtonPushedTime is None:
            self.programButtonPushedTime = timeNow;
        holdTime = timeNow - self.programButtonPushedTime;
        # Button no longer held down:
        self.programButtonPushedTime = None;
        
        # Held long enough to indicate intention to program?:
        if holdTime >= SpeakEasyGUI.PROGRAM_BUTTON_HOLD_TIME:
            self.programOneButton(buttonObj);
        else:
            self.playProgram(buttonObj);
            
    #----------------------------------
    # programOneButton 
    #--------------

    def programOneButton(self, buttonObj):
        
        if self.gui.speechInputFld.isEmpty():
            self.dialogService.showErrorMsg("You need to enter text in the input panel to program a button.");
            return;
        
        newButtonLabel = self.gui.getNewButtonLabel();
        if newButtonLabel is not None:
            self.gui.setButtonLabel(buttonObj,newButtonLabel);
        
        textToSave = self.gui.speechInputFld.getText();
        #******** TODO!!!!
        if self.gui.activeVoice() == "Male":
            ttsEngine = "festival"
        elif self.gui.activeVoice() == "David":
            ttsEngine = "cepstral"
        #******** End TODO
        programObj = ButtonProgram(buttonObj, textToSave, self.gui.activeVoice(), ttsEngine, self.gui.playOnceChecked());
        
        self.programs[buttonObj] = programObj; 
        
    #----------------------------------
    # playProgram 
    #--------------

    def playProgram(self, buttonObj):
        
        program = None;
        try:    
            program = self.programs[buttonObj];
        except KeyError:
            self.dialogService.showErrorMsg("This button does not contain a program. Press-and-hold for " +\
                                            str(int(SpeakEasyGUI.PROGRAM_BUTTON_HOLD_TIME)) +\
                                            " seconds to program.");
            return;
    
    
        onlyPlayOnce = program.playOnce;
        voice        = program.activeVoice;
        ttsEngine    = program.ttsEngine;
        self.sayText(program.getText(), voice, ttsEngine, onlyPlayOnce);
             
    #----------------------------------
    # actionSoundButtons 
    #--------------
        
    def actionSoundButtons(self, buttonObj):
        if buttonObj ==  self.gui.soundButtonDict[self.gui.interactionWidgets['SOUND_1']]:
            soundFile = SpeakEasyController.soundPaths['SOUND_1'];
        elif buttonObj ==  self.gui.soundButtonDict[self.gui.interactionWidgets['SOUND_2']]:
            soundFile = SpeakEasyController.soundPaths['SOUND_2'];
        elif buttonObj ==  self.gui.soundButtonDict[self.gui.interactionWidgets['SOUND_3']]:
            soundFile = SpeakEasyController.soundPaths['SOUND_3'];
        elif buttonObj ==  self.gui.soundButtonDict[self.gui.interactionWidgets['SOUND_4']]:
            soundFile = SpeakEasyController.soundPaths['SOUND_4'];
        elif buttonObj ==  self.gui.soundButtonDict[self.gui.interactionWidgets['SOUND_5']]:
            soundFile = SpeakEasyController.soundPaths['SOUND_5'];
        elif buttonObj ==  self.gui.soundButtonDict[self.gui.interactionWidgets['SOUND_6']]:
            soundFile = SpeakEasyController.soundPaths['SOUND_6'];
        elif buttonObj ==  self.gui.soundButtonDict[self.gui.interactionWidgets['SOUND_7']]:
            soundFile = SpeakEasyController.soundPaths['SOUND_7'];
        elif buttonObj ==  self.gui.soundButtonDict[self.gui.interactionWidgets['SOUND_8']]:
            soundFile = SpeakEasyController.soundPaths['SOUND_8'];
        elif buttonObj ==  self.gui.soundButtonDict[self.gui.interactionWidgets['SOUND_9']]:
            soundFile = SpeakEasyController.soundPaths['SOUND_9'];
        elif buttonObj ==  self.gui.soundButtonDict[self.gui.interactionWidgets['SOUND_10']]:
            soundFile = SpeakEasyController.soundPaths['SOUND_10'];
        elif buttonObj ==  self.gui.soundButtonDict[self.gui.interactionWidgets['SOUND_11']]:
            soundFile = SpeakEasyController.soundPaths['SOUND_11'];
        elif buttonObj ==  self.gui.soundButtonDict[self.gui.interactionWidgets['SOUND_12']]:
            soundFile = SpeakEasyController.soundPaths['SOUND_12'];
        else:
            raise ValueError("Unknown widget passed to actionSoundButton() method: " + str(buttonObj));

    	self.soundClient.sendMsg(SpeakEasyRequest.PLAY_FILE,
	                             SpeakEasyRequest.PLAY_START,
				                 soundFile)
          
        
if __name__ == "__main__":

    app = QApplication(sys.argv);
        
    # To find the sounds, we need the absolute directory
    # path to this script:
    scriptDir = os.path.dirname(os.path.abspath(sys.argv[0]));
    speakeasyController = SpeakEasyController(scriptDir);
    # Enter Qt application main loop
    sys.exit(app.exec_());
        
        
