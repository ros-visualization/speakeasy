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


# TODO:
#   - Robot ops with new msg
#   - Switch live between local and remote ops. 

# Note: Unfortunately, this code was written before I knew about QtCreator. So 
#       all of the UI is created in code (see file speakeasy_ui.py).

## Try importing ROS related modules. Remember whether
## that worked. In the SpeakEasyController __init__() method
## we'll switch to local, and warn user if appropriate:
try:
    import roslib; roslib.load_manifest('speakeasy');
    import rospy
    ROS_IMPORT_OK = True;
except ImportError:
    print('ROS not installed on this machine; run locally, but PYTHONPATH must be set to include all SpeakEasy file locations.')
    ROS_IMPORT_OK = False;

import sys
import os
import signal
import socket
import time
import subprocess
import re
import shutil
import threading
from functools import partial;
from threading import Timer;

from python_qt_binding.QtGui import QApplication, QMessageBox, QPushButton, QAction;
from python_qt_binding.QtCore import Qt, QSocketNotifier, QTimer, Slot;

from utilities.speakeasy_utils import SpeakeasyUtils; 

from sound_player import SoundPlayer;
from text_to_speech import TextToSpeechProvider;

from robot_interaction import RoboComm;

from speakeasy_ui import SpeakEasyGUI;
from speakeasy_ui import DialogService;
from speakeasy_ui import PlayLocation;
from speakeasy_ui import DEFAULT_PLAY_LOCATION;

from speakeasy_ui import alternateLookHandler;
from speakeasy_ui import standardLookHandler;

from speakeasy.speakeasy_persistence import ButtonSavior;

from speakeasy.buttonSetPopupSelector_ui import ButtonSetPopupSelector;
from speakeasy import speakeasy_persistence;

from markupManagement import MarkupManagement;
from speakeasy.speakeasy_persistence import ButtonSavior

# ----------------------------------------------- Class ButtonProgram ------------------------------------

class ButtonProgram(object):
    
    #----------------------------------
    # Initializer 
    #--------------
    
    def __init__(self, buttonLabel, textToSave, voice, ttsEngine, playOnce=True):
        '''
        Create an object that holds the playback parameters for a 
        programmed button. This initializer is used in two contexts. When the ui
        is first built, and when a button program set is reconstituted from an
        XML file.
        
        @param buttonLabel: The label on the button
        @type  buttonLabel: string
        @param textToSave: Text to play back with this button.
        @type  textToSave: string
        @param voice: The voice to use for the utterance
        @type  voice: string
        @param ttsEngine: The text-to-speech engine to use. (e.g. "festival", or "cepstral"
        @type  ttsEngine: string
        @param playOnce: Whether to play the utterance just once, or several times.
        @type  playOnce: bool
        '''
        self.buttonLabel = buttonLabel;
        self.textToSay   = textToSave;
        self.activeVoice = voice;
        self.ttsEngine   = ttsEngine;
        self.playOnce    = playOnce;
        
    #----------------------------------
    # getText 
    #--------------
        
        
    def getText(self):
        return self.textToSay;

    #----------------------------------
    # setText 
    #--------------
        
    def setText(self, newText):
        '''
        Change the button program's utterance text to newText. 
        @param newText: The new utterance.
        @type  newText: string
        '''
        self.textToSay = newText;

    #----------------------------------
    # getLabel
    #--------------
        
    def getLabel(self):
        return self.buttonLabel;
    
    #----------------------------------
    # getTtsEngine
    #--------------
        
    def getTtsEngine(self):
        return self.ttsEngine;
    
    #----------------------------------
    # getVoice
    #--------------
        
    def getVoice(self):
        return self.activeVoice;
    
    #----------------------------------
    # toXML
    #--------------

    def toXML(self):
        
        domOneButtonProgramRoot = ButtonSavior.createXMLElement(ButtonSavior.BUTTON_PROGRAM_TAG);
        
        domOneButtonProgramRoot.append(ButtonSavior.createXMLElement(ButtonSavior.BUTTON_LABEL_TAG, content=self.buttonLabel));
        domOneButtonProgramRoot.append(ButtonSavior.createXMLElement(ButtonSavior.BUTTON_TEXT_TO_SAY_TAG, content=self.textToSay));
        domOneButtonProgramRoot.append(ButtonSavior.createXMLElement(ButtonSavior.BUTTON_VOICE_TAG, content=self.activeVoice));
        domOneButtonProgramRoot.append(ButtonSavior.createXMLElement(ButtonSavior.BUTTON_TTS_ENGINE, content=self.ttsEngine));        
        domOneButtonProgramRoot.append(ButtonSavior.createXMLElement(ButtonSavior.BUTTON_PLAY_ONCE, content=str(self.playOnce)));                                       
        return domOneButtonProgramRoot

# ----------------------------------------------- Class ButtonSet ------------------------------------

class ButtonSet(object):
    '''
    Class to hold one set of ButtonProgram instances that together
    form a set of buttons that are visible in the UI.
    '''
    
    def __init__(self, buttonProgArray, fileName):
        '''
        The array of ButtonProgram objects that constitute the new
        ButtonSet, and the file name that represents its .xml equivalent
        on disk. 
        @param buttonProgArray: the ButtonProgram instances of the ButtonSet
        @type buttonProgArray: [ButtonProgram]
        @param fileName: basename of file that holds the XML encoded information of the new set. Not the full path; example: buttonSet1.xml.
        @type fileName: string
        '''
        self.buttonProgArray = buttonProgArray
        self.fileName = fileName
        
    def getXMLFileName(self):
        return self.fileName;
    
    def getButtonProgram(self):
        return self.buttonProgArray;

# ----------------------------------------------- Class SpeakEasyController ------------------------------------

class SpeakEasyController(object):
    '''
    Control logic behind the speakeasy GUI.
	
    Available voices:
        1. Festival: Usually voice_kal_diphone (male) on Ubuntu installations
        2. Cepstral: Depends on your installation. Voices are individually licensed.       
    '''
    
    VERSION = '1.2';
    PID_PUBLICATION_FILE = "/tmp/speakeasyPID";
    PROJECT_ROOT = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../..');
    CONFIG_PATH = os.path.join(os.getenv('HOME'), '.speakeasy');
    
    # Mapping from sound button names ('SOUND_1', 'SOUND_2', etc) to sound filename (just basename):
    soundPaths = {}
    # Mapping sound file basenames to their full file names:
    soundPathsFull = {};
    
    # Constant for repeating play of sound/music/voice forever: 
    FOREVER = -1;
    
    # Unix signals for use with clearing text remotely, and with
    # pasting and speech-triggering from remote:
    REMOTE_CLEAR_TEXT_SIG = signal.SIGUSR1;
    REMOTE_PASTE_AND_SPEAK_SIG = signal.SIGUSR2;
    
    def __init__(self, dirpath, unix_sig_notify_read_socket=None, stand_alone=None):

        if stand_alone is None:
            stand_alone = (DEFAULT_PLAY_LOCATION == PlayLocation.LOCALLY);
        originalStandAlone = stand_alone

        self.unix_sig_notify_read_socket = unix_sig_notify_read_socket;        
        self.stand_alone = stand_alone;
        self.gui = None;
        self.soundPlayer = None;
        self.textToSpeechPlayer = None;
        self.rosInitialized = False;
        self.speechReplayDemons = [];
        self.soundReplayDemons  = [];
        self.sound_file_names   = [];
        self.roboComm = None;
        
        localInit = False;
        robotInit = False;
        
        # Remember original default play location, so that
        # we can warn user of the import error condition further
        # down, when the GUI is up:
        DEFAULT_PLAY_LOCATION_ORIG = DEFAULT_PLAY_LOCATION;
    
        # If this is the first time SpeakEasy is started on this machine,
        # by this user, then create a .speakeasy subdirectory under the
        # user's HOME, and copy all sound effects and buttonPrograms
        # into that subdirectory:
        self.initConfigDir();
              
        if self.stand_alone or not ROS_IMPORT_OK:
            localInit = self.initLocalOperation();
        else: # Robot operation
            robotInit = self.initROSOperation();
            
        self.gui = SpeakEasyGUI(stand_alone=self.stand_alone, sound_effect_labels=self.sound_file_names);
        self.gui.setWindowTitle("SpeakEasy (V" + SpeakEasyController.VERSION + ")");
        
        self.dialogService = DialogService(self.gui);
        # Handler that makes program button temporarily
        # look different to indicate entry into program mode:
        self.gui.hideButtonSignal.connect(alternateLookHandler);
        # Handler that makes program button look normal:
        self.gui.showButtonSignal.connect(standardLookHandler);

        # Now that we have the GUI up, we can warn user
        # if ROS couldn't be imported, the ROS node wasn't running,
        # or no SpeakEasy node was available, yet this app was 
        # set to control a Ros node (rather than running locally):
        
        if originalStandAlone == PlayLocation.ROBOT and not robotInit:
            self.dialogService.showErrorMsg("Application was set to control sound on robot, but: %s. Switching to local operation." %
                                            str(self.rosInitException));

        if self.stand_alone:
            self.gui.setWhereToPlay(PlayLocation.LOCALLY);
        else:
            self.gui.setWhereToPlay(PlayLocation.ROBOT);
            
        self.dirpath = dirpath;
        
        # No speech buttons programmed yet:
        self.programs = {};

        self.currentButtonSetFile = os.path.join(ButtonSavior.SPEECH_SET_DIR, 'default.xml');
        
        # Accept SIGUSR1 and SIGUSR2 from other processes
        # to initiate PASTE and CLEAR operations, of the 
        # text area, respectively. Done via a socket from 
        # outside the application, which we connect to a QSocketNofifier
        # (see __main__ below):
        self.unixSigNotifier = QSocketNotifier(self.unix_sig_notify_read_socket.fileno(), QSocketNotifier.Read)
        
        self.connectWidgetsToActions()
        self.installDefaultSpeechSet();
        
        # Let other processes know out pid: 
        self.publishPID();

    def initConfigDir(self):
        SpeakEasyController.SOUND_DIR = os.path.join(SpeakEasyController.CONFIG_PATH, 'sounds');
        origSpeechSetDir =  ButtonSavior.SPEECH_SET_DIR;
        ButtonSavior.SPEECH_SET_DIR = os.path.join(SpeakEasyController.CONFIG_PATH, 
                                                   os.path.basename(ButtonSavior.SPEECH_SET_DIR));
        
        # Config dir already exists?                                           
        if not os.path.isdir(SpeakEasyController.CONFIG_PATH):
            # No. ==> First time start of SpeakEasy for this user on this machine.
            # Copy the default sounds and button programs to the config dir
            # ($HOME/.speakeasy):
            shutil.copytree(os.path.join(SpeakEasyController.PROJECT_ROOT,'sounds'), 
                            SpeakEasyController.SOUND_DIR);
                            
            # Button programs directory:                            
            shutil.copytree(origSpeechSetDir, ButtonSavior.SPEECH_SET_DIR);
        
    def findButtonFileOriginalForDefault(self):
        '''
        Attempt to find buttonProgram file that is the same as the 
        default.xml file, except for the title. Doesn't work; ran out of time.
        '''
        buttonSetDefaultFile = os.path.join(ButtonSavior.SPEECH_SET_DIR, 'default.xml');
        with open(buttonSetDefaultFile, 'r') as fd:
            defaultSet = fd.readlines();
        defaultSetInfoOnly = defaultSet[2:];
        
        fileAndDirsList = os.listdir(ButtonSavior.SPEECH_SET_DIR);
        for fileName in fileAndDirsList:
            if fileName == 'default.xml':
                continue;
            fullFileName = os.path.join(ButtonSavior.SPEECH_SET_DIR, fileName);
            with open(fullFileName, 'r') as fd:
                xmlContent = fd.readlines();
            xmlSetInfoOnly = xmlContent[2:];
            if xmlSetInfoOnly == defaultSetInfoOnly:
                return fullFileName;
        return None;
            
    #----------------------------------
    # shutdown 
    #--------------

    def shutdown(self):
        '''
        Delete the PID pub file. Not crucial, but nice to let other
        processes know that SpeakEasy is no longer running.
        '''
        try:
            self.gui.speechControls.shutdown();
            os.remove(SpeakEasyController.PID_PUBLICATION_FILE);
        except:
            pass

    #----------------------------------
    # initLocalOperation 
    #--------------
    
    def initLocalOperation(self):
        '''
        Initialize for playing sound and text-to-speech locally. 
        OK to call multiple times. Initializes
        self.sound_file_names to a list of sound file names
        for use with SoundPlayer instance.
        @return: True if initialization succeeded, else False.
        @rtype: boolean
        '''
        if self.soundPlayer is None:
            self.soundPlayer = SoundPlayer();
        if self.textToSpeechPlayer is None:
            self.textToSpeechPlayer = TextToSpeechProvider();
        self.sound_file_names = self.getAvailableSoundEffectFileNames(stand_alone=True);
        self.stand_alone = True;
        return True;

    #----------------------------------
    # initROSOperation 
    #--------------
    
    def initROSOperation(self):
        '''
        Try to initialize operation through ROS messages to 
        a SpeakEasy ROS node. If that init fails, revert to local
        operation.
        @return: True if ROS operation init succeeded. Else, if local ops was initiated instead, 
                 return False.
        @rtype: bool
        '''
        # Try to initialize ROS. If that does not work, instantiation
        # raises NotImplementedError, or IOError:
        try:
            self.roboComm = RoboComm();
            self.sound_file_names = self.roboComm.getSoundEffectNames();
            self.stand_alone = False;
            return True
        except Exception as rosInitFailure:
            self.rosInitException = rosInitFailure;
            # Robot init didn't work, fall back to local op:
            self.initLocalOperation();
            return False
        
    #----------------------------------
    # connectWidgetsToActions 
    #--------------
    
    def connectWidgetsToActions(self):
        self.gui.speechInputFld
        for recorderButton in self.gui.recorderButtonDict.values():
            recorderButton.clicked.connect(partial(self.actionRecorderButtons, recorderButton));
        self.connectProgramButtonsToActions();
        for soundButton in self.gui.soundButtonDict.values():
            soundButton.clicked.connect(partial(self.actionSoundButtons, soundButton));
        newSpeechSetButton = self.gui.speechSetButtonDict[SpeakEasyGUI.interactionWidgets['NEW_SPEECH_SET']];
        newSpeechSetButton.clicked.connect(self.actionNewSpeechSet);
        pickSpeechSetButton = self.gui.speechSetButtonDict[SpeakEasyGUI.interactionWidgets['PICK_SPEECH_SET']];
        pickSpeechSetButton.clicked.connect(self.actionPickSpeechSet);

        # Location where to play: Locally, or at the Robot:
        for radioButton in self.gui.playLocalityRadioButtonsDict.values():
            if radioButton.text() == "Play at robot":
                radioButton.clicked.connect(partial(self.actionWhereToPlayRadioButton, PlayLocation.ROBOT));
            else:
                radioButton.clicked.connect(partial(self.actionWhereToPlayRadioButton, PlayLocation.LOCALLY));
        
        self.gui.replayPeriodSpinBox.valueChanged.connect(self.actionRepeatPeriodChanged);

        pasteButton = self.gui.convenienceButtonDict[SpeakEasyGUI.interactionWidgets['PASTE']];
        pasteButton.clicked.connect(self.actionPaste);
        clearButton = self.gui.convenienceButtonDict[SpeakEasyGUI.interactionWidgets['CLEAR']];
        clearButton.clicked.connect(self.actionClear);
        speechControlButton = self.gui.convenienceButtonDict[SpeakEasyGUI.interactionWidgets['SPEECH_MODULATION']];
        speechControlButton.clicked.connect(self.actionSpeechControls);
        
        
        # Remote control of clearing text field, and speaking what's in the
        # text field from other applications. Handled via Unix signals SIGUSR1
        # and SIGUSR2. These are caught in handleOS_SIGUSR1_2() in __main__. The
        # handler writes the respective signal number to the socket, which triggerse
        # a socket notifier. We connect that notifier to a handler:
        self.unixSigNotifier.activated.connect(self.actionUnixSigReceived);
    
    @Slot(int)    
    def actionUnixSigReceived(self, socket):
        # Read the signal number (32 is the buff size):
        (sigNumStr, socketAddr) = self.unix_sig_notify_read_socket.recvfrom(32);
        sigNumStr = sigNumStr.strip();
        if sigNumStr == str(SpeakEasyController.REMOTE_CLEAR_TEXT_SIG):
            self.actionClear();
        elif sigNumStr == str(SpeakEasyController.REMOTE_PASTE_AND_SPEAK_SIG):
            self.actionPaste()
            playButton = self.gui.recorderButtonDict[self.gui.interactionWidgets['PLAY_TEXT']];  
            self.actionRecorderButtons(playButton);
        
    def connectProgramButtonsToActions(self):
        for programButton in self.gui.programButtonDict.values():
            programButton.pressed.connect(partial(self.actionProgramButtons, programButton));
            programButton.released.connect(partial(self.actionProgramButtonRelease, programButton));
            # Each program button gets a context menu. 
            # Context menu entry to copy programmed text to the text area:
            copyToTextAreaAction = QAction('Copy to text area', programButton);
            copyToTextAreaAction.triggered.connect(partial(self.programButtonContextMenuCopyToTextAreaAction, programButton));
            programButton.addAction(copyToTextAreaAction);
            programButton.setContextMenuPolicy(Qt.ActionsContextMenu);
    
    #----------------------------------
    # getAvailableSoundEffectFileNames
    #--------------
    
    def getAvailableSoundEffectFileNames(self, stand_alone=None):
        '''
        Determine all the available sound effect files. If this process
        operates stand-alone, the local '../../sounds' subdirectory is searched.
        Else, in a ROS environment, the available sound effect file names 
        are obtained from the 'speech_capabilities_inquiry' service call.
        @param stand_alone: False if referenced sounds are to be from the ROS environment.
        @type stand_alone: boolean
        @return: array of sound file basenames without extensions. E.g.: [rooster, birds, elephant]
        @rtype: [string]
        '''

        if stand_alone is None:
            stand_alone = self.stand_alone;
        
        # Standalone files are local to this process:
        if stand_alone:
            return self.getAvailableLocalSoundEffectFileNames();

        # Get sound effect names from SpeakEasy ROS node:
            return self.roboComm.getSoundEffectNames();
        
    #----------------------------------
    # getAvailableLocalSoundEffectFileNames 
    #--------------
        
    def getAvailableLocalSoundEffectFileNames(self):
        
#        scriptDir = os.path.dirname(os.path.realpath(__file__));
#        soundDir = os.path.join(scriptDir, "../../sounds");
        if not os.path.exists(SpeakEasyController.SOUND_DIR):
            raise ValueError("No sound files found.")
        
        fileAndDirsList = os.listdir(SpeakEasyController.SOUND_DIR);
        fileList = [];
        # Grab all usable sound file names:
        for fileName in fileAndDirsList:
            fileExtension = SpeakeasyUtils.fileExtension(fileName); 
            if (fileExtension == "wav") or (fileExtension == "ogg"):
                fileList.append(fileName);
         
        sound_file_basenames = [];
        for (i, full_file_name) in enumerate(fileList):
            baseName = os.path.basename(full_file_name);
            SpeakEasyController.soundPaths['SOUND_' + str(i)] = full_file_name;
            # Chop extension off the basename (e.g. rooster.wav --> rooster):
            sound_file_basenames.append(os.path.splitext(os.path.basename(full_file_name))[0]);
            # Map basename (e.g. 'rooster.wav') to its full file name.
            self.soundPathsFull[baseName] = os.path.join(SpeakEasyController.SOUND_DIR,full_file_name);
        return sound_file_basenames;
    
    #----------------------------------
    # sayText 
    #--------------
    
    def sayText(self, text, voice, ttsEngine="festival", sayOnce=True, stand_alone=None):
        '''
        Send message to SpeakEasy service to say text, with the
        given voice, using the given text-to-speech engine.
        </p>
        If the voice parameter is the Festival voice 'Male' it is a special case in
        that it refers to the Festival engine's "voice_kal_diphone". We convert this.
        
        @param text: Text to be uttered by the tts engine
        @type  string
        @param voice: Name of speaking voice to be used.
        @type string
        @param ttsEngine: Name of tts engine to use (e.g. "festival" (the default), "cepstral"
        @type string
        @param sayOnce: Whether repeat the utterance over and over, or just say it once.
        @type bool
        '''
        
        if stand_alone is None:
            stand_alone = self.stand_alone;
        
        if ttsEngine == "festival" and voice == "Male":
            voice = "voice_kal_diphone";
        # Repeat over and over? Or say once?
        if stand_alone:
            if sayOnce:
                try:
                    self.textToSpeechPlayer.say(text, voice, ttsEngine);
                except ValueError:
                    self.dialogService.showErrorMsg("Voice '" + str(voice) +
                                                    "' is not supported by the text-to-speech engine '" +
                                                    str(ttsEngine) + "'.");
            else:
                self.speechReplayDemons.append(SpeakEasyController.SpeechReplayDemon(text, 
                                                                                     voice, 
                                                                                     ttsEngine, 
                                                                                     self.gui.getPlayRepeatPeriod(),
                                                                                     self.textToSpeechPlayer));
                self.speechReplayDemons[-1].start();
        else:
            if sayOnce:
                self.roboComm.say(text, voice=voice, ttsEngine=ttsEngine);
            else:
                self.roboComm.say(text, voice=voice, ttsEngine=ttsEngine, numRepeats=SpeakEasyController.FOREVER, repeatPeriod=self.gui.getPlayRepeatPeriod());
        return;
   
    #----------------------------------
    # convertRawTextToSSML   
    #--------------
   
    def convertRawTextToSSML(self, rawText):
        '''
        Given a string with SpeakEasy speech modulation markup, convert the
        string to W3C SSML marked-up text, and return a new string. Example:
        'This is [P90my] test' --> 'This is <prosody pitch='+90%'>my</prosody> test'.
        Note: Only the Cepstral engine currently handles SSML.
        @param theStr: string to convert
        @type theStr: String
        '''
        try:
            ssmlText = MarkupManagement.convertStringToSSML(rawText);
        except ValueError as e:
            self.dialogService.showErrorMsg(`e`);
        return ssmlText;
    
    #----------------------------------
    # actionRecorderButtons
    #--------------
            
    def actionRecorderButtons(self, buttonObj):
        '''
        Handler for one of the recorder buttons pushed:
        Play Text, or Stop.
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
            if (voice == "voice_kal_diphone"):
                ttsEngine = "festival"
            else:
                ttsEngine = "cepstral"
            rawText = self.gui.speechInputFld.getText();
            if ttsEngine == 'cepstral':
                # Convert any speech modulation markup to official W3C SSML markup:
                ssmlText = self.convertRawTextToSSML(rawText);
            else:
                ssmlText = rawText;
            self.sayText(ssmlText, voice, ttsEngine, self.gui.playOnceChecked());
            return;
        
        # Stop button pushed?
        buttonKey = self.gui.interactionWidgets['STOP'];
        if buttonObj == self.gui.recorderButtonDict[buttonKey]:
            self.stopAll();
            
    #----------------------------------
    #  stopAll
    #--------------
    
    def stopAll(self):
        if self.stand_alone:
            if len(self.speechReplayDemons) > 0:
                for speechDemon in self.speechReplayDemons:
                    speechDemon.stop();
                self.speechReplayDemons = [];
            self.textToSpeechPlayer.stop();
            if len(self.soundReplayDemons) > 0:
                for soundDemon in self.soundReplayDemons:
                    soundDemon.stop();
            self.soundReplayDemons = []; 
            
            self.soundPlayer.stop();
            self.textToSpeechPlayer.stop();
        else: # Robot op
            self.roboComm.stopSaying();
            self.roboComm.stopSound();
            return;
    
    #----------------------------------
    #  actionWhereToPlayRadioButton
    #--------------
    
    def actionWhereToPlayRadioButton(self, playLocation):
        if playLocation == PlayLocation.LOCALLY:
            self.stopAll();
            self.initLocalOperation();
        elif playLocation == PlayLocation.ROBOT:
            self.stopAll();
            success = self.initROSOperation();
            if not success:
                self.dialogService.showErrorMsg('Could not communicate with robot. Is the rosmaster node running?');
                # Switch radio button selection back to 'Play Locally':
                self.gui.setWhereToPlay(PlayLocation.LOCALLY);
    
    #----------------------------------
    # programButtonContextMenuAction
    #--------------

    def programButtonContextMenuCopyToTextAreaAction(self, buttonObj):
        program = None;
        try:    
            program = self.programs[buttonObj];
        except KeyError:
            self.dialogService.showErrorMsg("This button does not contain a program. Press-and-hold for " +\
                                            str(int(SpeakEasyGUI.PROGRAM_BUTTON_HOLD_TIME)) +\
                                            " seconds to program.");
            return;
        ttsEngine    = program.ttsEngine;
        rawText = program.getText()
        if (len(rawText) == 0):
            self.dialogService.showErrorMsg("This button does not contain a program. Press-and-hold for " +\
                                            str(int(SpeakEasyGUI.PROGRAM_BUTTON_HOLD_TIME)) +\
                                            " seconds to program.");
            return;
        if ttsEngine == 'cepstral':
            # Convert any speech modulation markup to official W3C SSML markup:
            ssmlText = self.convertRawTextToSSML(rawText);
        else:
            ssmlText = rawText;
        textArea = self.gui.speechInputFld;
        textArea.append(ssmlText);
    
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
        if os.path.basename(self.currentButtonSetFile) == 'default.xml':
            self.dialogService.showInfoMessage("Before (re)programming a button, please choose an existing button set via the 'Pick different speech set' button, or create a new set via the 'Save speech set' button.");
            return;
        newButtonLabel = self.gui.getNewButtonLabel();
        if newButtonLabel is not None:
            self.gui.setButtonLabel(buttonObj,newButtonLabel);
        
        textToSave = self.gui.speechInputFld.getText();
        if self.gui.activeVoice() == "voice_kal_diphone":
            ttsEngine = "festival"
        else:
            ttsEngine = "cepstral"
        programObj = ButtonProgram(buttonObj.text(), textToSave, self.gui.activeVoice(), ttsEngine, self.gui.playOnceChecked());
        
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
        rawText = program.getText()
        if ttsEngine == 'cepstral':
            # Convert any speech modulation markup to official W3C SSML markup:
            ssmlText = self.convertRawTextToSSML(rawText);
        else:
            ssmlText = rawText;
        self.sayText(ssmlText, voice, ttsEngine, onlyPlayOnce);
             
    #----------------------------------
    # actionSoundButtons 
    #--------------
        
    def actionSoundButtons(self, buttonObj):
        
        soundIndx = 0;
        while True:
            soundKey = "SOUND_" + str(soundIndx);
            try:
                soundLabel = self.gui.interactionWidgets[soundKey];
                oneButtonObj  = self.gui.soundButtonDict[soundLabel];
            except KeyError:
                raise ValueError("Unknown widget passed to actionSoundButton() method: " + str(buttonObj));
            if buttonObj == oneButtonObj:
                # For local operation, sound effect button labels are keys
                # to a dict that maps to the local file names:
                if self.stand_alone:
                    soundFile = SpeakEasyController.soundPaths[soundKey];
                else:
                    soundFile = buttonObj.text();
                break;
            else:
                soundIndx += 1;
        
        if self.stand_alone:
            originalSoundFile = soundFile;
            try:
                if not os.path.exists(soundFile):
                    try:
                        soundFile = self.soundPathsFull[soundFile]
                    except KeyError:
                        self.dialogService.showErrorMsg("Sound file %s not found. Searched %s/../../sounds." % (originalSoundFile, __file__))
                        return;
                    soundInstance = self.soundPlayer.play(soundFile);
                if self.gui.playRepeatedlyChecked():
                    self.soundReplayDemons.append(SpeakEasyController.SoundReplayDemon(soundInstance,self.gui.getPlayRepeatPeriod(), self.soundPlayer));
                    self.soundReplayDemons[-1].start();

            except IOError as e:
                self.dialogService.showErrorMsg(str(e));
                return
        else: # Robot operation
            if self.gui.playRepeatedlyChecked():
                self.roboComm.playSound(soundFile, numRepeats=SpeakEasyController.FOREVER, repeatPeriod=self.gui.getPlayRepeatPeriod());
            else:
                self.roboComm.playSound(soundFile);
        
    # ------------------------- Changing and Adding Button Programs --------------
        
    #----------------------------------
    #  actionNewSpeechSet
    #--------------
        
        
    def actionNewSpeechSet(self):

        # Get an iterator over all the current program# button UI widgets:
        programButtonIt = self.gui.programButtonIterator();
        
        # Get an array of ButtonProgram objects that are associated
        # with those buttons:
        buttonProgramArray = [];
        while True:
            try:
                buttonObj = programButtonIt.next();
                buttonLabel = buttonObj.text();
                try:
                    buttonProgramArray.append(self.programs[buttonObj]);
                except KeyError:
                    # Button was not programmed. Create an empty ButtonProgram:
                    buttonProgramArray.append(ButtonProgram(buttonLabel, "", "Male", "festival"));
            except StopIteration:
                break;
        
        # Save this array of programs as XML:
        makeNewFile = self.dialogService.newButtonSetOrUpdateCurrent();
        if makeNewFile == DialogService.ButtonSaveResult.CANCEL:
            return;
        if makeNewFile == DialogService.ButtonSaveResult.NEW_SET:
            fileName = self.getNewSpeechSetName();
            ButtonSavior.saveToFile(buttonProgramArray, fileName, title=os.path.basename(fileName));
            self.currentButtonSetFile = fileName;
            buttonSetNum = self.getSpeechSetFromSpeechFileName(fileName);
            self.dialogService.showInfoMessage("New speech set %d created." % buttonSetNum);
        elif makeNewFile == DialogService.ButtonSaveResult.UPDATE_CURRENT:
            fileName = self.currentButtonSetFile;
            ButtonSavior.saveToFile(buttonProgramArray, fileName, title=os.path.basename(fileName));
            if os.path.basename(fileName) != 'default.xml':
                buttonSetNum = self.getSpeechSetFromSpeechFileName(self.currentButtonSetFile);
                self.dialogService.showInfoMessage("Saved to speech button set %d." % buttonSetNum);
            else:
                self.dialogService.showInfoMessage("Saved to speech button set 'default.xml'");

        if os.path.basename(fileName) != 'default.xml':
            try:
                shutil.copy(fileName, os.path.join(ButtonSavior.SPEECH_SET_DIR, "default.xml"));
            except:
                rospy.logerr("Could not copy new program XML file to default.xml.");

    #----------------------------------
    # actionPickSpeechSet
    #--------------

    def actionPickSpeechSet(self):
        
        # Build an array of ButtonProgram instances for each
        # of the XML files in the button set directory. Collect
        # these arrays in buttonProgramArray:
        
        xmlFileNames = self.getAllSpeechSetXMLFileNames();
        if xmlFileNames is None:
            self.dialogService.showErrorMsg("No additional button sets are stored on your disk.");
            return None;
        
        # Fill the following array with arrays of ButtonProgram:    
        buttonProgramArrays = [];
        # Associate each ButtonProgram array with the file name
        # from which it was built:
        buttonProgramSetFiles = {};
        for xmlFileName in xmlFileNames:
            if xmlFileName == 'default.xml':
                continue;
            try:
                (buttonSettingTitle, buttonProgram) = ButtonSavior.retrieveFromFile(xmlFileName, ButtonProgram); 
                buttonProgramArrays.append(buttonProgram);
                buttonSet = ButtonSet(buttonProgram, xmlFileName);
                buttonProgramSetFiles[buttonSet] = xmlFileName; 
            except ValueError as e:
                # Bad XML:
                rospy.logerr(`e`);
                return;
            
        buttonSetSelector = ButtonSetPopupSelector(iter(buttonProgramArrays));
        buttonSetSelected = buttonSetSelector.exec_();
        if buttonSetSelected == -1:
            self.dialogService.showErrorMsg('No button sets have been defined yet.')
            return;
        if buttonSetSelected == 0:
            # User cancelled:
            return;
        
        # Get the selected ButtonProgram array:
        buttonPrograms = buttonSetSelector.getCurrentlyShowingSet();
        self.replaceProgramButtons(buttonPrograms);
        
        # Update the disk file:
        #***buttonSetBaseFilename = buttonProgramSetFiles[buttonPrograms];
        buttonSetBaseFilename = None;
        for buttonSet in buttonProgramSetFiles.keys():
            if buttonSet.getButtonProgram() == buttonPrograms:
                buttonSetBaseFilename = buttonSet.getXMLFileName();
                break;
            
        if buttonSetBaseFilename is not None:
            self.currentButtonSetFile = os.path.join(ButtonSavior.SPEECH_SET_DIR, buttonSetBaseFilename);       
        
        # Copy this new XML file into default.xml, so that it will be
        # loaded next time the application starts:
        
        ButtonSavior.saveToFile(buttonPrograms, os.path.join(ButtonSavior.SPEECH_SET_DIR, "default.xml"), title="default.xml");      
    
    
    #----------------------------------
    # actionRepeatPeriodChanged 
    #--------------
      
    def actionRepeatPeriodChanged(self):
        # If the repeat period is changed on its spinbox,
        # automatically select 'Play repeatedly':
        self.gui.setPlayRepeatedlyChecked();
    
    #----------------------------------
    # actionClear
    #--------------
    
    def actionClear(self):
        self.gui.speechInputFld.clear();
    
    #----------------------------------
    # actionPaste 
    #--------------
        
    def actionPaste(self):
        # Also called by handleOS_SIGUSR1_2
        textArea = self.gui.speechInputFld;
        currCursor = textArea.textCursor();
        currCursor.insertText(QApplication.clipboard().text());
    
    #----------------------------------
    # actionSpeechControls 
    #--------------
        
    def actionSpeechControls(self):
        '''
        Raise the voice modulation control panel.
        '''
        self.gui.speechControls.show();
        self.gui.speechControls.raise_();
        
    #----------------------------------
    # installDefaultSpeechSet
    #--------------
                
    def installDefaultSpeechSet(self):
        defaultPath = os.path.join(ButtonSavior.SPEECH_SET_DIR, "default.xml");
        if not os.path.exists(defaultPath):
            return;
        (buttonSetTitle, buttonPrograms) =  ButtonSavior.retrieveFromFile(defaultPath, ButtonProgram);
        self.replaceProgramButtons(buttonPrograms);
        
    #----------------------------------
    #  replaceProgramButtons
    #--------------
        
    def replaceProgramButtons(self, buttonProgramArray):
        self.gui.replaceProgramButtons(buttonProgramArray);
        self.connectProgramButtonsToActions();
        # Update the button object --> ButtonProgram instance mapping:
        self.programs = {};
        buttonObjIt = self.gui.programButtonIterator();
        for buttonProgram in buttonProgramArray:
            try:
                self.programs[buttonObjIt.next()] = buttonProgram;
            except StopIteration:
                # Should not happen:
                raise ValueError("Fewer buttons than ButtonProgram instances.");
            
    #----------------------------------
    #  getAllSpeechSetXMLFileNames
    #--------------
        
    def getAllSpeechSetXMLFileNames(self):

        if not os.path.exists(ButtonSavior.SPEECH_SET_DIR):
            return None;
        
        xmlFileNames = []
        for fileName in os.listdir(ButtonSavior.SPEECH_SET_DIR):
            if fileName.endswith(".xml") or fileName.endswith(".XML"):
                xmlFileNames.append(fileName);
        if len(xmlFileNames) == 0:
            return None;
        
        return xmlFileNames;
         
    #----------------------------------
    # getNewSpeechSetName
    #--------------
  
    def getNewSpeechSetName(self):
        
        if not os.path.exists(ButtonSavior.SPEECH_SET_DIR):
            os.makedirs(ButtonSavior.SPEECH_SET_DIR);
        suffix = 1;
        newFileName = "buttonProgram1.xml";
        fileSet = set(os.listdir(ButtonSavior.SPEECH_SET_DIR));
        while newFileName in fileSet:
            suffix += 1;
            newFileName = "buttonProgram" + str(suffix) + ".xml";
        return os.path.join(ButtonSavior.SPEECH_SET_DIR, newFileName);
    
    #----------------------------------
    # getSpeechSetFromSpeechFileName
    #--------------
    
    def getSpeechSetFromSpeechFileName(self, filePath):
        '''
        Given a file path to a button set, return the button set's number.
        We assume that the path is well formed, and all button sets are named
        buttonProgramnnn.xml. If malformed path, shows error msg on screen, and 
        returns None.
        @param filePath: Path to xml file.
        @type filePath: string
        @return: Number encoded in file name (i.e. nnn)
        @rtype: int
        '''
        
        # Get something like: buttonSet2.xml:
        fileName = os.path.basename(filePath).split('.')[0];
        buttonSetNum = fileName[len('buttonProgram'):];
        try:
            return int(buttonSetNum)
        except ValueError:
            self.dialogService.showErrorMsg('Bad file path to button sets: %s' % filePath);
            return None;
    
    
    #----------------------------------
    # publishPID 
    #--------------
    
    def publishPID(self):
        with open(SpeakEasyController.PID_PUBLICATION_FILE,'w') as fd:
            fd.write(str(os.getpid()));

    #----------------------------------
    # handleOS_SIGUSR1_2 
    #--------------

    def handleOS_SIGUSR1_2(self, signum, stack):
        if signum == signal.SIGUSR1:
            self.actionPaste;
        elif signum == signal.SIGUSR2:
            self.actionClear();
    
    # --------------------------------------------   Replay Demon -------------------------------
    
    # Only used for local operation:
    class ReplayDemon(threading.Thread):
        
        def __init__(self, repeatPeriod):
            super(SpeakEasyController.ReplayDemon, self).__init__();
            self.repeatPeriod = repeatPeriod;
            self.stopped = True;
            
    class SoundReplayDemon(ReplayDemon):
        
        def __init__(self, soundInstance, repeatPeriod, soundPlayer):
            super(SpeakEasyController.SoundReplayDemon, self).__init__(repeatPeriod);
            self.soundInstance = soundInstance;
            self.soundPlayer = soundPlayer;
    
        def run(self):
            self.stopped = False;
            self.soundPlayer.waitForSoundDone(self.soundInstance);
            while not self.stopped:
                time.sleep(self.repeatPeriod);
                self.soundPlayer.play(self.soundInstance, blockTillDone=True);
        
        def stop(self):
            self.stopped = True;
            self.soundPlayer.stop(self.soundInstance);
            
    class SpeechReplayDemon(ReplayDemon):
            
        def __init__(self, text, voiceName, ttsEngine, repeatPeriod, textToSpeechPlayer):
            super(SpeakEasyController.SpeechReplayDemon, self).__init__(repeatPeriod);
            self.text = text;
            self.ttsEngine = ttsEngine;
            self.voiceName = voiceName;
            self.textToSpeechPlayer = textToSpeechPlayer;
            
        
        def run(self):
            self.stopped = False;
            self.textToSpeechPlayer.waitForSoundDone();
            while not self.stopped:
                time.sleep(self.repeatPeriod);
                try:
                    self.textToSpeechPlayer.say(self.text, self.voiceName, self.ttsEngine, blockTillDone=True);
                except:
                    # If any problem, stop this thread so that we don't keep
                    # generating that same error:
                    self.stop();
        
        def stop(self):
            self.stopped = True;
            self.textToSpeechPlayer.stop();
        
if __name__ == "__main__":

    # Create socket pair to communicate between the
    # Unix signal handler and the Qt event loop:
    rsock, wsock = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
 
    # Handler for SIGUSR1 and SIGUSR2
    def sigusr1_2_handler(signum, stack):
       print 'Received Signal:', signum
       wsock.send(str(signum) + "\n");
    
    app = QApplication(sys.argv);
        
    # To find the sounds, we need the absolute directory
    # path to this script:
    scriptDir = os.path.dirname(os.path.abspath(sys.argv[0]));
    #speakeasyController = SpeakEasyController(scriptDir, stand_alone=False);
    #speakeasyController = SpeakEasyController(scriptDir, stand_alone=True);
    
    if len(sys.argv) > 1:
        if sys.argv[1] == 'local':
            print "Starting SpeakEasy in local (i.e. non-ROS) mode."
            speakeasyController = SpeakEasyController(scriptDir, unix_sig_notify_read_socket=rsock, stand_alone=True);
        else:
            try:
                rospy.loginfo("Will attempt to start SpeakEasy in ROS mode. If fail, switch to local mode. Possibly a few seconds delay...");
            except:
                print("Will attempt to start SpeakEasy in ROS mode. If fail, switch to local mode. Possibly a few seconds delay...");
            speakeasyController = SpeakEasyController(scriptDir, unix_sig_notify_read_socket=rsock, stand_alone=None);
    else:
        try:
            rospy.loginfo("Will attempt to start SpeakEasy in ROS mode. If fail, switch to local mode. Possibly a few seconds delay...");
        except:
            print("Will attempt to start SpeakEasy in ROS mode. If fail, switch to local mode. Possibly a few seconds delay...");
        speakeasyController = SpeakEasyController(scriptDir, unix_sig_notify_read_socket=rsock, stand_alone=None);

    # Attach Unix signals USR1/USR2 to the sigusr1_2_handler().
    # (These signals are separate from the Qt signals!):
    signal.signal(signal.SIGUSR1, sigusr1_2_handler)
    signal.signal(signal.SIGUSR2, sigusr1_2_handler)  
    # Unix signals are delivered to Qt only when Qt 
    # leaves its event loop. Force that to happen
    # every half second: 
    timer = QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)   

    # Enter Qt application main loop
    sys.exit(app.exec_());
        
