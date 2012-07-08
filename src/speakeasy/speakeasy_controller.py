#!/usr/bin/python

# TODO:
#   - Robot ops with new msg
#   - Word completion
#   - Switch live between local and remote ops. 

import roslib; roslib.load_manifest('speakeasy');

import sys
import os
import time
import subprocess
import re
import shutil
import threading
from functools import partial;
from threading import Timer;

from python_qt_binding import QtBindingHelper;
from PyQt4.QtGui import QApplication

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

#TODO: Delete:
## Try importing ROS related modules. Remember whether
## that worked. In the SpeakEasyController __init__() method
## we'll switch to local, and warn user if appropriate:
#try:
#    import roslib; roslib.load_manifest('speakeasy');
#    import rospy
#    ROS_IMPORT_OK = True;
#except ImportError:
#    # Ros not installed on this machine; run locally:
#    ROS_IMPORT_OK = False;
# ----------------------------------------------- Class Program ------------------------------------

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


# ----------------------------------------------- Class SpeakEasyController ------------------------------------

class SpeakEasyController(object):
    '''
    Control logic behind the speakeasy GUI.
	
    Available voices:
        1. Festival: Usually voice_kal_diphone (male) on Ubuntu installations
        2. Cepstral: Depends on your installation. Voices are individually licensed.       
    '''
    
    VERSION = '1.0';
    
    # Mapping from sound button names ('SOUND_1', 'SOUND_2', etc) to sound filename (just basename):
    soundPaths = {}
    # Mapping sound file basenames to their full file names:
    soundPathsFull = {};
    
    # Constant for repeating play of sound/music/voice forever: 
    FOREVER = -1;
    
    def __init__(self, dirpath, stand_alone=None):

        if stand_alone is None:
            stand_alone = (DEFAULT_PLAY_LOCATION == PlayLocation.LOCALLY);
        originalStandAlone = stand_alone
        
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
              
        if self.stand_alone:
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

        if stand_alone:
            self.gui.setWhereToPlay(PlayLocation.LOCALLY);
        else:
            self.gui.setWhereToPlay(PlayLocation.ROBOT);
            
        self.dirpath = dirpath;
        
        # No speech buttons programmed yet:
        self.programs = {};
        self.connectWidgetsToActions()
        self.installDefaultSpeechSet();

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
        
    def connectProgramButtonsToActions(self):
        for programButton in self.gui.programButtonDict.values():
            programButton.pressed.connect(partial(self.actionProgramButtons, programButton));
            programButton.released.connect(partial(self.actionProgramButtonRelease, programButton));
    
    
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
        
        scriptDir = os.path.dirname(os.path.realpath(__file__));
        soundDir = os.path.join(scriptDir, "../../sounds");
        if not os.path.exists(soundDir):
            raise ValueError("No sound files found.")
        
        fileAndDirsList = os.listdir(soundDir);
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
            self.soundPathsFull[baseName] = os.path.join(soundDir,full_file_name);
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
    #   
    #--------------
   
    
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
            self.sayText(self.gui.speechInputFld.getText(), voice, ttsEngine, self.gui.playOnceChecked());
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
            self.initROSOperation();
    
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
        self.sayText(program.getText(), voice, ttsEngine, onlyPlayOnce);
             
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
        fileName = self.getNewSpeechSetName();
        ButtonSavior.saveToFile(buttonProgramArray, fileName, title=os.path.basename(fileName));
        try:
            shutil.copy(fileName, os.path.join(ButtonSavior.SPEECH_SET_DIR, "default.xml"));
        except:
            rospy.logerr("Could not copy new program XML file to default.xml.");
        self.dialogService.showInfoMessage("New speech set created.");

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
        for xmlFileName in xmlFileNames:
            try:
                (buttonSettingTitle, buttonProgram) = ButtonSavior.retrieveFromFile(xmlFileName, ButtonProgram); 
                buttonProgramArrays.append(buttonProgram);
            except ValueError as e:
                # Bad XML:
                rospy.logerr(e.toStr());
                return;
            
        buttonSetSelector = ButtonSetPopupSelector(iter(buttonProgramArrays));
        buttonSetSelected = buttonSetSelector.exec_();
        if buttonSetSelected != 1:
            return;
        
        # Get the selected ButtonProgram array:
        buttonPrograms = buttonSetSelector.getCurrentlyShowingSet();
        self.replaceProgramButtons(buttonPrograms);
        
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
        for filename in os.listdir(ButtonSavior.SPEECH_SET_DIR):
            if filename == newFileName:
                suffix += 1;
                newFileName = "buttonProgram" + str(suffix) + ".xml";
                continue;
            break;
        return os.path.join(ButtonSavior.SPEECH_SET_DIR, newFileName);
    
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

    app = QApplication(sys.argv);
        
    # To find the sounds, we need the absolute directory
    # path to this script:
    scriptDir = os.path.dirname(os.path.abspath(sys.argv[0]));
    #speakeasyController = SpeakEasyController(scriptDir, stand_alone=False);
    #speakeasyController = SpeakEasyController(scriptDir, stand_alone=True);
    if len(sys.argv) > 1:
        if sys.argv[1] == 'local':
            print "Starting SpeakEasy in local (i.e. non-ROS) mode."
            speakeasyController = SpeakEasyController(scriptDir, stand_alone=True);
        else:
            try:
                rospy.loginfo("Will attempt to start SpeakEasy in ROS mode. If fail, switch to local mode. Possibly a few seconds delay...");
            except:
                print("Will attempt to start SpeakEasy in ROS mode. If fail, switch to local mode. Possibly a few seconds delay...");
            speakeasyController = SpeakEasyController(scriptDir, stand_alone=None);
    else:
        try:
            rospy.loginfo("Will attempt to start SpeakEasy in ROS mode. If fail, switch to local mode. Possibly a few seconds delay...");
        except:
            print("Will attempt to start SpeakEasy in ROS mode. If fail, switch to local mode. Possibly a few seconds delay...");
        speakeasyController = SpeakEasyController(scriptDir, stand_alone=None);
            
    # Enter Qt application main loop
    sys.exit(app.exec_());
        
        
