#!/usr/bin/python

import roslib; roslib.load_manifest('speakeasy');
import rospy

import sys
import os
import time
import shutil
from functools import partial;
from threading import Timer;

import QtBindingHelper;
from QtGui import QApplication

from speakeasy.msg import SpeakEasyRequest;
from speakeasy.srv import SpeechCapabilitiesInquiry

from speakeasy_ui import SpeakEasyGUI;
from speakeasy_ui import DialogService;

from speakeasy.libspeakeasy import SoundClient;

from speakeasy_ui import alternateLookHandler;
from speakeasy_ui import standardLookHandler;

from speakeasy.speakeasy_persistence import ButtonSavior;

from speakeasy.buttonSetPopupSelector_ui import ButtonSetPopupSelector;

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
        @type string
        @param playOnce: Whether to play the utterance just once, or several times.
        @type  bool
        @param buttonLabel: Label visible on the button widget. If None, the label will be 
                            extracted from the buttonObj.
        @type string or None
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
    
    # Whether application runs in a ROS context, or
    # standalone:
    STAND_ALONE = False; 
    
    soundPaths = {}
    
    def __init__(self, dirpath, stand_alone=False):
        
        if not stand_alone:
            rospy.loginfo("Wait for speech capabilities service...");
            rospy.wait_for_service('speech_capabilities_inquiry')
            rospy.loginfo("Speech capabilities service online.");    
            capabilitiesService = rospy.ServiceProxy('speech_capabilities_inquiry', SpeechCapabilitiesInquiry)
            try:
                capabilitiesReply = capabilitiesService();
                self.sound_file_names = capabilitiesReply.sounds
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        soundFileNames = self.getAvailableSoundEffectFileNames(stand_alone=stand_alone);
        
        self.gui = SpeakEasyGUI(stand_alone=stand_alone, sound_effect_labels=soundFileNames);
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
        self.installDefaultSpeechSet();

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
    

    def connectProgramButtonsToActions(self):
        for programButton in self.gui.programButtonDict.values():
            programButton.pressed.connect(partial(self.actionProgramButtons, programButton));
            programButton.released.connect(partial(self.actionProgramButtonRelease, programButton));
    
    
    #----------------------------------
    # getAvailableSoundEffectFileNames
    #--------------
    
    def getAvailableSoundEffectFileNames(self, stand_alone=False):
        '''
        Determine all the available sound effect files. If this process
        operates stand-alone, the local 'sounds' subdirectory is searched.
        Else, in a ROS environment, the available sound effect file names 
        are obtained from the 'speech_capabilities_inquiry' service call:
        '''
        
        #********************
        return self.getAvailableLocalSoundEffectFileNames();
        
        # Standalone files are local to this process:
        if stand_alone:
            return self.getAvailableLocalSoundEffectFileNames();

        sound_file_basenames = [];
        for (i, full_file_name) in enumerate(self.sound_file_names):
            SpeakEasyController.soundPaths['SOUND_' + str(i)] = full_file_name;
            sound_file_basenames.append(os.path.splitext(os.path.basename(full_file_name))[0])
        return sound_file_basenames; 
        
    #----------------------------------
    # getAvailableLocalSoundEffectFileNames 
    #--------------
        
    def getAvailableLocalSoundEffectFileNames(self):
        
        scriptDir = os.path.dirname(os.path.realpath(__file__));
        soundDir = os.path.join(scriptDir, "../../sounds");
        if not os.path.exists(soundDir):
            raise ValueError("No sound files found.")
        
        fileList = os.listdir(soundDir);
        sound_file_basenames = [];
        for (i, full_file_name) in enumerate(fileList):
            # Isolate the file extension:
            fileExtension = os.path.splitext(full_file_name)[1][1:].strip()
            if (fileExtension == "wav") or (fileExtension == "ogg"):
                SpeakEasyController.soundPaths['SOUND_' + str(i)] = full_file_name;
                sound_file_basenames.append(os.path.splitext(os.path.basename(full_file_name))[0]);
        return sound_file_basenames;
    
    
    #----------------------------------
    # sayText 
    #--------------
    
    def sayText(self, text, voice, ttsEngine="festival", sayOnce=True):
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
        
        if ttsEngine == "festival" and voice == "Male":
            voice = "voice_kal_diphone";
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
                 soundFile = SpeakEasyController.soundPaths[soundKey];
                 break;
            else:
                soundIndx += 1;

        self.soundClient.sendMsg(SpeakEasyRequest.PLAY_FILE,
	                             SpeakEasyRequest.PLAY_START,
				                 soundFile)
        
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
    
        
        
if __name__ == "__main__":

    app = QApplication(sys.argv);
        
    # To find the sounds, we need the absolute directory
    # path to this script:
    scriptDir = os.path.dirname(os.path.abspath(sys.argv[0]));
    speakeasyController = SpeakEasyController(scriptDir, stand_alone=False);
        
    # Enter Qt application main loop
    sys.exit(app.exec_());
        
        
