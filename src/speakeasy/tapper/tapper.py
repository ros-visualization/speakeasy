#!/usr/bin/env python

import os
import sys
import threading
import time
import random

from speakeasy.speakeasy_ui import DialogService;

from speakeasy.music_player import MusicPlayer;
from speakeasy.music_player import TimeReference;
from speakeasy.music_player import PlayStatus;
from speakeasy.sound_player import SoundPlayer;


from python_qt_binding import QtBindingHelper;
from PyQt4.QtCore import QRect, Signal, QObject
from PyQt4.QtGui import QWidget, QStyleFactory, QApplication, QDialog, QMainWindow
from PyQt4.QtGui import QLabel, QVBoxLayout, QHBoxLayout, QTabWidget, QIcon

QT_CREATOR_UI_FILE_ROOT = os.path.join(os.path.dirname(os.path.realpath(__file__)), "QTCreatorFiles");
SOUND_DIRECTORY = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../../sounds");
SONG_DIRECTORY = os.path.join(SOUND_DIRECTORY, "music");
IMAGE_DIRECTORY = os.path.join(os.path.dirname(os.path.realpath(__file__)), "img"); 

class CommChannel(QObject):
    '''
    Combines signals into one place.
    '''
    pauseBlinkSignal        = Signal();
    playCounterUpdateSignal = Signal();

class PlayPauseIconState:
    PAUSE_ICON_SHOWING = 0;
    PLAY_ICON_SHOWING  = 1;

class SoundType:
    SONG = 0;
    SOUND_EFFECT = 1;

class Tapper(QWidget):
    
    # Singleton instance, obtainable via this
    # class var.
    ui = None;
    
    #WINDOW_SIZE = QRect(88, 682, 720, 380)    
    WINDOW_SIZE = QRect(88, 682, 720, 480)
    
    TAB_INDEX_PICK_SONG = 0;
    TAB_INDEX_TAP_BEAT  = 1;
    TAB_INDEX_USE_BEAT  = 2;
    
    EMPTY_SONG_LIST_TEXT = "No songs found";
    EMPTY_SOUND_LIST_TEXT = "No sound effects found";
    
    def __init__(self):
        super(Tapper, self).__init__();
        
        # Enforce singleton op for this class. 
        # We could simply return, if we detect
        # a second instantiation here, but it seems
        # better to clarify the correct operation: 
        if Tapper.ui is not None:
            raise RuntimeError("Can only have single instance of Tapper");
        
        Tapper.ui = self;
        self.musicPlayer = MusicPlayer();
        self.soundPlayer = SoundPlayer();
        self.commChannel = CommChannel();
        # Error message popup utility:
        self.dialogService = DialogService();
        self.initUI();
        
        # Initialize tapping panel data:
        self.resetTap();
        
        # Start thread that blinks the pause button when
        # paused, and updates the playhead counter while
        # playing: 
        self.maintenanceThread = OneSecondMaintenance(self, self.musicPlayer, self.commChannel.pauseBlinkSignal, self.commChannel.playCounterUpdateSignal);
        self.maintenanceThread.start();
        
    def initUI(self):
        
        self.setWindowTitle("Tapper");
        
        # No play stop time scheduled:
        self.scheduledPlayStopTime = None;
        
        # Load QtCreator's XML UI files:
        # Make QtCreator generated UIs children if this instance:
        self.loadUIs();

        
        self.toolStack =  QTabWidget()
        self.toolStack.addTab(self.pickSongWidget, "Pick a song");
        self.toolStack.addTab(self.tapBeatWidget, "Tap the beat");
        self.toolStack.addTab(self.beatChooserWidget, "Use the beat");
        self.toolStack.addTab(self.insertCommandsDialog, "Insert songs/sounds to program");
        self.toolStack.addTab(self.speechWidget, "Robot says...");
        
        self.tapeRecWidget.setParent(self.tapBeatWidget.recorderContainerWidget);

        layout =  QVBoxLayout()
        layout.addWidget(self.toolStack);
        self.setLayout(layout)
        #self.setGeometry(QRect(88, 682, 750, 489));
        self.setGeometry(Tapper.WINDOW_SIZE);

        # Make simple names for the widgets we care about:

                
        # Pick-a-song panel:
        self.pickSongList = self.pickSongWidget.pickSongList; # Combobox
        self.pickSongPlayExcerptButton = self.pickSongWidget.shortPlayButton
        self.pickSongStopExcerptButton = self.pickSongWidget.shortPlayStopButton
        self.pickSongSongSampleLenSpinBox = self.pickSongWidget.songSampleLength
        
        # Tap-the-beat panel:
        self.tapButton        = self.tapBeatWidget.tapButton;
        self.tapBeatPeriodLCD = self.tapBeatWidget.beatPeriodLCD;
        self.tapResetButton   = self.tapBeatWidget.tapResetButton;
        
        # Beat chooser panel:
        self.beatChooserSongName = self.beatChooserWidget.songName;
        self.beatChooserOneBeatButton = self.beatChooserWidget.oneBeat;
        self.beatChooserHalfBeatButton = self.beatChooserWidget.halfBeat;
        self.beatChooserQuarterBeatButton = self.beatChooserWidget.quarterBeat;
        self.beatChooserEighthBeatButton = self.beatChooserWidget.eighthBeat;
        self.beatChooserInserSongButton = self.beatChooserWidget.insertSongButton;

        # Tape recorder embedded in the Tap-the-beat panel:        
        self.playButton = self.tapeRecWidget.playButton;
        self.rewindButton = self.tapeRecWidget.rewindButton;
        self.backALittleButton =  self.tapeRecWidget.littleBitLeftButton;
        self.forwardALittleButton =  self.tapeRecWidget.littleBitRightButton;
        self.stopButton = self.tapeRecWidget.stopButton;
        self.incrementalMoveTimeSpinBox = self.tapeRecWidget.smallMoveTimeSpinBox;
        self.songPositionSpinBox = self.tapeRecWidget.songPositionSpinbox;
        
        # Song/Sound insert panel:
        self.insertCmdSongToInsert = self.insertCommandsDialog.songSelectComboBox;
        self.insertCmdStartPos = self.insertCommandsDialog.startPosSpinbox;
        self.insertCmdSongRepeats = self.insertCommandsDialog.repeatsSpinbox;
        self.insertCmdPlayDuration = self.insertCommandsDialog.playForSpinbox;
        
        self.insertCmdPauseSongButton = self.insertCommandsDialog.pauseSongButton;
        self.insertCmdUnpauseSongButton = self.insertCommandsDialog.unpauseSongButton;
        self.insertCmdStopSongButton = self.insertCommandsDialog.stopSongButton;
        self.insertCmdPauseSoundButton = self.insertCommandsDialog.pauseSoundButton;
        self.insertCmdUnpauseSoundButton = self.insertCommandsDialog.unpauseSoundButton;
        self.insertCmdStopSoundButton = self.insertCommandsDialog.stopSoundButton;
        
        self.insertCmdSoundToInsert = self.insertCommandsDialog.soundSelectComboBox;
        self.insertCmdSoundRepeats = self.insertCommandsDialog.soundRepeatsSpinbox;
        self.insertCmdSoundTestPlayButton = self.insertCommandsDialog.soundTestPlayButton;
        self.insertCmdSoundTestStopButton = self.insertCommandsDialog.soundTestStopButton;
        
        self.insertCmtSoundTestInsert = self.insertCommandsDialog.insertSoundButton;
        
        
        # Robot speech panel:
        self.speechText = self.speechWidget.speechTextBox;
        self.speechPlayButton = self.speechWidget.playButton;
        self.speechStopButton = self.speechWidget.stopButton;        self.speechDavidVoice = self.speechWidget.davidVoiceRadioButton;
        self.speechComputerVoice = self.speechWidget.davidVoiceRadioButton;
        self.speechInsertButton = self.speechWidget.speechInsertButton;

        self.connectWidgets()
        
        # Populate the song lists from the file system:
        self.populateSongList(self.pickSongList)
        self.populateSongList(self.beatChooserSongName);
        self.populateSongList(self.insertCmdSongToInsert);
        
        # Populate the sound list:
        self.populateSoundList(self.insertCmdSoundToInsert);
    
        # Prepare the play and pause icons for the tape recorder button:
        self.playIcon  = QIcon(os.path.join(IMAGE_DIRECTORY, "play.png"));
        self.pauseIcon = QIcon(os.path.join(IMAGE_DIRECTORY, "pause.png"));
        self.currentPlayPauseIconState = PlayPauseIconState.PAUSE_ICON_SHOWING;

        self.show();
        

    def loadUIs(self):
        self.pickSongWidget       = QtBindingHelper.loadUi(os.path.join(QT_CREATOR_UI_FILE_ROOT, "pickSong/pickSong.ui"));
        self.tapBeatWidget        = QtBindingHelper.loadUi(os.path.join(QT_CREATOR_UI_FILE_ROOT, "tapBeat/tapBeat.ui"));
        self.beatChooserWidget    = QtBindingHelper.loadUi(os.path.join(QT_CREATOR_UI_FILE_ROOT, "beatChooser/beatChooser.ui"));
        self.insertCommandsDialog = QtBindingHelper.loadUi(os.path.join(QT_CREATOR_UI_FILE_ROOT, "multiMediaOps/multimediaOperations.ui"));
        self.speechWidget         = QtBindingHelper.loadUi(os.path.join(QT_CREATOR_UI_FILE_ROOT, "saySomething/saySomething.ui"));
        self.tapeRecWidget        = QtBindingHelper.loadUi(os.path.join(QT_CREATOR_UI_FILE_ROOT, "tapeRecorder/tapeRecorder.ui"));


    def connectWidgets(self):
        
        # Tab switch signal:
        self.toolStack.currentChanged.connect(self.switchedTab);
        
        # Pick-a-song panel:
        self.pickSongPlayExcerptButton.clicked.connect(self.playSongExcerptAction);
        self.pickSongStopExcerptButton.clicked.connect(self.stopSongExcerptAction);

        # Tapping the beat:
        self.tapButton.clicked.connect(self.tapButtonAction);
        self.tapResetButton.clicked.connect(self.tapResetAction);
        
        # Tape recorder:
        self.playButton.clicked.connect(self.playAction);
        self.rewindButton.clicked.connect(self.rewindAction);
        self.backALittleButton.clicked.connect(self.backALittleAction);
        self.forwardALittleButton.clicked.connect(self.forwardALittleAction);
        self.stopButton.clicked.connect(self.stopAction);
        self.songPositionSpinBox.valueChanged.connect(self.setPlayheadFromSpinBoxAction);
        
        self.insertCmdPauseSongButton.clicked.connect(self.insertCommandPauseSongButtonAction);
        self.insertCmdUnpauseSongButton.clicked.connect(self.insertCommandUnpauseSongButtonAction);
        self.insertCmdStopSongButton.clicked.connect(self.insertCommandStopSongButtonAction);
        
        self.insertCmdPauseSoundButton.clicked.connect(self.insertCommandPauseSoundButtonAction);
        self.insertCmdUnpauseSoundButton.clicked.connect(self.insertCommandUnpauseSoundButtonAction);
        self.insertCmdStopSoundButton.clicked.connect(self.insertCommandStopSoundButtonAction);
        
        self.insertCmdSoundTestPlayButton.clicked.connect(self.playSoundAction);
        self.insertCmdSoundTestStopButton.clicked.connect(self.stopSoundAction);
        self.insertCmtSoundTestInsert.clicked.connect(self.insertSoundAction);
        
        
        # Blink pause icon on/off during paused playback:
        self.commChannel.pauseBlinkSignal.connect(self.blinkPauseButtonIcon);
        # Update playhead pos counter during playback:
        self.commChannel.playCounterUpdateSignal.connect(self.updatePlayheadCounter)





    def closeEvent(self, eventObj):
        try:
            self.maintenanceThread.stop();
            eventObj.accept();
        except:
            pass;
        
    def switchedTab(self, newTabIndex):
        #print str(self.geometry());
        if newTabIndex == Tapper.TAB_INDEX_PICK_SONG:
            self.populateSongList(self.pickSongList);
    
    def populateSongList(self, comboBoxToPopulate, songNameList=None):
        if songNameList is None:
            # Get songs from standard song directory
            filenames = os.listdir(SONG_DIRECTORY);
            self.allSongs = {};
            for filename in filenames:
                # From /foo/bar/blue.txt, get (blue, .txt):
                (fileBaseName, extension) = os.path.splitext(filename);
                try:
                    self.musicPlayer.formatSupported(extension);
                    # Map file name without extension (i.e. song name) to full path: 
                    self.allSongs[fileBaseName] = os.path.join(SONG_DIRECTORY,filename);
                except ValueError:
                    continue;
            if (len(self.allSongs) == 0):
                comboBoxToPopulate.addItem(Tapper.EMPTY_SONG_LIST_TEXT);
            else:
                comboBoxToPopulate.addItems(self.allSongs.keys());
        else:
            comboBoxToPopulate.addItems(songNameList); 

    def populateSoundList(self, comboBoxToPopulate, soundNameList=None):
        if soundNameList is None:
            # Get songs from standard song directory
            filenames = os.listdir(SOUND_DIRECTORY);
            self.allSounds = {};
            for filename in filenames:
                # From /foo/bar/blue.txt, get (blue, .txt):
                (fileBaseName, extension) = os.path.splitext(filename);
                try:
                    self.soundPlayer.formatSupported(extension);
                    # Map file name without extension (i.e. song name) to full path: 
                    self.allSounds[fileBaseName] = os.path.join(SOUND_DIRECTORY,filename);
                except ValueError:
                    continue;
            if (len(self.allSounds) == 0):
                comboBoxToPopulate.addItem(Tapper.EMPTY_SOUND_LIST_TEXT);
            else:
                comboBoxToPopulate.addItems(self.allSounds.keys());
        else:
            comboBoxToPopulate.addItems(soundNameList); 


    def setPlayPauseButtonIcon(self, icon):
        self.playButton.setIcon(icon);

    def rewindAction(self):
        self.musicPlayer.stop();
        self.setPlayPauseButtonIcon(self.playIcon);
        currentSongName = self.getCurrentSongName();
        if currentSongName is None:
            return;
        self.musicPlayer.play(currentSongName)
        #******self.setPlayPauseButtonIcon(self.pauseIcon);
        
    def backALittleAction(self):
        if self.musicPlayer.playStatus == PlayStatus.PLAYING:        
            self.musicPlayer.setPlayhead(-1 * self.getIncrementalMoveTime(), TimeReference.RELATIVE);
        
    def forwardALittleAction(self):
        if self.musicPlayer.playStatus == PlayStatus.PLAYING:
            self.musicPlayer.setPlayhead(self.getIncrementalMoveTime(), TimeReference.RELATIVE);
        
    def playAction(self):
        # The play button shares with the pause function.
        # If we are currently playing, this button press indicates
        # that the user wants to pause.
        playStatus = self.musicPlayer.getPlayStatus();
        if playStatus == PlayStatus.PAUSED:
            self.musicPlayer.unpause();
            #****self.setPlayPauseButtonIcon(self.pauseIcon);
            return;
        elif playStatus == PlayStatus.PLAYING:
            self.musicPlayer.pause()
            self.currentPlayPauseIconState = PlayPauseIconState.PAUSE_ICON_SHOWING;
            #****self.setPlayPauseButtonIcon(self.playIcon);
            return;
        else:
            # Play is stopped. Just play the song from the start again: 
            fullFilename = self.getCurrentSongName();
            self.musicPlayer.play(fullFilename);
            self.setPlayPauseButtonIcon(self.pauseIcon);
        
    def stopAction(self):
        self.musicPlayer.stop();
        self.setPlayPauseButtonIcon(self.playIcon);

    def setPlayheadFromSpinBoxAction(self, newVal):
        # Interrupt from user having changed the playhead spinbox:
        if not isinstance(newVal, float):
            return;
        self.musicPlayer.setPlayhead(newVal, TimeReference.ABSOLUTE);

    def playSongExcerptAction(self):
        
        if self.getCurrentSongName() is None:
            return
        sampleLen = self.pickSongSongSampleLenSpinBox.value();
        
        # Find a random spot in the song, and play for sampleLen 
        # seconds. Make sure that if the song is very short, move
        # start time closer to the start of the song:

        # Avg song is 3 minutes (180 seconds). Pick an upper
        # time bound below that, but not too low, so that random
        # snippets will be played from all over the song:
        highestStartTime = 170; # sec
        random.seed();        
        playedSample = False;
        while not playedSample:
            sampleStart = random.randint(0,highestStartTime); 
            self.musicPlayer.play(self.getCurrentSongName(), startTime=float(sampleStart), blockTillDone=False);
            if self.musicPlayer.getPlayStatus() != PlayStatus.PLAYING:
                highestStartTime = int(highestStartTime / 2);
                continue;
            else:
                # Schedule for playback to stop after sampleLen seconds.
                # This action will be taken by the maintenance thread:
                self.scheduledPlayStopTime = time.time() + sampleLen;
                return;

    def stopSongExcerptAction(self):
        '''
        Stop song snippet playing in pick-a-song panel.
        '''
        self.musicPlayer.stop();

    def tapButtonAction(self):
        
        # If more than 15 seconds since the previous tap,
        # or since initialization, start over:
        if (time.time() - self.mostRecentTapTime) > 15:
            self.resetTap();
        
        self.numTaps += 1;
        self.mostRecentTapTime = time.time();
        self.currentBeatPerSecs = float(self.numTaps) / (self.mostRecentTapTime - self.firstTapTime);
        # Convert to beats per minute:
        self.currentBeat       = self.currentBeatPerSecs * 60.0;
        self.tapBeatPeriodLCD.display(self.currentBeat); 
        
    def tapResetAction (self):
        self.resetTap();

    def resetTap(self):
        self.firstTapTime = time.time();
        self.mostRecentTapTime = self.firstTapTime;
        self.currentBeat = 0.0; 
        self.numTaps = 0;
        self.tapBeatPeriodLCD.display(0.0);


    def insertCmdPauseSongButtonAction(self):
        #TODO: implement
        self.dialogService.showErrorMsg("Pause song insertion not yet implemented.");
        
    def insertCmdUnpauseSongButtonAction(self):
        #TODO: implement
        self.dialogService.showErrorMsg("Unpause song insertion not yet implemented.");
        
    def insertCmdStopSongButtonAction(self):
        #TODO: implement
        self.dialogService.showErrorMsg("Stop song insertion not yet implemented.");
    
    def insertCmdPauseSoundButtonAction(self):
        #TODO: implement
        self.dialogService.showErrorMsg("Pause sound insertion not yet implemented.");
        
    def insertCmdUnpauseSoundButtonAction(self):
        #TODO: implement
        self.dialogService.showErrorMsg("Unpause sound insertion not yet implemented.");
        
    def insertCmdStopSoundButtonAction(self):
        #TODO: implement
        self.dialogService.showErrorMsg("Stop sound insertion not yet implemented.");


    def playSoundAction(self):
    #TODO: implement
        self.soundPlayer.play(self.getCurrentSoundName(), blockTillDone=False);
    
    def stopSoundAction(self):
    #TODO: implement
        self.soundPlayer.stop();

    def insertSoundAction(self):
        #TODO implement.
        self.dialogService.showErrorMsg("Sound insertion not yet implemented.");
        

    # -----------------------------------------------------  Signal Handlers -----------------------------
    
    def blinkPauseButtonIcon(self):
        '''
        Handler for signal self.pauseBlinkSignal. The one-second
        maintenance thread generates that signal if the music player
        is currently paused. The method turns the pause button on
        and off, toggling each time.
        '''
        if self.currentPlayPauseIconState == PlayPauseIconState.PLAY_ICON_SHOWING:
            self.playButton.setIcon(self.pauseIcon);
            self.currentPlayPauseIconState = PlayPauseIconState.PAUSE_ICON_SHOWING;
        else:
            self.playButton.setIcon(self.playIcon);
            self.currentPlayPauseIconState = PlayPauseIconState.PLAY_ICON_SHOWING;
    
    def updatePlayheadCounter(self):
        '''
        Hander for signal self.playCounterUpdate. The one-second
        maintenance thread generates that signal if the music player
        is currently playing.
        '''
        playheadPos = self.musicPlayer.getPlayheadPosition();
        self.songPositionSpinBox.setValue(playheadPos);

    def getCurrentSongName(self):
        return self.getCurrentFileName(self.pickSongList, SoundType.SONG);
    
    def getCurrentSoundName(self):
        return self.getCurrentFileName(self.insertCmdSoundToInsert, SoundType.SOUND_EFFECT);

    def getCurrentFileName(self, comboboxObj, soundType):
        '''
        Return full pathname for the song or sound name in the given pick-a-song's or
        pick-a-sound's combobox:
        @param comboboxObj: Instance of QT4 combobox, which contains the name of a song or sound
                            in the music/sound directories, respectively.
        @type comboboxObj: QComboBox
        @param soundType: indicator whether given combobox contains a song or sound file.
        @type soundType: SoundType
        @return: full pathname of the given song or sound.
        @returnt: string
        '''
        currentNameOnComboBox = comboboxObj.currentText();
        if soundType == SoundType.SONG:
            if currentNameOnComboBox == Tapper.EMPTY_SONG_LIST_TEXT:
                return None
            try:
                currentSong = self.allSongs[currentNameOnComboBox];
            except KeyError:
                return None;
            return currentSong;
        else:
            if currentNameOnComboBox == Tapper.EMPTY_SOUND_LIST_TEXT:
                return None
            try:
                currentSound = self.allSounds[currentNameOnComboBox];
            except KeyError:
                return None;
            return currentSound;

    def getIncrementalMoveTime(self):
        return self.incrementalMoveTimeSpinBox.value();

    def getSongPositionIndicator(self):
        return self.songPositionSpinBox.value();
    
    def setSongPositionIndicator(self, newValue):
        if not isinstance(newValue, float):
            raise ValueError("Song position must be a floating point number. Instead it was: " + str(newValue));
        self.songPositionSpinBox.setValue(newValue);
        

class OneSecondMaintenance(threading.Thread):
    '''
    Thread for 1-second period chores:
    <ul>
        <li>When tape recorder is paused, changes play button icon between
            play and pause symbols.</li>
        <li>For song snippet playing, stops playback after tapper.snippetLength
            seconds.</li>
    </ul>
       
    '''
    
    def __init__(self, tapper, musicPlayer, pauseBlinkSignal, playCounterSignal):
        '''
        The info the thread needs:
        @param tapper: instance of Tapper (to obtain various instance vars from it).
        @type tapper: Tapper
        @param musicPlayer: Instance of MusicPlayer (to check for play status, and to stop playback)
        @type musicPlayer: MusicPlayer
        @param pauseBlinkSignal: Signal to emit so that the GUI thread will do the play button
                                 icon switching, which only that thread is allowed to do.
        @type pauseBlinkSignal: Signal
        @param playCounterSignal: Signal to emit so that the playhead time counter will be advanced
                                  in the GUI thread.
        @type playCounterSignal: Signal
        '''
        super(OneSecondMaintenance, self).__init__();
        self.stopped = False;
        self.musicPlayer = musicPlayer;
        self.tapper = tapper;
        self.pauseBlinkSignal = pauseBlinkSignal;
        self.playCounterSignal = playCounterSignal;
        
    def run(self):
        while not self.stopped:
            playStatus = self.musicPlayer.getPlayStatus(); 
            if playStatus == PlayStatus.PAUSED:
                self.pauseBlinkSignal.emit();
            elif playStatus == PlayStatus.PLAYING:
                self.playCounterSignal.emit();

            if self.tapper.scheduledPlayStopTime is not None:
                if time.time() >= tapper.scheduledPlayStopTime:
                    self.musicPlayer.stop();
                    tapper.scheduledPlayStopTime = None;

            time.sleep(1.0);
            
    def stop(self):
        self.stopped = True;
        

if __name__ == "__main__":

    style = QStyleFactory.create("Cleanlooks");
    QApplication.setStyle(style);
    app = QApplication(sys.argv);
        
    tapper = Tapper();
    
    # Enter Qt application main loop
    #builderFullUI.show();
    sys.exit(app.exec_());
    
