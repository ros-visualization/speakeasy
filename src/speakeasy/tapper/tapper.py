#!/usr/bin/env python

import os
import sys
import threading
import time

from speakeasy.music_player import MusicPlayer;
from speakeasy.music_player import TimeReference
from speakeasy.music_player import PlayStatus


from python_qt_binding import QtBindingHelper;
from PyQt4.QtCore import QRect, Signal, QObject
from PyQt4.QtGui import QWidget, QStyleFactory, QApplication, QDialog, QMainWindow
from PyQt4.QtGui import QLabel, QVBoxLayout, QHBoxLayout, QTabWidget, QIcon

QT_CREATOR_UI_FILE_ROOT = os.path.join(os.path.dirname(os.path.realpath(__file__)), "QTCreatorFiles");
SONG_DIRECTORY = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../../sounds/music");
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

class Tapper(QWidget):
    
    # Singleton instance, obtainable via this
    # class var.
    ui = None;
    
    TAB_INDEX_PICK_SONG = 0;
    TAB_INDEX_TAP_BEAT  = 1;
    TAB_INDEX_USE_BEAT  = 2;
    
    EMPTY_SONG_LIST_TEXT = "No songs found";
    
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
        self.commChannel = CommChannel();
        self.initUI();
        
        # Start thread that blinks the pause button when
        # paused, and updates the playhead counter while
        # playing: 
        OneSecondMaintenance(self.musicPlayer, self.commChannel.pauseBlinkSignal, self.commChannel.playCounterUpdateSignal).start();
        
    def initUI(self):
        
        self.setWindowTitle("Tapper");
        
        # Load QtCreator's XML UI files:
        # Make QtCreator generated UIs children if this instance:
        self.loadUIs();

        
        self.toolStack =  QTabWidget()
        self.toolStack.addTab(self.pickSongWidget, "Pick a song");
        self.toolStack.addTab(self.tapBeatWidget, "Tap the beat");
        self.toolStack.addTab(self.beatChooserWidget, "Use the beat");
        
        self.tapeRecWidget.setParent(self.tapBeatWidget.recorderContainerWidget);

        layout =  QVBoxLayout()
        layout.addWidget(self.toolStack);
        self.setLayout(layout)
        self.setGeometry(QRect(88, 682, 750, 489));

        # Make simple names for the widgets we care about:
        self.pickSongList = self.pickSongWidget.pickSongList; # Combobox
        self.playButton = self.tapeRecWidget.playButton;
        self.rewindButton = self.tapeRecWidget.rewindButton;
        self.backALittleButton =  self.tapeRecWidget.littleBitLeftButton;
        self.forwardALittleButton =  self.tapeRecWidget.littleBitRightButton;
        self.stopButton = self.tapeRecWidget.stopButton;
        self.incrementalMoveTimeSpinBox = self.tapeRecWidget.smallMoveTimeSpinBox;
        self.songPositionSpinBox = self.tapeRecWidget.songPositionSpinbox;

        self.connectWidgets()
        
        # Populate the song list from the file system:
        self.populateSongList(self.pickSongList)
        
        # Prepare the play and pause icons for the tape recorder button:
        self.playIcon  = QIcon(os.path.join(IMAGE_DIRECTORY, "play.png"));
        self.pauseIcon = QIcon(os.path.join(IMAGE_DIRECTORY, "pause.png"));
        self.currentPlayPauseIconState = PlayPauseIconState.PAUSE_ICON_SHOWING;

        self.show();
        

    def loadUIs(self):
        self.pickSongWidget    = QtBindingHelper.loadUi(os.path.join(QT_CREATOR_UI_FILE_ROOT, "pickSong/pickSong.ui"));
        self.tapBeatWidget     = QtBindingHelper.loadUi(os.path.join(QT_CREATOR_UI_FILE_ROOT, "tapBeat/tapBeat.ui"));
        self.beatChooserWidget = QtBindingHelper.loadUi(os.path.join(QT_CREATOR_UI_FILE_ROOT, "beatChooser/beatChooser.ui"));
        self.tapeRecWidget     = QtBindingHelper.loadUi(os.path.join(QT_CREATOR_UI_FILE_ROOT, "tapeRecorder/tapeRecorder.ui"));


    def connectWidgets(self):
        self.toolStack.currentChanged.connect(self.switchedTab);
        self.playButton.clicked.connect(self.playAction);
        self.rewindButton.clicked.connect(self.rewindAction);
        self.backALittleButton.clicked.connect(self.backALittleAction);
        self.forwardALittleButton.clicked.connect(self.forwardALittleAction);
        self.stopButton.clicked.connect(self.stopAction);
        self.songPositionSpinBox.valueChanged.connect(self.setPlayheadFromSpinBoxAction);
        
        # Blink pause icon on/off during paused playback:
        self.commChannel.pauseBlinkSignal.connect(self.blinkPauseButtonIcon);
        # Update playhead pos counter during playback:
        self.commChannel.playCounterUpdateSignal.connect(self.updatePlayheadCounter)
        
        
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
                if self.musicPlayer.formatSupported(extension):
                    # Map file name without extension (i.e. song name) to full path: 
                    self.allSongs[fileBaseName] = os.path.join(SONG_DIRECTORY,filename);
            if (len(self.allSongs) == 0):
                comboBoxToPopulate.addItem(Tapper.EMPTY_SONG_LIST_TEXT);
            else:
                comboBoxToPopulate.addItems(self.allSongs.keys());
        else:
            comboBoxToPopulate.addItems(songNameList); 

    def setPlayPauseButtonIcon(self, icon):
        self.playButton.setIcon(icon);

    def rewindAction(self):
        self.musicPlayer.stop();
        self.setPlayPauseButtonIcon(self.playIcon);
        currentSongName = self.getCurrentSongName();
        if currentSongName is None:
            return;
        self.musicPlayer.play(self.allSongs[currentSongName]);
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
            fullFilename = self.allSongs[self.getCurrentSongName()];
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
        currentSongNameOnComboBox = self.pickSongList.currentText();
        if currentSongNameOnComboBox == Tapper.EMPTY_SONG_LIST_TEXT:
            return None
        return currentSongNameOnComboBox;

    def getIncrementalMoveTime(self):
        return self.incrementalMoveTimeSpinBox.value();

    def getSongPositionIndicator(self):
        return self.songPositionSpinBox.value();
    
    def setSongPositionIndicator(self, newValue):
        if not isinstance(newValue, float):
            raise ValueError("Song position must be a floating point number. Instead it was: " + str(newValue));
        self.songPositionSpinBox.setValue(newValue);
        

class OneSecondMaintenance(threading.Thread):
    
    def __init__(self, musicPlayer, pauseBlinkSignal, playCounterSignal):
        super(OneSecondMaintenance, self).__init__();
        self.stopped = False;
        self.musicPlayer = musicPlayer;
        self.pauseBlinkSignal = pauseBlinkSignal;
        self.playCounterSignal = playCounterSignal;
        
    def run(self):
        while not self.stopped:
            playStatus = self.musicPlayer.getPlayStatus(); 
            if playStatus == PlayStatus.PAUSED:
                self.pauseBlinkSignal.emit();
            elif playStatus == PlayStatus.PLAYING:
                self.playCounterSignal.emit();

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

        

    
