#!/usr/bin/env python

import os
import sys

from speakeasy.music_player import MusicPlayer;
from speakeasy.music_player import TimeReference


from python_qt_binding import QtBindingHelper;
from PyQt4.QtCore import QRect
from PyQt4.QtGui import QWidget, QStyleFactory, QApplication, QDialog, QMainWindow
from PyQt4.QtGui import QLabel, QVBoxLayout, QHBoxLayout, QTabWidget

QT_CREATOR_UI_FILE_ROOT = os.path.join(os.path.dirname(os.path.realpath(__file__)), "QTCreatorFiles");
SONG_DIRECTORY = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../../sounds/music");

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
        self.initUI();
        
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

        self.connectWidgets()
        
        # Populate the song list from the file system:
        self.populateSongList(self.pickSongList)

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

    def rewindAction(self):
        self.musicPlayer.stop();
        currentSongName = self.getCurrentSongName();
        if currentSongName is None:
            return;
        self.musicPlayer.play(self.allSongs[currentSongName]);
        
    def backALittleAction(self):
        self.musicPlayer.setPlayhead(-1 * self.incrementalMoveTime, TimeReference.RELATIVE);
        
    def forwardALittleAction(self):
        self.musicPlayer.setPlayhead(self.incrementalMoveTime, TimeReference.RELATIVE);
        
    def playAction(self):
        fullFilename = self.allSongs[self.getCurrentSongName()];
        self.musicPlayer.play(fullFilename);
        
    def stopAction(self):
        self.musicPlayer.stop();
        
    def getCurrentSongName(self):
        currentSongNameOnComboBox = self.pickSongList.currentText();
        if currentSongNameOnComboBox == Tapper.EMPTY_SONG_LIST_TEXT:
            return None
        return currentSongNameOnComboBox;

if __name__ == "__main__":

    style = QStyleFactory.create("Cleanlooks");
    QApplication.setStyle(style);
    app = QApplication(sys.argv);
        
    tapper = Tapper();
    
    # Enter Qt application main loop
    #builderFullUI.show();
    sys.exit(app.exec_());

        

    
