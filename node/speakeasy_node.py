#!/usr/bin/env python

# TODO:


import roslib; roslib.load_manifest('speakeasy')
import rospy

import os
import threading
import subprocess
import time
import sys
import tempfile

from speakeasy.msg import SpeakEasyStatus, SpeakEasyMusic, SpeakEasyPlayhead, SpeakEasySound, SpeakEasyTextToSpeech;
from speakeasy.msg import  TtsVoices;
from speakeasy.sound_player import SoundPlayer;
from speakeasy.text_to_speech import TextToSpeechProvider; 
from speakeasy.music_player import MusicPlayer;
from speakeasy.music_player import PlayStatus;

class SpeakEasyServer(object):

    STATUS_PUBLICATION_PERIOD   = 1.0; # seconds
    PLAYHEAD_PUBLICATION_PERIOD = 0.1; # seconds: Playhead time counter while playing

    SAY = 0;
    STOP= 1;

    PLAY = 0;
    STOP = 1;
    PAUSE = 2;
    UNPAUSE = 3;
    SET_VOL = 4;

    SET_PLAYHEAD = 5;
    
    def __init__(self):
        
        rospy.init_node('speakeasy')
        
        # We publish a latched message with this SpeakEasy node's speech, sound, and music capabilities:
        self.status_pub = rospy.Publisher("/speakeasy_status", SpeakEasyStatus, latch=False)
        # During music playback we also publish the playhead time position every 1/10 second:
        self.playhead_pub = rospy.Publisher("/speakeasy_playhead", SpeakEasyPlayhead);
        
        # Paths to where sound and music files are stored: In/below 'sounds' directory 
        # under package root dir:
        self.soundDir = os.path.join(os.path.dirname(__file__),'../sounds')
        self.musicDir = os.path.join(self.soundDir, 'music');
        
        self.lock = threading.Lock();
        
        self.ttsProvider = TextToSpeechProvider();
        self.soundPlayer = SoundPlayer();
        self.musicPlayer = MusicPlayer();
        
        self.subscriberStatusReq = rospy.Subscriber("speakeasy_status_req", SpeakEasyStatus, self.handleSpeakEasyStatusInquiry);
        self.subscriberMusicReq  = rospy.Subscriber("speakeasy_music_req", SpeakEasyMusic, self.handleMusicRequest);
        self.subscriberSoundReq  = rospy.Subscriber("speakeasy_sound_req", SpeakEasySound, self.handleSoundRequest);
        self.subscriberTTSReq    = rospy.Subscriber("speakeasy_text_to_speech_req", SpeakEasyTextToSpeech, self.handleTextToSpeechRequest)

        # Thread used during music playback to publish playhead position, and 
        # at less frequent period publish general status:
        self.playheadPublisherThread = PlayheadPublisher(self.musicPlayer, self.publishStatus).start();        
        
    def publishStatus(self):
        # Build a message listing the speech status (which engines, which sounds):
        statusMsg = self.buildStatusMsg()
        # Publish the status message just once, latched:
        self.status_pub.publish(statusMsg);
        
        
    def handleTextToSpeechRequest(self, req):
        
        ttsCmd = req.command;
        if ttsCmd == SpeakEasyServer.SAY:
            text       = req.text;
            engine     = req.engineName;
            voiceName  = req.voiceName;
            try:
                self.ttsProvider.say(text, voiceName, engine, destFileName)
            except:
                rospy.logerr("Error in received TextToSpeech.msg message caused text-to-speech error: " + str(sys.exc_info()[0]));
        elif ttxCmt == SpeakEasyServer.STOP:
            self.ttsProvider.stop();
        else:
            rospy.logerr("Incorrect command type in received TextToSpeech.msg message: " + str(ttsCmd));
    
    def handleSoundRequest(self, req):
        soundCmd = req.command;
        if soundCmd == SpeakEasyServer.PLAY:
            soundName = req.sound_name;
            volume    = req.volume;
            # Default volume?:
            if volume == -1.0:
                volume = None;
            
            try:
                self.soundPlayer.play(soundName, blockTillDone=False, volume=volume);
            except:
                rospy.logerr("Error in received SoundPlay.msg message caused sound play error: " + str(sys.exc_info()[0]));
        elif soundCmd == SpeakEasyServer.STOP:
            soundName = req.sound_name;
            try:
                self.soundPlayer.stop(soundName=soundName);
            except:
                rospy.logerr("Error while calling sound player command 'stop': " + str(sys.exc_info()[0]));
        elif soundCmd == SpeakEasyServer.PAUSE:
            soundName = req.sound_name;
            try:
                self.soundPlayer.pause(soundName=soundName);
            except:
                rospy.logerr("Error while calling sound player command 'pause': " + str(sys.exc_info()[0]));
        elif soundCmd == SpeakEasyServer.UNPAUSE:
            soundName = req.sound_name;
            try:
                self.soundPlayer.unpause(soundName=soundName);
            except:
                rospy.logerr("Error while calling sound player command 'unpause': " + str(sys.exc_info()[0]));
        elif soundCmd == SpeakEasyServer.SET_VOL:
            volume = req.volume;
            soundName = req.sound_name;
            try:
                self.soundPlayer.setSoundVolume(volume, soundName);
                # Update latched status message to reflect this new volume:
                self.publishStatus();
            except:
                rospy.logerr("Error while calling sound player command 'setSoundVolume': " + str(sys.exc_info()[0]));
        else:
            rospy.logerr("Incorrect command type in received SoundPlay.msg message: " + str(soundCmd));
            
    def handleMusicRequest(self, req):

        musicCmd = req.command;
        if musicCmd == SpeakEasyServer.PLAY:
            songName  = req.song_name;
            repeates  = req.repeats;
            startTime = req.time;
            volume    = req.volume;
            # Default volume?:
            if volume == -1.0:
                volume = None;
            try:
                self.musicPlayer.play(songName, repeats=repeats, startTime=startTime, blockTillDone=False, volume=volume);
            except:
                rospy.logerr("Error while calling music player command 'play': " + str(sys.exc_info()[0]));
        elif musicCmd == SpeakEasyServer.STOP:
            try:
                self.musicPlayer.stop();
            except:
                rospy.logerr("Error while calling music player command 'stop': " + str(sys.exc_info()[0]));
        elif musicCmd == SpeakEasyServer.PAUSE:
            try:
                self.musicPlayer.pause();
            except:
                rospy.logerr("Error while calling music player command 'pause': " + str(sys.exc_info()[0]));
        elif musicCmd == SpeakEasyServer.UNPAUSE:
            try:
                self.musicPlayer.unpause();
            except:
                rospy.logerr("Error while calling music player command 'unpause': " + str(sys.exc_info()[0]));
        elif musicCmd == SpeakEasyServer.SET_VOL:
            volume = req.volume;
            try:
                self.musicPlayer.setVol(volume);
                # Update latched status message to reflect this new volume:
                self.publishStatus();
            except:
                rospy.logerr("Error while calling music player command 'setVol': " + str(sys.exc_info()[0]));
        elif musicCmd == SpeakEasyServer.SET_PLAYHEAD:
            playheadTime  = req.time;
            timeReference = req.timeReference;
            try:
                self.musicPlayer.setPlayhead(playheadTime, timeReference=timeReference);
            except:
                rospy.logerr("Error while calling music player command 'setPlayhead': " + str(sys.exc_info()[0]));

    def handleSpeakEasyStatusInquiry(self, request):
        statusMsg = self.buildStatusMsg();
        #rospy.loginfo("Speech capability inquiry.")
        #return (statusMsg.ttsEngines,statusMsg.voices,statusMsg.sounds, voices,statusMsg.songs);
        return statusMsg;

    def buildStatusMsg(self):
        # Create an empty Status message:
        statusMsg = SpeakEasyStatus() 
        # initialize the text-to-speech engines field:
        statusMsg.ttsEngines = self.ttsProvider.availableTextToSpeechEngines();
        # Build ttsVoices structures for use in the Status
        # message's TtsVoices field, which is an array of:
        #    string ttsEngine
        #    string[] voices
        
        ttsVoicesFieldValue = [];
        # Get dict mapping each text-to-speech engine name to an array of voice names
        # that are available on that engine:
        ttsEnginesAndVoicesThisMachine = self.ttsProvider.availableVoices()
        for engine in ttsEnginesAndVoicesThisMachine.keys():
            ttsVoices = TtsVoices();
            ttsVoices.ttsEngine = engine;
            ttsVoices.voices    = ttsEnginesAndVoicesThisMachine[engine];
            ttsVoicesFieldValue.append(ttsVoices);
        
        statusMsg.voices = ttsVoicesFieldValue;
        
        # List of available sound files:
        filesInSoundDir = os.listdir(self.soundDir)
        self.soundFiles = []
        for soundFileName in filesInSoundDir:
            fileExtension = os.path.splitext(soundFileName)[1][1:].strip()
            if (fileExtension == "wav") or (fileExtension == "ogg"):
                self.soundFiles.append(soundFileName)
        
        statusMsg.sounds = self.soundFiles
        
        # List of available music files:
        filesInMusicDir = os.listdir(self.musicDir)
        self.musicFiles = []
        for musicFileName in filesInMusicDir:
            fileExtension = os.path.splitext(musicFileName)[1][1:].strip()
            if (fileExtension == "wav") or (fileExtension == "ogg"):
                self.musicFiles.append(musicFileName)
        
        statusMsg.sounds = self.musicFiles
        
        statusMsg.numSoundChannels = self.soundPlayer.numChannels();
        statusMsg.musicStatus = self.musicPlayer.getPlayStatus();
        statusMsg.soundVolume = self.soundPlayer.getSoundVolume(None) #**** Refine this
        statusMsg.musicVolume = self.musicPlayer.getSoundVolume()
        
        return statusMsg

    def sleep(self, duration):
        try:
            time.sleep(duration)
        except rospy.exceptions.ROSInterruptException:
            pass

class PlayheadPublisher(threading.Thread):
    '''
    Thread to publish playhead position while songs play.
    Rate is SpeakEasyServer.PLAYHEAD_PUBLICATION_PERIOD secs. 
    Additionally, thread publishes SpeakEasy status message
    every SpeakEasyServer.STATUS_PUBLICATION_PERIOD seconds.
    '''
    
    def __init__(self, musicPlayer, statusMsgSendMethod):
        super(PlayheadPublisher, self).__init__();
        self.musicPlayer = musicPlayer;
        self.statusMsgSendMethod = statusMsgSendMethod;
        self.playheadMsg = SpeakEasyPlayhead();
        self.stopped     = False;
                    
    def run(self):
        lastStatusPublish = rospy.Time.now();
        while not self.stopped:
            # Time to publish a general status message?
            if (rospy.Time.now() - lastStatusPublish).secs >= SpeakEasyServer.STATUS_PUBLICATION_PERIOD:
                self.statusMsgSendMethod();
                lastStatusPublish = rospy.Time.now();
            
            while self.musicPlayer.getPlayStatus() == PlayStatus.PLAYING:
                self.playheadMsg.playhead_time = self.musicPlayer.getPlayheadPosition();
                try:
                    self.playhead_pub.publish(self.playheadMsg);
                except:
                    pass;
                time.sleep(PLAYHEAD_PUBLICATION_PERIOD);
                # Time to publish a general status message?
                if (rospy.Time.now() - lastStatusPublish).secs >= SpeakEasyServer.STATUS_PUBLICATION_PERIOD:
                    self.statusMsgSendMethod();
                    lastStatusPublish = rospy.Time.now();
                    
    def stop(self):
        self.stopped = True;
            
if __name__ == '__main__':
    speakEasyServer = SpeakEasyServer();
    print "Going to spin..."
    rospy.spin();
