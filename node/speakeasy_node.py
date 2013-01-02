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

    # Paths to where sound and music files are stored: In/below 'sounds' directory 
    # under package root dir:
    soundDir = None;
    musicDir = None;
        
    # Dicts mapping sound and music filenames to their full paths.
    # Each sound or music file has two entrys: one keyed by the 
    # full file name (e.g. 'foo.wav'), the other by the file name
    # without extenseion (e.g. 'foo'). These dicts are populated
    # by the maintenance thread that also publishes the status
    # messages. That's why these are class vars, not instance vars:
    soundNameToFileDict = {};
    musicNameToFileDict = {};
    
    def __init__(self):
        
        rospy.init_node('speakeasy')
        
        # We publish a latched message with this SpeakEasy node's speech, sound, and music capabilities:
        self.status_pub = rospy.Publisher("/speakeasy_status", SpeakEasyStatus, latch=False)
        # During music playback we also publish the playhead time position every 1/10 second:
                
        SpeakEasyServer.soundDir = os.path.join(os.path.dirname(__file__),'../sounds')
        SpeakEasyServer.musicDir = os.path.join(SpeakEasyServer.soundDir, 'music');
        
        self.lock = threading.Lock();
        
        self.ttsProvider = TextToSpeechProvider();
        self.soundPlayer = SoundPlayer();
        self.musicPlayer = MusicPlayer();
                
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

        rospy.logdebug("T2S request: " + str(req));        
        with self.lock:
            ttsCmd = req.command;
            if ttsCmd == SpeakEasyServer.SAY:
                text       = req.text;
                engine     = req.engineName;
                voiceName  = req.voiceName;
                
                # Defaulted tts engine and voice names come as
                # empty strings in the request message. Change
                # those to None, so that the ttsProvider will 
                # understand to use default(s):
                if len(engine) == 0:
                    engine = None;
                if len(voiceName) == 0:
                    voiceName = None;
                try:
                    self.ttsProvider.say(text, voiceName, engine)
                except:
                    rospy.logerr("Error in received TextToSpeech.msg message caused text-to-speech error: " + self.makeMsg(sys.exc_info()));
            elif ttsCmd == SpeakEasyServer.STOP:
                self.ttsProvider.stop();
            else:
                rospy.logerr("Incorrect command type in received TextToSpeech.msg message: " + str(ttsCmd));
    
    def handleSoundRequest(self, req):
        
        with self.lock:
            soundCmd = req.command;
            if soundCmd == SpeakEasyServer.PLAY:
                soundName = req.sound_name;
                volume    = req.volume;
                # Default volume?:
                if volume == -1.0:
                    volume = None;
                
                # Ensure best effort to find the file:
                try:
                    soundName = self.toFullPath(soundName, SpeakEasyServer.soundNameToFileDict);
                except ValueError:
                    rospy.logerr("SpeakEasy error playing a sound: " + self.makeMsg(sys.exc_info()));
                    return;                    
                try:
                    self.soundPlayer.play(soundName, blockTillDone=False, volume=volume);
                except:
                    rospy.logerr("Error in received SoundPlay.msg message caused sound play error: " + self.makeMsg(sys.exc_info()));
            elif soundCmd == SpeakEasyServer.STOP:
                soundName = req.sound_name;
                if len(soundName) == 0:
                    soundName = None;
                # Ensure best effort to find the file:
                if soundName is not None:
                    try:                
                        soundName = self.toFullPath(soundName, SpeakEasyServer.soundNameToFileDict);
                    except ValueError:
                        rospy.logerr("SpeakEasy error stopping a sound: " + self.makeMsg(sys.exc_info()));
                        return;                    
                try:
                    self.soundPlayer.stop(whatToStop=soundName);
                except:
                    rospy.logerr("Error while calling sound player command 'stop': " + self.makeMsg(sys.exc_info()));
            elif soundCmd == SpeakEasyServer.PAUSE:
                soundName = req.sound_name;
                if len(soundName) == 0:
                    soundName = None;
                if soundName is not None:
                    # Ensure best effort to find the file:
                    try:
                        soundName = self.toFullPath(soundName, SpeakEasyServer.soundNameToFileDict);
                    except ValueError:
                        rospy.logerr("SpeakEasy error pausing a sound: " + self.makeMsg(sys.exc_info()));
                        return;                    
                try:
                    self.soundPlayer.pause(whatToPause=soundName);
                except:
                    rospy.logerr("Error while calling sound player command 'pause': " + self.makeMsg(sys.exc_info()));
            elif soundCmd == SpeakEasyServer.UNPAUSE:
                soundName = req.sound_name;
                if len(soundName) == 0:
                    soundName = None;
                if soundName is not None:
                    # Ensure best effort to find the file:
                    try:
                        soundName = self.toFullPath(soundName, SpeakEasyServer.soundNameToFileDict);
                    except ValueError:
                        rospy.logerr("SpeakEasy error un-pausing a sound: " + self.makeMsg(sys.exc_info()));
                        return;                    
                    
                try:
                    self.soundPlayer.unpause(whatToUnPause=soundName);
                except:
                    rospy.logerr("Error while calling sound player command 'unpause': " + self.makeMsg(sys.exc_info()));
            elif soundCmd == SpeakEasyServer.SET_VOL:
                volume = req.volume;
                soundName = req.sound_name;
                if len(soundName) == 0:
                    soundName = None;

                if soundName is not None:
                    # Ensure best effort to find the file:
                    try: 
                        soundName = self.toFullPath(soundName, SpeakEasyServer.soundNameToFileDict);
                    except ValueError:
                        rospy.logerr("SpeakEasy error setting volume for a sound: " + self.makeMsg(sys.exc_info()));
                        return;                    
                try:
                    self.soundPlayer.setSoundVolume(volume, whatToSetVolFor=soundName);
                    # Update latched status message to reflect this new volume:
                    self.publishStatus();
                except:
                    rospy.logerr("Error while calling sound player command 'setSoundVolume': " + self.makeMsg(sys.exc_info()));
            else:
                rospy.logerr("Incorrect command type in received SoundPlay.msg message: " + str(soundCmd));
            
    def handleMusicRequest(self, req):

        with self.lock:
            musicCmd = req.command;
            if musicCmd == SpeakEasyServer.PLAY:
                songName  = req.song_name;
                repeats  = req.repeats;
                startTime = req.time;
                volume    = req.volume;
                # Default volume?:
                if volume == -1.0:
                    volume = None;
    
                # Ensure best effort to find the file:
                try:
                    songName = self.toFullPath(songName, SpeakEasyServer.musicNameToFileDict);
                except ValueError:
                    rospy.logerr("SpeakEasy error playing a song: " + self.makeMsg(sys.exc_info()));
                    return;                    
                
                try:
                    self.musicPlayer.play(songName, repeats=repeats, startTime=startTime, blockTillDone=False, volume=volume);
                except:
                    rospy.logerr("Error while calling music player command 'play': " + self.makeMsg(sys.exc_info()));
            elif musicCmd == SpeakEasyServer.STOP:
                try:
                    self.musicPlayer.stop();
                except:
                    rospy.logerr("Error while calling music player command 'stop': " + self.makeMsg(sys.exc_info()));
            elif musicCmd == SpeakEasyServer.PAUSE:
                try:
                    self.musicPlayer.pause();
                except:
                    rospy.logerr("Error while calling music player command 'pause': " + self.makeMsg(sys.exc_info()));
            elif musicCmd == SpeakEasyServer.UNPAUSE:
                try:
                    self.musicPlayer.unpause();
                except:
                    rospy.logerr("Error while calling music player command 'unpause': " + self.makeMsg(sys.exc_info()));
            elif musicCmd == SpeakEasyServer.SET_VOL:
                volume = req.volume;
                try:
                    self.musicPlayer.setSoundVolume(volume);
                    # Update latched status message to reflect this new volume:
                    self.publishStatus();
                except:
                    rospy.logerr("Error while calling music player command 'setVol': " + self.makeMsg(sys.exc_info()));
            elif musicCmd == SpeakEasyServer.SET_PLAYHEAD:
                playheadTime  = req.time;
                timeReference = req.timeReference;
                try:
                    self.musicPlayer.setPlayhead(playheadTime, timeReference=timeReference);
                except:
                    rospy.logerr("Error while calling music player command 'setPlayhead': " + self.makeMsg(sys.exc_info()));

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
        filesInSoundDir = os.listdir(SpeakEasyServer.soundDir)
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
        
        statusMsg.music = self.musicFiles
        
        statusMsg.numSoundChannels = self.soundPlayer.numChannels();
        statusMsg.musicStatus = self.musicPlayer.getPlayStatus();
        statusMsg.textToSpeechBusy = self.ttsProvider.busy();
        statusMsg.soundVolume = self.soundPlayer.getSoundVolume(None) #**** Refine this
        statusMsg.musicVolume = self.musicPlayer.getSoundVolume()

        # Since we just collected all sound and music files, update
        # the self.soundNameToFileDict and self.musicNameToFileDict
        # maps to full file names:
        
        for soundName in self.soundFiles:
            soundPath = os.path.join(SpeakEasyServer.soundDir, soundName);
            soundFileName = os.path.basename(soundPath);
            SpeakEasyServer.soundNameToFileDict[soundFileName] = soundPath;
            SpeakEasyServer.soundNameToFileDict[os.path.splitext(soundFileName)[0]] = soundPath;
            
        for musicName in self.musicFiles:
            musicPath = os.path.join(SpeakEasyServer.musicDir, musicName);
            musicFileName = os.path.basename(musicPath);
            SpeakEasyServer.musicNameToFileDict[musicFileName] = musicPath;
            SpeakEasyServer.musicNameToFileDict[os.path.splitext(musicFileName)[0]] = musicPath;
        
        return statusMsg

    def sleep(self, duration):
        try:
            time.sleep(duration)
        except rospy.exceptions.ROSInterruptException:
            pass

    def makeMsg(self, systemExceptionInfo):
        '''
        Return the error string portion of a sys.exc_info() result.
        @param systemExceptionInfo: sys.exc_info() result
        @type systemExceptionInfo: [exceptionTypeObj, string, tracebackObj];
        '''
        return str(systemExceptionInfo[1]);

    def toFullPath(self, soundName, musicOrSoundDict):
        '''
        Given one of SpeakEasyServer.soundNameToFileDict or SpeakEasyServer.musicNameToFileDict
        and an alleged sound or music file name, return the file's full path
        if is not already an absolute path.
        @param soundName: Sound or music file name. Might be "/foo/bar.wav", "foo.wav," or "foo"
        @type soundName: string
        @param musicOrSoundDict: one of SpeakEasyServer.soundNameToFileDict and SpeakEasyServer.musicNameToFileDict, which
                                 are assumed to be initialized to map short versions of file names (e.g. 'foo' and 'foo.wav')
                                 to full paths.
        @type musicOrSoundDict: {string : string}
        @raise ValueError: if file is not known. 
        '''
        # Already an absolute path?
        if os.path.isabs(soundName):
            return soundName;
        
        # No: is the non-absolute name known as is?
        baseName = os.path.basename(soundName);
        try:
            return musicOrSoundDict[baseName];
        except KeyError:
            pass
        
        # Final possibility: is the name known once the
        # extension is removed?
        baseNameNoExt = os.path.splitext(baseName)[0];
        try:
            return musicOrSoundDict[baseNameNoExt]
        except KeyError:
            raise ValueError("Sound or music name '%s' is unknown on this SpeakEasy node." % soundName);
            
        

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
        self.playhead_pub = rospy.Publisher("/speakeasy_playhead", SpeakEasyPlayhead);
        self.stopped     = False;
                    
    def run(self):
        lastStatusPublish = rospy.Time.now();
        while not self.stopped and not rospy.is_shutdown():
            # Time to publish a general status message?
            if (rospy.Time.now() - lastStatusPublish).secs >= SpeakEasyServer.STATUS_PUBLICATION_PERIOD:
                    try:
                        self.statusMsgSendMethod();
                    except:
                        rospy.logerr(str(sys.exc_info()[1]));
                    lastStatusPublish = rospy.Time.now();
            
            while (self.musicPlayer.getPlayStatus() == PlayStatus.PLAYING) and not self.stopped and not rospy.is_shutdown():
                self.playheadMsg.playhead_time = self.musicPlayer.getPlayheadPosition();
                try:
                    self.playhead_pub.publish(self.playheadMsg);
                except:
                    rospy.logerr(str(sys.exc_info()[1]));
                rospy.sleep(SpeakEasyServer.PLAYHEAD_PUBLICATION_PERIOD);
                # Time to publish a general status message?
                if (rospy.Time.now() - lastStatusPublish).secs >= SpeakEasyServer.STATUS_PUBLICATION_PERIOD:
                    try:
                        self.statusMsgSendMethod();
                    except:
                        rospy.logerr(str(sys.exc_info()[1]));
                    lastStatusPublish = rospy.Time.now();
                    
    def stop(self):
        self.stopped = True;
            
if __name__ == '__main__':
    speakEasyServer = SpeakEasyServer();
    rospy.loginfo("SpeakEasy ROS node is ready...")
    while not rospy.is_shutdown():
        rospy.spin();
    if speakEasyServer.playheadPublisherThread is not None:
        speakEasyServer.playheadPublisherThread.stop();
    rospy.loginfo("Exiting ROS node SpeakEasy...")
    sys.exit();
    
