#!/usr/bin/env python

import threading;
import unittest;

from msg import SpeakEasyStatus;
from msg import SpeakEasyMusic, SpeakEasySound, SpeakEasyPlayhead, SpeakEasyTextToSpeech;

from music_player import TimeReference, PlayStatus; 

# Try importing ROS related modules. Remember whether
# that worked.
try:
    import roslib; roslib.load_manifest('speakeasy');
    import rospy
    ROS_IMPORT_OK = True;
except ImportError:
    # Ros not installed on this machine; run locally:
    ROS_IMPORT_OK = False;


from utilities.speakeasy_utils import SpeakeasyUtils 


class TTSCommands:
    SAY  = 0;
    STOP = 1;
    
class MusicCommands:
    PLAY = 0
    STOP = 1
    PAUSE = 2
    UNPAUSE = 3
    SET_VOL = 4
    SET_PLAYHEAD = 5

class SoundCommands:
    PLAY=0
    STOP=1
    PAUSE=2
    UNPAUSE=3
    SET_VOL=4
 

class RoboComm(object):

    # Timeout for awaiting SpeakEasy status message:
    STATUS_MSG_TIMEOUT = 5; # seconds

    def __init__(self):
        '''
        Initialize sound to play at the robot. Initializes
        self.sound_file_names to a list of sound file names
        for use with play-sound calls to the Robot via ROS.
        @raise NotImplementedError: if ROS initialization failed.
        @raise IOError: if SpeakEasy node is online, but service call to it failed. 
        '''
        
        if not ROS_IMPORT_OK:
            raise NotImplementedError("ROS is not installed on this machine.");
        
        self.rosSpeakEasyNodeAvailable = False;
        self.latestCapabilitiesReply = None;

        # init_node hangs indefinitely if roscore is not running.
        # Therefore: check for that. If roscore isn't running,

        if not SpeakeasyUtils.processRunning('rosmaster'):
            raise NotImplementedError("The roscore process is not running.");
        
        # We now know that ROS is installed, and that roscore is running.
        
        # Declare us to be a ROS node.
        # Allow multiple GUIs to run simultaneously. Therefore
        # the anonymous=True:
        rospy.init_node('speakeasy_remote_gui', anonymous=True);
    
        # Wait for Ros SpeakEasy service for a limited time; there might be none:
        waitTime = 4; # seconds
        try:
            SpeakeasyUtils.waitForRosNode('speakeasy', 
                                          timeout=waitTime, 
                                          waitMessage='Wait for sound and speech capabilities service: ', 
                                          provideTimeInfo=True)
        except NotImplementedError:
            rospy.loginfo("Speech/sound/music capabilities service is offline.");
            self.rosSpeakEasyNodeAvailable = False;    
            raise NotImplementedError("Speech capabilities service is offline.");
                
        rospy.loginfo("Speech capabilities service online.");
        
        # Publishers of requests for sound effects, music, and text-to-speech:
        self.rosSoundRequestor = rospy.Publisher('speakeasy_sound_req', SpeakEasySound);
        self.rosMusicRequestor = rospy.Publisher('speakeasy_music_req', SpeakEasyMusic);
        self.rosTTSRequestor = rospy.Publisher('speakeasy_text_to_speech_req', SpeakEasyTextToSpeech);
        
        # Prepare request messages for easy reuse:
        self.musicRequestMsg = SpeakEasyMusic();
            
    def robotAvailable(self):
        return self.rosSpeakEasyNodeAvailable
        
    def getSpeakEasyNodeStatus(self, cachedOK=False):
        if cachedOK and self.latestCapabilitiesReply is not None:
            return self.latestCapabilitiesReply;
        try:
            self.latestCapabilitiesReply = rospy.wait_for_message('/speakeasy_status', SpeakEasyStatus, self.STATUS_MSG_TIMEOUT);
        except rospy.ROSException:
            rospy.loginfo("SpeakEasy status messages not being received.")
            return None;
        return self.latestCapabilitiesReply;

    def getSoundEffectNames(self, cachedOK=False):
        capabilities = self.getSpeakEasyNodeStatus(cachedOK=cachedOK);
        if capabilities is not None:
            return capabilities.sounds;
        else:
            return [];
        
    def getSongNames(self, cachedOK=False):
        capabilities = self.getSpeakEasyNodeStatus(cachedOK=cachedOK);
        if capabilities is not None:
            return capabilities.music;
        else:
            return [];
    
    def getTextToSpeechEngineNames(self,cachedOK=False):
        capabilities = self.getSpeakEasyNodeStatus(cachedOK=cachedOK);
        if capabilities is not None:
            return capabilities.ttsEngines;
        else:
            return [];
    
    def getVoiceNames(self, ttsEngine=None, cachedOK=False):
        capabilities = self.getSpeakEasyNodeStatus(cachedOK=cachedOK);
        if capabilities is not None:
            ttsVoicesEnums = capabilities.voices;
            if ttsVoicesEnums == None:
                return [];
            
            voices = [];
            for ttsEngineVoiceEnum in ttsVoicesEnums:
                if ttsEngine is None or ttsEngineVoiceEnum.ttsEngine == ttsEngine:
                    voices.extend(ttsEngineVoiceEnum.voices);
            return voices;
        else:
            return [];
    
    def getNumSoundChannels(self,cachedOK=False):
        capabilities = self.getSpeakEasyNodeStatus(cachedOK=cachedOK);
        if capabilities is not None:
            return capabilities.numSoundChannels;
        else:
            return [];
    
    def getSoundVolume(self,cachedOK=False):
        capabilities = self.getSpeakEasyNodeStatus(cachedOK=cachedOK);
        if capabilities is not None:
            return capabilities.soundVolume;
        else:
            return [];
    
    def getMusicVolume(self,cachedOK=False):
        capabilities = self.getSpeakEasyNodeStatus(cachedOK=cachedOK);
        if capabilities is not None:
            return capabilities.musicVolume;
        else:
            return [];
    
    def getPlayStatus(self,cachedOK=False):
        capabilities = self.getSpeakEasyNodeStatus(cachedOK=cachedOK);
        if capabilities is not None:
            return capabilities.musicStatus;
        else:
            return [];

    def getMusicBusy(self):
        return self.getPlayStatus() != PlayStatus.STOPPED;
    
    def getTextToSpeechBusy(self, cachedOK=False):
        capabilities = self.getSpeakEasyNodeStatus(cachedOK=cachedOK);
        if capabilities is not None:
            return capabilities.textToSpeechBusy;
        else:
            return False;
        
    #--------------------------------  Text to Speech -----------------------------
    
    def say(self, text, voice=None, ttsEngine=None, numRepeats=0, repeatPeriod=0, blockTillDone=False):
        '''
		# Command Codes:
		int8 SAY = 0
		int8 STOP= 1
		
		# Parameters:
		int8 command  	    # One of the above Command codes
		string text		    # mandatory
		string voiceName    # optional. Use default voice if unspecified
		string engineName   # optional. Use default engine if unspecified
        '''
        
        if not SpeakeasyUtils.ensureType(numRepeats, int):
            raise TypeError("Number of repeats must be an integer. Was " + str(numRepeats));
        if not SpeakeasyUtils.ensureType(repeatPeriod, int) and not SpeakeasyUtils.ensureType(repeatPeriod, float):
            raise TypeError("Repeat period must be an integer or a float. Was " + str(repeatPeriod));

        ttsRequestMsg = SpeakEasyTextToSpeech();
        ttsRequestMsg.command = TTSCommands.SAY;
        ttsRequestMsg.text    = text;
        if voice is not None:
            ttsRequestMsg.voiceName = voice;
        else:
            ttsRequestMsg.voiceName = '';
        if ttsEngine is not None:
            ttsRequestMsg.engineName = ttsEngine;
        else:
            ttsRequestMsg.engineName = '';
            
        self.rosTTSRequestor.publish(ttsRequestMsg);
        if numRepeats > 0:
            RoboComm.SpeechReplayDemon(text, voice, ttsEngine, numRepeats, repeatPeriod, self).start();
            
        if blockTillDone:
            while self.getTextToSpeechBusy():
                rospy.sleep(0.5);
        
    def stopSaying(self):
        self.ttsRequestMsg.command = TTSCommands.STOP;
        self.rosTTSRequestor.publish(self.ttsRequestMsg);
    
    
    # --------------------------------------------   Sound Effects  -------------------------------
    
    def playSound(self, soundName, volume=None, numRepeats=0, repeatPeriod=0):
        
        if not SpeakeasyUtils.ensureType(numRepeats, int):
            raise TypeError("Number of repeats must be an integer. Was " + str(numRepeats));
        if not SpeakeasyUtils.ensureType(repeatPeriod, int) and not SpeakeasyUtils.ensureType(repeatPeriod, float):
            raise TypeError("Repeat period must be an integer or a float. Was " + str(repeatPeriod));
        
        soundReqMsg = SpeakEasySound();
        soundReqMsg.command = SoundCommands.PLAY;
        soundReqMsg.sound_name = soundName;
        if volume is None:
            soundReqMsg.volume = -1.0;
        else:
            soundReqMsg.volume = volume;

        self.rosSoundRequestor.publish(soundReqMsg);
        if numRepeats > 0:
            RoboComm.SoundReplayDemon(soundName, numRepeats, repeatPeriod, self).start();

    def stopSound(self):
        soundReqMsg = SpeakEasySound();
        soundReqMsg.command = SoundCommands.STOP;
        self.rosSoundRequestor.publish(soundReqMsg);
        
    def pauseSound(self):
        soundReqMsg = SpeakEasySound();
        soundReqMsg.command = SoundCommands.PAUSE;
        self.rosSoundRequestor.publish(soundReqMsg);
        
    def unPauseSound(self):
        soundReqMsg = SpeakEasySound();
        soundReqMsg.command = SoundCommands.UNPAUSE;
        self.rosSoundRequestor.publish(soundReqMsg);
        
    def setSoundVolume(self, vol):
        if not SpeakeasyUtils.ensureType(vol, float):
            raise TypeError("Volume must be a float between 0.0 and 1.0. Was " + str(vol));
        
        soundReqMsg = SpeakEasySound();
        soundReqMsg.command = SoundCommands.SET_VOL;
        soundReqMsg.volume = vol;
        self.rosSoundRequestor.publish(soundReqMsg);

    # --------------------------------------------   Music Streaming  -------------------------------
    
    def playMusic(self, songName, volume=None, playheadTime=0.0, timeReference=TimeReference.ABSOLUTE, numRepeats=0, repeatPeriod=0):
        
        if not SpeakeasyUtils.ensureType(numRepeats, int):
            raise TypeError("Number of repeats must be an integer. Was " + str(numRepeats));
        if not SpeakeasyUtils.ensureType(repeatPeriod, int) and not SpeakeasyUtils.ensureType(repeatPeriod, float):
            raise TypeError("Repeat period must be an integer or a float. Was " + str(repeatPeriod));
        if not SpeakeasyUtils.ensureType(playheadTime, int) and not SpeakeasyUtils.ensureType(playheadTime, float):
            raise TypeError("Playhead must be an integer or a float. Was " + str(playheadTime));
        if (timeReference != TimeReference.ABSOLUTE) and (timeReference != TimeReference.RELATIVE):
            raise TypeError("Time reference must be TimeReference.RELATIVE, or TimeReference.ABSOLUTE. Was " + str(timeReference));
        
        musicReqMsg = SpeakEasyMusic();
        musicReqMsg.command = MusicCommands.PLAY;
        musicReqMsg.song_name = songName;
        musicReqMsg.time = playheadTime;
        musicReqMsg.timeReference = timeReference;
        if volume is None:
            musicReqMsg.volume = -1.0;
        else:
            musicReqMsg.volume = volume;

        self.rosMusicRequestor.publish(musicReqMsg);
        if numRepeats > 0:
            RoboComm.MusicReplayDemon(songName, volume, playheadTime, timeReference, numRepeats, repeatPeriod, self).start();

    def stopMusic(self):
        musicReqMsg = SpeakEasyMusic();
        musicReqMsg.command = MusicCommands.STOP;
        self.rosMusicRequestor.publish(musicReqMsg);
        
    def pauseSound(self):
        musicReqMsg = SpeakEasyMusic();
        musicReqMsg.command = MusicCommands.PAUSE;
        self.rosMusicRequestor.publish(musicReqMsg);
        
    def unPauseSound(self):
        musicReqMsg = SpeakEasyMusic();
        musicReqMsg.command = MusicCommands.UNPAUSE;
        self.rosMusicRequestor.publish(musicReqMsg);
        
    def setSoundVolume(self, vol):
        if not SpeakeasyUtils.ensureType(vol, float):
            raise TypeError("Volume must be a float between 0.0 and 1.0. Was " + str(vol));
        
        musicReqMsg = SpeakEasyMusic();
        musicReqMsg.command = MusicCommands.SET_VOL;
        musicReqMsg.volume = vol;
        self.rosSoundRequestor.publish(musicReqMsg);

    def setPlayhead(self, playheadTime):
        if not SpeakeasyUtils.ensureType(playheadTime, int) and not SpeakeasyUtils.ensureType(playheadTime, float): 
            raise TypeError("Playhead must be an int or float indicating seconds. Was " + str(playheadTime));
        
        musicReqMsg = SpeakEasyMusic();
        musicReqMsg.command = MusicCommands.SET_PLAYHEAD;
        musicReqMsg.time = playheadTime;
        self.rosSoundRequestor.publish(musicReqMsg);
    
    # --------------------------------------------   Replay Demons -------------------------------
    
    class ReplayDemon(threading.Thread):
        
        def __init__(self, repeatPeriod):
            super(RoboComm.ReplayDemon, self).__init__();
            self.repeatPeriod = repeatPeriod;
            self.stopped = True;
            
    class SoundReplayDemon(ReplayDemon):
        
        def __init__(self, soundName, roboComm, volume=None, repeatPeriod=0, numRepeats=0):
            super(RoboComm.SoundReplayDemon, self).__init__(repeatPeriod);
            self.soundName = soundName;
            self.volume = volume;
            self.numRepeats = numRepeats;
            self.roboComm = roboComm;
    
        def run(self):
            self.stopped = False;
            while not self.stopped and (self.numRepeats > 0):
                rospy.sleep(self.repeatPeriod);
                self.roboComm.playSound(self.soundName, volume=self.volume)
                self.numRepeats -= 1;
        
        def stop(self):
            self.stopped = True;
            self.roboComm.stopSound();

    class MusicReplayDemon(ReplayDemon):
        
        def __init__(self, songName, roboComm, volume=None, playhead=0.0, timeReference=TimeReference.ABSOLUTE, numRepeats=0, repeatPeriod=0):
            super(RoboComm.SoundReplayDemon, self).__init__(repeatPeriod);
            self.songName = songName
            self.roboComm = roboComm
            self.volume = volume
            self.playhead = playhead
            self.timeReference = timeReference
            self.numRepeats = numRepeats
            self.repeatPeriod = repeatPeriod
    
        def run(self):
            self.stopped = False;
            while not self.stopped and (self.numRepeats > 0):
                rospy.sleep(self.repeatPeriod);
                self.roboComm.playMusic(self.songName,
                                        volume=self.volume, 
                                        playhead=self.playhead, 
                                        timeReference=self.timeReference); 
                self.numRepeats -= 1;
        
        def stop(self):
            self.stopped = True;
            self.roboComm.stopMusic();

            
    class SpeechReplayDemon(ReplayDemon):
            
        def __init__(self, text, voiceName, ttsEngine, numRepeats, repeatPeriod, roboComm):
            super(RoboComm.SpeechReplayDemon, self).__init__(repeatPeriod);
            self.text = text;
            self.voiceName = voiceName;
            self.ttsEngine = ttsEngine;
            self.numRepeats = numRepeats;
            self.roboComm = roboComm;
            
        def run(self):
            self.stopped = False;
            while self.roboComm.getTextToSpeechBusy():
                rospy.sleep(0.5);
            while not self.stopped and (self.numRepeats > 0):
                rospy.sleep(self.repeatPeriod);
                self.roboComm.say(self.text, voice=self.voiceName, ttsEngine=self.ttsEngine, blockTillDone=True);
                self.numRepeats -= 1;
        
        def stop(self):
            self.stopped = True;
            self.roboComm.stopSaying();
    
     
#---------------------------------------- Testing -------------------------------
    
class TestGroup:
    STATUS_MSGS = 0;
    TTS = 1;
    SOUND = 2;
    MUSIC = 3;
    PLAYHEAD = 4;
    
groupsToTest = [TestGroup.STATUS_MSGS];
#groupsToTest = [TestGroup.TTS];
    
class TestRobotInteraction(unittest.TestCase):
    
    def setUp(self):
        unittest.TestCase.setUp(self);
        try: 
            self.roboComm = RoboComm();
        except Exception as e:
            print "ROS SpeakEasy service not available. No tests run: %s" %e
            import sys;
            sys.exit();
            
    def tearDown(self):
        pass;

    unittest.skipUnless(TestGroup.STATUS_MSGS in groupsToTest, 'Not testing for status message processing.')
    def testVolumeGetting(self):
        import math;
        self.assertEqual(math.ceil(self.roboComm.getMusicVolume()), 1.0, "Get music volume failed.");
        self.assertEqual(math.ceil(self.roboComm.getSoundVolume()), 1.0, "Get music volume failed.");

    unittest.skipUnless(TestGroup.STATUS_MSGS in groupsToTest, 'Not testing for status message processing.')
    def testPlayStatus(self):
        self.assertEqual(self.roboComm.getPlayStatus(), 0, "Play status is not 'stopped'");

    unittest.skipUnless(TestGroup.STATUS_MSGS in groupsToTest, 'Not testing for status message processing.')
    def testNumChannels(self):
        self.assertEqual(self.roboComm.getNumSoundChannels(cachedOK=True), 8, "Wrong number of sound channels.");

    unittest.skipUnless(TestGroup.STATUS_MSGS in groupsToTest, 'Not testing for status message processing.')
    def testVoiceAndTtsEngineNames(self):
        self.assertIn('voice_kal_diphone', self.roboComm.getVoiceNames('festival',cachedOK=True), "Festival engine not reported to contain 'voice_kal_diphone'");
        self.assertIn('festival', self.roboComm.getTextToSpeechEngineNames(), "Festival engine not reported as present.");
    
    unittest.skipUnless(TestGroup.STATUS_MSGS in groupsToTest, 'Not testing for status message processing.')    
    def testSongAndSoundNames(self):
        self.assertIn('cottonFields.ogg', self.roboComm.getSongNames(), "Cottonfields not reported as present in available music.");
        self.assertIn('drill.wav', self.roboComm.getSoundEffectNames(), "Drill.wav not reported as present in sounds.");
    
    unittest.skipUnless(TestGroup.TTS in groupsToTest, 'Not testing for text-to-speech processing.')    
    def testSay(self):
        self.roboComm.say("This is a test.");
        rospy.spin();
      
if __name__ == '__main__':
    #unittest.main();
    roboComm = RoboComm();
    #roboComm.say("Testing", ttsEngine='festival', numRepeats=2, blockTillDone=False);
    #roboComm.playSound("drill", numRepeats=2, repeatPeriod=4);
    #roboComm.playMusic("cottonFields");
    roboComm.stopMusic();
    rospy.sleep(5);
#    while not rospy.is_shutdown():
#        rospy.spin();
