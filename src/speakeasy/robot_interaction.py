#!/usr/bin/env python

import threading;
import unittest;

from speakeasy.msg import SpeakEasyStatus;
from speakeasy.msg import SpeakEasyMusic, SpeakEasySound, SpeakEasyPlayhead, SpeakEasyTextToSpeech;

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
    '''
    Provides all methods required to operate the SpeakEasy ROS node remotely.
    Intention is for GUIs or other programs to use one instance of this class
    to control all text-to-speech, sound effects, and music play functions if
    those functions are provided by a SpeakEasy ROS node. 
    <p>
    For an exmple to use if these three functions are provided locally, without 
    using ROS, see speakeasy_controller.py.  
    '''

    # Timeout for awaiting SpeakEasy status message:
    STATUS_MSG_TIMEOUT = 5; # seconds
    
    # Timeout for awaiting SpeakEasy playhead message.
    # Note: these messages are only sent while music is
    #       playing. At that point the transmission period
    #       is speakeasy_node.PLAYHEAD_PUBLICATION_PERIOD:
    
    PLAYHEAD_MSG_TIMEOUT = 0.5; # seconds
    

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
        
        self.nodeStatusLock   = threading.Lock();
        self.textToSpeechLock = threading.Lock();
        self.musicLock        = threading.Lock();
        self.soundLock        = threading.Lock();
        
        # Place to remember threads that repeat voice/sound/music.
        # These lists are used when stopping those threads:
        self.speechThreads = [];
        self.soundThreads  = [];
        self.musicThreads  = [];
        

        # init_node hangs indefinitely if roscore is not running.
        # Therefore: check for that. If roscore isn't running,

        if not SpeakeasyUtils.processRunning('rosmaster'):
            raise NotImplementedError("The roscore process is not running.");
        
        # We now know that ROS is installed, and that roscore is running.
        
        # Publishers of requests for sound effects, music, and text-to-speech:
        self.rosSoundRequestor = rospy.Publisher('speakeasy_sound_req', SpeakEasySound);
        self.rosMusicRequestor = rospy.Publisher('speakeasy_music_req', SpeakEasyMusic);
        self.rosTTSRequestor = rospy.Publisher('speakeasy_text_to_speech_req', SpeakEasyTextToSpeech);
        
        # Declare us to be a ROS node.
        # Allow multiple GUIs to run simultaneously. Therefore
        # the anonymous=True:
        nodeInfo = rospy.init_node('speakeasy_remote_gui', anonymous=True);
        # Don't know why, but without a bit of delay after this init, the first
        # published message will not be transmitted, no matter what the message type:
        rospy.sleep(1.0);
    
        # Wait for Ros SpeakEasy service for a limited time; there might be none:
        waitTime = 4; # seconds
        secsWaited = 0;
        while not self.robotAvailable() and (secsWaited < waitTime):
            rospy.loginfo("No SpeakEasy node available. Keep checking for %d more second(s)..." % (waitTime - secsWaited));
            rospy.sleep(1);
            secsWaited += 1;
        if secsWaited >= waitTime:
            rospy.logerr("Speech/sound/music capabilities service is offline.");
            self.rosSpeakEasyNodeAvailable = False;    
            raise NotImplementedError("Speech capabilities service is offline.");
                
        rospy.loginfo("Speech capabilities service online.");
        
    def robotAvailable(self):
        '''
        Return True if a SpeakEasy service is available, i.e. if a SpeakEasy ROS
        node is running. Else return False.
        @return: Result of checking whether a live SpeakEasy node is detected.
        @rtype: bool 
        '''
        if self.getSpeakEasyNodeStatus() is None:
            return False;
        else:
            return True;
        
    def getSpeakEasyNodeStatus(self, cachedOK=False):
        '''
        Return a SpeakEasy status message instance as received from a live
        SpeakEasy node. See msg.SpeakEasyStatus.msg for field details.
        @param cachedOK: Set True if OK to use a previously cached status message.
        @type cachedOK: bool
        @return: Message instance if available, else None. Message instances are unavailable if no SpeakEasy node is running.
        @rtype: SpeakEasyStatus
        '''
        with self.nodeStatusLock:
            if cachedOK and self.latestCapabilitiesReply is not None:
                return self.latestCapabilitiesReply;
            try:
                self.latestCapabilitiesReply = rospy.wait_for_message('/speakeasy_status', SpeakEasyStatus, self.STATUS_MSG_TIMEOUT);
            except rospy.ROSException:
                rospy.loginfo("SpeakEasy status messages not being received.")
                return None;
            return self.latestCapabilitiesReply;

    def getSoundEffectNames(self, cachedOK=False):
        '''
        Return a list of the sound effects that the SpeakEasy node is offering.
        These effects may be invoked via the playSound() method.
        @param cachedOK: Set True if OK to use a previously cached status message.
        @type cachedOK: bool
        @return: Array of sound effect names.
        @rtype: [string] 
        '''
        capabilities = self.getSpeakEasyNodeStatus(cachedOK=cachedOK);
        if capabilities is not None:
            return capabilities.sounds;
        else:
            return [];
        
    def getSongNames(self, cachedOK=False):
        '''
        Return a list of the music files that the SpeakEasy node is offering.
        These songs may be played via the playMusic() method.
        
        @param cachedOK: Set True if OK to use a previously cached status message.
        @type cachedOK: bool
        @return: Array of music files.
        @rtype: [bool]
        '''
        capabilities = self.getSpeakEasyNodeStatus(cachedOK=cachedOK);
        if capabilities is not None:
            return capabilities.music;
        else:
            return [];
    
    def getTextToSpeechEngineNames(self,cachedOK=False):
        '''
        Return array of text-to-speech engines that are available at the SpeakEasy node.
        Examples: Festival, the Linux open source engine, or Cepstral, the commercial engine.
        @param cachedOK: Set True if OK to use a previously cached status message.
        @type cachedOK: bool
        @return: List of engine names.
        @rtype: [string]
        '''
        capabilities = self.getSpeakEasyNodeStatus(cachedOK=cachedOK);
        if capabilities is not None:
            return capabilities.ttsEngines;
        else:
            return [];
    
    def getVoiceNames(self, ttsEngine=None, cachedOK=False):
        '''
        Return a list of text-to-speech voices that the SpeakEasy node provides.
        If a particular text-to-speech engine is specified, only that engine's 
        voices are returned, else all voices of all available engines are
        returned in one list.
        @param ttsEngine: Name of one available text-to-speech engine.
        @type ttsEngine: string
        @param cachedOK: Set True if OK to use a previously cached status message.
        @type cachedOK: bool
        @return: List of voice names, or empty list. List may be empty either because
                 the SpeakEasy node does not offer text-to-speech services, or because
                 the node has terminated, and is no longer broadcasting status messages.
        @rtype: [string]
        '''
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
        '''
        Return the number of simultaneous sound effects that the SpeakEasy node provides.
        Note that SpeakEasy nodes can be configured to provide many sound effect channels,
        though this API does not provide such control. Default is eight.
        @param cachedOK: Set True if OK to use a previously cached status message.
        @type cachedOK: bool
        @return: Number of sound effect channels.
        @rtype: int
        '''
        capabilities = self.getSpeakEasyNodeStatus(cachedOK=cachedOK);
        if capabilities is not None:
            return capabilities.numSoundChannels;
        else:
            return [];
    
    def getSoundVolume(self,cachedOK=False):
        '''
        Return default sound effect volume level. Note that sound volume can be set
        on a case-by-case basis in calls to playSound();
        @param cachedOK: Set True if OK to use a previously cached status message.
        @type cachedOK: bool
        @return: Default sound volume as a number between 0.0 and 1.0.
        @rtype: float
        '''
        capabilities = self.getSpeakEasyNodeStatus(cachedOK=cachedOK);
        if capabilities is not None:
            return capabilities.soundVolume;
        else:
            return [];
    
    def getMusicVolume(self,cachedOK=False):
        '''
        Return default music volume level. Note that music volume can be set
        on a case-by-case basis in calls to playMusic();
        @param cachedOK: Set True if OK to use a previously cached status message.
        @type cachedOK: bool
        @return: Default sound volume as a number between 0.0 and 1.0.
        @rtype: float
        '''
        capabilities = self.getSpeakEasyNodeStatus(cachedOK=cachedOK);
        if capabilities is not None:
            return capabilities.musicVolume;
        else:
            return [];
    
    def getPlayStatus(self,cachedOK=False):
        '''
        Return current music play state. These states are defined in music_player.py.
        For reference, at the time of this writing:
            class PlayStatus:
				STOPPED = 0;
				PAUSED  = 1;
				PLAYING = 2;
        @param cachedOK: Set True if OK to use a previously cached status message.
        @type cachedOK: bool
        @return: Status of the music playback engine on the SpeakEasy node.
        @rtype: PlayStatus
        '''
        capabilities = self.getSpeakEasyNodeStatus(cachedOK=cachedOK);
        if capabilities is not None:
            return capabilities.musicStatus;
        else:
            return [];

    def getMusicBusy(self):
        '''
        Convenience method to determine whether music is currently either
        playing or paused.
        @return: True/False to indicate whether music status is currently PlayStatus.STOPPED.
        @rtype: bool
        '''
        return self.getPlayStatus() != PlayStatus.STOPPED;
    
    def getTextToSpeechBusy(self, cachedOK=False):
        '''
        Return whether SpeakEasy node is currently generating speech from text.
        @return: True/False to indicate whether text-to-speech engine is busy.
        @rtype: bool
        '''
        capabilities = self.getSpeakEasyNodeStatus(cachedOK=cachedOK);
        if capabilities is not None:
            return capabilities.textToSpeechBusy;
        else:
            return False;
        
    #--------------------------------  Text to Speech -----------------------------
    
    def say(self, text, voice=None, ttsEngine=None, numRepeats=0, repeatPeriod=0, blockTillDone=False):
        '''
        Given a piece of text, generate corresponding speech at the SpeakEasy node site.
        @param text: Words to speak.
        @type text: string
        @param voice: Name of text-to-speech voice to use.
        @type voice: string
        @param ttsEngine: Name of text-to-speech engine to use.
        @type ttsEngine: string
        @param numRepeats: Number of times the utterance is to be repeated after the first time. Use -1 if forever.
        @type numRepeats: int
        @param repeatPeriod: Time period in fractional seconds to wait between repeated utterances.
        @type repeatPeriod: float
        @param blockTillDone: Request to return immediately, or block until the utterance if finished.
        @type blockTillDone: bool
        @raises TypeError: if one of the parameters is of incorrect type.
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
            
        with self.textToSpeechLock:        
            self.rosTTSRequestor.publish(ttsRequestMsg);
            
        # Keep this block out of the lock! Thread registration will 
        # acquire the lock (thus leading to deadlock, if lock is owned here):
        if numRepeats > 0 or numRepeats == -1:
            speechRepeatThread = RoboComm.SpeechReplayDemon(text, voice, ttsEngine, numRepeats, repeatPeriod, self);
            self.registerSpeechRepeaterThread(speechRepeatThread); 
            speechRepeatThread.start();
            
        if blockTillDone:
            while self.getTextToSpeechBusy():
                rospy.sleep(0.5);
        
    def stopSaying(self):
        '''
        Stop text-to-speech utterance. No effect if text-to-speech is currently inactive.
        '''
        ttsRequestMsg = SpeakEasyTextToSpeech();
        ttsRequestMsg.command = TTSCommands.STOP;
        with self.textToSpeechLock:
            self.rosTTSRequestor.publish(ttsRequestMsg);

        # Keep this following statement out of the lock! Thread unregistration will 
        # acquire the lock (thus leading to deadlock, if lock is owned here):
        self.killSpeechRepeatThreads(self.speechThreads);
    
    # --------------------------------------------   Sound Effects  -------------------------------
    
    def playSound(self, soundName, volume=None, numRepeats=0, repeatPeriod=0):
        '''
        Play a sound effect at the SpeakEasy node.
        @param soundName: Name of the sound effect. (see getSoundEffectNames()).
        @type soundName: string
        @param volume: Loudness for the sound effect. If None, current volume is used. Volume must be in range 0.0 to 1.0
        @type volume: float.
        @param numRepeats: Number of times the sound effect is to be repeated after the first time.  Use -1 to play forever.
        @type numRepeats: int
        @param repeatPeriod: Time period in fractional seconds to wait between repeats.
        @type repeatPeriod: float
        @raise TypeError: if any parameter is of the wrong type. 
        '''
        
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

        with self.soundLock:
            self.rosSoundRequestor.publish(soundReqMsg);
            
        # Keep this block out of the lock! Thread registration will 
        # acquire the lock (thus leading to deadlock, if lock is owned here):
        if numRepeats > 0 or numRepeats == -1:
            soundRepeatThread = RoboComm.SoundReplayDemon(soundName, self, volume=volume, numRepeats=numRepeats, repeatPeriod=repeatPeriod);
            self.registerSoundRepeaterThread(soundRepeatThread) 
            soundRepeatThread.start();

    def stopSound(self):
        '''
        Stop the all currently playing sound effects. Method has no effect if no sound is currently playing.
        '''
        soundReqMsg = SpeakEasySound();
        soundReqMsg.command = SoundCommands.STOP;
        with self.soundLock:        
            self.rosSoundRequestor.publish(soundReqMsg);
            
        # Keep this following statement out of the lock! Thread unregistration will 
        # acquire the lock (thus leading to deadlock, if lock is owned here):
        self.killSoundRepeatThreads(soundThreads);
        
    def pauseSound(self):
        '''
        Pause the all currently playing sound effects. Method has no effect if no sound is currently playing.
        '''
        soundReqMsg = SpeakEasySound();
        soundReqMsg.command = SoundCommands.PAUSE;
        with self.soundLock:        
            self.rosSoundRequestor.publish(soundReqMsg);
        
    def unPauseSound(self):
        '''
        Un-pause all currently paused sound effects. Method has no effect if no sound is currently paused.
        '''
        soundReqMsg = SpeakEasySound();
        soundReqMsg.command = SoundCommands.UNPAUSE;
        with self.soundLock:        
            self.rosSoundRequestor.publish(soundReqMsg);
        
    def setSoundVolume(self, volume, soundName=None):
        '''
        Change the sound effect default volume.
        @param volume: New default volume for sound effects. Value must be in range 0.0 to 1.0.
        @type volume: float
        @param soundName: Optionally the name of the sound whose volume is to be changed.
        @type soundName: string
        @raises TypeError: if any parameters are of incorrect type.
        '''
        if not SpeakeasyUtils.ensureType(volume, float):
            raise TypeError("Volume must be a float between 0.0 and 1.0. Was " + str(volume));
        
        soundReqMsg = SpeakEasySound();
        soundReqMsg.command = SoundCommands.SET_VOL;
        if soundName is None:
            soundName = '';
        soundReqMsg.sound_name = soundName;
        soundReqMsg.volume = volume;
        with self.soundLock:
            self.rosSoundRequestor.publish(soundReqMsg);

    # --------------------------------------------   Music Streaming  -------------------------------
    
    def playMusic(self, songName, volume=None, playheadTime=0.0, timeReference=TimeReference.ABSOLUTE, numRepeats=0, repeatPeriod=0):
        '''
        Play a piece of music (a sound file) at the SpeakEasy node.
        @param songName: Name of sound file. (See getSongNames()). 
        @type songName: string
        @param volume: Loudness at which to play. Default uses current volume. Else must be in range 0.0 to 1.0
        @type volume: {None | float}
        @param playheadTime: Offset in fractional seconds into where to start the song. Default: start at beginning. 
        @type playheadTime: float
        @param timeReference: If playheadTime is provided, specifies whether the given time is intended as absolute (relative to the
                              beginning of the song), or relative to the current playhead position.
        @type timeReference: TimeReference
        @param numRepeats: Number of times the song is to be repeated after the first time. Use -1 to play forever.
        @type numRepeats: int
        @param repeatPeriod: Time period in fractional seconds to wait between repeats.
        @type repeatPeriod: float
        @raise TypeError: if any parameters are of incorrect type. 
        '''
        
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

        with self.musicLock:
            self.rosMusicRequestor.publish(musicReqMsg);
            
        # Keep this block out of the lock! Thread registration will 
        # acquire the lock (thus leading to deadlock, if lock is owned here):
        if numRepeats > 0 or numRepeats == -1:
            musicRepeatThread = RoboComm.MusicReplayDemon(songName, volume, playheadTime, timeReference, numRepeats, repeatPeriod, self);
            self.registerMusicRepeaterThread(musicRepeatThread);
            musicRepeatThread.start();

    def stopMusic(self):
        '''
        Stop currently playing music. No effect if nothing playing.
        '''
        musicReqMsg = SpeakEasyMusic();
        musicReqMsg.command = MusicCommands.STOP;
        with self.musicLock:
            self.rosMusicRequestor.publish(musicReqMsg);

        # Keep this following statement out of the lock! Thread unregistration will 
        # acquire the lock (thus leading to deadlock, if lock is owned here):
        self.killMusicRepeatThreads(self.musicThreads);
        
    def pauseMusic(self):
        '''
        Pause currently playing music. No effect if nothing playing.
        '''
        musicReqMsg = SpeakEasyMusic();
        musicReqMsg.command = MusicCommands.PAUSE;
        with self.musicLock:
            self.rosMusicRequestor.publish(musicReqMsg);
        
    def unPauseMusic(self):
        '''
        Un-pause currently playing music. No effect if nothing paused or playing.
        '''
        musicReqMsg = SpeakEasyMusic();
        musicReqMsg.command = MusicCommands.UNPAUSE;
        with self.musicLock:
            self.rosMusicRequestor.publish(musicReqMsg);
        
    def setMusicVolume(self, vol):
        '''
        Set default volume of music playback.
        @param vol: Loudness value between 0.0 and 1.0.
        @type vol: float
        @raise TypeError: if any parameters are of incorrect type. 
        '''
        if not SpeakeasyUtils.ensureType(vol, float):
            raise TypeError("Volume must be a float between 0.0 and 1.0. Was " + str(vol));
        
        musicReqMsg = SpeakEasyMusic();
        musicReqMsg.command = MusicCommands.SET_VOL;
        musicReqMsg.volume = vol;
        with self.musicLock:
            self.rosMusicRequestor.publish(musicReqMsg);

    def setPlayhead(self, playheadTime, timeReference=TimeReference.ABSOLUTE):
        '''
        Change the music playhead position to a particular time within a song.
        Time may be specified absolute (i.e. relative to the start of the song), 
        or relative to the current playhead. The playhead may be changed during
        playback, or while a song is paused.
        @param playheadTime: New time where to position the playhead.
        @type playheadTime: float
        @param timeReference: TimeReference.ABSOLUTE or TimeReference.RELATIVE
        @type timeReference: TimeReference.
        @raise TypeError: if any parameters are of incorrect type. 
        '''
        if not SpeakeasyUtils.ensureType(playheadTime, int) and not SpeakeasyUtils.ensureType(playheadTime, float): 
            raise TypeError("Playhead must be an int or float indicating seconds. Was " + str(playheadTime));
        
        musicReqMsg = SpeakEasyMusic();
        musicReqMsg.command = MusicCommands.SET_PLAYHEAD;
        musicReqMsg.time = playheadTime;
        musicReqMsg.timeReference = timeReference;
        with self.musicLock:
            self.rosMusicRequestor.publish(musicReqMsg);
            
    def getPlayhead(self):
        '''
        Return song position in fractional seconds. Return None
        if no music is currently playing.
        @return: Position in fractional seconds.
        @rtype: float
        '''
        try:
            playHeadMsg = rospy.wait_for_message('/speakeasy_playhead', SpeakEasyPlayhead, self.PLAYHEAD_MSG_TIMEOUT);
            return playHeadMsg.playhead_time;
        except:
            return None;
    
    
    # --------------------------------------------   Repeater Thread Methods -------------------------------
    
    #  NOTE:  Do not call methods in this section with the respective
    #         locks set. You will deadlock. 'Respective' means one
    #         of self.textToSpeechLock, self.soundLock, self.musicLock.
    #         See the methods for which lock is relevant.
    #  These methods are already thread safe.
    
    def registerSpeechRepeaterThread(self, speechThread):
        with self.textToSpeechLock:
            self.speechThreads.append(speechThread);
        
    def killSpeechRepeatThreads(self, speechThreads):
        speechThread.stop();
        
        with self.textToSpeechLock:
            # Copy list for the loop, b/c unregisterRepeatThread() 
            # modifies the pass-in list in place:
            for speechThread in list(speechThreads):
                self.unregisterRepeatThread(speechThread, self.speechThreads);
                 
    def registerSoundRepeaterThread(self, soundThread):
        with self.soundLock:
            self.soundThreads.append(soundThread);
        
    def killSoundRepeatThreads(self, soundThreads):
        soundThread.stop();
        with self.soundLock:
            # Copy list for the loop, b/c unregisterRepeatThread() 
            # modifies the pass-in list in place:
            for soundThread in list(soundThreads):
                self.unregisterRepeatThread(soundThread, self.soundThreads);
    
    def registerMusicRepeaterThread(self, musicThread):
        with self.musicLock:
            self.musicThreads.append(musicThread);
        
    def killMusicRepeatThreads(self, musicThreads):
        musicThread.stop();
        with self.musicLock:
            # Copy list for the loop, b/c unregisterRepeatThread() 
            # modifies the pass-in list in place:
            for musicThread in list(musicThreads):
                self.unregisterRepeatThread(musicThread, self.musicThreads);
            
    def unregisterRepeatThread(self, threadObj, threadList):
        # NOTE: this method is not re-entrant. Ensure this condition in callers:
        try:
            threadList.remove(threadObj);
        except:
            pass;
    
    # --------------------------------------------   Replay Demon Threads -------------------------------
    
    class ReplayDemon(threading.Thread):
        '''
        Abstract class of all repeat demons: text-to-speech, sound effects, and music.
        '''
        
        def __init__(self, repeatPeriod):
            super(RoboComm.ReplayDemon, self).__init__();
            self.repeatPeriod = repeatPeriod;
            self.stopped = True;
            
    class SoundReplayDemon(ReplayDemon):
        '''
        Responsible for repeating sound effects at appropriate intervals. Runs as thread.
        '''
        
        def __init__(self, soundName, roboComm, volume=None, numRepeats=0, repeatPeriod=0):
            super(RoboComm.SoundReplayDemon, self).__init__(repeatPeriod);

            self.soundName = soundName;
            self.volume = volume;
            self.numRepeats = numRepeats;
            if numRepeats == -1:
                self.playForever = True;
            else:
                self.playForever = False;
            self.roboComm = roboComm;
    
        def run(self):
            self.stopped = False;
            while not self.stopped and ((self.numRepeats > 0) or self.playForever):
                rospy.sleep(self.repeatPeriod);
                self.roboComm.playSound(self.soundName, volume=self.volume)
                if not self.playForever:
                    self.numRepeats -= 1;
                    
            with self.roboComm.soundLock:
                self.roboComm.unregisterRepeatThread(self, self.roboComm.soundThreads);
        
        def stop(self):
            self.stopped = True;
            self.roboComm.stopSound();
            with self.soundLock:
                self.roboComm.unregisterRepeatThread(self, self.roboComm.soundThreads);

    class MusicReplayDemon(ReplayDemon):
        '''
        Responsible for repeating songs at appropriate intervals. Runs as thread.
        '''
        
        def __init__(self, songName, roboComm, volume=None, playhead=0.0, timeReference=TimeReference.ABSOLUTE, numRepeats=0, repeatPeriod=0):
            super(RoboComm.SoundReplayDemon, self).__init__(repeatPeriod);
            self.songName = songName
            self.roboComm = roboComm
            self.volume = volume
            self.playhead = playhead
            self.timeReference = timeReference
            self.numRepeats = numRepeats
            if numRepeats == -1:
                self.playForever = True;
            else:
                self.playForever = False;
            
            self.repeatPeriod = repeatPeriod
    
        def run(self):
            self.stopped = False;
            while not self.stopped and ((self.numRepeats > 0) or self.playForever):
                rospy.sleep(self.repeatPeriod);
                self.roboComm.playMusic(self.songName,
                                        volume=self.volume, 
                                        playhead=self.playhead, 
                                        timeReference=self.timeReference); 
                if not self.playForever:
                    self.numRepeats -= 1;

            with self.roboComm.musicLock:
                self.roboComm.unregisterRepeatThread(self, self.roboComm.musicThreads);            
        
        def stop(self):
            self.stopped = True;
            self.roboComm.stopMusic();
            with self.roboComm.musicLock:
                self.roboComm.unregisterRepeatThread(self, self.roboComm.musicThreads);            
            
    class SpeechReplayDemon(ReplayDemon):
        '''
        Responsible for repeating text-to-speech utterances at appropriate intervals. Runs as thread.
        '''
            
        def __init__(self, text, voiceName, ttsEngine, numRepeats, repeatPeriod, roboComm):
            super(RoboComm.SpeechReplayDemon, self).__init__(repeatPeriod);
            self.text = text;
            self.voiceName = voiceName;
            self.ttsEngine = ttsEngine;
            if numRepeats == -1:
                self.playForever = True;
            else:
                self.playForever = False;
            self.numRepeats = numRepeats;
            self.roboComm = roboComm;
            
        def run(self):
            self.stopped = False;
            while self.roboComm.getTextToSpeechBusy():
                rospy.sleep(0.5);
            while not self.stopped and ((self.numRepeats > 0) or self.playForever):
                rospy.sleep(self.repeatPeriod);
                self.roboComm.say(self.text, voice=self.voiceName, ttsEngine=self.ttsEngine, blockTillDone=True);
                if not self.playForever:
                    self.numRepeats -= 1;
                    
            with self.roboComm.textToSpeechLock:
                self.roboComm.unregisterRepeatThread(self, self.roboComm.speechThreads);                     
        
        def stop(self):
            self.stopped = True;
            self.roboComm.stopSaying();
            with self.roboComm.textToSpeechLock:
                self.roboComm.unregisterRepeatThread(self, self.roboComm.speechThreads);                     
     
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
    
    try:
        roboComm = RoboComm();
    except NotImplementedError:
        rospy.logerr("You must start a SpeakEasy service first.");
        import sys
        sys.exit();

    # Test text to speech:
    rospy.loginfo("Testing text-to-speech...");
    roboComm.say("Testing", ttsEngine='festival', numRepeats=2, blockTillDone=False);
    rospy.loginfo("Done testing text-to-speech...\n------------------");
    while len(roboComm.speechThreads) != 0:
        rospy.loginfo("Waiting for %d speech thread(s) to terminate..." % len(roboComm.speechThreads));
        rospy.sleep(1.0);
    
    # Test sound effects:
#    rospy.loginfo("Testing sound effects...");
    #roboComm.playSound("drill", numRepeats=2, repeatPeriod=6);
#    roboComm.setSoundVolume(0.1, None);
#    roboComm.playSound("drill");
#    rospy.sleep(5);
#    roboComm.playSound("drill", 0.8);
#    rospy.sleep(5);
#    roboComm.setSoundVolume(0.5, 'drill');
#    roboComm.playSound("drill");
#    roboComm.setSoundVolume(0.9, None);
#    rospy.loginfo("Done testing sound effects.\n------------------");

    # Test music playing (Play 4 secs, pause 4 secs, play 10 secs, stop):
#    rospy.loginfo("Testing music play/pause/unpause/stop...")
#    roboComm.playMusic("cottonFields");
#    rospy.sleep(4);
#    roboComm.pauseMusic()
#    rospy.sleep(4);
#    roboComm.unPauseMusic()
#    rospy.sleep(10);
#    roboComm.stopMusic();
#    rospy.sleep(5);
#
#    # Volume setting: 4 secs on half vol, 4 secs on full vol:
#    roboComm.setMusicVolume(0.5);
#    roboComm.playMusic("cottonFields");
#    rospy.sleep(4);
#    roboComm.stopMusic();
#    roboComm.playMusic("cottonFields", volume=0.9);
#    rospy.sleep(4);
#    roboComm.stopMusic();
#    rospy.loginfo("Done testing music play/pause/unpause/stop.\n------------------");
    
    # Set playhead:
#    rospy.loginfo("Testing music playhead setting...")    
#    rospy.sleep(3);
#    roboComm.playMusic("cottonFields");
#    rospy.sleep(4);
#    roboComm.setPlayhead(10);
#    if not int(roboComm.getPlayhead()) in range(10,13):
#        rospy.logerr("Playhead was not positioned (during playback).")
#    rospy.sleep(4);
#    roboComm.pauseMusic();
#    rospy.sleep(2);
#    roboComm.setPlayhead(20);
#    roboComm.unPauseMusic();
#    if not int(roboComm.getPlayhead()) in range(20,23):
#            rospy.logerr("Playhead was not positioned (after unpause playback).")
#    rospy.sleep(4);
#    roboComm.stopMusic();
#    rospy.loginfo("Done testing music playhead setting.\n------------------");    
    
    rospy.loginfo("Done testing robot_interaction.py")
