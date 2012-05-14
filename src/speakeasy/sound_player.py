#!/usr/bin/env python

import pygame
import os
import time
from operator import itemgetter
import threading


# Modify to allow more than 8 simultaneous sounds:
NUM_SIMULTANEOUS_SOUNDS = 8;
NUM_OF_CACHED_SOUNDS    = 500;

class SoundPlayer(object):
    '''
    Plays up to NUM_SIMULTANEOUS_SOUNDS .wav/.ogg files simultaneously. Allows pause/unpause on
    any of these sounds. (.ogg untested)
    <p>
    Theory of operation: The class uses the pygame module. This module involves Sound instances
    and Channel instances. A Sound is created from a .wav or .ogg file, and can play on one or 
    more Channel instances. 
    <p>
    All channels can simultaneously play, one Sound per channel. The
	number of channels can be set via the module-global constant
	NUM_SIMULTANEOUS_SOUNDS. Default is 8. Channels and Sounds are managed
	by pygame.mixer. 
	<p>
	This class uses parameter polymorphism to unify all these concepts as
	much as possible. The only public methods are
	<code>play(), stop(), pause(), unpause(),set_volume(),
	get_volume()</code>. Each of these methods takes either the path to
	a sound file, a Sound instances, or a Channel instance. The
	SoundPlayer takes care of creating Sound instances by loading them
	from files, caching sounds, and tracking which Sounds are playing on
	which Channel at any time. Callers of these methods need to deal only
	with the sound file paths. These paths, when passed to, say, the
	pause() method, will do the right thing; Sound instances are cached,
	so they will not be loaded repeatedly.
	<p>
	Note that this wonderful collapsing of complex underlying pygame
	concepts comes at a price in code ugliness.  Since Python does not
	have parameter based polymorphism, methods must figure out incoming
	parameter types via duck typing methods: Treat the parameters as some
	type and see whether an error occurs. Terrible, but the 'pythonic' way.
	<p>
	
	As background: The pygame API provides methods three entities:
	Mixer, Channel, and Sound. The main methods are:
	
	<ul>
	    <li>Mixer:
	        stop(),pause(),unpause(),get_num_channels(),set_num_channels()</li>
	    <li>Channel: play(),stop(),pause(),unpause()</li>
	    <li>Sound: play(),stop(),pause(),unpause()</li>
	</ul>
	<p>   
    '''
    
    # Used to enforce Singleton pattern:
    singletonInstanceRunning = False;    
    
    #--------------------------------
    # Initializer
    #---------------
    
    def __init__(self):
        if SoundPlayer.singletonInstanceRunning:
            raise RuntimeError("Must only instantiate SoundPlayer once; an instance is already running.")
        else:
            SoundPlayer.singletonInstanceRunning = True;
        pygame.mixer.init();
        self.lock = threading.Lock(); 
        self.loadedSounds = {};    # Map filenames to Sound instances
        self.loadedFilenames = {}  # Map Sound instances to filenames
        self.soundChannelBindings = {};
        self.lastUsedTime = {};
        pygame.mixer.set_num_channels(NUM_SIMULTANEOUS_SOUNDS);
        
    #--------------------------------
    # play
    #---------------
    
    def play(self, whatToPlay, blockTillDone=False, volume=None):
        '''
        Play a file, or an already loaded Sound instance.
        Offers choice of blocking return until the sound is finished, or
        returning immediately. 
        
        @param whatToPlay: Full path to .wav or .ogg file, or Sound instance.
        @type whatToPlay: {string | Sound}
        @param blockTillDone: True to delay return until sound is done playing.
        @type blockTillDone: boolean
        @param volume: Volume to play at. Float between 0.0 and 1.0. None: use current volume.
        @type volume: float
        @return: The sound instance.
        @raise IOError: if given sound file path does not exist, or some other playback error occurred. 
        @raise ValueError: if given volume is not between 0.0 and 1.0
        @raise TypeError: if whatToPlay is not a filename (string), or Sound instance.
        '''

        if (volume is not None) and ( (volume < 0.0) or (volume > 1.0) ):
            raise ValueError("Volume must be between 0.0 and 1.0"); 

        with self.lock:
            self.cleanupSoundChannelBindings();
                    
            try:
                # Play a file?
                (sound, channel) = self.playFile(whatToPlay, volume);
                # Remember that this sound is now playing on that channel:
                if channel is not None:
                    self.addSoundToChannelBinding(sound, channel);
                    self.lastUsedTime[sound] = time.time();
                else:
                    raise IOError("Could not play sound '" + str(whatToPlay) + "'");
            except TypeError:
                # Nope, not a file, must be a Sound instance or something illegal:
                try: 
                    # Hypothesis: whatToPlay is a Sound instance:
                    sound   = whatToPlay;
                    channel = sound.play();
                    self.addSoundToChannelBinding(sound, channel);
                    self.lastUsedTime[sound] = time.time(); 
                except AttributeError:
                    # whatToPlay is not a file path (i.e. string), nor a Sound instance:
                    raise TypeError("Play method takes the path to a sound file, or a Sound instance. Instead received:" + 
                                     str(whatToPlay));
    
        # At this point, sound and channel vars are correctly set.
        # Caller wants to block till sound done? If so, we release
        # the lock (exit the 'with' block), and hang out:
        if blockTillDone:
            self.waitForSoundDone(sound)
            with self.lock:
                # Protect the sound-channel binding data structure: 
                self.cleanupSoundChannelBindings();
        return sound;

    #--------------------------------
    # stop
    #---------------
    
    def stop(self, whatToStop=None):
        '''
        Stop either all currently playing sounds, or a particular sound.
        The sound to stop (parameter whatToStop) may be specified as a 
        the full-path filename of the respective .wav/.ogg file, as a 
        Sound instance, or as a Channel instance. 
        @param whatToStop: If None, top all currently playing sounds. If any a
                           Sound instance, stop this sound on all channels on which
                           it might currently be playing. If a Channel instance,
                           stop only whatever is currently playing on this channel. 
        @type whatToStop: {NoneType | string | Sound | Channel}
        '''
        with self.lock:
            try:
                if whatToStop is None:
                    pygame.mixer.stop();
                    return;
                try:
                    self.stopFile(whatToStop);
                    return;
                except TypeError:
                    pass
                
                # Must be a Sound or Channel instance, which
                # support stop() (or something illegal):
                try:
                    whatToStop.stop();
                except:
                    raise TypeError("Parameter whatToStop must be a filename, Sound instance, or Channel instance.")
            finally:
                self.cleanupSoundChannelBindings();
    
    #--------------------------------
    # pause
    #---------------

    def pause(self, whatToPause=None):
        '''
        Pause either all currently playing sounds, or a particular sound.
        The sound to pause (parameter whatToStop) may be specified as a 
        the full-path filename of the respective .wav/.ogg file, as a 
        Sound instance, or as a Channel instance. 
        @param whatToPause: If None, top all currently playing sounds. If any a
                            Sound instance, pause this sound on all channels on which
                            it might currently be playing. If a Channel or array of 
                            channel instances, pause whatever is currently playing 
                            on the given channel(s). 
        @type whatToPause: {NoneType | string | Sound | Channel | [Channel]}
        @raise TypeError: if whatToPause is of illegal type.
        '''
        
        with self.lock:
            self.cleanupSoundChannelBindings();        
            
            if whatToPause is None:
                # Pause everything:
                pygame.mixer.pause();
                return
    
            try:
                # Pause by filename?
                if self.pauseFile(whatToPause) is not None:
                    # whatToPause was a filename, but it wasn't
                    # playing, or Pause succeeded. Either way
                    # we're done. 
                    return;
            except TypeError:
                # whatToPause is not a filename:
                pass;
    
            # whatToPause is a Channel or Sound instance:
            # Is it a Sound instance? 
            channels = self.getChannelsFromSound(whatToPause);
            if channels is None:
                # No sound bound to whatToPause is playing on any channel.
                # So maybe whatToUnPause is a channel or array of channels.
                # Ensure that we have an array:
                channels = whatToPause;
                try:
                    len(channels)
                except TypeError:
                    channels = [whatToPause];
                
            # Must be an array of channels at this point, or something illegal:
            try:
                for channel in channels:
                    if channel.get_busy():
                        channel.pause();
            except:
                # The passed-in whatToPause is neither, string (i.e. filename),
                # nor Sound instance, nor Channel instance. Chastise caller:
                raise TypeError("Parameter whatToPause must be a filename, Sound instance, Channel instance, or iterable of channel instances.")
            return;

    #--------------------------------
    # unpause
    #---------------

    def unpause(self, whatToUnPause=None):
        '''
        Unpause either all currently playing sounds, or a particular sound.
        The sound to unpause (parameter whatToStop) may be specified as a 
        the full-path filename of the respective .wav/.ogg file, as a 
        Sound instance, or as a Channel instance. 
        @param whatToPause: If None, unpause all currently playing sounds. If whatToUnPause
                            is a Sound instance, pause this sound on all channels on which
                            it might currently be playing. If whatToUnPause is a Channel or array of 
                            channel instances, pause whatever is currently playing 
                            on the given channel(s). 
        @type whatToUnPause: {NoneType | string | Sound | Channel}
        '''
        
        with self.lock():
            self.cleanupSoundChannelBindings();        
            
            if whatToUnPause is None:
                # Unpause everything:
                pygame.mixer.unpause();
                return
    
            try:
                # Unpause by filename?
                if self.unpauseFile(whatToUnPause) is not None:
                    # whatToUnPause was a filename, but it wasn't
                    # playing, or Pause succeeded. Either way
                    # we're done. 
                    return;
            except TypeError:
                # whatToUnPause is not a filename:
                pass;
    
            # whatToUnPause is a Channel or Sound instance:
            # Is it a Sound? 
            channels = self.getChannelsFromSound(whatToUnPause);
            if channels is None:
                # No sound that is whatToUnPause is playing on any channel.
                # So maybe whatToUnPause is a Channel instance or array:
                # Ensure that we have an array:
                channels = whatToUnPause;
                try:
                    len(channels)
                except TypeError:
                    channels = [whatToUnPause];
    
            try:
                for channel  in channels:
                    if channel.get_busy():
                        channel.unpause();
            except:
                # The passed-in whatToUnPause is neither, string (i.e. filename),
                # nor Sound instance, nor Channel instance. Chastise caller:
                raise TypeError("Parameter whatToUnPause must be a filename, Sound instance, or Channel instance.")
            return;

    #--------------------------------
    # setSoundVolume
    #---------------

    def setSoundVolume(self, whatToSetVolFor, volume):
        '''
        Set sound volume for a particular sound or channel.
        The entity for which to set the volume (i.e. whatToSetVolFor) 
        may be the full-path filename of a .wav/.ogg file, a Sound instance,
        or a Channel instance. Here is the interaction between settings
        of Sound vs. Channel volume:
    		  sound.set_volume(0.9)   # Now plays at 90% of full volume.
    		  sound.set_volume(0.6)   # Now plays at 60% (previous value replaced).
    		  channel.set_volume(0.5) # Now plays at 30% (0.6 * 0.5).
    	Passing in a filename will load the file, and set the volume of the 
    	corresponding Sound.
        @param whatToSetVolFor: The soundfile, Sound, or Channel instance whose volume to set. 
        @type whatToSetVolFor: {NoneType | string | Sound | Channel}
        @param volume: Value between 0.0 and 1.0.
        @type volume: float
        @raise OSError: if given filename that does not exist. 
        '''

        if (volume is not None) and ( (volume < 0.0) or (volume > 1.0) ):
            raise ValueError("Sound volume must be between 0.0 and 1.0. Was " + str(volume));

        with self.lock:
            # If whatToSetVolFor is a Sound or Channel instance,
            # a set_volume() method call will work:
            try:
                whatToSetVolFor.set_volume(volume);
                return;
            except:
                # Was not a sound or channel instance
                pass
    
            # whatToSetVolFor must be a filename (or something illegal).
            # Is this sound cached?
            sound = self.getSoundFromFileName(whatToSetVolFor);
            if sound is not None:
                # Yep, set the Sound instance's volume:
                sound.set_volume(volume);
                return;
            else:
                # Try loading the sound. Will barf with OSError if file not found:
                sound = self.loadSound(whatToSetVolFor);
                sound.set_volume(volume);
                return;
        
    #--------------------------------
    # getSoundVolume
    #---------------
        
    def getSoundVolume(self, whatToGetVolFor):
        '''
        Get sound volume for a particular sound or channel.
        The entity for which to get the volume for (i.e. whatToSetVolFor) 
        may be the full-path filename of a .wav/.ogg file, a Sound instance,
        or a Channel instance. Here is the interaction between settings
        of Sound vs. Channel volume:
    		  sound.set_volume(0.9)   # Now plays at 90% of full volume.
    		  sound.set_volume(0.6)   # Now plays at 60% (previous value replaced).
    		  channel.set_volume(0.5) # Now plays at 30% (0.6 * 0.5).
    	Passing in a filename will load the file, and get the volume of the 
    	corresponding Sound.
    	
        @param whatToSetVolFor: The soundfile, Sound, or Channel instance whose volume to get. 
        @type whatToSetVolFor: {NoneType | string | Sound | Channel}
        @raise OSError: if given filename that does not exist. 
        '''

        with self.lock:        
            # If whatToGetVolFor is a Sound or Channel instance,
            # a get_volume() method call will work:
            try:
                return whatToGetVolFor.get_volume();
            except:
                pass
    
            # whatToGetVolFor must be a filename (or something illegal):
            sound = self.getSoundFromFileName(whatToGetVolFor);
            if sound is not None:
                return sound.get_volume();
            else:
                # Try loading the sound. Will barf with OSError if file not found:
                sound = self.loadSound(whatToGetVolFor);
                return sound.get_volume();

    #--------------------------------
    # numChannels
    #---------------

    def numChannels(self):
        '''
        Number of sounds to play simultaneously. Controlled by module variable NUM_SIMULTANEOUS_SOUNDS. 
        '''
        return pygame.mixer.get_num_channels();

    # -----------------------------------   Private Methods ----------------------------
    
    #--------------------------------
    # loadSound
    #---------------
    
    def loadSound(self, filename):
        '''
        Create a Sound instance from the given file. Cache the sound,
        and initialize the LRU cache last-used time.
        @param filename: Full path to sound file (.wav/.ogg)
        @type filename: string
        @raise IOError: if file does not exist. 
        '''
        try:
            # Already loaded?
            return self.loadedSounds[filename];
        except KeyError:
            if not os.path.exists(filename):
                raise IOError("Sound file " + str(filename) + " does not exist.");
            sound = pygame.mixer.Sound(filename);
            # See whether cache is full, (and make room if not):
            self.checkCacheStatus();
            self.loadedSounds[filename] = sound;
            self.loadedFilenames[sound] = filename;
            # Initialize last-used time for this sound to
            # be load time. Whenever the sound is played,
            # this time will be updated:
            self.lastUsedTime[sound] = time.time();
            return sound;
        
    #--------------------------------
    # checkCacheStatus
    #---------------
    
    def checkCacheStatus(self):
        '''
        Ensures that number of cached sounds does not exceed
        NUM_OF_CACHED_SOUNDS. If it does, half the cache is
        emptied in order of least-recently-used first.
        '''
        
        if len(self.loadedSounds.keys()) < NUM_OF_CACHED_SOUNDS:
            return;
        
        # Must unload some sounds by removing references to half of
        # the Sound instances we loaded. We go by LRU discipline.
        # From the dict {sound:time-last-used} get a list of 
        # (sound,time-last-used) sorted by descending time.
        # The 'itemgetter(1)' is a special operator imported above
        # from the operator module. It expects one element of the sort
        # to be passed in, and returns the key to sort on. In this 
        # case the actuals will be sound/time pairs, and the operator
        # returns the time part:
        
        soundTimePairList = sorted(self.lastUsedTime.items(), key=itemgetter(1));   
        for soundTimePair in soundTimePairList[:NUM_OF_CACHED_SOUNDS / 2]:
            self.unloadSound(soundTimePair[0]);

    #--------------------------------
    # unloadSound 
    #---------------
    
    def unloadSound(self, sound):
        '''
        Remove Sound instance from cache and all other references. 
        @param sound: Sound instance to remove
        @type sound: Sound
        '''
        
        soundFilename = None
        # If this sound playing on one or more channels, stop them all
        # before unloading:
        
        channels = self.getChannelsFromSound(sound);
        if channels is not None:
            self.stop(sound);        
            
        try:
            soundFilename = self.loadedFilenames[sound];
            del self.loadedSounds[soundFilename];
        except KeyError:
            # Already unloaded, but clear out other 
            # possible references anyway:
            pass;
        
        if soundFilename is not None:
            try:
                del self.loadedFilenames[sound]
            except:
                pass;
        
        try: 
            del self.soundChannelBindings[sound];
        except KeyError:
            # Already unloaded, but clear out other 
            # possible references anyway:
            pass;
        try:
            del self.lastUsedTime[sound];
        except KeyError:
            pass;
        
        return;
    
    #--------------------------------
    # playFile
    #---------------

    def playFile(self, filename, volume=None):
        '''
        Private Method! Called by play();
        Given a filename, load it into a Sound instance, if it is not
        already cached. Set the volume, if given, and play. It is assumed
        that the volume value, if given, has been verified by the caller
        to be between 0.0 and 1.0. 
        @param filename: Name of sound file to play
        @type filename: string
        @param volume: Volume to play at: 0.0 to 1.0
        @type volume: float
        @return: Sound and channel instances
        @returnt: (Sound, Channel)
        @raise OSError: if file does not exist. 
        '''
        if not isinstance(filename, basestring):
            raise TypeError("Filename must be a string.")
        
        sound = self.getSoundFromFileName(filename);
        if sound is None:
            sound = self.loadSound(filename);
        if volume is not None:
            self.setSoundVolume(sound, volume);
        channel = sound.play();
        return (sound, channel);

    #--------------------------------
    # stopFile
    #---------------
    
    def stopFile(self, filename):
        '''
        Private Method! Called by stop();
        Stops currently playing sound from the given filename.
        Does nothing if this file is not currently playing.
        @param filename: Filename from which the Sound instance to be stopped was created.
        @type filename: string
        @raise TypeError: filename is not a string.
        '''
        
        if not isinstance(filename, basestring):
            raise TypeError("Filename must be a string.")
        sound = self.getSoundFromFileName(filename);
        if sound is None:
            # This file can't be playing, b/c it's not loaded:
            return None;
        sound.stop();
        return sound;
        
    #--------------------------------
    # pauseFile
    #---------------
        
    def pauseFile(self, filename):
        '''
        Private Method! Called by pause();
        Pauses currently playing sound from the given filename.
        Does nothing if this file is not currently playing.
        @param filename: Filename from which the Sound instance to be paused was created.
        @type filename: string
        @return: Sound instance
        @raise TypeError: filename is not a string.
        '''
        if not isinstance(filename, basestring):
            raise TypeError("Filename must be a string.")
        
        sound = self.getSoundFromFileName(filename);
        if sound is None:
            # This file can't be playing, b/c it's not loaded.
            # Not an error condition, but all done with the pause operation: 
            return True;
        # Sound exists in cache; get the all the Channel instances that are currently
        # playing that Sound instance:
        channels = self.getChannelsFromSound(sound);
        if channels is None:
            # Sound is loaded, but not currently playing.
            # Not an error, but all done with the pause operation: 
            return True;
        for channel in channels:
            channel.pause();
        return sound;

    #--------------------------------
    # unpauseFile
    #---------------

    def unpauseFile(self, filename):
        '''
        Unpauses currently paused sound from the given filename.
        Does nothing if this file is not currently playing or paused.
        @param filename: Filename from which the Sound instance to be unpaused was created.
        @type filename: string
        @return: Sound instance
        @returnt: Sound
        @raise TypeError: filename is not a string.
        '''
        
        if not isinstance(filename, basestring):
            raise TypeError("Filename must be a string.")
        
        sound = self.getSoundFromFileName(filename);
        if sound is None:
            # This file can't be playing, b/c it's not loaded:
            # Not an error, but all done with the unpause operation: 
            return True; 
        channels = self.getChannelsFromSound(sound);
        if channels is None:
            # Not an error, but all done with the pause operation: 
            return True;
        for channel in channels:
            channel.unpause();
        return sound;
    
    #--------------------------------
    # getSoundFromFileName
    #---------------

    def getSoundFromFileName(self, filename):
        '''
        Given a sound file path name, return the corresponding
        Sound instance, if the file was loaded. Else return None.
        @param filename: Filename from which the Sound instance was created.
        @type filename: string
        @return: Sound instance created from the given file. None if file not yet loaded.
        @returnt: {Sound | NoneType}
        '''
        try:
            return self.loadedSounds[filename]
        except KeyError:
            # This file hasn't been loaded:
            return None;

    #--------------------------------
    # getSoundFromChannel
    #---------------
    
    def getSoundFromChannel(self, channel):
        '''
        Return Sound instance that is currently playing on the given Channel instance.
        None if channel is inactive.
        @param channel: Channel to be investigated.
        @type channel: Channel
        '''
        if not channel.get_busy():
            return None;
        return channel.get_sound();        
    
    #--------------------------------
    # getChannelsFromSound
    #---------------
    
    def getChannelsFromSound(self, sound):
        '''
        Return all channels on which the given Sound instance 
        is currently playing.
        @param sound: Sound instance to be hunted down.
        @type sound: Sound
        @return: Array of Channel instances, or None, if Sound is not currently playing on any channel.
        @returnt: {[Channel] | NoneType}
        '''
        self.cleanupSoundChannelBindings();
        try:
            return self.soundChannelBindings[sound];
        except (KeyError, TypeError):
            return None;

    #--------------------------------
    # addSoundToChannelBinding
    #---------------

    def addSoundToChannelBinding(self, sound, channel):
        '''
        Register that a Sound is beginning to play on a given Channel.  
        @param sound: Sound to bind
        @type sound: Sound
        @param channel: Channel to bind to
        @type channel: Channel
        '''
        
        self.cleanupSoundChannelBindings();
        
        # Is this sound already playing on some channel?
        channels = self.getChannelsFromSound(sound);
        if channels is not None:
            # Add this channel to the ones that are already playing this sound:
            channels.append(channel);
            self.soundChannelBindings[sound] = channels;
        else:
            self.soundChannelBindings[sound] = [channel];

    #--------------------------------
    # cleanupSoundChannelBindings
    #---------------

    def cleanupSoundChannelBindings(self):
        '''
        Runs through the sound-to-channel bindings and removes
        the entries of channels that are done playing.
        '''
        maybePlayingSounds = self.soundChannelBindings.keys();
        for sound in maybePlayingSounds:
            # Get all the channels that are currently playing this sound:
            channels = self.soundChannelBindings[sound];
            # Find all channels that are no longer busy:
            channelsCopy = list(channels);
            for channel in channelsCopy:
                if not channel.get_busy():
                    channels.remove(channel);
            if len(channels) != 0:
                self.soundChannelBindings[sound] = channels;
            else:
                del self.soundChannelBindings[sound];

    #--------------------------------
    # waitForSoundDone
    #---------------
    
    def waitForSoundDone(self, sound):
        '''
        Block until sound is done playing on all channels 
        on which it is currently playing. 
        @param sound: Sound to monitor
        @type sound: Sound
        '''
        # Find all channels the Sound instance is currently playing on:
        channels = self.getChannelsFromSound(sound);
        if channels is None:
            return;
        for channel in channels:
            while channel.get_busy():
                time.sleep(0.3);
            
            
    # ---------------------------------------  Testing  -----------------------
if __name__ == '__main__':
    
    import os
    
    def playPauseUnpause(channel, whatToPause):
        while channel.get_busy():
            time.sleep(1);
            if not channel.get_busy():
                break;
            player.pause(whatToPause);
            time.sleep(3);
            player.unpause(whatToPause);
        
    testFileRooster = os.path.join(os.path.dirname(__file__), "../../sounds/rooster.wav");
    testFileSeagulls = os.path.join(os.path.dirname(__file__), "../../sounds/seagulls_shore.wav");
    testFileDrill = os.path.join(os.path.dirname(__file__), "../../sounds/drill.wav");
    testFileMoo2 = os.path.join(os.path.dirname(__file__), "../../sounds/moo.wav");
    testFileSteam = os.path.join(os.path.dirname(__file__), "../../sounds/steam.wav");
    
    player = SoundPlayer();
    print "Test one sound..."
    channel = player.play(testFileRooster, blockTillDone=True);
    print "Done test one sound..."
    print "---------------"
    
    # Test all-mixer pause: Play sound for 1 second, pause for 3 sec, play to completion:
    print "Test pause/unpause all channels. Expect 1sec sound, 3sec pause, 1 second sound, 3sec pause, rest of sound...";
#    channel = player.play(testFileRooster, blockTillDone=False);
#    playPauseUnpause(channel, None)
    print "Done pause/unpause all channels.";
    print "---------------"
    
    print "Test pause/unpause by filename. Expect repeated cycles of 1sec sound, 3sec pause...";
#    channel = player.play(testFileSeagulls, blockTillDone=False);
#    playPauseUnpause(channel, testFileSeagulls);
    print "Done test pause/unpause by filename.";
    print "---------------"
        
    print "Test two sounds after another: seagulls, then rooster..."
#    channelSeagulls = player.play(testFileSeagulls, blockTillDone=True);
#    channelRooster  = player.play(testFileRooster, blockTillDone=False);   
#    while channelRooster.get_busy() or channelSeagulls.get_busy():
#        time.sleep(0.5)
    print "Done testing two sounds after another: seagulls, then rooster."
    print "---------------"
    

    print "Test pause/unpause while another sound is running. Expect repeated cycles of 1sec rooster and seagull sounds, 3sec, both sounds again for 1 sec,...";
#    channelSeagulls = player.play(testFileSeagulls, blockTillDone=False);
#    channelRooster  = player.play(testFileRooster, blockTillDone=False);
#    playPauseUnpause(channelSeagulls, None) # None--> Both channels paused/unpaused
#    
#    while channelRooster.get_busy() or channelSeagulls.get_busy():
#        time.sleep(0.5)
    print "Done test pause/unpause while another sound is running. Expect repeated cycles of 1sec rooster and seagull sounds, 3sec, both sounds again for 1 sec,...";        
    print "---------------"

    print "Test attempt to play on more than 8 channels at once..."
#    channel1 = player.play(testFileSeagulls, blockTillDone=False);
#    channel2 = player.play(testFileSeagulls, blockTillDone=False);
#    channel3 = player.play(testFileSeagulls, blockTillDone=False);
#    
#    channel4 = player.play(testFileRooster, blockTillDone=False);
#    channel5 = player.play(testFileRooster, blockTillDone=False);
#    channel6 = player.play(testFileDrill, blockTillDone=False);
#    channel7 = player.play(testFileRooster, blockTillDone=False);
#    channel8 = player.play(testFileRooster, blockTillDone=False);
#    
#    channel9 = player.play(testFileDrill, blockTillDone=False);
#    
#    try:
#        while channel3.get_busy() or channel7.get_busy() or channel9.get_busy():
#            time.sleep(0.5)
#    except AttributeError:
#        print "Loaded sounds dict is: " + str(player.loadedSounds);
#        print "SoundChannelBindings dict is: " + str(player.soundChannelBindings);
    print "Done test attempt to play on more than 8 channels at once."
    print "---------------"
            
    print "Test pausing by filename when respective sound is playing on multiple channels..."
#    channel1 = player.play(testFileRooster, blockTillDone=False);
#    channel2 = player.play(testFileRooster, blockTillDone=False);
#    channel3 = player.play(testFileRooster, blockTillDone=False);
#
#    # Test pause/unpause by filename:
#    time.sleep(1);
#    player.pause(testFileRooster);
#    time.sleep(3);
#    player.unpause(testFileRooster);
#    print "SoundChannelBindings dict while playing three copies of rooster is: " + str(player.soundChannelBindings);
#    time.sleep(1);
#    player.pause(testFileRooster);
#    time.sleep(3);
#    player.unpause(testFileRooster);
#    while channel1.get_busy() or channel2.get_busy() or channel3.get_busy():
#        time.sleep(0.5)
#    
#    # Test pausing/unpausing arrays of channels:
#    channel1 = player.play(testFileRooster, blockTillDone=False);
#    channel2 = player.play(testFileRooster, blockTillDone=False);
#    channel3 = player.play(testFileRooster, blockTillDone=False);
#    time.sleep(1);
#    player.pause([channel1, channel2, channel3]);
#    time.sleep(3);
#    player.unpause([channel1, channel2, channel3]);
#    while channel1.get_busy() or channel2.get_busy() or channel3.get_busy():
#        time.sleep(0.5)
#    
#    # Test pausing/unpausing individual channels:
#    channel1 = player.play(testFileRooster, blockTillDone=False);
#    channel2 = player.play(testFileRooster, blockTillDone=False);
#    channel3 = player.play(testFileRooster, blockTillDone=False);
#    time.sleep(1);
#    player.pause(channel1)
#    player.pause(channel2)
#    player.pause(channel3)
#    time.sleep(3);
#    player.unpause(channel1);
#    while channel1.get_busy():
#        time.sleep(0.5)
#    player.unpause(channel2);
#    while channel2.get_busy():
#        time.sleep(0.5)
#    player.unpause(channel3);
#    while channel3.get_busy():
#        time.sleep(0.5)
#    while channel1.get_busy() or channel2.get_busy() or channel3.get_busy():
#        time.sleep(0.5)

#    # Check the dictionaries:
#    player.cleanupSoundChannelBindings()
#    print "Loaded sounds dict is: " + str(player.loadedSounds);
#    print "SoundChannelBindings dict before clean is: " + str(player.soundChannelBindings);
#    player.cleanupSoundChannelBindings()
#    print "SoundChannelBindings dict after clean is: " + str(player.soundChannelBindings);    
    print "Done test pausing by filename when respective sound is playing on multiple channels..."
    print "---------------"
    
    print "Test volume control. Hear rooster three times, once at full volume, then twice at the same lower volume"
#    player.play(testFileRooster, blockTillDone=True, volume=1.0);
#    player.play(testFileRooster, blockTillDone=True, volume=0.3);
#    # Sound should play at same low volume even without vol spec:
#    player.play(testFileRooster, blockTillDone=True);
    print "Done test volume control. Hear rooster three times, once at full volume, then twice at the same lower volume"
    print "---------------"
    
    print "Test sound cache management; three second delay is normal."
#    player.loadSound(testFileRooster);
#    # Sleep a sec between each load to get a spread of last-used times:
#    time.sleep(1);
#    player.loadSound(testFileDrill);
#    time.sleep(1);
#    player.loadSound(testFileSeagulls);
#    time.sleep(1);
#    player.loadSound(testFileMoo2);
#    assert(len(player.loadedSounds) == 4)
#    assert(len(player.lastUsedTime) == 4)
#    assert(len(player.soundChannelBindings) == 0);
#    #print "Keys loadedSounds before cache reduction: " + str(player.loadedSounds);
#    #print "Keys soundChannelBindings before cache reduction: " + str(player.soundChannelBindings);
#    #print "Keys lastUsedTime before cache reduction: " + str(player.lastUsedTime);
#    
#    NUM_OF_CACHED_SOUNDS_SAVED = NUM_OF_CACHED_SOUNDS
#    NUM_OF_CACHED_SOUNDS = 4;    
#    player.loadSound(testFileSteam);
#    
#    assert(len(player.loadedSounds) == 3)
#    assert(len(player.lastUsedTime) == 3)
#    assert(len(player.soundChannelBindings) == 0);
#    
#    #print "Keys loadedSounds after cache reduction: " + str(player.loadedSounds);
#    #print "Keys soundChannelBindings after cache reduction: " + str(player.soundChannelBindings);
#    #print "Keys lastUsedTime after cache reduction: " + str(player.lastUsedTime);
#    
#    NUM_OF_CACHED_SOUNDS = NUM_OF_CACHED_SOUNDS_SAVED
    print "Done test sound cache management; three second delay is normal."
    print "---------------"
    
    print "Test singleton enforcement"
    try:
        player1 = SoundPlayer();
        raise ValueError("Should have raised a RuntimeError.")
    except RuntimeError:
        pass;
    print "Done testing singleton enforcement"    
    print "---------------"
            
    print "All  done"