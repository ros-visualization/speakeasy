#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE

import pygame
import os
import time
from operator import itemgetter
import threading

# TODO:
#    - play needs block option impl.

class TimeReference:
    RELATIVE = 0;
    ABSOLUTE = 1;

class PlayStatus:
    STOPPED = 0;
    PAUSED  = 1;
    PLAYING = 2;


class MusicPlayer(object):
    '''
    Plays music files, currently ogg and wav. In contrast to SoundPlayer,
    which is optimized for dealing with lots of short sounds, this facility
    is for longer files, which are streamed, rather than loaded. Also in
    contrast to SoundPlayer, MusicPlayer can only play one song at a time.

    For ogg files the method setPlayhead() allows clients to move foward
    and back within a song as it plays.

    Public methods:

       1. play()
       2. pause()
       3. unpause()
       4. setSoundVolume()
       5. getSoundVolume()
       6. setPlayhead()
       7. getPlayheadPosition()
       8. getPlayStatus()

	Requires pygame.    
    '''
    
    # Used to enforce Singleton pattern:
    singletonInstanceRunning = False;
    
    supportedFormats = ['ogg', 'wav'];    
    
    #--------------------------------
    # Initializer
    #---------------
    
    def __init__(self):
        if MusicPlayer.singletonInstanceRunning:
            raise RuntimeError("Must only instantiate MusicPlayer once; an instance is already running.")
        else:
            MusicPlayer.singletonInstanceRunning = True;
        pygame.init();
        self.lock = threading.Lock();
        self.playStatus = PlayStatus.STOPPED;
        self.currentAudioFormat = 'ogg';
        
        # Use the first available pygame user event number as 
        # a 'play ended naturally or via stop()' event:
        self.PLAY_ENDED_EVENT = pygame.USEREVENT; 
        
    #--------------------------------
    # play
    #---------------
    
    def play(self, whatToPlay, repeats=0, startTime=0.0, blockTillDone=False, volume=None):
        '''
        Play an .mp3 or .ogg file, or File class instance. Note that pygame does
        not support startTime for .wav files. They will play, but startTime is ignored.
        Offers choice of blocking return until the music is finished, or
        returning immediately. 
        
        @param whatToPlay: Full path to .wav or .ogg file, or File instance.
        @type whatToPlay: {string | File}
        @param repeats: Number of times to repeat song after the first time. If -1: repeat forever, or until another song is played.
        @type repeats: int
        @param startTime: Time in seconds into the song to start the playback.
        @type startTime: float
        @param blockTillDone: True to delay return until music is done playing.
        @type blockTillDone: boolean
        @param volume: How loudly to play (0.0 to 1.0). None: current volume.
        @type volume: float
        @raise IOError: if given music file path does not exist, or some other playback error occurred. 
        @raise ValueError: if given volume is not between 0.0 and 1.0
        @raise TypeError: if whatToPlay is not a filename (string), or Sound instance.
        @raise NotImplementedError: if startTime is other than 0.0, but the underlying music engine does not support start time control.
        '''

        if (volume is not None) and ( (volume < 0.0) or (volume > 1.0) ):
            raise ValueError("Volume must be between 0.0 and 1.0");
        
        if not (isinstance(repeats, int) and (repeats >= -1)): 
            raise TypeError("Number of repeats must be an integer greater or equal to -1");
        
        if not (isinstance(startTime, float) and (startTime >= 0.0)):
            raise ValueError("Start time must be a float of value zero or greater."); 
        
        # Check type explicitly, b/c pygame's exceptions are 
        # not useful for incorrect type errors:
        if not (isinstance(whatToPlay, basestring) or isinstance(whatToPlay, file)):
            raise TypeError("Song must be a string or a Python file object.")
        
        # Guaranteed that whatToPlay is string or file obj.
        # Ensure existence of file:
        try:
            # Assume whatToPlay is a string:
            if not os.path.exists(whatToPlay):
                raise IOError("Music filename %s does not exist." % whatToPlay);
            filename = whatToPlay;
        except TypeError:
            # Was a file object; check *it* for existence:
            if not os.path.exists(whatToPlay.name):
                raise IOError("Music filename %s does not exist." % whatToPlay.name);
            filename = whatToPlay.name;

        # Now filename has a legal file. Which audio format?
        (fileBaseName, extension) = os.path.splitext(filename);
        if extension.startswith("."):
            extension = extension[1:]
        if not extension in MusicPlayer.supportedFormats:
            raise ValueError("Unsupported file format '%s'. Legal formats in order of feature power: %s." % (os.path.basename(filename), 
                                                                                                             str(MusicPlayer.supportedFormats)));
        with self.lock:
            
            self.currentAudioFormat = extension;
            # Convert start time to msecs:
            self.initialStartPos = startTime * 1000.0
        
            pygame.mixer.music.load(filename);
            self.loadedFile = filename;
            if volume is not None:
                self.setSoundVolume(volume);
                
            # Clear the event queue of any old 'done playing'
            # events:
            pygame.event.clear(self.PLAY_ENDED_EVENT);
            # Ensure that pygame will queue such an event
            # when done:
            pygame.mixer.music.set_endevent(self.PLAY_ENDED_EVENT);
            
            # Pygame play method wants total number of times
            # to play the song; therefore: add 1. Start time is
            # in seconds. If file is a .wav file, leave out the
            # start time:
            if filename.endswith('.wav'):
                pygame.mixer.music.play(repeats+1);
            else:
                try:
                    pygame.mixer.music.play(repeats+1,startTime);
                except:
                    self.playStatus = PlayStatus.STOPPED;
                    return;
            self.playStatus = PlayStatus.PLAYING;

        if blockTillDone:
            self.waitForSongDone();

    #--------------------------------
    # stop
    #---------------
    
    def stop(self):
        '''
        Stop music if any is playing.
        '''
        with self.lock:
            pygame.mixer.music.stop();
            self.playStatus = PlayStatus.STOPPED;
    
    #--------------------------------
    # pause
    #---------------

    def pause(self):
        '''
        Pause either currently playing song, if any.
        '''
        
        if self.playStatus != PlayStatus.PLAYING:
            return;
        with self.lock:
            pygame.mixer.music.pause();
            self.playStatus = PlayStatus.PAUSED;

    #--------------------------------
    # unpause
    #---------------

    def unpause(self):
        '''
        Unpause a paused song.
        '''
        
        if self.playStatus != PlayStatus.PAUSED:
            return;
        with self.lock:
            pygame.mixer.music.unpause();
            self.playStatus = PlayStatus.PLAYING;

    #--------------------------------
    # setSoundVolume
    #---------------

    def setSoundVolume(self, volume):
        '''
        Set sound playback volume.
        @param volume: Value between 0.0 and 1.0.
        @type volume: float
        @raise TypeError: if volume is not a float.
        @raise ValueError: if volume is not between 0.0 and 1.0
        '''
        if (volume is not None) and ( (volume < 0.0) or (volume > 1.0) ):
            raise ValueError("Sound volume must be between 0.0 and 1.0. Was " + str(volume));

        try:
            # Lock if not already locked. Lock is already acquired if 
            # this call to setSoundVolume() did not originate from a 
            # client, but from a method within MusicPlayer(), which 
            # acquired the lock. In that case, remember that the lock
            # was already set, so that we don't release it on exit:
            wasUnlocked = self.lock.acquire(False);

            pygame.mixer.music.set_volume(volume);
        except:
            pass
        finally:
            # Only release the lock if the call to this method came from
            # a client of SoundPlayer, not from a method within SoundPlayer:
            if wasUnlocked:
                self.lock.release();
        
    #--------------------------------
    # getSoundVolume
    #---------------
        
    def getSoundVolume(self):
        '''
        Get currently set sound volume.
        @return: Volume number between 0.0 and 1.0
        @rtype: float
        '''

        try:
            # Lock if not already locked. Lock is already acquired if 
            # this call to setSoundVolume() did not originate from a 
            # client, but from a method within MusicPlayer(), which 
            # acquired the lock. In that case, remember that the lock
            # was already set, so that we don't release it on exit:
            wasUnlocked = self.lock.acquire(False);

            return pygame.mixer.music.get_volume();
        finally:
            # Only release the lock if the call to this method came from
            # a client of SoundPlayer, not from a method within SoundPlayer:
            if wasUnlocked:
                self.lock.release();
            
    #--------------------------------
    # setPlayhead
    #---------------

    def setPlayhead(self, secs, timeReference=TimeReference.ABSOLUTE):
        '''
        Set playhead to 'secs' seconds into the currently playing song. If nothing is being
        played, this method has no effect. If a song is currently paused, the song will
        be unpaused, continuing to play at the new playhead position.
        @param secs: number of (possibly fractional) seconds to start into the song.
        @type secs: float
        @param timeReference: whether to interpret the secs parameter as absolute from the song start, or relative from current position.
                              Options are TimeRerence.ABSOLUTE and TimeRerence.RELATIVE
        @type timeReference: TimeReference
        @raise NotImplementedError: if called while playing song whose format does not support playhead setting in pygame. 
        '''

        if self.currentAudioFormat != 'ogg':
            raise NotImplementedError("Playhead setting is currently implemented only for the ogg format. Format of currently playing file: " + 
                                      str(self.currentAudioFormat));
                                      
        if self.playStatus == PlayStatus.STOPPED:
            return;
        
        if not isinstance(secs, float):
            raise ValueError("The playhead position must be a positive float. Instead it was: " + str(secs));
        
        if (timeReference == TimeReference.ABSOLUTE) and (secs < 0.0):
            raise ValueError("For absolute playhead positioning, the playhead position must be a positive float. Instead it was: " + str(secs));
        
        with self.lock:
            currentlyAt = self.getPlayheadPosition(); # fractional seconds
            if timeReference == TimeReference.RELATIVE:
                newAbsolutePlayheadPos = currentlyAt + secs;
            else:
                newAbsolutePlayheadPos = secs;
                
            # New 'initial play position' is the target playhead position:
            self.initialStartPos = newAbsolutePlayheadPos * 1000.0;
            
        pygame.mixer.music.stop();
        self.play(self.loadedFile, startTime=newAbsolutePlayheadPos);
    
    #--------------------------------
    # getPlayheadPosition 
    #---------------

    def getPlayheadPosition(self):
        '''
        Return number of (possibly fractional) seconds to where the current
        song is currently playing. If currently playing nothing, return 0.0.
        @return: number of fractional seconds where virtual playhead is positioned.
        @rtype: float
        '''
        if pygame.mixer.music.get_busy() != 1:
            return 0.0;
        
        # Get time played so far in msecs. Returns only time played,
        # not considering start offset:
        timePlayedSinceStart = pygame.mixer.music.get_pos()
        # Add the start position (also kept in msgecs):
        trueTimePlayed = timePlayedSinceStart + self.initialStartPos
        
        # return seconds:
        return trueTimePlayed / 1000.0;
        
    #--------------------------------
    # currentlyPlaying() 
    #---------------
        
    def getPlayStatus(self):
        '''
        Return one of PlayStatus.STOPPED, PlayStatus.PLAYING, PlayStatus.PAUSED to
        reflect what the player is currently doing.
        '''
        if pygame.mixer.music.get_busy() != 1:
            self.playStatus = PlayStatus.STOPPED;
        return self.playStatus;

    # -----------------------------------   Private Methods ----------------------------
    

    #--------------------------------
    # waitForSoundDone
    #---------------
    
    def waitForSongDone(self, timeout=None):
        '''
        Block until song is done playing. Used in play() method.
        @param timeout: Maximum time to wait in seconds.
        @type timeout: {int | float}
        @return: True if song ended, False if timeout occurred.
        @rtype: boolean
        '''
        if self.getPlayStatus() == PlayStatus.STOPPED:
            return;
        if timeout is not None:
            startTime = time.time();
            while (pygame.mixer.music.get_busy() == 1) and ((time.time() - startTime) < timeout):
                time.sleep(0.3);
            if pygame.mixer.music.get_busy() == 0:
                return True;
            else:
                return False;
        else:
            while (pygame.mixer.music.get_busy() == 1):
                time.sleep(0.3);
            return True;

        
    #--------------------------------
    # formatSupported
    #---------------
    
    def formatSupported(self, fileExtension):
        '''
        Checks whether the given file extension implies a supported
        sound format.
        @param fileExtension: file extension with or without leading period. Example: ".ogg"
        @type fileExtension: string
        @return: True if the format is supported, else False.
        @rtype: boolean
        @raise ValueError: if fileExtension is anything other than a string with length > 0. 
        '''
        if (fileExtension is None) or (not isinstance(fileExtension, basestring)) or (len(fileExtension) == 0):
            raise ValueError("File format specification must be the format's file extension string.");
        if fileExtension[0] == '.':
            fileExtension = fileExtension[1:];
        return fileExtension in MusicPlayer.supportedFormats;
            
        
        return fileExtension in MusicPlayer.supportedFormats;
           
            
    # ---------------------------------------  Testing  -----------------------
if __name__ == '__main__':
    
    import os
    
    def playPauseUnpause():
        while player.getPlayStatus() != PlayStatus.STOPPED:
            time.sleep(2);
            print "First pause...";
            player.pause();
            time.sleep(3);
            player.unpause();
            time.sleep(2);
            print "Second pause...";
            player.pause();
            time.sleep(3);
            player.unpause();
            print "Stopping playback."
            player.stop();
            
    
    def testPlayheadMove():
        while player.getPlayStatus() != PlayStatus.STOPPED:
            time.sleep(3.0);
            player.pause();
            print "Playhead position after 3 seconds: " + str(player.getPlayheadPosition());
            time.sleep(2.0);
            print "Set Playhead position to 10 seconds and play...";
            player.setPlayhead(10.0);
            time.sleep(3.0); # Play from 10sec mark on for 3secs
            print "Set Playhead position back to 10 seconds without pausing...";
            player.setPlayhead(10.0);
            time.sleep(3.0);
            player.pause();
            print "Playhead position: " + str(player.getPlayheadPosition());
            time.sleep(2.0);
            print "set playhead position to -3, which should be 10 again...";
            player.setPlayhead(-3.0, TimeReference.RELATIVE);
            print "Playhead position after relative setting back to 10: " + str(player.getPlayheadPosition());
            time.sleep(3.0);
            print "Stopping."
            player.stop();
            
    
    #testFileCottonFields = os.path.join(os.path.dirname(__file__), "../../sounds/music/cottonFields.wav");
    testFileCottonFields = os.path.join(os.path.dirname(__file__), "../../sounds/music/cottonFields.ogg");
    testFileRooster = os.path.join(os.path.dirname(__file__), "../../sounds/rooster.wav");
    
    player = MusicPlayer();
    print "Test existing song."
#    player.play(testFileCottonFields, blockTillDone=True);
#    time.sleep(10);
#    player.stop();
    print "Done test existing song..."
    print "---------------"

    print "Test pause/unpause";
#    player.play(testFileCottonFields, blockTillDone=False);
#    playPauseUnpause();
    print "Done testing pause/unpause";
    print "---------------"
    
    print "Test playhead settings. Expect: Play from start for 3sec, play from 10 for 3sec, play from 10 again for 3sec. Stop."
#    player.play(testFileCottonFields);
#    testPlayheadMove();
    print "Done testing playhead settings."
    print "---------------"
    
    print "Test waitForSongDone()"
#    print "Immediate return when stopped:..."
#    player.waitForSongDone();
#    print "Immediate return when stopped OK."
#    player.play(testFileRooster);
#    print "Return when rooster done..."
#    player.waitForSongDone();
#    print "Return when rooster done OK."
    print "Wait for song end with 10 second timeout:"
#    player.play(testFileCottonFields);
#    player.waitForSongDone(timeout=10);
    
    print "Done testing waitForSongDone()"
    print "---------------"

    print "Request play of .wav file with startTime (which should be ignored), and with start time longer than song:"
#    player.play(testFileRooster, startTime=0.3, blockTillDone=True);
#    # Plays from the 10sec mark for 10 seconds:
#    player.play(testFileCottonFields, startTime=10.0);    
#    time.sleep(10);
#    # This one would go past: 
#    player.play(testFileCottonFields, startTime=3600.0);

    print  "Done requesting play with start time longer than song:"
    print "---------------"
    
    print "All  done"