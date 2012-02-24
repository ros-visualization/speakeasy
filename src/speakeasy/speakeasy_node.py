#!/usr/bin/env python

# TODO:
#   - publish latched sound files
#   - publish latched sound engines
#   - test Cepstral
#   - update GUI


#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

# Author: Blaise Gassend

# This node is a derivative of Blaise's original soundplay_node.py.
# The modifications and additions were kept to a minimum, and backward
# comnpatibility for clients of soundplay_node.py was retained.
# Here are the changes:

# Andreas Paepcke: I kludged a 'fix' for identical texts not playing
#                  when submitted in short intervals. Only the first
#                  text would play, the others would not. Playback started
#                  working only after a considerable time period, like
#                  40 seconds or such. However, if any change at all is made
#                  to subsequent texts, then consecutive play of those
#                  almost-identical texts is fine. 
#                  Suspicion is that the system's cache is not working properly.
#                  The small text variations cause a cache miss and new
#                  text-to-speech conversion each time. The critical point
#                  is at:
#             elif data.sound == SpeakEasyRequest.SAY:
#            --->        if not data.arg in self.voicesounds.keys():
#                     rospy.logdebug('command for uncached text: "%s"' % data.arg)
#                     txtfile = tempfile.NamedTemporaryFile(prefix='sound_play', suffix='.txt')
#                     wavfile = tempfile.NamedTemporaryFile(prefix='sound_play', suffix='.wav')
#                                      ...
#                  A true fix would track down the cache corruption. As
#                  a stop-gap, I modified the 'if' to 'if True', thereby
#                  forcing text conversion each time.
#
# NOTE: hardcoded Festival voice list to be ["kal_diphone"]. To fix: do the equivalent of
#       entering festival on the command line, and typing (voice.list).
#
#
# Andreas Paepcke: 

import roslib; roslib.load_manifest('speakeasy')

import rospy
import threading
from speakeasy.msg import SpeakEasyRequest
import os
import time
import logging
import sys
import subprocess
import traceback
import tempfile
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue, DiagnosticArray
from msg import Capabilities, TtsVoices

try:
    import pygst
    pygst.require('0.10')
    import gst
    import gobject
except:
    str="""
**************************************************************
Error opening pygst. Is gstreamer installed? (sudo apt-get install python-gst0.10 
**************************************************************
"""
    rospy.logfatal(str)
    print str
    exit(1)

def sleep(t):
    try:
        rospy.sleep(t)
    except:
        pass


class soundtype:
    STOPPED = 0
    LOOPING = 1
    COUNTING = 2
    #sound = gst.element_factory_make("playbin2","player")

    def __init__(self, file, volume = 1.0):
        self.lock = threading.RLock()
        self.state = self.STOPPED
        self.sound = gst.element_factory_make("playbin","player")
        if (":" in file):
            uri = file
        elif os.path.isfile(file):
            uri = "file://" + os.path.abspath(file)
        else:
          rospy.logerr('Error: URI is invalid: %s'%file)

        self.uri = uri
        self.volume = volume
        self.sound.set_property('uri', uri)
        self.sound.set_property("volume",volume)
        self.staleness = 1
        self.file = file

    def loop(self):  
        self.lock.acquire()
        try:
            self.staleness = 0
            if self.state == self.COUNTING:
                self.stop()
            
            if self.state == self.STOPPED:
              self.sound.seek_simple(gst.FORMAT_TIME, gst.SEEK_FLAG_FLUSH, 0)
              self.sound.set_state(gst.STATE_PLAYING)
            
            self.state = self.LOOPING
        finally:
            self.lock.release()

    def stop(self):
        if self.state != self.STOPPED:
            self.lock.acquire()
            try:
                self.sound.set_state(gst.STATE_NULL)
                self.state = self.STOPPED
            finally:
                self.lock.release()

    def single(self):
        self.lock.acquire()
        try:
            self.staleness = 0
            if self.state == self.LOOPING:
                self.stop()
            
            self.sound.seek_simple(gst.FORMAT_TIME, gst.SEEK_FLAG_FLUSH, 0)
            self.sound.set_state(gst.STATE_PLAYING)
        
            self.state = self.COUNTING
        finally:
            self.lock.release()

    def command(self, cmd):
         if cmd == SpeakEasyRequest.PLAY_STOP:
             self.stop()
         elif cmd == SpeakEasyRequest.PLAY_ONCE:
             self.single()
         elif cmd == SpeakEasyRequest.PLAY_START:
             self.loop()

    def get_staleness(self):
        self.lock.acquire()
        position = 0
        duration = 0
        try:
            position = self.sound.query_position(gst.FORMAT_TIME)[0]
            duration = self.sound.query_duration(gst.FORMAT_TIME)[0]
        except Exception, e:
            position = 0
            duration = 0
        finally:
            self.lock.release()

        if position != duration:
            self.staleness = 0
        else:
            self.staleness = self.staleness + 1
        return self.staleness


class soundplay:
    def stopdict(self,dict):
        for sound in dict.values():
            sound.stop()
    
    def stopall(self):
        self.stopdict(self.builtinsounds)
        self.stopdict(self.filesounds)
        self.stopdict(self.voicesounds)

    def callback(self,data):
        if not self.initialized:
            rospy.logerr("Speakeasy received sound request, but speakeasy node is not initialized.")
            return
        self.mutex.acquire()
        
        # Force only one sound at a time
        self.stopall()
        try:
            if data.sound == SpeakEasyRequest.ALL and data.command == SpeakEasyRequest.PLAY_STOP:
                self.stopall()
            else:
                if data.sound == SpeakEasyRequest.PLAY_FILE:
                    if not data.arg in self.filesounds.keys():
                        rospy.logdebug('command for uncached wave: "%s"'%data.arg)
                        try:
                            self.filesounds[data.arg] = soundtype(data.arg)
                        except:
                            print "Exception"
                            rospy.logerr('Error setting up to play "%s". Does this file exist on the machine on which speakeasy_node.py is running?'%data.arg)
                            return
                    else:
                        print "cached"
                        rospy.logdebug('command for cached wave: "%s"'%data.arg)
                    sound = self.filesounds[data.arg]
                elif data.sound == SpeakEasyRequest.SAY:
                    # data.sound is the text of a text-to-speech request:
                     
                    # Original paysound_node.py had caching, but
                    # caching does not work. It causes any cached sounds to
                    # not be played until they are purged from the cache.
                    # This takes several seconds. We deviate here from the original
                    # soundplay_node.py code, and always re-generate the text-to-speech:
                    rospy.logdebug('command for uncached text: "%s"' % data.arg)
                    
                    ttsEngine = None
                    if len(data.text_to_speech_engine) == 0:
                        data.text_to_speech_engine = "festival";
                    if (data.text_to_speech_engine == "festival"):
                        ttsEngine = Festival();
                    elif (data.text_to_speech_engine == "cepstral"):
                        ttsEngine = Cepstral();
                    else:
                        rospy.logerr("Request for text-to-speech engine " + str(data.text_to_speech_engine) + ", which is unsupported.")
                        return;
                    
                    voice = data.arg2
                    text  = data.arg
                    try:
                        try:
                            ttsEngine.runTextToSpeech(voice, text)
                        except OSError as e:
                            rospy.logerr(e.getMessage())
                            return
                        self.voicesounds[data.arg] = soundtype(ttsEngine.getWaveFileName());
                    finally:
                        if (ttsEngine is not None):
                            ttsEngine.cleanup();
                    sound = self.voicesounds[data.arg]
                else:
                    rospy.logdebug('command for builtin wave: %i'%data.sound)
                    if not data.sound in self.builtinsounds:
                        params = self.builtinsoundparams[data.sound]
                        self.builtinsounds[data.sound] = soundtype(params[0], params[1])
                    sound = self.builtinsounds[data.sound]
                if sound.staleness != 0 and data.command != SpeakEasyRequest.PLAY_STOP:
                    # This sound isn't counted in active_sounds
                    #print "activating %i %s"%(data.sound,data.arg)
                    self.active_sounds = self.active_sounds + 1
                    sound.staleness = 0
#                    if self.active_sounds > self.num_channels:
#                        mixer.set_num_channels(self.active_sounds)
#                        self.num_channels = self.active_sounds
                sound.command(data.command)
        except Exception, e:
            rospy.logerr('Exception in callback: %s'%str(e))
            rospy.loginfo(traceback.format_exc())
        finally:
            self.mutex.release()
            #print "done callback"

    # Purge sounds that haven't been played in a while.
    def cleanupdict(self, dict):
        purgelist = []
        for (key,sound) in dict.iteritems():
            try:
                staleness = sound.get_staleness()
            except Exception, e:
                rospy.logerr('Exception in cleanupdict for sound (%s): %s'%(str(key),str(e)))
                staleness = 100 # Something is wrong. Let's purge and try again.
            #print "%s %i"%(key, staleness)
            if staleness >= 10:
                purgelist.append(key)
            if staleness == 0: # Sound is playing
                self.active_sounds = self.active_sounds + 1
        for key in purgelist:
           del dict[key]
    
    def cleanup(self):
        self.mutex.acquire()
        try:
            self.active_sounds = 0
            self.cleanupdict(self.filesounds)
            self.cleanupdict(self.voicesounds)
            self.cleanupdict(self.builtinsounds)
        except:
            rospy.loginfo('Exception in cleanup: %s'%sys.exc_info()[0])
        finally:
            self.mutex.release()

    def diagnostics(self, state):
        try:
            da = DiagnosticArray()
            ds = DiagnosticStatus()
            ds.name = rospy.get_caller_id().lstrip('/') + ": Node State"
            if state == 0:
                ds.level = DiagnosticStatus.OK
                ds.message = "%i sounds playing"%self.active_sounds
                ds.values.append(KeyValue("Active sounds", str(self.active_sounds)))
                ds.values.append(KeyValue("Allocated sound channels", str(self.num_channels)))
                ds.values.append(KeyValue("Buffered builtin sounds", str(len(self.builtinsounds))))
                ds.values.append(KeyValue("Buffered wave sounds", str(len(self.filesounds))))
                ds.values.append(KeyValue("Buffered voice sounds", str(len(self.voicesounds))))
            elif state == 1:
                ds.level = DiagnosticStatus.WARN
                ds.message = "Sound device not open yet."
            else:
                ds.level = DiagnosticStatus.ERROR
                ds.message = "Can't open sound device. See http://pr.willowgarage.com/wiki/sound_play/Troubleshooting"
            da.status.append(ds)
            da.header.stamp = rospy.get_rostime()
            self.diagnostic_pub.publish(da)
        except Exception, e:
            rospy.loginfo('Exception in diagnostics: %s'%str(e))

    def __init__(self):
        rospy.init_node('speakeasy')
        self.diagnostic_pub = rospy.Publisher("/diagnostics", DiagnosticArray)
        self.capabilities_pub = rospy.Publisher("/capabilities", Capabilities, latch=True)

        # Path to where sound files are stored: In 'sounds' directory under package root dir:
        self.soundDir = os.path.join(os.path.dirname(__file__),'../../sounds')
        
        self.builtinsoundparams = {
                SpeakEasyRequest.BACKINGUP              : (os.path.join(self.soundDir, 'BACKINGUP.ogg'), 0.1),
                SpeakEasyRequest.NEEDS_UNPLUGGING       : (os.path.join(self.soundDir, 'NEEDS_UNPLUGGING.ogg'), 1),
                SpeakEasyRequest.NEEDS_PLUGGING         : (os.path.join(self.soundDir, 'NEEDS_PLUGGING.ogg'), 1),
                SpeakEasyRequest.NEEDS_UNPLUGGING_BADLY : (os.path.join(self.soundDir, 'NEEDS_UNPLUGGING_BADLY.ogg'), 1),
                SpeakEasyRequest.NEEDS_PLUGGING_BADLY   : (os.path.join(self.soundDir, 'NEEDS_PLUGGING_BADLY.ogg'), 1),
                }

        # List of sound engines. Examples "festival", "cepstral":
        self.soundEngines = [];
        
        festivalPath = self.which("text2wave");
        if (festivalPath is not None):
            self.soundEngines.append("festival");
            festivalVoices = self.getFestivalVoices(festivalPath)
            
        cepstralSwiftPath = self.which("swift");
        if (cepstralSwiftPath is not None):
            self.soundEngines.append("cepstral");
            cepstralVoices = self.getCepstralVoices(cepstralSwiftPath);
        
        # Create an empty Capabilities message:
        capabilitiesMsg = Capabilities();
        # initialize the text-to-speech engines field:
        capabilitiesMsg.ttsEngines = self.soundEngines;
        
        # Build ttsVoices structures for use in the Capabilities
        # message's TtsVoices field:
        
        ttsFestivalVoices = TtsVoices()
        ttsFestivalVoices.ttsEngine = "festival"
        ttsFestivalVoices.voices = festivalVoices;
        
        ttsCepstralVoices = TtsVoices()
        ttsCepstralVoices.ttsEngine = "cepstral"
        ttsCepstralVoices.voices = cepstralVoices;
        
        capabilitiesMsg.voices = [ttsFestivalVoices, ttsCepstralVoices];
        
        # List of available sound files:
        filesInSoundDir = os.listdir(self.soundDir);
        self.soundFiles = [];
        for soundFileName in filesInSoundDir:
            fileExtension = os.path.splitext(soundFileName)[1][1:].strip() 
            if (fileExtension == "wav") or (fileExtension == "ogg"):
                self.soundFiles.append(soundFileName);
        capabilitiesMsg.sounds = self.soundFiles;
        
        # Publish the capabilities message just once, latched:
        self.capabilities_pub.publish(capabilitiesMsg);
        
        
        self.mutex = threading.Lock()
        sub = rospy.Subscriber("robotsound", SpeakEasyRequest, self.callback)
        self.mutex.acquire()
        self.no_error = True
        self.initialized = False
        self.active_sounds = 0
        self.sleep(0.5) # For ros startup race condition
        self.diagnostics(1)

        while not rospy.is_shutdown():
            while not rospy.is_shutdown():
                self.init_vars()
                self.no_error = True
                self.initialized = True
                self.mutex.release()
                try:
                    self.idle_loop()
                    # Returns after inactive period to test device availability
                    #print "Exiting idle"
                except:
                    rospy.loginfo('Exception in idle_loop: %s'%sys.exc_info()[0])
                finally:
                    self.mutex.acquire()

            self.diagnostics(2)
        self.mutex.release()

    def init_vars(self):
        self.num_channels = 10
        self.builtinsounds = {}
        self.filesounds = {}
        self.voicesounds = {}
        self.hotlist = []
        if not self.initialized:
            rospy.loginfo('speakeasy node is ready to play sound')
            
    def sleep(self, duration):
        try:
            #rospy.sleep(duration)   
            time.sleep(duration)
        except rospy.exceptions.ROSInterruptException:
            pass
    
    def idle_loop(self):
        self.last_activity_time = rospy.get_time()
        while (rospy.get_time() - self.last_activity_time < 10 or
                 len(self.builtinsounds) + len(self.voicesounds) + len(self.filesounds) > 0) \
                and not rospy.is_shutdown():
            #print "idle_loop"
            self.diagnostics(0)
            self.sleep(1)
            self.cleanup()
        #print "idle_exiting"


    def which(self, program):
        '''
        Implements the Unix 'which' shell command. 
        @param program: Name of the executable with or without path
        @type: string
        @return: None if no executable found, else full path to executable.
        '''
        def is_exe(fpath):
            return os.path.exists(fpath) and os.access(fpath, os.X_OK)
    
        def ext_candidates(fpath):
            yield fpath
            for ext in os.environ.get("PATHEXT", "").split(os.pathsep):
                yield fpath + ext
    
        fpath, fname = os.path.split(program)
        if fpath:
            if is_exe(program):
                return program
        else:
            for path in os.environ["PATH"].split(os.pathsep):
                exe_file = os.path.join(path, program)
                for candidate in ext_candidates(exe_file):
                    if is_exe(candidate):
                        return candidate
    
        return None

    def getCepstralVoices(self, absSwiftPath):
        p = subprocess.Popen([absSwiftPath, "--voices"], stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        tbl, errors = p.communicate()
        lines = tbl.splitlines();
        
        # Throw out header lines:
        for lineNum, line in enumerate(lines):
            if not line.startswith("-"):
                continue;
            break
        voiceNames = []
        for voiceLine in lines[lineNum:]:
            if not voiceLine.startswith("-"):
                voiceNames.append(str(voiceLine.split()[0]))
        return voiceNames    

    def getFestivalVoices(self, absFestivalPath):
        #TODO: run festival, and issue Scheme command (voice.list)
        #      to get the real list of voices.
        return ["kal_diphone"]


class TextToSpeechEngine(object):
    
    def __init__(self):
        self.txtfile = tempfile.NamedTemporaryFile(prefix='speakeasy', suffix='.txt')
        self.wavfile = tempfile.NamedTemporaryFile(prefix='speakeasy', suffix='.wav')
        self.txtfilename = self.txtfile.name
        self.wavfilename = self.wavfile.name

    def initTextFile(self, text):
        # Just in case client calls getCommandString() twice on the same
        # TTS instance:
        self.txtfile.truncate();
        
    def runTextToSpeechHelper(self, text, commandLine, failureMsg):
        try:
            self.txtfile.write(text)
            self.txtfile.flush()
            os.system(commandLine);
            if os.stat(self.wavfilename).st_size == 0:
                raise OSError(failureMsg);
        except Exception as e:
            rospy.logerr(e.getMessage());
            

    def getWaveFileName(self):
        return self.wavfilename;
        
    def cleanup(self):
        self.txtfile.close();
        #self.wavfile.close(); # Maybe race condition?
        
class Festival(TextToSpeechEngine):
    
    def __init__(self):
        super(Festival, self).__init__()
        
    def runTextToSpeech(self, voice, text):
        self.initTextFile(text)
        failureMsg = "Sound synthesis failed. Is festival installed? Is a festival voice installed? Try running 'rosdep satisfy speakeasy|sh'. Refer to http://pr.willowgarage.com/wiki/sound_play/Troubleshooting"
        self.runTextToSpeechHelper(text,
                                   "text2wave -eval '(" + str(voice) + ")' " + self.txtfilename + " -o " + self.wavfilename,
                                   failureMsg);
        return self.wavfilename
    
class Cepstral(TextToSpeechEngine):
    
    def __init__(self):
        super(Cepstral, self).__init__()
        
    def runTextToSpeech(self, voice, text):
        self.initTextFile(text)        
        failureMsg = "Sound synthesis failed. Is Cepstral's swift installed? Is a Cepstral voice installed? Try running 'rosdep satisfy speakeasy|sh'. Refer to http://pr.willowgarage.com/wiki/sound_play/Troubleshooting"
        self.runTextToSpeechHelper(text,
                                   "swift -d " + str(voice) + " -f " + self.txtfilename + " -o " + self.wavfilename,
                                   failureMsg);
        return self.wavfilename

if __name__ == '__main__':
    soundplay()
