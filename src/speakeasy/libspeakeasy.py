#!/usr/bin/env python

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

# Andreas Paepcke: NOTE: this file is used only by speakeasy_node.py, not when speakeasy_controller.py
#                        operates in stand-alone mode.

import rospy
import os, sys
from speakeasy.msg import SpeakEasyRequest

## \brief Class that publishes messages to the speakeasy node.
##
## This class is a helper class for communicating with the speakeasy node
## via the \ref speakeasy.SpeakEasyRequest message. It has two ways of being used:
##
## - It can create Sound classes that represent a particular sound which
##   can be played, repeated or stopped.
##
## - It provides methods for each way in which the speakeasy.SpeakEasyRequest
##   message can be invoked.

class Sound:
    def __init__(self, client, snd, arg):
        self.client = client
        self.snd = snd
        self.arg = arg
    
## \brief Play the Sound.
## 
## This method causes the Sound to be played once.

    def play(self):
        self.client.sendMsg(self.snd, SpeakEasyRequest.PLAY_ONCE, self.arg)

## \brief Play the Sound repeatedly.
##
## This method causes the Sound to be played repeatedly until stop() is
## called.
    
    def repeat(self):
       self.client.sendMsg(self.snd, SpeakEasyRequest.PLAY_START, self.arg)

## \brief Stop Sound playback.
##
## This method causes the Sound to stop playing.

    def stop(self):
        self.client.sendMsg(self.snd, SpeakEasyRequest.PLAY_STOP, self.arg)

## This class is a helper class for communicating with the speakeasy node
## via the \ref speakeasy.SpeakEasyRequest message. There is a one-to-one mapping
## between methods and invocations of the \ref speakeasy.SpeakEasyRequest message.

class SoundClient:
    def __init__(self):
        self.pub = rospy.Publisher('robotsound', SpeakEasyRequest)

## \brief Create a voice Sound.
##
## Creates a Sound corresponding to saying the indicated text.
##
## \param s Text to say
 
    def voiceSound(self, s):
        return Sound(self, SpeakEasyRequest.SAY, s)

## \brief Create a wave Sound.
##
## Creates a Sound corresponding to indicated file.
##
## \param s File to play. Should be an absolute path that exists on the
## machine running the speakeasy node.
    def waveSound(self, sound):
        if sound[0] != "/":
	  rootdir = os.path.join(os.path.dirname(__file__),'../..','sounds')
          sound = rootdir + "/" + sound
        return Sound(self, SpeakEasyRequest.PLAY_FILE, sound)
    
## \brief Create a builtin Sound.
##
## Creates a Sound corresponding to indicated builtin wave.
##
## \param id Identifier of the sound to play.

    def builtinSound(self, id):
        return Sound(self, id, "")

## \brief Say a string
## 
## Send a string to be said by the sound_node. The vocalization can be
## stopped using stopSaying or stopAll.
## 
## \param text String to say
    def say(self,text, voice='voice_kal_diphone', ttsEngine="festival"):
        self.sendMsg(SpeakEasyRequest.SAY, SpeakEasyRequest.PLAY_ONCE, text, voice, ttsEngine=ttsEngine)

## \brief Say a string repeatedly
## 
## The string is said repeatedly until stopSaying or stopAll is used.
## 
## \param text String to say repeatedly

    def repeat(self,text, voice="voice_kal_diphone", ttsEngine="festival"):
        self.sendMsg(SpeakEasyRequest.SAY, SpeakEasyRequest.PLAY_START, text, voice, ttsEngine)

## \brief Stop saying a string
## 
## Stops saying a string that was previously started by say or repeat. The
## argument indicates which string to stop saying.
## 
## \param text Same string as in the say or repeat command

    def stopSaying(self,text):
        self.sendMsg(SpeakEasyRequest.SAY, SpeakEasyRequest.PLAY_STOP, text)
    
## \brief Plays a WAV or OGG file
## 
## Plays a WAV or OGG file once. The playback can be stopped by stopWave or
## stopAll.
## 
## \param sound Filename of the WAV or OGG file. Must be an absolute path valid
## on the computer on which the speakeasy node is running

    def playWave(self, sound):
        if sound[0] != "/":
	  rootdir = os.path.join(os.path.dirname(__file__),'../..','sounds')
          sound = rootdir + "/" + sound
        self.sendMsg(SpeakEasyRequest.PLAY_FILE, SpeakEasyRequest.PLAY_ONCE, sound)
    
## \brief Plays a WAV or OGG file repeatedly
## 
## Plays a WAV or OGG file repeatedly until stopWave or stopAll is used.
## 
## \param sound Filename of the WAV or OGG file. Must be an absolute path valid
## on the computer on which the speakeasy node is running.

    def startWave(self, sound):
        if sound[0] != "/":
	  rootdir = os.path.join(os.path.dirname(__file__),'../..','sounds')
          sound = rootdir + "/" + sound
        self.sendMsg(SpeakEasyRequest.PLAY_FILE, SpeakEasyRequest.PLAY_START, sound)

##  \brief Stop playing a WAV or OGG file
## 
## Stops playing a file that was previously started by playWave or
## startWave.
## 
## \param sound Same string as in the playWave or startWave command

    def stopWave(self,sound):
        if sound[0] != "/":
	  rootdir = os.path.join(os.path.dirname(__file__),'../..','sounds')
          sound = rootdir + "/" + sound
        self.sendMsg(SpeakEasyRequest.PLAY_FILE, SpeakEasyRequest.PLAY_STOP, sound)

## \brief Play a buildin sound
##
## Starts playing one of the built-in sounds. built-ing sounds are documented
## in \ref SpeakEasyRequest.msg. Playback can be stopped by stopall.
##
## \param sound Identifier of the sound to play.

    def play(self,sound):
        self.sendMsg(sound, SpeakEasyRequest.PLAY_ONCE, "")

## \brief Play a buildin sound repeatedly
##
## Starts playing one of the built-in sounds repeatedly until stop or
## stopall is used. Built-in sounds are documented in \ref SpeakEasyRequest.msg.
##
## \param sound Identifier of the sound to play.
    
    def start(self,sound):
        self.sendMsg(sound, SpeakEasyRequest.PLAY_START, "")

## \brief Stop playing a built-in sound
##
## Stops playing a built-in sound started with play or start. 
##
## \param sound Same sound that was used to start playback
    
    def stop(self,sound):
        self.sendMsg(sound, SpeakEasyRequest.PLAY_STOP, "")

## \brief Stop all currently playing sounds
##
## This method stops all speech, wave file, and built-in sound playback.
  
    def stopAll(self):
        self.stop(SpeakEasyRequest.ALL)

    def sendMsg(self, snd, cmd, s, arg2="", ttsEngine="festival"):
        msg = SpeakEasyRequest()
        msg.sound = snd
        msg.command = cmd
        msg.arg = s
        msg.arg2 = arg2 
        msg.text_to_speech_engine = ttsEngine
        self.pub.publish(msg)
        ## @todo this should be a warn once warns become visible on the console.
        if self.pub.get_num_connections() < 1:
            rospy.logerr("Sound command issued, but no node is subscribed to the topic. Perhaps you forgot to run soundplay_node.py");
