#!/usr/bin/env python

# TODO:
#    - add service request for status

import math

import roslib; roslib.load_manifest('speakeasy');
import rospy;

from speakeasy.msg import SpeakEasyMusic, SpeakEasySound, SpeakEasyStatus, SpeakEasyTextToSpeech,TtsVoices,SpeakEasyPlayhead 

# Commands
class Commands:
	PLAY = 0
	STOP = 1
	PAUSE = 2
	UNPAUSE = 3
	SET_VOL = 4



class SpeakEasyTester(object):
    
    def __init__(self):
        
        self.soundReqPub  = rospy.Publisher('speakeasy_sound_req', SpeakEasySound);
        self.musicReqPub  = rospy.Publisher('speakeasy_music_req', SpeakEasyMusic);
        self.ttsReqPub    = rospy.Publisher('speakeasy_text_to_speech_req', SpeakEasyTextToSpeech);
        
        rospy.init_node('speakeasy_tester', anonymous=True);
        
        statusMsg = self.getSpeakEasyStatus();
        rospy.loginfo("SpeakEasy status message in idle state:")
        rospy.loginfo(str(statusMsg));
        
    
    def testSoundPlay(self):
	   #  play(soundName, volume=)
	   #  stop(soundName=)
	   #  pause(soundName=)
	   #  unpause(soundName=)
	   #  setVol(newVol, soundName)
       
       # Make rooster sound:
       msg = self.createSoundMsg(Commands.PLAY, "rooster")
       self.soundReqPub.publish(msg);
       
    def createSoundMsg(self, command, soundName=None, volume=None):
        if soundName is None:
            soundName = "";
        if volume is None:
            volume = -1;
        msg = SpeakEasySound(command=command, sound_name=soundName, volume=volume);
        return msg;
       
       
#    def createSoundMsg(self, command=None, soundName=None, volume=None):
#        msg = SpeakEasySound();
#        msg.command = command;
#        if soundName is not None:
#            msg.sound_name = soundName;
#        if volume is not None:
#            msg.volume = volume;
#        return msg;
    
    def getSpeakEasyStatus(self, timeout=None):
        
        reportToUserPeriod = 2.0;
        
        if timeout is None:
            while True:
                try:
                    return rospy.wait_for_message('speakeasy_status', SpeakEasyStatus, reportToUserPeriod);
                except rospy.ROSException:
                    rospy.loginfo("Waiting for SpeakEasy node to begin publishing status messages...");
        else:
            for waitLoop in range(math.ceil(timeout/reportToUserPeriod)):
                try:
                    return rospy.wait_for_message('speakeasy_status', SpeakEasyStatus, reportToUserPeriod);
                except rospy.ROSException:
                    rospy.loginfo("Waiting for SpeakEasy node to begin publishing status messages...");
        
        
if __name__ == "__main__":
    
    tester = SpeakEasyTester();
    tester.testSoundPlay();
    
    #rospy.spin();