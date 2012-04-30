#!/usr/bin/env python

import rospy
from speakeasy.srv import SpeechCapabilitiesInquiry

def testSpeechCapabilitiesInquiry():
    print "Wait for speech capabilities service..."
    rospy.wait_for_service('speech_capabilities_inquiry')
    print "Speech capabilities service online."    
    try:
        capabilitiesService = rospy.ServiceProxy('speech_capabilities_inquiry', SpeechCapabilitiesInquiry)
        capabilities = capabilitiesService("foo");
        print str(capabilities);
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    testSpeechCapabilitiesInquiry();
    