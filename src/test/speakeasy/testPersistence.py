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

import unittest;
import tempfile;

from speakeasy.speakeasy_controller  import ButtonProgram;
from speakeasy.speakeasy_persistence import ButtonSavior;

class ButtonProgramNoButtonObj(ButtonProgram):
    '''
    Class behaves like ButtonProgram, but without needing
    a Qt button object. Used for testing XML creation and parsing.
    '''
    buttonNameCount = 0;

    def __init__(self, textToSave, voice, ttsEngine, playOnce=True):
        ButtonProgramNoButtonObj.buttonNameCount += 1;
        self.buttonLabel = "Button " + str(ButtonProgramNoButtonObj.buttonNameCount);
        self.textToSay   = textToSave;
        self.activeVoice = voice;
        self.ttsEngine   = ttsEngine;
        self.playOnce    = playOnce;

class TestButtonProgramPersistence(unittest.TestCase):
    
    def setUp(self):
        self.buttonSetting1 = ButtonProgramNoButtonObj("Utterance number 1", "David", "cepstral");
        self.buttonSetting2 = ButtonProgramNoButtonObj("Utterance number 2", "David", "cepstral");
        self.buttonSetting3 = ButtonProgramNoButtonObj("Utterance number 3", "Male", "festival");
        
    def test_createIndividualProgramXML(self):
        correctInnerTexts = ["Button 1", "Utterance number 1", "David", "cepstral", "True"];        
        trueInnerTexts = [];
        domOneButtonProgram = self.buttonSetting1.toXML();
        innerTextIterator = domOneButtonProgram.itertext();
        for buttonProgramText in innerTextIterator:
            # print buttonProgramText;
            trueInnerTexts.append(buttonProgramText);
        self.assertEqual(correctInnerTexts, trueInnerTexts, "Single button program did not produced correct XML.");
        
    def test_writeOneButtonSetToFile(self):
        tmpFile = "/tmp/buttonSettingsThreeButtons.xml";
        ButtonSavior.saveToFile([self.buttonSetting1, self.buttonSetting2, self.buttonSetting3], 
                                tmpFile, 
                                title="Household Set");
        
if __name__ == "__main__":
    unittest.main();        
        
        