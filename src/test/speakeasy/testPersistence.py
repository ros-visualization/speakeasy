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
        
        