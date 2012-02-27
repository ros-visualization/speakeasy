from xml.etree import ElementTree
import os
import rospy
import sys

# ------------------------------------------------    Support Function --------------------------

#----------------------------------
# openTag
#--------------

def openTag(tagName):
    return "<" + tagName + ">"

#----------------------------------
# closeTag
#--------------

def closeTag(tagName):
    return "</" + tagName + ">"

#----------------------------------
# indent 
#--------------

def indent(elem, level=0):
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

# ------------------------------------------------    Class ButtonSavior --------------------------

class ButtonSavior(object):

    BUTTON_PROGRAM_SET_TAG             = "buttonProgramSet"
    BUTTON_PROGRAM_SET_TITLE_TAG       = "buttonProgramSetTitle"
    BUTTON_PROGRAM_TAG                 = "buttonSetting";
    BUTTON_LABEL_TAG                   = "label";
    BUTTON_TEXT_TO_SAY_TAG             = "text";
    BUTTON_VOICE_TAG                   = "voice";
    BUTTON_TTS_ENGINE                  = "engine";
    BUTTON_PLAY_ONCE                   = "playOnce";


    SPEECH_SET_DIR = os.path.join(os.path.dirname(__file__),'../..','buttonPrograms');

    #----------------------------------
    # initializer
    #--------------

    def __init__(self):
        pass;

    #----------------------------------
    # saveToFile
    #--------------

    @staticmethod
    def saveToFile(buttonSettings, fileName, title=None):
        '''
        Given an array of ButtonProgram objects, turn them to 
        XML and save them in a new file of the given name.
        If the file exists, it will be overwritten.
        @param buttonSettings: Array of ButtonProgram instances
        @type  buttonSettings: [ButtonProgram]
        @param fileName: Name of file to which the XML is to be written.
        @type fileName: string
        @param title: An arbitrary name for this set
        @type title: string
        '''
        try:
            fd = open(fileName, 'w');
        except Exception:
            ButtonSavior.reportError("Cannot open file '" + str(fileName) + "' for saving buttons.");
            sys.exit(-1);

        if title is None:
            title = ButtonSavior.findDefaultButtonSetTitle();

        domRoot    = ElementTree.XML(openTag(ButtonSavior.BUTTON_PROGRAM_SET_TAG) +\
                                     closeTag(ButtonSavior.BUTTON_PROGRAM_SET_TAG));
        domTitle = ElementTree.XML(openTag(ButtonSavior.BUTTON_PROGRAM_SET_TITLE_TAG) +\
                                   title +\
                                   closeTag(ButtonSavior.BUTTON_PROGRAM_SET_TITLE_TAG));
        domRoot.append(domTitle);
        
        for buttonSetting in buttonSettings:
            domOneButtonSetting = buttonSetting.toXML();
            domRoot.append(domOneButtonSetting);

        indent(domRoot);
        domTree = ElementTree.ElementTree(domRoot);
        domTree.write(fd);

    @staticmethod
    def retrieveFromFile(fileName, buttonProgramClass):
        '''
        Given an absolute path to a button program XML file,
        return an iterator over the DOM tree program elements.
        The caller passes in a class object. This class will
        be instantiated with the data extracted from the XML
        of each button program.
        
        Example structure:
        <buttonProgramSet>
          <buttonProgramSetTitle>Household Set</buttonProgramSetTitle>
          <buttonSetting>
            <label>Button 4</label>
            <text>Utterance number 1</text>
            <voice>David</voice>
            <engine>cepstral</engine>
            <playOnce>True</playOnce>
          </buttonSetting>
          <buttonSetting>
            <label>Button 5</label>
            <text>Utterance number 2</text>
            <voice>David</voice>
            <engine>cepstral</engine>
            <playOnce>True</playOnce>
          </buttonSetting>
        </buttonProgramSet>
        
        @param fileName: name of XML file.
        @type fileName: string
        @param buttonProgramClass: class whose instances are to be created to hold button settings.
        @type buttonProgramClass: ButtonProgram
        @return: A tuple containing the title of the button set, and an array of instances of
                the passed-in class.
        @returnt: (string, [ButtonProgram])
        '''
        try:
            fd = open(fileName, 'w');
        except Exception:
            ButtonSavior.reportError("Cannot open file '" + str(fileName) + "' for saving button settings set.");
            sys.exit(-1);
        try:
            domTree = ElementTree.parse(fd);
        except Exception:
            ButtonSavior.reportError("Cannot open file '" + str(fileName) + "' for retrieving button settings set.");
    
        domTreeIt = domTree.iter();
        buttonProgramObjects = [];
        
        try:
            domButtonSetTitleEl = domTreeIt.next();
            programSetTitle = domButtonSetTitleEl.text;
            while True:
                # Get the program setting for One button:
                domProgramEl = domTreeIt.next();
                
                domButtonLabelEl = domProgramEl.find("label");
                label = domButtonLabelEl.text;
                
                domTextEl = domProgramEl.find("text");
                text = domTextEl.text;
                 
                domVoiceEl = domProgramEl.find("voice");
                voice = domVoiceEl.text;
                
                domEngineEl = domProgramEl.find("engine");
                engine = domEngineEl.text;

                domPlayOnceEl = domProgramEl.find("playOnce");
                playOnce = domPlayOnceEl.text;
                if playOnce == "True":
                    playOnce = True;
                else:
                    playOnce = False;
                                 
                buttonProgram = buttonProgramClass(label, text, voice, engine, playOnce);
                buttonProgramObjects.append(buttonProgram);
        except StopIteration:
            pass;
        
        return (programSetTitle, buttonProgramObjects);
        
    #----------------------------------
    # findDefaultButtonSetTitle
    #--------------

    @staticmethod
    def findDefaultButtonSetTitle():
        return "Default Title" #TODO

    #----------------------------------
    # createXMLElement
    #--------------

    @staticmethod
    def createXMLElement(tagName, content="", attrDict={}):
        domEl = ElementTree.XML(openTag(tagName) + content + closeTag(tagName));
        for attrName in attrDict.keys():
            domEl.set(attrName, attrDict[attrName]);
        return domEl;


    #----------------------------------
    # reportError
    #--------------

    def reportError(self, msg):
        rospy.logerr(msg);
        #print msg;

if __name__ == "__main__":
    pass
