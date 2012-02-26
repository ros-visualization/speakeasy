import sys
import rospy
from xml.etree import ElementTree

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
    def retrieveFromFile(fileName):
        '''
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
        
        return domTree.iter();
        
    #----------------------------------
    # findDefaultButtonSetTitle
    #--------------

    @staticmethod
    def findDefaultButtonSetTitle():
        pass #TODO

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
