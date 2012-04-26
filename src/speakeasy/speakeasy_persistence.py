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

# ------------------------------------------------    Class ElementreeIterator --------------------------

class Python2_7ElementTree(object):
    '''
    This facility is built in to Python 2.7's ElementTree as
    element.iter(). Also built in with Python 2.7 is findall(tagName).
    We need to use 2.6 for now, so we recreate the facilities 

    '''
    
    def __init__(self, python2_6ElementTree):
        '''
        Pass in an element tree of the old kind.
        @param python2_6ElementTree: ElementTree instance
        @type python2_6ElementTree: ElementTree
        '''
        self.elTree = python2_6ElementTree;
        self.iterIndx = None;
        
    def iter(self):
        '''
        Return an object that supports methods next(), hasNext(), and closeIter().
        '''
        if self.iterIndx is not None:
            raise ValueError("Can only have one iterator going at a time. Call method closeIter() first.");
        self.flatNodeList = self.elTree.getiterator();
        self.iterIndx = -1;
        return self;
    
    def next(self):
        '''
        Return next subelement in xml tree.
        @raise ValueError: if no more elements are available. 
        '''
        if self.iterIndx is None:
            raise ValueError("Iterator must first be started with call to method iter()");
        self.iterIndx += 1;
        return self.flatNodeList[self.iterIndx];
    
    def hasNext(self):
        '''
        Return True if at least one element has still not been 
        retrieved via the next() method.
        '''
        if self.iterIndx is None:
            raise ValueError("Iterator must first be started with call to method iter()");
        return self.iterIndx < len(self.flatNodeList);
        
    def closeIter(self):
        '''
        Indicate that the iterator is no longer needed.
        Only after calling this method can a new iterator
        be obtained via iter().
        '''
        self.iterIndx = None;
    

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
        @raise ValueError: if XML is bad. 
        '''
        try:
            absFileName = os.path.join(ButtonSavior.SPEECH_SET_DIR, fileName);
            fd = open(absFileName, 'r');
        except Exception:
            ButtonSavior.reportError("Cannot open file '" + str(absFileName) + "' for saving button settings set.");
            return [];
        try:
            domTree = ElementTree.parse(fd);
        except Exception:
            ButtonSavior.reportError("Cannot open file '" + str(absFileName) + "' for retrieving button settings set.");
            return [];
    
        #domTreeIt = domTree.iter(); # Pythyon 2.7
        domTreeIt = Python2_7ElementTree(domTree).iter();
        buttonProgramObjects = [];
        
        try:
            try:
                # Get the root (outermost) element 'buttonProgramSet':
                domButtonSetRootEl = domTreeIt.next();
                domButtonSetTitleEl = domTreeIt.next();
                programSetTitle = domButtonSetTitleEl.text;

                # Get all 'buttonSetting' elements as ElementObject instances:
                domButtonSettingElArr = domButtonSetRootEl.findall("buttonSetting")

                
                for domButtonSettingEl in domButtonSettingElArr: 
                    domButtonLabelEl = domButtonSettingEl.find("label");
                    label = domButtonLabelEl.text;
                    
                    domTextEl = domButtonSettingEl.find("text");
                    text = domTextEl.text;
                     
                    domVoiceEl = domButtonSettingEl.find("voice");
                    voice = domVoiceEl.text;
                    
                    domEngineEl = domButtonSettingEl.find("engine");
                    engine = domEngineEl.text;
    
                    domPlayOnceEl = domButtonSettingEl.find("playOnce");
                    playOnce = domPlayOnceEl.text;
                    if playOnce == "True":
                        playOnce = True;
                    else:
                        playOnce = False;
                    
                    if text is None:
                        text = "";       
                    buttonProgram = buttonProgramClass(label, text, voice, engine, playOnce);
                    buttonProgramObjects.append(buttonProgram);
            except StopIteration:
                pass;
        except Exception:
            # Bad XML:
            raise ValueError("Bad XML in file " + fileName);
        
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

    @staticmethod
    def reportError(msg):
        rospy.logerr(msg);
        #print msg;


# ---------------------------------------------------------   Testing   --------------------------------

if __name__ == "__main__":
    from speakeasy_controller import ButtonProgram;
    import tempfile;
    
    buttonProgs = [
                   ButtonProgram("Test Label 1", "Test text 1", "David", "Festival"),
                   ButtonProgram("Test Label 2", "Test text 2", "Maria", "Cepstral"),
                   ]
    for buttonProg in buttonProgs:
        print str(buttonProg);
    tmpFile = tempfile.NamedTemporaryFile(delete=False);
    buttonSavior = ButtonSavior();
    buttonSavior.saveToFile(buttonProgs, tmpFile.name, "Title1");
    (title, progs) = buttonSavior.retrieveFromFile(tmpFile.name, ButtonProgram);
    tmpFile.close();
    os.remove(tmpFile.name)

    buttonProg = progs[0];
    
    if (buttonProg.getLabel() != "Test Label 1") or\
        (buttonProg.getText() != "Test text 1") or\
        (buttonProg.getVoice() != "David") or\
        (buttonProg.getTtsEngine() != "Festival"):
        raise ValueError("Button program 1 is incorrect: " +
                         "Label is '%s', should be '%s'. " % (buttonProg.getLabel(), "Test Label 1") +
                         "Text is '%s', should be '%s'. " % (buttonProg.getText(), "Test text 1") +
                         "Voice is '%s', should be '%s'. " % (buttonProg.getVoice(), "David") +
                         "TtsEngine is '%s', should be '%s'. " % (buttonProg.getTtsEngine(), "Festival"))
        
    buttonProg = progs[1];
    if buttonProg.getLabel() != "Test Label 2" or buttonProg.getText() != "Test text 2" or buttonProg.getVoice() != "Maria" or buttonProg.getTtsEngine() != "Cepstral":
        raise ValueError("Button program 2 is incorrect.");

    print "SpeakEasy button program save/restore test successful."
    
