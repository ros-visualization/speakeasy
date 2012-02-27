import sys
import os

from functools import partial;
from threading import Timer;

from PySide.QtCore import * #@UnusedWildImport
from PySide.QtGui import * #@UnusedWildImport


#TODO: From speakeasy_node's capabilities message, find all available sound files, and build sound buttons from those file names.
#TODO: Play repeatedly.
#TODO: From speakeasy_node's capabilities message, find all voices and their tts engines. Represent them in the radio buttons.


class Option:
    YES = True;
    NO  = False;

class Orientation:
    HORIZONTAL = 0;
    VERTICAL   = 1;

class Alignment:
    LEFT = 0;
    CENTER = 1;
    RIGHT = 2;
    TOP = 3;
    BOTTOM = 4;
    
class LocationSpec():
    LEFT = 0;
    RIGHT = 1;
    TOP = 2;
    BOTTOM = 3;

class CheckboxGroupBehavior:
    RADIO_BUTTONS = 0;
    CHECKBOXES    = 1;


#--------------------------------  TextPanel Class ---------------------------
class TextPanel(QTextEdit):
    '''
    Text input field whose number of lines may be specified approximately.
    '''
    
    # Class whose instances implements a sizeable, editable text input field.

    #----------------------------------
    # Initializer 
    #--------------

    def __init__(self, numLines=None):
        super(TextPanel, self).__init__(None);
        
        if numLines is not None:
            # Want the text area to be numLines high.
            # Get the app's font, get its height, and use it
            # to specify the text field's maximum height:
            currentFontHeight = self.fontMetrics().height();
            # add 5 pixels for each line:
            self.setMinimumHeight((currentFontHeight * numLines) + (5 * numLines));
            self.setMinimumWidth(600); #px

    #----------------------------------
    # getText
    #--------------
    
    def getText(self):
        # Get text field text as plain text, and convert
        # unicode to ascii, ignoring errors:
        return self.toPlainText().encode('ascii', 'ignore');
    
    #----------------------------------
    # isEmpty
    #--------------

    def isEmpty(self):
        return len(self.toPlainText()) == 0;

# ----------------------------------------------- Class CommChannel ------------------------------------


#class CommChannel(QObject):
#    '''
#    Combines signals into one place.
#    '''
#    hideButtonSignal  	= Signal(QPushButton); # hide the given button
#    showButtonSignal  	= Signal(QPushButton); # show the hidden button
#    
#    def __init__(self):
#        super(CommChannel, self).__init__();
    
# ----------------------------------------------- Class DialogService ------------------------------------

class DialogService(object):

    #----------------------------------
    # Initializer
    #--------------

    def __init__(self, parent=None):
        
        # All-purpose error popup message:
        # Used by self.showErrorMsgByErrorCode(<errorCode>), 
        # or self.showErrorMsg(<string>).
        self.errorMsgPopup = QErrorMessage.qtHandler();
        self.errorMsgPopup.setStyleSheet(SpeakEasyGUI.stylesheetAppBG);
#        if parent is not None:
#            self.errorMsgPopup.setParent(parent);

    
    #----------------------------------
    # showErrorMsg
    #--------------
    QErrorMessage
    def showErrorMsg(self,errMsg):
        '''
        Given a string, pop up an error dialog.
        @param errMsg: The message
        @type errMsg: string
        '''
        self.errorMsgPopup.showMessage(errMsg);
    

#----------------------------------------------------  SpeakEasyGUI Class ---------------------------

class SpeakEasyGUI(QMainWindow):
    '''
    One instance of this class builds the entire sound play UI.
    Instance variable that hold widgets of interest to controllers:
    
    - C{speechInputFld}
    - C{onceOrRepeatDict}
    - C{voicesRadioButtonsDict}
    - C{recorderDict}
    - C{programButtonDict}
    - C{soundButtonDict}

    '''

    # ---------------------- Durations --------------------
    
    # Number of seconds a program button must be pressed to
    # go into program mode:
    PROGRAM_BUTTON_HOLD_TIME = 3.0;

    # Amount of time the program button stays in alternate look
    # to indicate that it is changing to programming mode: 

    PROGRAM_BUTTON_LOOK_CHANGE_DURATION = 0.2; # seconds

    # ---------------------- Button and Font Sizes -------------------- 

    # Minimum height of buttons:
    BUTTON_MIN_HEIGHT = 30; # pixels
    # Pushbutton minimum font size:
    BUTTON_LABEL_FONT_SIZE = 16; # pixels
    
    # Radio button minimum font size:
    RADIO_BUTTON_LABEL_FONT_SIZE = 16; # pixels
    
    EDIT_FIELD_TEXT_SIZE = 18; # pixels
    
    NUM_OF_PROGRAM_BUTTON_COLUMNS = 4;
    NUM_OF_SOUND_BUTTON_COLUMNS   = 4;
    
    # ---------------------- Names for Voices ----------------------
    
    voices = {'VOICE_1': 'Male',
              'VOICE_2':'David'
              };
        
    # ---------------------- Names for Widgets ----------------------
    
    interactionWidgets = {
                          'PLAY_ONCE': 'Play once',
                          'PLAY_REPEATEDLY': 'Play repeatedly',
                          'VOICE_1': 'Male',
                          'VOICE_2': 'David',
                          'PLAY_TEXT': 'Play Text',
                          'STOP': 'Stop',
                          'STOP_ALL' : 'Stop All',
                          
                          'SPEECH_1' : 'Speech 1',
                          'SPEECH_2' : 'Speech 2',
                          'SPEECH_3' : 'Speech 3',
                          'SPEECH_4' : 'Speech 4',
                          'SPEECH_5' : 'Speech 5',
                          'SPEECH_6' : 'Speech 6',
                          'SPEECH_7' : 'Speech 7',
                          'SPEECH_8' : 'Speech 8',
                          'SPEECH_9' : 'Speech 9',
                          'SPEECH_10' : 'Speech 10',
                          'SPEECH_11' : 'Speech 11',
                          'SPEECH_12' : 'Speech 12',
                          
                          'SOUND_1' : 'Rooster',
                          'SOUND_2' : 'Drill',
                          'SOUND_3' : 'Bull call',
                          'SOUND_4' : 'Clown horn',
                          'SOUND_5' : 'Cash register',
                          'SOUND_6' : 'Glass breaking',
                          'SOUND_7' : 'Cell door',
                          'SOUND_8' : 'Cow',
                          'SOUND_9' : 'Birds',
                          'SOUND_10' : 'Big truck',
                          'SOUND_11' : 'Seagulls',
                          'SOUND_12' : 'Lift',
                          
                          'NEW_SPEECH_SET' : 'Create new speech set',
                          'PICK_SPEECH_SET' : 'Pick different speech set',
                          } 
        
    # ---------------------- Application Background styling --------------------     

    # Application background:
    veryLightBlue = QColor(230, 255,255);
    stylesheetAppBG = 'QDialog {background-color: %s}' % veryLightBlue.name();
    
    defaultStylesheet = stylesheetAppBG;
    defaultStylesheetName = "Default";
    
    # ---------------------- Edit field styling -----------------

    editFieldBGColor =  QColor(12,21,109);
    editFieldTextColor = QColor(244,244,246);

    inputFldStylesheet =\
        'TextPanel {background-color: ' + editFieldBGColor.name() +\
        '; color: ' + editFieldTextColor.name() +\
        '; font-size: ' + str(EDIT_FIELD_TEXT_SIZE) + 'pt' +\
        '}';
    
    # ---------------------- Pushbutton styling -------------------- 
    
    # Button color definitions:
    recorderButtonBGColor = QColor(176,220,245); # Light blue
    recorderButtonDisabledBGColor = QColor(187,200,208); # Gray-blue
    recorderButtonTextColor = QColor(0,0,0);     # Black
    programButtonBGColor = QColor(117,150,169);  # Middle blue
    programButtonTextColor = QColor(251,247,247);# Off-white
    soundButtonBGColor = QColor(110,134,211);      # Dark blue
    soundButtonTextColor = QColor(251,247,247);  # Off-white

    # Button stylesheets:    
    recorderButtonStylesheet =\
        'QPushButton {background-color: ' + recorderButtonBGColor.name() +\
        '; color: ' + recorderButtonTextColor.name() +\
        '; font-size: ' + str(BUTTON_LABEL_FONT_SIZE) + 'px' +\
        '}';
        
    recorderButtonDisabledStylesheet =\
        'QPushButton {background-color: ' + recorderButtonDisabledBGColor.name() +\
        '; color: ' + recorderButtonTextColor.name() +\
        '; font-size: ' + str(BUTTON_LABEL_FONT_SIZE) + 'px' +\
        '}';
        
    programButtonStylesheet =\
        'QPushButton {background-color: ' + programButtonBGColor.name() +\
        '; color: ' + programButtonTextColor.name() +\
        '; font-size: ' + str(BUTTON_LABEL_FONT_SIZE) + 'px' +\
        '}';

    # Stylesheet for when program button look is temporarily
    # changed to indicate transition to programming mode:                                                          
    programButtonModeTransitionStylesheet =\
        'QPushButton {background-color: ' + editFieldBGColor.name() +\
        '; color: ' + programButtonTextColor.name() +\
        '; font-size: ' + str(BUTTON_LABEL_FONT_SIZE) + 'px' +\
        '}';
                                                          
                                                          
    soundButtonStylesheet =\
        'QPushButton {background-color: ' + soundButtonBGColor.name() +\
        '; color: ' + soundButtonTextColor.name() +\
        '; font-size: ' + str(BUTTON_LABEL_FONT_SIZE) + 'px' +\
        '}';

    # ---------------------- Radiobutton styling -----------------
    
    # Radiobutton color definitions:
    playOnceRepeatButtonBGColor = QColor(121,229,230); # Very light cyan
    voicesButtonBGColor = QColor(97,164,165); # Darker cyan

    
    playOnceRepeatButtonStylesheet =\
     'font-size: ' + str(RADIO_BUTTON_LABEL_FONT_SIZE) + 'px' +\
     '; background-color: ' + playOnceRepeatButtonBGColor.name();

    voiceButtonStylesheet =\
     'font-size: ' + str(RADIO_BUTTON_LABEL_FONT_SIZE) + 'px' +\
     '; background-color: ' + voicesButtonBGColor.name();


    # ---------------------- Signals -----------------
    
    hideButtonSignal  	= Signal(QPushButton); # hide the given button
    showButtonSignal  	= Signal(QPushButton); # show the hidden button

    #----------------------------------
    # Initializer 
    #--------------
        
    def __init__(self, parent=None, mirrored=True):
        
        super(SpeakEasyGUI, self).__init__(parent);
        
        #self.setMaximumWidth(1360);
        #self.setMaximumHeight(760);
        
            # Vertical box to hold all top level widget groups
        appLayout = QVBoxLayout();
        appWidget = QDialog();      
        appWidget.setStyleSheet(SpeakEasyGUI.defaultStylesheet);
        # Activate the window resize handle in the lower right corner
        # of the app window:
        appWidget.setSizeGripEnabled(True);
        
        self.setCentralWidget(appWidget);

        self.addTitle(appLayout);        
        self.addTxtInputFld(appLayout);
        self.buildTapeRecorderButtons(appLayout);
        self.addOnceOrRepeat_And_VoiceRadioButtons(appLayout);
        
        #TODO: play repeatedly is not working:
        self.onceOrRepeatDict[SpeakEasyGUI.interactionWidgets['PLAY_REPEATEDLY']].setDisabled(True)
        
        self.buildHorizontalDivider(appLayout);
        self.buildProgramButtons(appLayout);
        self.buildSoundButtons(appLayout);
        self.buildButtonSetControls(appLayout);
        
        #self.connectSignalsToWidgets();        
        
        appWidget.setLayout(appLayout);
        
        self.show();

    #----------------------------------
    # programButtonIterator 
    #--------------

    def programButtonIterator(self, gridLayout=None):
        
        if gridLayout is None:
            gridLayout = self.programButtonGridLayout;
        
        programButtonArray = [];
        # Collect the buttons into a flat array:
        for row in range(gridLayout.rowCount()):
            for col in range(SpeakEasyGUI.NUM_OF_PROGRAM_BUTTON_COLUMNS):
                programButtonArray.append(self.programButtonGridLayout.itemAtPosition(row, col).widget());
        return iter(programButtonArray);

    #----------------------------------
    # replaceProgramButtons
    #--------------

    def replaceProgramButtons(self, buttonProgramsArray):
        
        programButtonObjIt = self.programButtonIterator();
        try:
            programButtonObj = programButtonObjIt.next();
            self.programButtonGridLayout.removeWidget(programButtonObj);
            programButtonObj.deleteLater();
        except StopIteration:
            pass
        
        # Make an array of button labels from the ButtonProgram
        # instances in buttonProgramsArray:
        buttonLabelArr = [];
        for buttonProgram in buttonProgramsArray:
            buttonLabelArr.append(buttonProgram.getLabel());
            
        # No more program buttons present. Make 
        # new ones, and add them to the grid layout:
        (newProgramButtonGridLayout, self.programButtonDict) =\
            SpeakEasyGUI.buildButtonGrid(buttonLabelArr,
                                         SpeakEasyGUI.NUM_OF_PROGRAM_BUTTON_COLUMNS);
        for buttonObj in self.programButtonDict.values():
            buttonObj.setStyleSheet(SpeakEasyGUI.programButtonStylesheet);
            buttonObj.setMinimumHeight(SpeakEasyGUI.BUTTON_MIN_HEIGHT);

        # Transfer the buttons from this new gridlayout object to the
        # original layout that is embedded in the application Dialog:
        newButtonsIt = self.programButtonIterator(gridLayout=newProgramButtonGridLayout);
        try:
            self.programButtonGridLayout.addWidget(newButtonsIt.next());
        except StopIteration:
            pass;
    
    #----------------------------------
    # connectSignalsToWidgets 
    #--------------
    
#    def connectSignalsToWidgets(self):
#        self.commChannel = CommChannel();
#        self.commChannel.hideButtonSignal.connect(SpeakEasyGUI.hideButtonHandler);
#        self.commChannel.showButtonSignal.connect(SpeakEasyGUI.showButtonHandler);


    #----------------------------------
    # addTitle 
    #--------------

    def addTitle(self,layout):
        title = QLabel("<H1>SpeakEasy</H1>");
        hbox = QHBoxLayout();
        hbox.addStretch(1);
        hbox.addWidget(title);
        hbox.addStretch(1);
        
        layout.addLayout(hbox);
        
        
    #----------------------------------
    # addTxtInputFld 
    #--------------
        
    def addTxtInputFld(self, layout):
        '''
        Creates text input field label and text field
        in a horizontal box layout. Adds that hbox layout
        to the passed-in layout.
        
        Sets instance variables:
        1. C{self.speechInputFld}
         
        @param layout: Layout object to which the label/txt-field C{hbox} is to be added.
        @type  layout: QLayout
        '''
        speechControlsLayout = QHBoxLayout();
        speechControlsLayout.addStretch(1);
        speechInputFldLabel = QLabel("<b>What to say:</b>")
        speechControlsLayout.addWidget(speechInputFldLabel);
        
        self.speechInputFld = TextPanel(numLines=5);
        self.speechInputFld.setStyleSheet(SpeakEasyGUI.inputFldStylesheet);
        self.speechInputFld.setFontPointSize(SpeakEasyGUI.EDIT_FIELD_TEXT_SIZE);
        
        speechControlsLayout.addWidget(self.speechInputFld);
        
        layout.addLayout(speechControlsLayout);
    
    #----------------------------------
    # addVoiceRadioButtons
    #--------------
            
    def addOnceOrRepeat_And_VoiceRadioButtons(self, layout):
        '''
        Creates radio buttons for selecting whether a
        sound is to play once, or repeatedly until stopped.
        Also adds radio buttons for selecting voices.
        Places all in a horizontal box layout. Adds 
        that hbox layout to the passed-in layout.
        
        Sets instance variables:
        1. C{self.onceOrRepeatDict}
        2. C{self.voicesRadioButtonsDict}
         
        @param layout: Layout object to which the label/txt-field C{hbox} is to be added.
        @type  layout: QLayout
         '''
        
        hbox = QHBoxLayout();

        (self.onceOrRepeatGroup, onceOrRepeatButtonLayout, self.onceOrRepeatDict) =\
            self.buildRadioButtons([SpeakEasyGUI.interactionWidgets['PLAY_ONCE'], 
                                    SpeakEasyGUI.interactionWidgets['PLAY_REPEATEDLY']
                                    ],
                                   Orientation.HORIZONTAL,
                                   Alignment.LEFT,
                                   activeButton=SpeakEasyGUI.interactionWidgets['PLAY_ONCE'],
                                   behavior=CheckboxGroupBehavior.RADIO_BUTTONS);
        
        (self.voicesGroup, voicesButtonLayout, self.voicesRadioButtonsDict) =\
            self.buildRadioButtons([SpeakEasyGUI.interactionWidgets['VOICE_1'],
                                    SpeakEasyGUI.interactionWidgets['VOICE_2']
                                    ],
                                   Orientation.HORIZONTAL,
                                   Alignment.RIGHT,
                                   activeButton=SpeakEasyGUI.interactionWidgets['VOICE_1'],
                                   behavior=CheckboxGroupBehavior.RADIO_BUTTONS);
                                   
        # Style all the radio buttons:
        for playFreqButton in self.onceOrRepeatDict.values():
            playFreqButton.setStyleSheet(SpeakEasyGUI.playOnceRepeatButtonStylesheet); 
        for playFreqButton in self.voicesRadioButtonsDict.values():
            playFreqButton.setStyleSheet(SpeakEasyGUI.voiceButtonStylesheet); 
                                   
        hbox.addLayout(onceOrRepeatButtonLayout);
        hbox.addStretch(1);
        hbox.addLayout(voicesButtonLayout);
        layout.addLayout(hbox);
        
    #----------------------------------
    # buildTapeRecorderButtons 
    #--------------
        
    def buildTapeRecorderButtons(self, layout):
        '''
        Creates tape recorder buttons (Play Text, Stop,etc.).
        Places all in a row, though the layout is a 
        QGridLayout. Adds QGridLayout to the passed-in 
        layout.
        
        Sets instance variables:
        1. C{self.recorderDict}
         
        @param layout: Layout object to which the label/txt-field C{QGridlayout} is to be added.
        @type  layout: QLayout
        '''
        
        (buttonGridLayout, self.recorderButtonDict) =\
            SpeakEasyGUI.buildButtonGrid([SpeakEasyGUI.interactionWidgets['PLAY_TEXT'],
                                          SpeakEasyGUI.interactionWidgets['STOP'],
                                          SpeakEasyGUI.interactionWidgets['STOP_ALL']
                                          ],
                                         3); # Three columns
        for buttonObj in self.recorderButtonDict.values():
            buttonObj.setStyleSheet(SpeakEasyGUI.recorderButtonStylesheet);
            buttonObj.setMinimumHeight(SpeakEasyGUI.BUTTON_MIN_HEIGHT);
        layout.addLayout(buttonGridLayout);                                 
         
    #----------------------------------
    # buildProgramButtons
    #--------------
        
    def buildProgramButtons(self, layout):
        '''
        Creates grid of buttons for saving sounds.
        Adds the resulting QGridLayout to the passed-in 
        layout.
        
        Sets instance variables:
        1. C{self.programButtonDict}
         
        @param layout: Layout object to which the label/txt-field C{QGridlayout} is to be added.
        @type  layout: QLayout
        '''

        buttonLabelArr = [];
        for i in range(12):
            key = "SPEECH_" + str(i+1);
            buttonLabelArr.append(SpeakEasyGUI.interactionWidgets[key]);
                    
        (self.programButtonGridLayout, self.programButtonDict) =\
            SpeakEasyGUI.buildButtonGrid(buttonLabelArr,
                                         SpeakEasyGUI.NUM_OF_PROGRAM_BUTTON_COLUMNS);
        for buttonObj in self.programButtonDict.values():
            buttonObj.setStyleSheet(SpeakEasyGUI.programButtonStylesheet);
            buttonObj.setMinimumHeight(SpeakEasyGUI.BUTTON_MIN_HEIGHT);
            
        layout.addLayout(self.programButtonGridLayout);                                 
        
    #----------------------------------
    # buildSoundButtons
    #--------------
        
    def buildSoundButtons(self, layout):
        '''
        Creates grid of buttons for playing canned sounds.
        Adds the resulting QGridLayout to the passed-in 
        layout.
        
        Sets instance variables:
        1. C{self.soundButtonDict}
         
        @param layout: Layout object to which the label/txt-field C{QGridlayout} is to be added.
        @type  layout: QLayout
        '''
        buttonLabelArr = [];
        for i in range(12):
            key = "SOUND_" + str(i+1);
            buttonLabelArr.append(SpeakEasyGUI.interactionWidgets[key]);
        
        (buttonGridLayout, self.soundButtonDict) =\
            SpeakEasyGUI.buildButtonGrid(buttonLabelArr, SpeakEasyGUI.NUM_OF_SOUND_BUTTON_COLUMNS);
        for buttonObj in self.soundButtonDict.values():
            buttonObj.setStyleSheet(SpeakEasyGUI.soundButtonStylesheet);
            buttonObj.setMinimumHeight(SpeakEasyGUI.BUTTON_MIN_HEIGHT);
                        
        layout.addLayout(buttonGridLayout);                                 

    #----------------------------------
    # buildButtonSetControls
    #--------------

    def buildButtonSetControls(self, layout):
        
        buttonLabelArray = [self.interactionWidgets['NEW_SPEECH_SET'], self.interactionWidgets['PICK_SPEECH_SET']]; 
        # Two columns of buttons:
        (buttonGridLayout, self.speechSetButtonDict) = SpeakEasyGUI.buildButtonGrid(buttonLabelArray, 2);
        for buttonObj in self.speechSetButtonDict.values():
            buttonObj.setStyleSheet(SpeakEasyGUI.programButtonStylesheet);
            buttonObj.setMinimumHeight(SpeakEasyGUI.BUTTON_MIN_HEIGHT);
        layout.addLayout(buttonGridLayout);
        
    #----------------------------------
    # buildHorizontalDivider 
    #--------------
        
    def buildHorizontalDivider(self, layout):
        frame = QFrame();
        frame.setFrameShape(QFrame.HLine);
        frame.setFrameStyle(QFrame.Shadow.Sunken);
        frame.setLineWidth(3);
        frame.setMidLineWidth(3);
        layout.addWidget(frame);
        
        
    #----------------------------------
    # buildRadioButtons
    #--------------
        
    # Note: Whenever a button is switched on or off it emits the 
    # PySide.QtGui.QAbstractButton.toggled() signal. Connect to this 
    # signal if you want to trigger an action each time the button changes state.         
    
    def buildRadioButtons(self, 
                          labelTextArray,
                          orientation,
                          alignment,
                          activeButton=None, 
                          behavior=CheckboxGroupBehavior.RADIO_BUTTONS):
        '''
        @param labelTextArray: Names of buttons
        @type labelTextArray: [string]
        @param orientation: Whether to arrange the buttons vertically or horizontally.
        @type orientation: Orientation
        @param alignment: whether buttons should be aligned Left/Center/Right for horizontal,
                          Top/Center/Bottom for vertical:/c
        @type  alignment: Alignment
        @param activeButton: Name of the button that is to be checked initially. Or None.
        @type activeButton: string
        @param behavior: Indicates whether the button group is to behave like Radio Buttons, or like Checkboxes.
        @type behavior: CheckboxGroupBehavior
        @return 1. The button group that contains the related buttons. Caller: ensure that 
                   this object does not go out of scope. 
                2. The button layout, which callers will need to add to
                   their own layouts.
                3. and a dictionary mapping button names to button objects that
                   were created within this method. This dict is needed by the
                   controller.
        @returnt (QButtonGroup, QLayout, dict<string,QRadioButton>).
        '''
        
        # Button control management group. It has no visible
        # representation. It allows the radio button bahavior, for instance:
        buttonGroup = QButtonGroup();
        
        if behavior == CheckboxGroupBehavior.CHECKBOXES:
            buttonGroup.setExclusive(False);
        else:
            buttonGroup.setExclusive(True);
        
        if orientation == Orientation.VERTICAL:
            buttonCompLayout = QVBoxLayout();
            # Arrange for the alignment (Part 1):
            if (alignment == Alignment.CENTER) or\
               (alignment == Alignment.BOTTOM):
                buttonCompLayout.addStretch(1);
        else:
            buttonCompLayout = QHBoxLayout();
            # Arrange for the alignment (Part 1):
            if (alignment == Alignment.CENTER) or\
               (alignment == Alignment.RIGHT):
                buttonCompLayout.addStretch(1);
            
        # Build the buttons:
        buttonDict  = {};
        
        for label in labelTextArray:
            button = QRadioButton(label);
            buttonDict[label] = button;
            # Add button to the button management group:
            buttonGroup.addButton(button);
            # Add the button to the visual group:
            buttonCompLayout.addWidget(button);
            if label == activeButton:
                button.setChecked(True);

        if orientation == Orientation.VERTICAL:
            buttonCompLayout = QVBoxLayout();
            # Arrange for the alignment (Part 2):
            if (alignment == Alignment.CENTER) or\
               (alignment == Alignment.TOP):
                buttonCompLayout.addStretch(1);
        else:
            # Arrange for the alignment (Part 2):
            if (alignment == Alignment.CENTER) or\
               (alignment == Alignment.LEFT):
                buttonCompLayout.addStretch(1);
                        
        return (buttonGroup, buttonCompLayout, buttonDict);
    
    #----------------------------------
    # buildButtonGrid
    #--------------

    @staticmethod
    def buildButtonGrid(labelTextArray, 
                        numColumns):
        '''
        Creates a grid of QPushButton widgets. They will be 
        
        @param labelTextArray: Button labels.
        @type labelTextArray: [string]
        @param numColumns: The desired width of the button grid.
        @type numColumns: int
        @return: 1. a grid layout with the button objects inside.
                 2. a dictionary mapping button labels to button objects.
        @returnt: QGridLayout 
        '''

        buttonLayout = QGridLayout();
        
        # Compute number of button rows:
        numRows = len(labelTextArray) / numColumns;
        # If this number of rows leaves a few buttons left over (< columnNum),
        # then add a row for those:
        if (len(labelTextArray) % numColumns) > 0:
            numRows += 1;
        
        buttonDict = {}
        
        row = 0;
        col = 0;
        for buttonLabel in labelTextArray:
            button = QPushButton(buttonLabel);
            buttonDict[buttonLabel] = button;
            buttonLayout.addWidget(button, row, col);
            col += 1;
            if (col > (numColumns - 1)):
                col = 0;
                row += 1;
                
        return (buttonLayout, buttonDict);
        
    #----------------------------------
    # getNewButtonLabel 
    #--------------

    def getNewButtonLabel(self):
        '''
        Requests a new button label from the user.
        Returns None if user canceled out, or a string
        with the new button label. 
        @return: None if user canceled, else string from input field.
        @returnt: {None | string}
        
        '''
        prompt = "New label for this button:";
        dialogBox = QInputDialog(parent=self);
        dialogBox.setStyleSheet(SpeakEasyGUI.stylesheetAppBG);
        dialogBox.setLabelText(prompt);
        dialogBox.setCancelButtonText("Keep existing label");
        userChoice = dialogBox.exec_();

        if userChoice == QDialog.Accepted:
            return dialogBox.textValue();
        else:
            return None;
        
    #----------------------------------
    # playOnceChecked 
    #--------------

    def playOnceChecked(self):
        return self.onceOrRepeatDict[SpeakEasyGUI.interactionWidgets['PLAY_ONCE']].isChecked();
    
    
    #----------------------------------
    # playRepeatedlyChecked 
    #--------------

    def playRepeatedlyChecked(self):
        return self.onceOrRepeatDict[SpeakEasyGUI.interactionWidgets['PLAY_REPEATEDLY']].isChecked();
    
    #----------------------------------
    # activeVoice
    #--------------

    def activeVoice(self):
        if self.voicesRadioButtonsDict[SpeakEasyGUI.interactionWidgets['VOICE_1']].isChecked():
            return SpeakEasyGUI.voices['VOICE_1'];
        elif self.voicesRadioButtonsDict[SpeakEasyGUI.interactionWidgets['VOICE_2']].isChecked():
            return SpeakEasyGUI.voices['VOICE_2'];
    
    #----------------------------------
    # setButtonLabel 
    #--------------
    
    def setButtonLabel(self, buttonObj, label):
        buttonObj.setText(label);

    #----------------------------------
    # blinkButton
    #--------------

    def blinkButton(self, buttonObj, turnOn):
        '''
        Used to make a program button blink in some way to indicate
        that it is changing into programming mode. Since this method
        is triggered by a timer thread, it cannot make any GUI changes.
        Instead, it sends a signal to have the GUI thread place the
        button into an alternative look. It then schedules a call
        to itself for a short time later. At that point it sends a
        signal to the GUI thread to return the button to its usual
        look:
        @param buttonObj: The button to be blinked
        @type  buttonObj: QPushButton
        @param turnOn:    Indicates whether to turn the button back into 
                          its normal state, or whether to make it take on
                          its alternate look. 
        @type  turnOn:    bool
        '''
        if turnOn:
            self.showButtonSignal.emit(buttonObj);
        else:
            self.hideButtonSignal.emit(buttonObj);
            self.buttonBlinkTimer = Timer(SpeakEasyGUI.PROGRAM_BUTTON_LOOK_CHANGE_DURATION, partial(self.blinkButton, buttonObj, True));
            self.buttonBlinkTimer.start();
    

def alternateLookHandler(buttonObj):
    #buttonObj.hide();
    buttonObj.setStyleSheet(SpeakEasyGUI.programButtonModeTransitionStylesheet);
    
def standardLookHandler(buttonObj):
    #buttonObj.show();
    buttonObj.setStyleSheet(SpeakEasyGUI.programButtonStylesheet);

if __name__ == "__main__":

    # Create a Qt application
    
#    style = QStyleFactory.create("Plastique");
#    style = QStyleFactory.create("Motif");
#    style = QStyleFactory.create("GTK+");
#    style = QStyleFactory.create("Windows");

    style = QStyleFactory.create("Cleanlooks");
    QApplication.setStyle(style);
    app = QApplication(sys.argv);
        
    soundPlayGui = SpeakEasyGUI();
    soundPlayGui.hideButtonSignal.connect(alternateLookHandler);
    soundPlayGui.showButtonSignal.connect(standardLookHandler);
    
    # Enter Qt application main loop
    sys.exit(app.exec_());
        
        
