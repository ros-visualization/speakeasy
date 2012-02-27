#!/usr/bin/env python

import sys

from PySide.QtCore import * #@UnusedWildImport
from PySide.QtGui import * #@UnusedWildImport

from speakeasy_ui import SpeakEasyGUI;
from speakeasy_ui import DialogService;

class NextPrev:
    NEXT     = 0;
    PREVIOUS = 1;

#----------------------------------------------------- ButtonSetPopupSelector Class -----------------------------  

class ButtonSetPopupSelector(QDialog):
    
    def __init__(self, buttonNameArrayIterator):
        
        super(ButtonSetPopupSelector, self).__init__();
        # For error message popups:
        self.dialogService = DialogService();
        
        self.setStyleSheet(SpeakEasyGUI.stylesheetAppBG);
        
        self.buttonLabelArrayIt = buttonNameArrayIterator;
        self.shownLabelArrays = [];
        self.currentlyShowingSetIndex = None;
        self.rootLayout = None;
        
        self.cancelButton = QPushButton("Cancel");
        self.cancelButton.setStyleSheet(SpeakEasyGUI.recorderButtonStylesheet);
        self.cancelButton.setMinimumHeight(SpeakEasyGUI.BUTTON_MIN_HEIGHT);
        #self.cancelButton.clicked.connect(partial(QDialog.done, self, 0));
        self.cancelButton.clicked.connect(self.clickedCancelButton);
        
        self.OKButton = QPushButton("Pick this One");
        self.OKButton.setStyleSheet(SpeakEasyGUI.recorderButtonStylesheet);
        self.OKButton.setMinimumHeight(SpeakEasyGUI.BUTTON_MIN_HEIGHT);
        #self.OKButton.clicked.connect(partial(QDialog.done, self, 1));
        self.OKButton.clicked.connect(self.clickedOKButton);
        
        self.currentNextPrevDirection = NextPrev.NEXT;

        self.nextButton = QPushButton("Next");
        self.nextButton.setStyleSheet(SpeakEasyGUI.recorderButtonStylesheet);
        self.nextButton.setMinimumHeight(SpeakEasyGUI.BUTTON_MIN_HEIGHT);
        self.nextButton.clicked.connect(self.clickedNextButton);

        self.prevButton = QPushButton("Previous");
        self.prevButton.setStyleSheet(SpeakEasyGUI.recorderButtonStylesheet);
        self.prevButton.setMinimumHeight(SpeakEasyGUI.BUTTON_MIN_HEIGHT);
        self.prevButton.clicked.connect(self.clickedPrevButton);

        self.setNextPrevButtonsEnabledness();
        
        self.endOfSetsLabel = QLabel("<b>No more button sets.</b>")
        self.noSetsLabel = QLabel("<b>No button sets available.</b>")

        self.offerNewButtonSet();
        
    def getCurrentlyShowingSet(self):
        return self.shownLabelArrays[self.currentlyShowingSetIndex];

    def offerNewButtonSet(self):

        # Make sure the 'No more button sets in this direction" 
        # message gets deleted:
        self.endOfSetsLabel.hide();
        self.enableActionButton(self.OKButton);

        # Want to display a previously displayed button set?        
        if self.currentNextPrevDirection == NextPrev.PREVIOUS:
            # Are we showing the first set, i.e. is there no set
            # before the current one?
            if self.currentlyShowingSetIndex == 0:
                # Show the 'nothing before this one' label,
                # and switch directions:
                self.flipNextPrevDirection();
                return;
            self.currentlyShowingSetIndex -= 1;
            self.buildButtonSet(self.getCurrentlyShowingSet());
            self.setNextPrevButtonsEnabledness();
            return;
        
        # Wants next button set. Is there one we haven't shown yet?:
        try:
            # Get next set of button labels from caller:
            buttonLabelArray = self.buttonLabelArrayIt.next();
            
            # If this is the very first time we pull a button set
            # from the caller, then our current index will be None
            # from the initialization. We use that to check for 
            # the case that the caller has no sets at all to feed
            # out. Since we didn't bomb with StopIteration, we know
            # that there is at least one available button set: 
            if self.currentlyShowingSetIndex is None:
                self.currentlyShowingSetIndex = -1;
                
            # Save that list of button names:
            self.shownLabelArrays.append(buttonLabelArray);
            self.currentlyShowingSetIndex += 1;
            self.buildButtonSet(buttonLabelArray);
            self.setNextPrevButtonsEnabledness();
            return;
        except StopIteration:
            # If we fell through the very first time, there is no
            # button set available at all:
            if self.currentlyShowingSetIndex is None:
                self.terminateWithWarning();
                return;
            # No more button sets available from caller's iterator:
            self.currentlyShowingSetIndex += 1;
        
        if self.currentlyShowingSetIndex >= len(self.shownLabelArrays):
            # Neither are there more new sets, or are be going
            # through the already shown sets going forward a second
            # time. So reverse direction to go backwards through the
            # the already shown sets:     
            self.flipNextPrevDirection();
            return;

        self.buildButtonSet(self.getCurrentlyShowingSet());
        self.setNextPrevButtonsEnabledness();

        return;
        
    def flipNextPrevDirection(self):

        self.clearDialogPane();
        
        self.rootLayout.addWidget(self.endOfSetsLabel);
        # Show the 'End of sets' label, and 
        # de-activate the OK button:
        self.endOfSetsLabel.show();
        self.disableActionButton(self.OKButton);
        
        if (self.currentNextPrevDirection == NextPrev.PREVIOUS):
            self.currentNextPrevDirection = NextPrev.NEXT;
            self.currentlyShowingSetIndex = -1;
        else:
            self.currentNextPrevDirection = NextPrev.PREVIOUS;
            self.currentlyShowingSetIndex = len(self.shownLabelArrays);
            
        self.setNextPrevButtonsEnabledness();
        self.addChoiceButtons(self.rootLayout);
        self.setLayout(self.rootLayout);

    def clearDialogPane(self):
        if self.rootLayout is None:
            return;
        
        self.rootLayout.removeWidget(self.cancelButton);
        self.rootLayout.removeWidget(self.OKButton);
        self.rootLayout.removeWidget(self.nextButton);
        self.rootLayout.removeWidget(self.prevButton);
        self.rootLayout.removeItem(self.buttonGridLayout);
        self.buttonGritLayout = None;
        for buttonObj in self.programButtonDict.values():
            if buttonObj is not None:
                buttonObj.hide();
        for key in self.programButtonDict.keys():
            self.programButtonDict[key] = None;

        self.setLayout(self.rootLayout);

    def buildButtonSet(self, buttonLabelArray):

        # If we never made the root layout, make it now:
        if self.rootLayout is None:
            self.rootLayout = QVBoxLayout();
        else:
            # A previous button set is being displayed,
            # Empty out the rootLayout:
            self.clearDialogPane();
            
        # Get a layout filled with the button objects
        # in the proper styling. Also get a dictionary
        # mapping button labels to button objects:
        
        (self.buttonGridLayout, self.programButtonDict) =\
            SpeakEasyGUI.buildButtonGrid(buttonLabelArray,
                                         SpeakEasyGUI.NUM_OF_PROGRAM_BUTTON_COLUMNS);
    
        for buttonObj in self.programButtonDict.values():
            buttonObj.setStyleSheet(SpeakEasyGUI.programButtonStylesheet);
            buttonObj.setMinimumHeight(SpeakEasyGUI.BUTTON_MIN_HEIGHT);

        # Place the grid into a V-layout, and add
        # the next/previous, cancel, and OK buttons;
        self.rootLayout.addLayout(self.buttonGridLayout);
        self.addChoiceButtons(self.rootLayout);
                                                
        # Set the popup window's layout to the button grid
        # plus choice button combo:
        self.setLayout(self.rootLayout);
        
    def addChoiceButtons(self, layout):
        choiceButtonRowLayout = QHBoxLayout();
        choiceButtonRowLayout.addWidget(self.nextButton);
        choiceButtonRowLayout.addWidget(self.prevButton);
        choiceButtonRowLayout.addWidget(self.cancelButton);
        choiceButtonRowLayout.addWidget(self.OKButton);
        layout.addLayout(choiceButtonRowLayout);

    def setNextPrevButtonsEnabledness(self):
        if self.currentlyShowingSetIndex >= len(self.shownLabelArrays):
            # Can only go backwards:
            self.disableActionButton(self.nextButton);
            self.enableActionButton(self.prevButton);
        elif self.currentlyShowingSetIndex <= 0:
            # Can only go forward:
            self.enableActionButton(self.nextButton);
            self.disableActionButton(self.prevButton);
        else:
            self.enableActionButton(self.nextButton);
            self.enableActionButton(self.prevButton);

    def enableActionButton(self, buttonObj):
        buttonObj.setEnabled(True)
        buttonObj.setStyleSheet(SpeakEasyGUI.recorderButtonStylesheet);

    def disableActionButton(self, buttonObj):
        buttonObj.setEnabled(False)
        buttonObj.setStyleSheet(SpeakEasyGUI.recorderButtonDisabledStylesheet);

    def clickedNextButton(self):
        if self.currentNextPrevDirection == NextPrev.PREVIOUS:
            self.currentNextPrevDirection = NextPrev.NEXT;
        self.offerNewButtonSet();
    
    def clickedPrevButton(self):
        if self.currentNextPrevDirection == NextPrev.NEXT:
            self.currentNextPrevDirection = NextPrev.PREVIOUS;
        self.offerNewButtonSet();

    def clickedCancelButton(self):
        self.clearDialogPane();
        self.done(0);

    def clickedOKButton(self):
        self.clearDialogPane();
        self.done(1);

    def terminateWithWarning(self):
        # No buttons at all:
        self.clearDialogPane();
        
        if self.rootLayout is None:
            self.close();
        self.rootLayout.addWidget(self.noSetsLabel);
        # Show the 'Nothing available' label, and 
        self.noSetsLabel.show();
        self.disableActionButton(self.OKButton);

#-----------------------------------------------------  Tester Class -----------------------------
          
class Tester(QDialog):
    
    def __init__(self):
        super(Tester, self).__init__();
        vlayout = QVBoxLayout();
        self.testButton = QPushButton("Exactly one row of buttons");
        vlayout.addWidget(self.testButton);
        self.testButton.clicked.connect(self.exactlyOneRow);
        
        self.setLayout(vlayout);
        self.show();
        
    def exactlyOneRow(self):
        # Exactly one row even:    
        testButtonSetArray = [
                              ["Button1.1", "Button2.1", "Button3.1", "Button4.1"],
                              ["Button1.2", "Button2.2", "Button3.2", "Button4.2"],
                              ["Button1.3", "Button2.3", "Button3.3", "Button4.3"]
                              ];
        buttonSetChoiceDialog = ButtonSetPopupSelector(iter(testButtonSetArray));
        buttonSetChoiceDialog.show();
        choiceResult = buttonSetChoiceDialog.exec_();
        print "Choice result: " + str(choiceResult); 
        buttonSetChoiceDialog = None;
        
        # Rewire the test button for the next text:
        self.testButton.setText("One button available");
        self.testButton.clicked.disconnect();
        self.testButton.clicked.connect(self.oneButtonAvailable);
        
    def oneButtonAvailable(self):
        # One button available
        testButtonSetArray = [["Single button."]];
        buttonSetChoiceDialog = ButtonSetPopupSelector(iter(testButtonSetArray));
        buttonSetChoiceDialog.show();
        choiceResult = buttonSetChoiceDialog.exec_();
        print "Choice result: " + str(choiceResult);
        buttonSetChoiceDialog = None; 

        self.testButton.setText("No button set available");
        self.testButton.clicked.connect(self.noSetAvailable);
        
    def noSetAvailable(self):
        testButtonSetArray = [];
        buttonSetChoiceDialog = ButtonSetPopupSelector(iter(testButtonSetArray));
        buttonSetChoiceDialog.show();
        choiceResult = buttonSetChoiceDialog.exec_();
        print "Choice result: " + str(choiceResult);
        buttonSetChoiceDialog = None; 
        
        # Rewire the test button for the next text:
        self.testButton.setText("Exit");
        self.testButton.clicked.connect(self.close);
        
    def exitApp(self):
        self.close();
    
if __name__ == "__main__":
    
    style = QStyleFactory.create("Cleanlooks");
    QApplication.setStyle(style);
    app = QApplication(sys.argv);

    theTester = Tester();
    
        
    # Enter Qt application main loop
    sys.exit(app.exec_());    
