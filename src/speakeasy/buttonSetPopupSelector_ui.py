#!/usr/bin/env python

import sys
from functools import partial

from PySide.QtCore import * #@UnusedWildImport
from PySide.QtGui import * #@UnusedWildImport

from speakeasy_ui import SpeakEasyGUI;

class NextPrev:
    NEXT     = 0;
    PREVIOUS = 1;

class ButtonSetPopupSelector(QDialog):
    
    def __init__(self, buttonNameArrayIterator):
        
        super(ButtonSetPopupSelector, self).__init__();
        
        self.setStyleSheet(SpeakEasyGUI.stylesheetAppBG);
        
        self.buttonLabelArrayIt = buttonNameArrayIterator;
        self.shownLabelArrays = [];
        self.currentlyShowingSetIndex = -1;
        self.rootLayout = None;
        
        self.cancelButton = QPushButton("Cancel");
        self.cancelButton.setStyleSheet(SpeakEasyGUI.recorderButtonStylesheet);
        self.cancelButton.setMinimumHeight(SpeakEasyGUI.BUTTON_MIN_HEIGHT);
        self.cancelButton.clicked.connect(partial(QDialog.done, self, 0));
        
        self.OKButton = QPushButton("Pick this One");
        self.OKButton.setStyleSheet(SpeakEasyGUI.recorderButtonStylesheet);
        self.OKButton.setMinimumHeight(SpeakEasyGUI.BUTTON_MIN_HEIGHT);
        self.OKButton.clicked.connect(partial(QDialog.done, self, 1));
        
        self.currentNextPrevDirection = NextPrev.NEXT;

        self.nextPrevButton = QPushButton("Next");
        self.setNextPrevButtonDirection();
        self.nextPrevButton.setStyleSheet(SpeakEasyGUI.recorderButtonStylesheet);
        self.nextPrevButton.setMinimumHeight(SpeakEasyGUI.BUTTON_MIN_HEIGHT);
        self.nextPrevButton.clicked.connect(self.offerNewButtonSet);
        
        self.endOfSetsLabel = QLabel("<b>No more button sets.</b>")

        self.offerNewButtonSet();
        
    def getCurrentlyShowingSet(self):
        return self.shownLabelArrays[self.currentlyShowingSetIndex];

    def offerNewButtonSet(self):

        # Make sure the 'No more button sets in this direction" 
        # message gets deleted:
        self.endOfSetsLabel.hide();
        self.OKButton.setEnabled(True);

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
            return;
        
        # Wants next button set. Is there one we haven't shown yet?:
        try:
            # Get next set of button labels from caller:
            buttonLabelArray = self.buttonLabelArrayIt.next();
            # Save that list of button names:
            self.shownLabelArrays.append(buttonLabelArray);
            self.currentlyShowingSetIndex += 1;
            self.buildButtonSet(buttonLabelArray);
            return;
        except StopIteration:
            # no more button sets available from caller's iterator:
            self.currentlyShowingSetIndex += 1;
        
        if self.currentlyShowingSetIndex >= len(self.shownLabelArrays):
            # Neither are there more new sets, or are be going
            # through the already shown sets going forward a second
            # time. So reverse direction to go backwards through the
            # the already shown sets:     
            self.flipNextPrevDirection();
            return;

        self.buildButtonSet(self.getCurrentlyShowingSet());

        return;
        
    def flipNextPrevDirection(self):

        self.clearDialogPane();
        
        self.rootLayout.addWidget(self.endOfSetsLabel);
        # Show the 'End of sets' label, and 
        # de-activate the OK button:
        self.endOfSetsLabel.show();
        self.OKButton.setEnabled(False);
        
        if (self.currentNextPrevDirection == NextPrev.PREVIOUS):
            self.currentNextPrevDirection = NextPrev.NEXT;
            self.currentlyShowingSetIndex = -1;
        else:
            self.currentNextPrevDirection = NextPrev.PREVIOUS;
            self.currentlyShowingSetIndex = len(self.shownLabelArrays);
            
        self.setNextPrevButtonDirection();
        self.addChoiceButtons(self.rootLayout);
        self.setLayout(self.rootLayout);

    def clearDialogPane(self):
        self.rootLayout.removeWidget(self.cancelButton);
        self.rootLayout.removeWidget(self.OKButton);
        self.rootLayout.removeWidget(self.nextPrevButton);
        self.rootLayout.removeItem(self.buttonGridLayout);
        for buttonObj in self.programButtonDict.values():
            buttonObj.hide();

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
        self.setNextPrevButtonDirection();
        choiceButtonRowLayout.addWidget(self.nextPrevButton);
        choiceButtonRowLayout.addWidget(self.cancelButton);
        choiceButtonRowLayout.addWidget(self.OKButton);
        layout.addLayout(choiceButtonRowLayout);

    def setNextPrevButtonDirection(self):
        if self.currentNextPrevDirection == NextPrev.NEXT:
            self.nextPrevButton.setText("Next");
        else:
            self.nextPrevButton.setText("Previous");

        
if __name__ == "__main__":
    
    style = QStyleFactory.create("Cleanlooks");
    QApplication.setStyle(style);
    app = QApplication(sys.argv);
    
    testButtonSetArray = [
                          ["Button1.1", "Button2.1", "Button3.1", "Button4.1"],
                          ["Button1.2", "Button2.2", "Button3.2", "Button4.2"],
                          ["Button1.3", "Button2.3", "Button3.3", "Button4.3"]
                          ];
    buttonSetChoiceDialog = ButtonSetPopupSelector(iter(testButtonSetArray));
    #buttonSetChoiceDialog.show();
    choiceResult = buttonSetChoiceDialog.exec_();
    print "Choice result: " + str(choiceResult); 
    
    # Enter Qt application main loop
    sys.exit(app.exec_());    
