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

import sys

from speakeasy_ui import SpeakEasyGUI;

from python_qt_binding.QtGui import QVBoxLayout, QHBoxLayout, QGridLayout, QDialog, QLabel, QPushButton, QStyleFactory, QApplication


class NextPrev:
    NEXT     = 0;
    PREVIOUS = 1;

#----------------------------------------------------- ButtonSetPopupSelector Class -----------------------------  

class ButtonSetPopupSelector(QDialog):
    '''
    An interactive dialog that displays successive sets of
    buttons. When each set is displayed, user may ask for the
    next available set, the previous. already seen set, or the
    user may accept the currently displayed set. A cancel is 
    available as well. 
    
    The call and return protocol is as follows:

      - The caller creates a fresh dialog instance of this class,
        passing an iterator. The iterator's next() method returns 
        an array of ButtonProgram instances each time it is called. (When the
        iterator is exhausted, the user is able to use a 'Previous'
        button to view the previously seen sets again).
      - The caller invokes the exec_() method on the dialog instance.
      - The exec_() method returns:
          
              - -1, if the iterator yielded no button label arrays at all.
              - 0, if the user canceled, and 
              - 1 if the user accepted one of the sets.

      - If the exec_() method returned 1, the caller may obtain an array with the
        ButtonProgram instances of the currently showing (and therefore accepted) buttons.
        The array is obtained from the dialog instance via method getCurrentlyShowingSetLabels(self). 
    '''
    #------------------------------------------------------   Public  Methods ---------------------------
    
    def __init__(self, buttonProgramArrayIterator):
        
        super(ButtonSetPopupSelector, self).__init__();

        self.setStyleSheet(SpeakEasyGUI.stylesheetAppBG);
        
        self.programButtonDict = {};
        self.buttonProgramArrayIt = buttonProgramArrayIterator;
        self.buttonProgramArrays = []; # saved ButtonProgram arrays
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

        self.noAvailableSets = False;
        self.offerNewButtonSet();
    
    
    def exec_(self):
        '''
        If the prior initialization revealed that the caller does not
        deliver any sets of button labels at all, then return -1, without
        displaying any dialog.
        @return: -1 if no button sets available. 1 if user accepted one of the button
                 label sets as the one they want, or 0, if the user cancelled.
        @rtype: int
        '''
        if self.noAvailableSets:
            return -1;
        return super(ButtonSetPopupSelector, self).exec_();

    def getCurrentlyShowingSet(self):
        '''
        Returns an array of ButtonProgram instances that are currently showing on
        the dialog, or were showing when the user clicked OK.
        @return: Array of ButtonProgram instances.
        @rtype: [ButtonProgram]
        '''
        return self.buttonProgramArrays[self.currentlyShowingSetIndex];

    def getCurrentlyShowingSetLabels(self):
        '''
        Returns an array of labels of buttons that are currently showing on
        the dialog, or were showing when the user clicked OK.
        @return: Array of labels.
        @rtype: [string]
        '''
        allLabels = [];
        for buttonProgram in self.buttonProgramArrays[self.currentlyShowingSetIndex]:
            allLabels.append(buttonProgram.getLabel()); 
        return allLabels;



    #------------------------------------------------------   Private  Methods ---------------------------
    
    def offerNewButtonSet(self):
        '''
        Responsible for displaying the next, or previous set of buttons.
        During successive calls to this method, the method obtains new 
        button label sets from the caller's iterator until the iterator
        is exhausted. The sets are collected in <code>self.shownLabelArrays</code>.
        The method is also responsible for showing previously shown sets again. 
        The direction is controlled by the handlers of the Previous and Next buttons.
        They set instance variable <code>currentNextPrevDirection</code>. 
        '''

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
            self.buildButtonSet(self.getCurrentlyShowingSetLabels());
            #***************
            self.setButtonSetTitleInWindow(self.currentlyShowingSetIndex + 1);
            #***************
            self.setNextPrevButtonsEnabledness();
            return;
        
        # Wants next button set. Is there one we haven't shown yet?:
        try:
            # Get next set of ButtonProgram instances from caller:
            buttonProgramArray = self.buttonProgramArrayIt.next();
            # Save this ButtonProgram instance:
            self.buttonProgramArrays.append(buttonProgramArray);
            
            # From the ButtonProgram objects array, create an
            # array of just the button labels:
            buttonLabelArray = [];
            for buttonProgram in buttonProgramArray:
                buttonLabelArray.append(buttonProgram.getLabel());
            
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
            #**********
            self.setButtonSetTitleInWindow(self.currentlyShowingSetIndex + 1);
            #**********
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
            #**********
            self.setButtonSetTitleInWindow(self.currentlyShowingSetIndex + 1);
            #**********
        
        if self.currentlyShowingSetIndex >= len(self.shownLabelArrays):
            # Neither are there more new sets, nor are we going
            # through the already shown sets going forward a second
            # time. So reverse direction to go backwards through the
            # the already shown sets:
            self.setButtonSetTitleInWindow(-1)
            self.flipNextPrevDirection();
            return;

        self.buildButtonSet(self.getCurrentlyShowingSetLabels());
        self.setNextPrevButtonsEnabledness();

        return;
        
    def setButtonSetTitleInWindow(self, buttonSetIndex):
        '''
        Given a button set number, displays a help text in the 'pick button set' dialog.
        @param buttonSetIndex: number of button set (1-based by convention, but 0 is legal).
        set to -1 if wish to indicate absence of a relevant set. Will show as 'Showing button set --'.
        @type buttonSetIndex: int
        '''
        if buttonSetIndex > -1:
            self.setWindowTitle('Showing button set %d' % buttonSetIndex);
        else:
            self.setWindowTitle('Showing button set --');
        
    def flipNextPrevDirection(self):
        '''
        Called when no more button label sets are available
        either in the next, or previous direction.
        '''

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
        '''
        Cleans out the dialog's layout. Does not destroy
        the control buttons (Next, Previous, Cancel, and OK),
        but does trigger deletion of all the button objects
        in the button grid. Attempts to None out references
        to UI widgets to enable garbage collection.
        '''
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
                buttonObj.deleteLater()

        for key in self.programButtonDict.keys():
            self.programButtonDict[key] = None;

        # Set this dialog's layout to the empty layout:
        self.setLayout(self.rootLayout);

    def buildButtonSet(self, buttonLabelArray):
        '''
        Constructs one set of buttons, based on passed in 
        button labels.
        
        @param buttonLabelArray: Array of button labels.
        @type buttonLabelArray: [string]
        '''

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
        '''
        Appends the existing Next/Previous/Cancel/OK buttons
        into the passed-in layout.
        @param layout:
        '''
        choiceButtonRowLayout = QHBoxLayout();
        choiceButtonRowLayout.addWidget(self.nextButton);
        choiceButtonRowLayout.addWidget(self.prevButton);
        choiceButtonRowLayout.addWidget(self.cancelButton);
        choiceButtonRowLayout.addWidget(self.OKButton);
        layout.addLayout(choiceButtonRowLayout);

    def setNextPrevButtonsEnabledness(self):
        '''
        Examines the instance variables <code>shownLabelArrays</code>
        and </code>currentlyShowingSetIndex</code> to determine whether
        the Next or Previous buttons should be enabled. Acts on the result.
        '''
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
        '''
        Handler for Next button clicks. Triggers show of next
        button set in order.
        '''
        if self.currentNextPrevDirection == NextPrev.PREVIOUS:
            self.currentNextPrevDirection = NextPrev.NEXT;
        self.offerNewButtonSet();
    
    def clickedPrevButton(self):
        '''
        Handler for Next button clicks. Triggers show of previous
        button set in order.
        '''
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
        '''
        Called when no button sets are available from the caller at all.
        Records this event in instance variable <code>noAvailableSets</code>,
        and returns.
        '''
        # No buttons at all:
        self.clearDialogPane();
        self.noAvailableSets = True;

#-----------------------------------------------------  Tester Class -----------------------------
          
class Tester(QDialog):
    '''
    Testing class. Invoked from main() of this file. Shows different
    button sets. Not automatic, must be run by hand.
    '''
    
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
        buttonSetChoiceDialog.deleteLater();
        
        # Rewire the test button for the next text:
        self.testButton.setText("One button available");
        self.testButton.clicked.disconnect(self.exactlyOneRow);
        self.testButton.clicked.connect(self.oneButtonAvailable);
        
    def oneButtonAvailable(self):
        # One button available
        testButtonSetArray = [["Single button."]];
        buttonSetChoiceDialog = ButtonSetPopupSelector(iter(testButtonSetArray));
        buttonSetChoiceDialog.show();
        choiceResult = buttonSetChoiceDialog.exec_();
        print "Choice result: " + str(choiceResult);
        buttonSetChoiceDialog.deleteLater();

        self.testButton.setText("No button set available");
        self.testButton.clicked.disconnect(self.oneButtonAvailable);        
        self.testButton.clicked.connect(self.noSetAvailable);
        
    def noSetAvailable(self):
        testButtonSetArray = [];
        buttonSetChoiceDialog = ButtonSetPopupSelector(iter(testButtonSetArray));
        #buttonSetChoiceDialog.show();
        choiceResult = buttonSetChoiceDialog.exec_();
        print "Choice result: " + str(choiceResult);
        buttonSetChoiceDialog.deleteLater();
        
        # Rewire the test button for the next text:
        self.testButton.setText("Exit");
        self.testButton.clicked.disconnect(self.noSetAvailable);        
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
