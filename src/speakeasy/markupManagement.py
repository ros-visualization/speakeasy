#!/usr/bin/env python

import unittest;
#from unittest import assertEquals
import re;


class Markup:
    SILENCE  = 0
    RATE     = 1
    PITCH    = 2
    VOLUME   = 3
    EMPHASIS = 4



class MarkupManagement(object):
    
    openMark  = '<';    
    closeMark = '>';
    marks = {Markup.SILENCE : openMark +'S',
             Markup.RATE    : openMark +'R',
             Markup.PITCH   : openMark +'P',
             Markup.VOLUME  : openMark +'V',
             Markup.EMPHASIS: openMark +'E'
             }
    markupOpeningLen = max(map(len,marks.items())); # max length of markup openings
    splitter = re.compile(r'[,;\s!?:.<>]+');
    letterChecker = re.compile(r'[a-zA-Z]');
    digitChecker  = re.compile(r'[0-9]');
    
    @staticmethod
    def addMarkup(theStr, markupType, startPos, length=None, numWords=None, value=0):
        
        if len(theStr) == 0:
            return theStr;
        
        if length is None:
            if numWords is None:
                raise ValueError('Either length or numWords must be provided. Both are None.')
            length = MarkupManagement.getLenFromNumWords(theStr, startPos, numWords);
        
        beforeMarkup = theStr[0:startPos].strip();
        markedStr    = theStr[startPos:startPos+length].strip();
        afterMarkup  = theStr[startPos+len(markedStr)+1:].strip();

        newStr = '' if len(beforeMarkup)==0 else beforeMarkup + ' ';
        newStr +=MarkupManagement.marks[markupType] +\
                 str(value) +\
                 markedStr +\
                 MarkupManagement.closeMark
        newStr += '' if len(afterMarkup) == 0 else ' ' + afterMarkup;
        return newStr; 
    
    @staticmethod
    def removeMarkup(theStr, startPos):
        markupStartPos = theStr[startPos:].find(MarkupManagement.openMark);
        if markupStartPos < 0:
            # No opening mark found:
            return theStr;
        markupOpeningEnd = markupStartPos +\
                           MarkupManagement.markupOpeningLen +\
                           MarkupManagement.findFirstNonDigit(theStr[MarkupManagement.markupOpeningLen + markupStartPos:]);
        beforeMarkup = theStr[0:startPos+markupStartPos];
        markupEndPos = theStr[startPos:].find(MarkupManagement.closeMark);
        retStr = '' if len(beforeMarkup) == 0 else beforeMarkup;
        if markupEndPos < 0:
            # Found open of markup, but not the close:
            retStr += theStr[markupOpeningEnd:];
            return retStr;
        # Have opening and closing of markup:
        afterMarkup = theStr[markupEndPos+len(MarkupManagement.closeMark):];
        retStr += theStr[markupOpeningEnd:markupEndPos];
        retStr += '' if len(afterMarkup) == 0 else afterMarkup; 
        return retStr;    
    
    @staticmethod
    def changeValue(theStr, startPos, newValue):
        
        # Find the opening markup to left of startPos:
        openMarkPos = MarkupManagement.pointerToEnclosingMarkup(theStr, startPos);
        if openMarkPos is None:
            return None;
        
        # Check that markup syntax is correct, and get pointer to the value within the string:
        valueStartIndex = MarkupManagement.isProperMarkupOpening(theStr, openMarkPos);
             
        numMatch = re.match(r'\d+',theStr[valueStartIndex:]);
        numStr = numMatch.group(0);
        newStr = theStr[0:valueStartIndex] + str(newValue) + theStr[valueStartIndex+len(numStr):];
        return newStr;
    
    @staticmethod
    def pointerToEnclosingMarkup(theStr, cursorPos):
        '''
        Given a string and a cursor position into it, return the cursor position
        of the nearest opening markup char.
        @param theStr: string to examine.
        @type theStr: String
        @param cursorPos: starting position of the search.
        @type cursorPos: int
        @return: Cursor position resPos such that theStr[resPos] == MarkupManagement.openMark. None if no enclosing openMark is found.
        @raise ValueError if passed-in cursorPos is out of range.: 
        
        '''
        #  Already pointing to markup opening?
        if theStr[cursorPos] == MarkupManagement.openMark:
            return cursorPos;
        for pos in reversed(range(cursorPos + 1)):
            if theStr[pos] == MarkupManagement.openMark:
                return pos;
        return None
    
    @staticmethod
    def getLenFromNumWords(str, startPos, numWords):
        
        wordsPlusRest = str[startPos:];
        tokens = MarkupManagement.splitter.split(wordsPlusRest);
        # Remove empty-str words:
        cleanTokens = [];
        for token in tokens:
            if len(token) > 0:
                cleanTokens.append(token);
        tokens = cleanTokens;
        searchPat = r'';
        for word in tokens[0:numWords]:
            searchPat += '[^a-zA-Z]*' + word;
        wordSearcher = re.compile(searchPat);
        substrMatch = wordSearcher.match(str[startPos:]);
        return substrMatch.end();

    @staticmethod
    def isProperMarkupOpening(theStr, cursorPos=0):
        '''
        Return True if theStr contains a legal prosidy markup opening at cursorPos.
        Else throw error with illuminating text.
        @param theStr: string to check
        @type theStr: String
        @param cursorPos: Starting position
        @type cursorPos: int
        @return: Index to start of the markup opening's value, if cursorPos points to a legal markup opening within theStr.
        @rtype: int
        @raise ValueError: if anything wrong. 
        '''
        # Is the substring after cursorPos even long enough to hold a markup opening? 
        if len(theStr) < MarkupManagement.markupOpeningLen:
            raise ValueError("Given string ('%s') is shorter than minimum markup opening length, which is %d." % (theStr, 
                                                                                                                  MarkupManagement.markupOpeningLen));
        # Is there a legal markup at cursorPos?:
        if theStr[cursorPos:cursorPos + MarkupManagement.markupOpeningLen] not in MarkupManagement.marks.values():
            raise ValueError("Markup opening must be one of %s. But it was '%s'." % (str(MarkupManagement.marks.values()), 
                                                                                         theStr[cursorPos:cursorPos+MarkupManagement.markupOpeningLen]));
        # Next must be an integer (possibly after spaces or tabs:):
        valueStart = None;
        for pos in range(cursorPos + MarkupManagement.markupOpeningLen, len(theStr)):
            if MarkupManagement.digitChecker.match(theStr[pos]) is not None:
                valueStart = pos;
                break;
            elif theStr[pos] == ' ' or theStr[pos] == '\t':
                # Allow spaces and tabs after markup opening and before the value: 
                continue;
            else:
                # Found a non-digit, non-space char after the opening markup marker; illegal opening marker format:
                break;
        if valueStart is None:
            raise ValueError("Bad prosidy markup: markup opening found, but no number follows after index %d in '%s'." % (cursorPos + MarkupManagement.markupOpeningLen,
                                                                                                                          theStr))
        return valueStart;
    
    @staticmethod
    def isLetter(oneChar):
        return MarkupManagement.letterChecker.match(oneChar) is not None;

    @staticmethod
    def findFirstNonDigit(theStr):
        for pos, oneChar in enumerate(theStr):
            if MarkupManagement.digitChecker.match(oneChar) is None:
                return pos;
        # All digits:
        return None;
        
# ------------------------------------------

class MarkupTest(unittest.TestCase):
    
    def setUp(self):
        self.testStr = "This little light";
        
    def testLenFromNumWords(self):
        theLen = MarkupManagement.getLenFromNumWords('this is me.', 0, 1);
        self.assertEqual(theLen, 4, 'Failed find len one word from start. Expected %d, got %d.' % (4, theLen));
        
        theLen = MarkupManagement.getLenFromNumWords('this is me.', 0, 2);
        self.assertEqual(theLen, 7, 'Failed find len two words from start. Expected %d, got %d.' % (7, theLen));
        
        theLen = MarkupManagement.getLenFromNumWords('this is me.', 5, 1);
        self.assertEqual(theLen, 2, 'Failed find len one word from second word. Expected %d, got %d.' % (2, theLen));

        theLen = MarkupManagement.getLenFromNumWords('this is me.', 5, 2);
        self.assertEqual(theLen, 5, 'Failed find len two words from second word. Expected %d, got %d.' % (5, theLen));

        theLen = MarkupManagement.getLenFromNumWords('this is me.', 5, 3);
        self.assertEqual(theLen, 5, 'Failed find len two words from second word. Expected %d, got %d.' % (5, theLen));

        theLen = MarkupManagement.getLenFromNumWords('  this   is   me.', 0, 1); # 2 spaces before 'this', 3 spaces before 'is' and 'me'
        self.assertEqual(theLen, 6, 'Failed find len one word from start of str with spaces. Expected %d, got %d.' % (6, theLen));
        
        theLen = MarkupManagement.getLenFromNumWords('  this   is   me.', 6, 1); # 2 spaces before 'this', 3 spaces before 'is' and 'me'
        self.assertEqual(theLen, 5, 'Failed find len one word from end of 2nd word in string with spaces. Expected %d, got %d.' % (5, theLen));
        
        theLen = MarkupManagement.getLenFromNumWords('  this   is   me.', 6, 2); # 2 spaces before 'this', 3 spaces before 'is' and 'me'
        self.assertEqual(theLen, 10, 'Failed find len 2 words from end of 2nd word in string with spaces. Expected %d, got %d.' % (10, theLen));

    
    def testAddMarkupEmptyStr(self):
        self.assertEqual(MarkupManagement.addMarkup('', Markup.EMPHASIS, 0, 10, value=29), '', "Markup for empty str failed.");
    
    def testMarkup(self):
        newStr = MarkupManagement.addMarkup(self.testStr, Markup.EMPHASIS, 0, numWords=1, value=10);
        self.assertEqual(newStr, '<E10This> little light', 'Failed adding emphasis to first word. Result was "%s".' % newStr);
        
        newStr = MarkupManagement.addMarkup(self.testStr, Markup.EMPHASIS, 5, numWords=1, value=10);
        self.assertEqual(newStr, 'This <E10little> light', 'Failed adding emphasis to second word. Result was "%s".' % newStr);
        
        newStr = MarkupManagement.addMarkup(self.testStr, Markup.EMPHASIS, 5, numWords=2, value=10);
        self.assertEqual(newStr, 'This <E10little light>', 'Failed adding emphasis to second and third word. Result was "%s".' % newStr);

        newStr = MarkupManagement.addMarkup(self.testStr, Markup.EMPHASIS, 11, numWords=1, value=10);
        self.assertEqual(newStr, 'This little <E10light>', 'Failed adding emphasis to last word. Result was "%s".' % newStr);

        newStr = MarkupManagement.addMarkup(self.testStr, Markup.EMPHASIS, 0, numWords=3, value=10);
        self.assertEqual(newStr, '<E10This little light>', 'Failed adding emphasis to all words. Result was "%s".' % newStr);

    def testZeroLenMarkedStr(self):
        pass
    
    def testLenLongerThanStr(self):
        pass


    def testRemoveMarkup(self):
        newStr = MarkupManagement.removeMarkup('<E10This> little light', 0);
        self.assertEqual(newStr, self.testStr, 'Failed to remove markup from first word. Got "%s"' % newStr);
        
        newStr = MarkupManagement.removeMarkup('<E10This little> light', 0);
        self.assertEqual(newStr, self.testStr, 'Failed to remove markup from first and second word. Got "%s"' % newStr);
        
        newStr = MarkupManagement.removeMarkup('<E10This little light>', 0);
        self.assertEqual(newStr, self.testStr, 'Failed to remove markup from first, second and third word. Got "%s"' % newStr);
        
        newStr = MarkupManagement.removeMarkup('This <E10little> light', 0);
        self.assertEqual(newStr, self.testStr, 'Failed to remove markup from second word. Got "%s"' % newStr);

        newStr = MarkupManagement.removeMarkup('This little light', 0);
        self.assertEqual(newStr, self.testStr, 'Failed to remove markup when no markup present. Got "%s"' % newStr);

        newStr = MarkupManagement.removeMarkup('<E10This little light', 0);
        self.assertEqual(newStr, self.testStr, 'Failed to remove markup from first word when closing mark missing. Got "%s"' % newStr);

    def testChangeValue(self):
        
        tstStr = '<E10This> little light';
        newStr = MarkupManagement.changeValue(tstStr, 0, 20); # String, pos within marked-up sequence, new value:
        self.assertEqual(newStr, '<E20This> little light', 'Failed to change value with markup first in string. Got "%s".' % newStr); 
    
        tstStr = 'This <E10little> light';
        newStr = MarkupManagement.changeValue(tstStr, 9, 30);
        self.assertEqual(newStr, 'This <E30little> light', 'Failed to change value with markup second in string. Got "%s".' % newStr); 

        tstStr = '<E40This little light>';
        newStr = MarkupManagement.changeValue(tstStr, 9, 50);
        self.assertEqual(newStr, '<E50This little light>', 'Failed to change value with markup second in string. Got "%s".' % newStr); 


    def isProperMarkupOpening(self):
        MarkupManagement.isProperMarkupOpening("Good <E10Work", len('Good '));

if __name__ == '__main__':
    unittest.main();
