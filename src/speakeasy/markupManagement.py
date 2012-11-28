#!/usr/bin/env python

import unittest;
#from unittest import assertEquals
import re;



class Markup:
    SILENCE  = 'S' #0
    RATE     = 'R' #1
    PITCH    = 'P' #2
    VOLUME   = 'V' #3
    EMPHASIS = 'E' #4
    
    @staticmethod
    def baseType():
        '''
        Needed to enable signals with Markup as a parameter.
        PyQt signals cannot accommodate parameter types that
        do not have a C++ equivalent. As a compromise this
        method provides that base type without breaking
        encapsulation tooooo badly.
        '''
        return type(Markup.SILENCE);

class MarkupManagement(object):
    
    openMark  = '[';    
    closeMark = ']';
    marks = {Markup.SILENCE : openMark +'S',
             Markup.RATE    : openMark +'R',
             Markup.PITCH   : openMark +'P',
             Markup.VOLUME  : openMark +'V',
             Markup.EMPHASIS: openMark +'E'
             }
    
    emphasisVals = {0 : 'none',
                    1 : 'moderate',
                    2 : 'strong'
                    };

    units = {Markup.SILENCE : 'ms',
             Markup.RATE    : '',
             Markup.PITCH   : '%',
             Markup.VOLUME  : '%',
             Markup.EMPHASIS: ''
             }


    
    ssmlOpener  = {
                   Markup.SILENCE : '<break time=',
				   Markup.RATE    : '<prosody rate=',
				   Markup.PITCH   : '<prosody pitch=',
				   Markup.VOLUME  : '<prosody volume=',
				   Markup.EMPHASIS: '<emphasis level='
    }
    
    ssmlCLoser  = {
                   Markup.SILENCE : ' />',
				   Markup.RATE    : '</prosody>',
				   Markup.PITCH   : '</prosody>',
				   Markup.VOLUME  : '</prosody>',
				   Markup.EMPHASIS: '</emphasis>'
    }


    markupOpeningLen = max(map(len,marks.items())); # max length of markup openings
    splitter = re.compile(r'[,;\s!?:.<>]+');
    letterChecker = re.compile(r'[a-zA-Z]');
    digitChecker  = re.compile(r'[-+]{0,1}[0-9]');
    
    @staticmethod
    def addMarkup(theStr, markupType, startPos, length=None, numWords=None, value=0):
        '''
        Given appropriate information, enclose parts of a string in a SpeakEasy speech markup.
        Note that these are the less intrusive markups (an opening char plus a char identifying
        the type of markup (speech rate, pitch, volume, level, silence length). 
        @param theStr: phrase containing the substring to be marked.
        @type theStr: String
        @param markupType: indicator for what type of markup is intended
        @type markupType: MarkupManagement.Marks
        @param startPos: first string index to be inclosed in the mark
        @type startPos: int
        @param length: number of chars to be enclosed in the mark. (If used, do not use numWords)
        @type length: int
        @param numWords: number of words to be enclosed in the mark (If used, do not use length)
        @type numWords: int
        @param value: the magnitude of the mark.
        @type value: {int | MarkupManagement.emphasisVals}
        '''
        
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
    def removeMarkup(theStr, curPos):
        '''
        Remove one SpeakEasy markup from a string.  
        @param theStr: string containing markup to remove
        @type theStr: String
        @param curPos: position somewhere inside the marked-up text, or right on the opening marker. 
        @type curPos: int
        @return: a new string with the markup removed. If no enclosing markup found, returns original string.
        @rtype: String
        '''
        markupStartPos = MarkupManagement.pointerToEnclosingMarkup(theStr,curPos);
        if markupStartPos is None :
            # No opening mark found:
            return theStr;
        # Get position of first char after the opening markup:
        markupOpeningEnd = markupStartPos +\
                           MarkupManagement.markupOpeningLen +\
                           MarkupManagement.findFirstNonDigit(theStr[MarkupManagement.markupOpeningLen + markupStartPos:]);
        # Get all string parts up to the start of the markup:
        beforeMarkup = theStr[0:markupStartPos];
        # Get position of the closing mark, starting the search from
        # within the markup:
        markupEndPos = theStr.find(MarkupManagement.closeMark, markupOpeningEnd);
        # Init return string with everything up to the markup opening:
        retStr = '' if len(beforeMarkup) == 0 else beforeMarkup;
        if markupEndPos < 0:
            # Found open of markup, but not the close; add remainder of string past opening markup:
            retStr += theStr[markupOpeningEnd:];
            return retStr;
        # Have opening and closing of markup. Gather the string part
        # after the closing mark: 
        afterMarkup = theStr[markupEndPos+len(MarkupManagement.closeMark):];
        # Add str fragment between the end of the opening markup and the closing mark: 
        retStr += theStr[markupOpeningEnd:markupEndPos];
        #***retStr += '' if len(afterMarkup) == 0 else afterMarkup; 
        retStr += '' if len(afterMarkup) == 0 else afterMarkup; 
        return retStr;    

    @staticmethod
    def getValue(theStr, startPos):
        '''
        Given a string and position within the string, Find the immediately enclosing
        markup, and return its magnitude.
        @param theStr: string containing the markup under consideration.
        @type theStr: String
        @param startPos: index into the string, including the opening char of the mark.
        @type startPos: int
        @return: magnitude value of the markup
        @rtype: int
        '''
        # Find the opening markup to left of startPos:
        openMarkPos = MarkupManagement.pointerToEnclosingMarkup(theStr, startPos);
        if openMarkPos is None:
            return None;
        
        # Check that markup syntax is correct, and get pointer to the value within the string:
        valueStartIndex = MarkupManagement.isProperMarkupOpening(theStr, openMarkPos);
             
        numMatch = re.match(r'[-+]{0,1}\d+',theStr[valueStartIndex:]);
        numStr = numMatch.group(0);
        return int(numStr);
        
    
    @staticmethod
    def changeValue(theStr, startPos, newValue):
        '''
        Change magnitude part of an existing markup.
        @param theStr: string containing the markup under consideration.
        @type theStr: String
        @param startPos: index into the marked-up text, including the opening marker.
        @type startPos: int
        @param newValue: new value for the markup
        @type newValue: int
        @return: a new string with the respective value modified.
        @rtype: String.
        '''
        
        # Find the opening markup to left of startPos:
        openMarkPos = MarkupManagement.pointerToEnclosingMarkup(theStr, startPos);
        if openMarkPos is None:
            return None;
        
        # Check that markup syntax is correct, and get pointer to the value within the string:
        valueStartIndex = MarkupManagement.isProperMarkupOpening(theStr, openMarkPos);
             
        numMatch = re.match(r'[-+]{0,1}\d+',theStr[valueStartIndex:]);
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
        @raise ValueError: if passed-in cursorPos is out of range.: 
        
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
        '''
        Given a string, a start position within the string, and a number of words,
        return number of chars to end of start plus numWords words.
        @param str: string to examine
        @type str: String
        @param startPos: start position for counting letters
        @type startPos: int
        @param numWords: number of words to include in the count
        @type numWords: int
        @return: number of chars between startPos and the end of the numWord's word.
        @rtype: int
        '''
        
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
        Return True if theStr contains a legal speech markup opening at cursorPos.
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
            if MarkupManagement.digitChecker.match(theStr[pos:]) is not None:
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

    @staticmethod
    def convertStringToSSML(theStr):
        '''
        Given a string with SpeakEasy markups, replace all SpeakEasy markups with official W3C SSML.
        @param theStr: string containing the markup under consideration.
        @type theStr: String
        @return: new string with only SSML markup
        @rtype: String.
        '''
        moreMarkups = True;
        while moreMarkups:
            moreMarkups = False;
            for i,char in enumerate(theStr):
                if char != MarkupManagement.openMark:
                    continue;
                # What type of markup? Volume? Pause? Pitch?...:
                markupType = theStr[i+1];
                # Value of markup:
                valNum = MarkupManagement.getValue(theStr, i);
                if markupType == Markup.RATE:
                    val = valNum / 100.;
                elif markupType == Markup.EMPHASIS:
                    val = MarkupManagement.emphasisVals[valNum];
                else:
                    val = valNum;
                units = MarkupManagement.units[markupType];
                if markupType == Markup.SILENCE:
                    newStr = theStr[0:i] +\
                    MarkupManagement.ssmlOpener[markupType] +\
                    "'" + str(val) + units + "'" +\
                    MarkupManagement.restToSSML(markupType, theStr[i+2+len(str(val)):]);
                    theStr = newStr;
                    moreMarkups = True
                    break;
                else:
                    newStr = theStr[0:i] +\
                    MarkupManagement.ssmlOpener[markupType] +\
                    "'" + str(val) + units + "'" + '>' +\
                    MarkupManagement.restToSSML(markupType, theStr[i+2+len(str(valNum)):]);
                    theStr = newStr;
                    moreMarkups = True;
                    break;
        return newStr;
        
    @staticmethod
    def restToSSML(markupType, theStrRest):
        for i,char in enumerate(theStrRest):
            if char == MarkupManagement.closeMark:
                newStr = theStrRest[0:i] + MarkupManagement.ssmlCLoser[markupType] + theStrRest[i+1:];
                return newStr;
        
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
        self.assertEqual(newStr,
                         '%sE10This%s little light' % (MarkupManagement.openMark,
                                                       MarkupManagement.closeMark), 
                         'Failed adding emphasis to first word. Result was "%s".' % newStr);
        
        newStr = MarkupManagement.addMarkup(self.testStr, Markup.EMPHASIS, 5, numWords=1, value=10);
        self.assertEqual(newStr,
                         'This %sE10little%s light' % (MarkupManagement.openMark,
                                                       MarkupManagement.closeMark), 
                          'Failed adding emphasis to second word. Result was "%s".' % newStr);
        
        newStr = MarkupManagement.addMarkup(self.testStr, Markup.EMPHASIS, 5, numWords=2, value=10);
        self.assertEqual(newStr,
                         'This %sE10little light%s' % (MarkupManagement.openMark,
                                                       MarkupManagement.closeMark), 
                         'Failed adding emphasis to second and third word. Result was "%s".' % newStr);

        newStr = MarkupManagement.addMarkup(self.testStr, Markup.EMPHASIS, 11, numWords=1, value=10);
        self.assertEqual(newStr,
                         'This little %sE10light%s' % (MarkupManagement.openMark,
                                                       MarkupManagement.closeMark), 
                         'Failed adding emphasis to last word. Result was "%s".' % newStr);

        newStr = MarkupManagement.addMarkup(self.testStr, Markup.EMPHASIS, 0, numWords=3, value=10);
        self.assertEqual(newStr,
                         '%sE10This little light%s' % (MarkupManagement.openMark,
                                                        MarkupManagement.closeMark), 
                         'Failed adding emphasis to all words. Result was "%s".' % newStr);

    def testZeroLenMarkedStr(self):
        pass
    
    def testLenLongerThanStr(self):
        pass


    def testRemoveMarkup(self):
        newStr = MarkupManagement.removeMarkup('%sE10This%s little light' % (MarkupManagement.openMark,
                                                                             MarkupManagement.closeMark), 
                                               0);
        self.assertEqual(newStr, self.testStr, 'Failed to remove markup from first word. Got "%s"' % newStr);
        
        newStr = MarkupManagement.removeMarkup('%sE10This little%s light' % (MarkupManagement.openMark,
                                                                             MarkupManagement.closeMark), 
                                               0);
        self.assertEqual(newStr, self.testStr, 'Failed to remove markup from first and second word. Got "%s"' % newStr);
        
        newStr = MarkupManagement.removeMarkup('%sE10This little light%s' % (MarkupManagement.openMark,
                                                                             MarkupManagement.closeMark), 
                                               0);
        self.assertEqual(newStr, self.testStr, 'Failed to remove markup from first, second and third word. Got "%s"' % newStr);
        
        newStr = MarkupManagement.removeMarkup('This %sE10little%s light' % (MarkupManagement.openMark,
                                                                             MarkupManagement.closeMark), 
                                               9);
        self.assertEqual(newStr, self.testStr, 'Failed to remove markup from second word. Got "%s"' % newStr);

        newStr = MarkupManagement.removeMarkup('This little light', 0);
        self.assertEqual(newStr, self.testStr, 'Failed to remove markup when no markup present. Got "%s"' % newStr);

        newStr = MarkupManagement.removeMarkup('%sE10This little light' % (MarkupManagement.openMark),
                                               0);
        self.assertEqual(newStr, self.testStr, 'Failed to remove markup from first word when closing mark missing. Got "%s"' % newStr);

    def testChangeValue(self):
        
        tstStr = '%sE10This%s little light' % (MarkupManagement.openMark,
                                               MarkupManagement.closeMark); 
        newStr = MarkupManagement.changeValue(tstStr, 0, 20); # String, pos within marked-up sequence, new value:
        self.assertEqual(newStr, 
                         '%sE20This%s little light' % (MarkupManagement.openMark,
                                                       MarkupManagement.closeMark), 
                          'Failed to change value with markup first in string. Got "%s".' % newStr); 
    
        tstStr = 'This %sE10little%s light' % (MarkupManagement.openMark,
                                               MarkupManagement.closeMark); 
        newStr = MarkupManagement.changeValue(tstStr, 9, 30);
        self.assertEqual(newStr,
                         'This %sE30little%s light' % (MarkupManagement.openMark,
                                                       MarkupManagement.closeMark), 
                         'Failed to change value with markup second in string. Got "%s".' % newStr); 

        tstStr = '%sE40This little light%s' % (MarkupManagement.openMark,
                                               MarkupManagement.closeMark);
        newStr = MarkupManagement.changeValue(tstStr, 9, 50);
        self.assertEqual(newStr,
                         '%sE50This little light%s' % (MarkupManagement.openMark,
                                                       MarkupManagement.closeMark), 
                         'Failed to change value with markup second in string. Got "%s".' % newStr); 

    def testConvertStringToSSMLSingleMarkups(self):

        # Silence
        tstStr = '%sS10%sThis little light' % (MarkupManagement.openMark,
                                               MarkupManagement.closeMark);
        newStr = MarkupManagement.convertStringToSSML(tstStr);
        self.assertEqual(newStr, "<break time='10ms' />This little light", "Failed Silence at start of sentence. Got '%s'" % newStr);
        
        tstStr = 'This little %sS10%s light' % (MarkupManagement.openMark,
                                               MarkupManagement.closeMark);
        newStr = MarkupManagement.convertStringToSSML(tstStr);
        self.assertEqual(newStr, "This little <break time='10ms' /> light", "Failed Silence in middle of sentence. Got '%s'" % newStr);
        
        # Rate
        tstStr = '%sR10 This%s little light' % (MarkupManagement.openMark,
                                               MarkupManagement.closeMark);
        newStr = MarkupManagement.convertStringToSSML(tstStr);
        self.assertEqual(newStr, "<prosody rate='0.1'> This</prosody> little light");
    
        tstStr = 'This little %sR10 light%s' % (MarkupManagement.openMark,
                                               MarkupManagement.closeMark);
        newStr = MarkupManagement.convertStringToSSML(tstStr);
        self.assertEqual(newStr, "This little <prosody rate='0.1'> light</prosody>");
        
        # Pitch
        tstStr = '%sP10 This%s little light' % (MarkupManagement.openMark,
                                               MarkupManagement.closeMark);
        newStr = MarkupManagement.convertStringToSSML(tstStr);
        self.assertEqual(newStr, "<prosody pitch='10%'> This</prosody> little light");
        
        tstStr = 'This %sP10little light%s' % (MarkupManagement.openMark,
                                               MarkupManagement.closeMark);
        newStr = MarkupManagement.convertStringToSSML(tstStr);
        self.assertEqual(newStr, "This <prosody pitch='10%'>little light</prosody>");

        # Volume
        tstStr = '%sV10 This%s little light' % (MarkupManagement.openMark,
                                               MarkupManagement.closeMark);
        newStr = MarkupManagement.convertStringToSSML(tstStr);
        self.assertEqual(newStr, "<prosody volume='10%'> This</prosody> little light");

        tstStr = '%sV-10 This%s little light' % (MarkupManagement.openMark,
                                               MarkupManagement.closeMark);
        newStr = MarkupManagement.convertStringToSSML(tstStr);
        self.assertEqual(newStr, "<prosody volume='-10%'> This</prosody> little light");

        # Emphasis
        tstStr = '%sE0 This%s little light' % (MarkupManagement.openMark,
                                              MarkupManagement.closeMark);
        newStr = MarkupManagement.convertStringToSSML(tstStr);
        self.assertEqual(newStr, "<emphasis level='none'> This</emphasis> little light");

        tstStr = '%sE0This little light%s' % (MarkupManagement.openMark,
                                              MarkupManagement.closeMark);
        newStr = MarkupManagement.convertStringToSSML(tstStr);
        self.assertEqual(newStr, "<emphasis level='none'>This little light</emphasis>");
        
    def testConvertStringToSSMLMultipleMarkups(self):
        
        # No nesting of markups:
        tstStr = '%sE0 This%s little %sV60light%s' % (MarkupManagement.openMark,
                                                      MarkupManagement.closeMark,
                                                      MarkupManagement.openMark,
                                                      MarkupManagement.closeMark);
        newStr = MarkupManagement.convertStringToSSML(tstStr);
        self.assertEqual(newStr, "<emphasis level='none'> This</emphasis> little <prosody volume='60%'>light</prosody>");
    
        # Nested markup:
        tstStr = '%sV40This %sP60little%s light%s' % (MarkupManagement.openMark,
                                                      MarkupManagement.openMark,
                                                      MarkupManagement.closeMark,
                                                      MarkupManagement.closeMark);
        newStr = MarkupManagement.convertStringToSSML(tstStr);
        self.assertEqual(newStr, "<prosody volume='40%'>This <prosody pitch='60%'>little</prosody> light</prosody>");
        
    
    def isProperMarkupOpening():
        MarkupManagement.isProperMarkupOpening("Good %sE10Work" % MarkupManagement.openMark, len('Good '));



if __name__ == '__main__':
    unittest.main();
