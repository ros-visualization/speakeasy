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
    
    emphasisStrs = {0 : 'none',
                    1 : 'moderate',
                    2 : 'strong'
                    };

    emphasisCodes = {'none' : 0,
                     'moderate' : 1, 
                     'strong' : 2 
                    };

    units = {Markup.SILENCE : 'ms',
             Markup.RATE    : '%',
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
    
    # Recognize a string that's an unclosed markup. Examples:
    #   o foo [E40bar    -> Yes
    #   o foo E40bar     -> No (no opening markup char)
    #   o foo [E40bar]   -> No (closing markup char present)
    #   o foo [K40bar    -> No (K is not a markup code)
    # With the opening and closing marker chars, and the marker letters 
    # as they are defined as of this writing, the regex would be: '[^[]*\[[REPVS][+-]{0,1}[0-9]*[^\]]+$'
    # with REPV standing for Rate, Emphasis, Pitch, Volume, and Silence. We piece this 
    # regex together using the symbolic names for the markers and mark letters:
    
    unclosedMarkupChecker = re.compile(r'[^' + openMark + ']*\\' + openMark +\
                                       '[' + Markup.RATE + Markup.EMPHASIS + Markup.PITCH + Markup.VOLUME + Markup.SILENCE +\
                                       '][+-]{0,1}[0-9]*[^\\' + closeMark + ']+$');
    
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
        @param startPos: first string index to be inclosed in the mark. In case of silence insertion,
                         this position should be between two words or on the first char of a word.
        @type startPos: int
        @param length: number of chars to be enclosed in the mark. (If used, do not use numWords)
        @type length: int
        @param numWords: number of words to be enclosed in the mark (If used, do not use length)
        @type numWords: int
        @param value: the magnitude of the mark.
        @type value: {int | MarkupManagement.emphasisStrs}
        @raise ValueError: if markup span is neither specified in length nor in number of words. 
        '''
        
        if len(theStr) == 0:
            return theStr;

        if markupType == Markup.SILENCE:
            if startPos != 0 and startPos < len(theStr) and theStr[startPos] != ' ' and theStr[startPos-1] != ' ':
                raise ValueError("Silence can only be inserted at the start of a string, or between words. Pos %d in '%s' violates this rule." % (startPos,theStr));
            length = 0; 
        
        if length is None:
            if numWords is None:
                raise ValueError('Either length or numWords must be provided. Both are None.')
            length = MarkupManagement.getLenFromNumWords(theStr, startPos, numWords);
        
        beforeMarkup = theStr[0:startPos].strip();
        markedStr    = theStr[startPos:startPos+length].strip();
        afterMarkup  = theStr[startPos+len(markedStr):].strip();

        newStr = '';
        if len(beforeMarkup) > 0:
            newStr += beforeMarkup;
            if not MarkupManagement.isUnclosedMarkup(beforeMarkup):
                newStr += ' ';
        newStr +=MarkupManagement.marks[markupType] +\
                 str(value) +\
                 markedStr +\
                 MarkupManagement.closeMark
        # Re-insert a space after the closing mark, unless the next
        # char is again a closing mark (of an outer-nested markup):
        if len(afterMarkup) > 0: 
            if afterMarkup[0] != MarkupManagement.closeMark:
                newStr += ' ' + afterMarkup;
            else:
                newStr += afterMarkup;
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
        retStr = '';
        if len(beforeMarkup) > 0:
            retStr += beforeMarkup;
            # Take care of a squished-together nested markup: Deleting the inner
            # markup around 'light' in 'This [E20Little[V30light]] of mine'
            # should come out as 'This [E20Little light] of mine',
            # not 'This [E20Littlelight] of mine',
            if MarkupManagement.isUnclosedMarkup(beforeMarkup) and beforeMarkup[-1] != ' ':
                retStr += ' ';
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
    def isUnclosedMarkup(theStr):
        '''
        Returns a match object if theStr ends with an unclosed markup,
        such as 'foo[E20bar', else returns None.
        @param theStr: string to examine
        @type theStr: String
        @return: None or match object
        @rtype {None | sre.SRE_Match}
        '''
        return MarkupManagement.unclosedMarkupChecker.match(theStr);

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
#               # OK to feed rate as percentage in spite of examples for that
#               # approach missing. All examples are decimal nums for rates.
#               # So the following is commented out
#                if markupType == Markup.RATE:
#                    val = valNum / 100.;
                if markupType == Markup.EMPHASIS:
                    val = MarkupManagement.emphasisStrs[valNum];
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
                    # Note: CEPSTRAL (maybe also W3C's?) SSML require a plus sign
                    #       before positive percentages. (And of course a minus sign
                    #       for negative numbers):
                    if markupType != Markup.EMPHASIS and val > 0:
                        valStr = '+' + str(val);
                    else:
                        valStr = str(val); 
                    newStr = theStr[0:i] +\
                    MarkupManagement.ssmlOpener[markupType] +\
                    "'" + valStr + units + "'" + '>' +\
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

    def testUnclosedMarkupMatcher(self):
        self.assertIsNotNone(MarkupManagement.unclosedMarkupChecker.match('foo[E20bar'), "Did not recognize 'foo[E20bar' as unclosed markup.");
        self.assertIsNotNone(MarkupManagement.unclosedMarkupChecker.match('[E20bar'), "Did not recognize '[E20bar' as unclosed markup.");  
        self.assertIsNotNone(MarkupManagement.unclosedMarkupChecker.match('[V20'), "Did not recognize '[V20' as unclosed markup.");  
        self.assertIsNotNone(MarkupManagement.unclosedMarkupChecker.match('foo[R20'), "Did not recognize 'foo[R20' as unclosed markup.");  
        self.assertIsNone(MarkupManagement.unclosedMarkupChecker.match('fooE20bar'), "Incorrectly recognized 'fooE20bar' as unclosed markup.");
        self.assertIsNone(MarkupManagement.unclosedMarkupChecker.match('foo[K20bar'), "Incorrectly recognized 'foo[K20bar' as unclosed markup.");
        
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

        newStr = MarkupManagement.addMarkup(self.testStr, Markup.EMPHASIS, 12, numWords=1, value=10);
        self.assertEqual(newStr,
                         'This little %sE10light%s' % (MarkupManagement.openMark,
                                                       MarkupManagement.closeMark), 
                         'Failed adding emphasis to last word. Result was "%s".' % newStr);

        newStr = MarkupManagement.addMarkup(self.testStr, Markup.EMPHASIS, 0, numWords=3, value=10);
        self.assertEqual(newStr,
                         '%sE10This little light%s' % (MarkupManagement.openMark,
                                                        MarkupManagement.closeMark), 
                         'Failed adding emphasis to all words. Result was "%s".' % newStr);
                         
    def testSilenceMarkup(self):
        newStr = MarkupManagement.addMarkup(self.testStr, Markup.SILENCE, 0, value=10);
        self.assertEqual(newStr,
                         '%sS10%s This little light' % (MarkupManagement.openMark,
                                                       MarkupManagement.closeMark),
                         'Failed adding silence at start of word. Result was "%s".' % newStr);

        self.assertRaises(ValueError, MarkupManagement.addMarkup, self.testStr, Markup.SILENCE, 3, value=10);
        
        newStr = MarkupManagement.addMarkup(self.testStr, Markup.SILENCE, 4, value=20);
        self.assertEqual(newStr,
                         'This %sS20%s little light' % (MarkupManagement.openMark,
                                                       MarkupManagement.closeMark),
                         'Failed adding silence in space between two words. Result was "%s".' % newStr);

        newStr = MarkupManagement.addMarkup(self.testStr, Markup.SILENCE, 5, value=30);
        self.assertEqual(newStr,
                         'This %sS30%s little light' % (MarkupManagement.openMark,
                                                       MarkupManagement.closeMark),
                         'Failed adding silence just before second word. Result was "%s".' % newStr);

                         
    def testMarkupNested(self):
        tstStr = 'This %sE10Little light%s of mine.' % (MarkupManagement.openMark, MarkupManagement.closeMark);
        newStr = MarkupManagement.addMarkup(tstStr, Markup.PITCH, 16, 5, value=20);
        self.assertEqual(newStr, 'This %sE10Little%sP20light%s%s of mine.' % (MarkupManagement.openMark,
                                                                               MarkupManagement.openMark,
                                                                               MarkupManagement.closeMark,
                                                                               MarkupManagement.closeMark));
        newStr = MarkupManagement.addMarkup(tstStr, Markup.PITCH, 9, 6, value=30);
        self.assertEqual(newStr, 'This %sE10%sP30Little%s light%s of mine.' % (MarkupManagement.openMark,
                                                                               MarkupManagement.openMark,
                                                                               MarkupManagement.closeMark,
                                                                               MarkupManagement.closeMark));
                                                                                    


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
        
    def testRemoveNestedMarkup(self):
        newStr =  MarkupManagement.removeMarkup('This %sE10Little %sP20light%s%s of mine.' % (MarkupManagement.openMark,
                                                                                                MarkupManagement.openMark,
                                                                                                MarkupManagement.closeMark,
                                                                                                MarkupManagement.closeMark),
                                                21); # cursor on 'g' of 'light'.
        self.assertEqual(newStr, 'This %sE10Little light%s of mine.' % (MarkupManagement.openMark, MarkupManagement.closeMark));                                                                                                
        
        newStr =  MarkupManagement.removeMarkup('This %sE10Little%sP20light%s%s of mine.' % (MarkupManagement.openMark,
                                                                                                MarkupManagement.openMark,
                                                                                                MarkupManagement.closeMark,
                                                                                                MarkupManagement.closeMark),
                                                21); # cursor on 'g' of 'light'.
        self.assertEqual(newStr, 'This %sE10Little light%s of mine.' % (MarkupManagement.openMark, MarkupManagement.closeMark));                                                                                                

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
        self.assertEqual(newStr, "<prosody rate='+10%'> This</prosody> little light");
    
        tstStr = 'This little %sR10 light%s' % (MarkupManagement.openMark,
                                               MarkupManagement.closeMark);
        newStr = MarkupManagement.convertStringToSSML(tstStr);
        self.assertEqual(newStr, "This little <prosody rate='+10%'> light</prosody>");
        
        # Pitch
        tstStr = '%sP10 This%s little light' % (MarkupManagement.openMark,
                                               MarkupManagement.closeMark);
        newStr = MarkupManagement.convertStringToSSML(tstStr);
        self.assertEqual(newStr, "<prosody pitch='+10%'> This</prosody> little light");
        
        tstStr = 'This %sP10little light%s' % (MarkupManagement.openMark,
                                               MarkupManagement.closeMark);
        newStr = MarkupManagement.convertStringToSSML(tstStr);
        self.assertEqual(newStr, "This <prosody pitch='+10%'>little light</prosody>");

        # Volume
        tstStr = '%sV10 This%s little light' % (MarkupManagement.openMark,
                                               MarkupManagement.closeMark);
        newStr = MarkupManagement.convertStringToSSML(tstStr);
        self.assertEqual(newStr, "<prosody volume='+10%'> This</prosody> little light");

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
        self.assertEqual(newStr, "<emphasis level='none'> This</emphasis> little <prosody volume='+60%'>light</prosody>");
    
        # Nested markup:
        tstStr = '%sV40This %sP60little%s light%s' % (MarkupManagement.openMark,
                                                      MarkupManagement.openMark,
                                                      MarkupManagement.closeMark,
                                                      MarkupManagement.closeMark);
        newStr = MarkupManagement.convertStringToSSML(tstStr);
        self.assertEqual(newStr, "<prosody volume='+40%'>This <prosody pitch='+60%'>little</prosody> light</prosody>");
        
    
    def isProperMarkupOpening():
        MarkupManagement.isProperMarkupOpening("Good %sE10Work" % MarkupManagement.openMark, len('Good '));



if __name__ == '__main__':
    unittest.main();
