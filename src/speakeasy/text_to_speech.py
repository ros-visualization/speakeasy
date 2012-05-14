#!/usr/bin/env python

import os
import time
import subprocess
import tempfile

from sound_player import SoundPlayer;

class TextToSpeechProvider(object):
    '''
    Abstraction for interacting with text-to-speech engines on 
    Ubuntu, Mac, and (unimplemented:) Windows. Detects Festival and
    Cepstral engines on Ubuntu. Main public facilities: Speak an 
    utterance immediately, given a string, and generate a .wav file
    from the text-to-speech conversion.
    '''
    
    def __init__(self):
        self.t2sEngines = {};
        self.defaultEngine = self.findAvailableTTSEngines();
        self.lastUsedEngineObj = None;
        
    # -----------------------------------------------  Public Methods ---------------------------------
    
    def sayToFile(self, text, voiceName=None, t2sEngineName=None, destFileName=None):
        '''
        Create a sound file with the result of turning the
        string passed in parameter 'text' to sound. The 
        given voice engine and voice are used. If no destination file
        is provided, a temporary file is created, and its filename is
        returned after the file is closed. The caller bears responsibility
        for removing that temporary file.
        
        @param text: String to convert into sound.
        @type text: string
        @param t2sEngineName: Name of text-to-speech engine: "festival", "cepstral", "mact2s"
        @type t2sEngineName: string
        @param voiceName: Name of voice to use. Must be a voice supported by the given t2s engine.
        @type voiceName: string
        @param destFileName: Path into which the resulting sound should be directed. If not
                             provided, a temp file is created.
        @type destFileName: string 
        @raise NotImplementedError: if voice is not supported by the given engine.
        @raise ValueError: if given engine name does not correspond to any know text-to-speech engine. 
        '''
        self.lastUsedEngineObj = self.getEngineObj(t2sEngineName)
        return self.lastUsedEngineObj.sayToFile(text, voiceName, destFileName);

    def say(self, text, voiceName=None, t2sEngineName=None, blockTillDone=False):
        '''
        Immediately speak the given string with the given voice on the given engine.
        If voice or engine are not provided, defaults are used.
        @param text: String to speak
        @type text: string
        @param voiceName: Designation of the voice to use.
        @type voiceName: string
        @param t2sEngineName: Name of text-to-speech engine to use.
        @type t2sEngineName: string
        @raise ValueError: if unknown text-to-speech engine.
        @raise OSError: if failure in running the text-to-speech command in underlying shell 
        '''

        self.lastUsedEngineObj = self.getEngineObj(t2sEngineName)
        self.lastUsedEngineObj.say(text, voiceName);
        # Caller wants to block till sound done?
        if blockTillDone:
            self.waitForSoundDone()
        

    def stop(self):
        self.lastUsedEngineObj.stop();

    def busy(self):
        '''
        Return True if any of the text-to-speech engines is currently
        synthesizing. Else return False;
        '''
        if self.lastUsedEngineObj is None:
            return False;
        else:
            return self.lastUsedEngineObj.busy();

    def waitForSoundDone(self):
        '''
        Block until speech finished playing.
        '''
        while self.busy():
            time.sleep(0.3);

    # -----------------------------------------------  Private Methods ---------------------------------
    
    def getEngineObj(self, engineName):
        '''
        From a text-to-speech engine name that may be None,
        return an engine object.
        @param engineName: Name of text-to-speech engine to use
        @type engineName: string
        @return: Subclass of TextToSpeechEngine
        '''
        try:
            if engineName is None:
                return self.defaultEngine;
            else:
                return self.t2sEngines[str(engineName).lower()];
        except KeyError:
            raise ValueError("Unknown text-to-speech engine: " + str(engineName));
    
    def findAvailableTTSEngines(self):
        '''
        Try to sense the underlying OS. Then identify the available
        text-to-speech engines. Return the default engine to be used.
        @return: Default engine instance.
        @returnt: TextToSpeechEngine subclass
        @raise ValueError: if no speech engine is found. 
        '''
        defaultEngine = None;
        if os.uname()[0].lower().find('linux') > -1:
            defaultEngine = self.linuxTTSEngines();
        if os.uname()[0].lower().find('cygwin') > -1:
            defaultEngine = self.windowsTTSEngines();
        if os.uname()[0].lower().find('mac') > -1:
            defaultEngine = self.macTTSEngines();
        
        if len(self.t2sEngines) == 0:
            raise ValueError("No text-to-speech engine found.");
        return defaultEngine;
        
    def linuxTTSEngines(self):
        '''
        Called if underlying machine is Linux. Explores which
        text-to-speech engines are available. Festival is built
        into Ubuntu. Cepstral is a for-pay engine.
        @return: Text-to-speech engine instance to use as default. None if no 
                 text-to-speech-engine is available.
        @returnt: TexToSpeechEngine
        '''
        
        festivalPath = TextToSpeechProvider.which("text2wave");
        if (festivalPath is not None):
            self.t2sEngines["festival"] = Festival();
            
        cepstralSwiftPath = TextToSpeechProvider.which("swift");
        if (cepstralSwiftPath is not None):
            self.t2sEngines["cepstral"] = Cepstral();
            
        # If Cepstral is available, make it the default engine:
        try:
            return  self.t2sEngines["cepstral"];
        except KeyError:
            pass;
        try:
            return self.t2sEngines["festival"];
        except KeyError:
            pass;

        return None;
        

    def macTTSEngines(self):
        '''
        Called if underlying machine is Mac. Explores which
        text-to-speech engines are available.
        @return: Text-to-speech engine instance to use as default. None if no 
                 text-to-speech-engine is available.
        @returnt: TexToSpeechEngine
        '''
        self.t2sEngines["mact2s"] = MacTextToSpeech();
        return self.t2sEngines["mact2s"];
        
    def windowsTTSEngines(self):
        raise NotImplementedError("Windows text-to-speech not yet implemented.");

    @staticmethod
    def which(program):
        '''
        Implements the Unix 'which' shell command, extended to consider
        not just $PATH, but also $PYTHONPATH when searching for an
        executable of the given name (the program parameter). $PATH
        is given preference; it is searched first.
        @param program: Name of the executable with or without path
        @type: string
        @return: None if no executable found, else full path to executable.
        '''
        def is_exe(fpath):
            return os.path.exists(fpath) and os.access(fpath, os.X_OK)
    
        def ext_candidates(fpath):
            yield fpath
            for ext in os.environ.get("PATHEXT", "").split(os.pathsep):
                yield fpath + ext
    
        fpath, fname = os.path.split(program)
        if fpath:
            if is_exe(program):
                return program
        else:
            for path in os.environ["PATH"].split(os.pathsep):
                exe_file = os.path.join(path, program)
                for candidate in ext_candidates(exe_file):
                    if is_exe(candidate):
                        return candidate

        def is_exe(fpath):
            return os.path.exists(fpath) and os.access(fpath, os.X_OK)
    
        def ext_candidates(fpath):
            yield fpath
            for ext in os.environ.get("PATHEXT", "").split(os.pathsep):
                yield fpath + ext
    
        fpath, fname = os.path.split(program)
        if fpath:
            if is_exe(program):
                return program
        else:
            for path in os.environ["PATH"].split(os.pathsep):
                exe_file = os.path.join(path, program)
                for candidate in ext_candidates(exe_file):
                    if is_exe(candidate):
                        return candidate
    

        # Try the Pythonpath:
        pPath = os.getenv("PYTHONPATH")
        if pPath is None:
           return None
        for path in pPath.split(os.pathsep):
            exe_file = os.path.join(path, program)
            for candidate in ext_candidates(exe_file):
                if is_exe(candidate):
                    return candidate
    
        return None
        
# ----------------------------------  Text to Speech Engine Classes and Subclasses ---------------

class TextToSpeechEngine(object):
    '''
    Abstract class from which text-to-speech engine classes are derived.
    '''
    
    def __init__(self):
        self.fullPathToExecutable = None;

    def getEngineName(self):
        return self.ttsEngineName;

    def getT2SDestFilename(self):
        return self.t2sDestFilename;
        
    def checkVoiceValid(self, voice, defaultVoice):
        '''
        Called from subclasses. Given the name of a voice, check whether it is either None,
        or a voice that is supported by the sound engine. If voice is None, the name of the 
        engine's default voice is returned.
        @param voice: Name of voice, or None
        @type voice: {string | NoneType}
        '''
        if voice is None:
            return defaultVoice;
        else:
            try:
                self.getVoiceList().index(voice);
            except ValueError:
                raise ValueError("Voice engine %s does not support a voice named %s." % (self.getEngineName(), str(voice)));
            return voice;
        
        
class Festival(TextToSpeechEngine):
    '''
    Wrapper for the Linux Festival text-to-speech engine.
    '''
    
    def __init__(self):
        super(Festival, self).__init__()
        self.ttsEngineName = "festival";
        self.fullPathToExecutable = TextToSpeechProvider.which("festival");
        if self.fullPathToExecutable is None:
            raise NotImplementedError("Festival text-to-speech engine is not implemented.")
        self.voiceList = None;
        self.getVoiceList();
        self.txtfile = tempfile.NamedTemporaryFile(prefix='speakeasy', suffix='.txt')
        self.txtfilename = self.txtfile.name

    def say(self, text, voice=None):
        # Too complicated to set a voice for now. Ignore that parameter.
        commandLine = 'echo "' + str(text) + '" | padsp festival --tts &';
        os.system(commandLine);
        
    def sayToFile(self, text, voice=None, destFileName=None):
        
        voice = self.checkVoiceValid(voice, self.defaultVoice);
        if destFileName is None:
            (destFile, destFileName) = tempfile.mkstemp(prefix='speakeasy', suffix='.wav')
        else:
            destFile = os.open(destFileName, 'w')
        
        failureMsg = "Sound synthesis failed. Is the Festival engine installed? Is a festival voice installed? Try running 'rosdep satisfy speakeasy|sh'. Refer to http://pr.willowgarage.com/wiki/sound_play/Troubleshooting"
        self.txtfile.write(text)
        self.txtfile.flush()
        commandLine = "text2wave -eval '(" + str(voice) + ")' " + self.txtfilename + " -o " + str(destFileName) + " &";
        os.system(commandLine);
        self.txtfile.close();
        if os.stat(destFileName).st_size == 0:
            raise OSError(failureMsg);
        return destFileName;
   
    def stop(self):
        os.system("killall " + str("audsp"));

    def busy(self):
        for line in os.popen("ps xa"):
            if  line.find("festival") > -1:
                return True;
        return False;
    
    def getVoiceList(self):
        #TODO: run festival, and issue Scheme command (voice.list)
        #      to get the real list of voices.
        
        if self.voiceList is not None:
            return self.voiceList;
        
        self.voiceList = ["voice_kal_diphone"];
        self.defaultVoice = "voice_kal_diphone";
        return self.voiceList; 
    
    def initTextFile(self, text):
        # Just in case client calls getCommandString() twice on the same
        # TTS instance:
        self.txtfile.truncate();
        
    
    
class Cepstral(TextToSpeechEngine):
    '''
    Wrapper for Cepstral text-to-speech engine.
    '''
    
    def __init__(self):
        super(Cepstral, self).__init__()
        
        self.fullPathToExecutable = TextToSpeechProvider.which("swift");
        if self.fullPathToExecutable is None:
            raise NotImplementedError("Cepstral text-to-speech engine is not implemented.")
        
        self.t2sDestFilename = tempfile.NamedTemporaryFile(prefix='speakeasy', suffix='.wav')
        self.voiceList = None;
        self.getVoiceList();
        self.ttsEngineName = "cepstral";

    def say(self, text, voice=None):
        voice = self.checkVoiceValid(voice, self.defaultVoice);
        failureMsg = "Sound synthesis failed. Is Cepstral's swift installed? Is a Cepstral voice installed? Try running 'rosdep satisfy speakeasy|sh'. Refer to http://pr.willowgarage.com/wiki/sound_play/Troubleshooting"
        commandLine = "padsp swift -n " + str(voice) + " '" + str(text) + "' &";
        os.system(commandLine);
        
    def stop(self):
        os.system("killall --quiet " + str("swift.bin"));
        
    def sayToFile(self, text, voice=None, destFileName=None):
        
        voice = self.checkVoiceValid(voice, self.defaultVoice);
        if destFileName is None:
            (destFile, destFileName) = tempfile.mkstemp(prefix='speakeasy', suffix='.wav')
        else:
            destFile = os.open(destFileName, 'w')
        
        failureMsg = "Sound synthesis failed. Is Cepstral's swift installed? Is a Cepstral voice installed? Try running 'rosdep satisfy speakeasy|sh'. Refer to http://pr.willowgarage.com/wiki/sound_play/Troubleshooting"
        commandLine = 'padsp swift -n ' + str(voice) + ' "' + str(text) + '"' + ' -o ' + str(destFileName) + " &";
        os.system(commandLine);
        os.close(destFile);
        if os.stat(destFileName).st_size == 0:
            raise OSError(failureMsg);
        return destFileName;

    def busy(self):
        for line in os.popen("ps xa"):
            if  line.find("swift") > -1:
                return True;
        return False;

    def getVoiceList(self):
        
        if self.voiceList is not None:
            return self.voiceList;
        
        absSwiftPath = TextToSpeechProvider.which("swift");
        if absSwiftPath is None:
            raise NotImplemented("Cannot find the Cepstral executable 'swift'. Make sure it is in $PATH or $PYTHONPATH of the shell where this Python program started.")
        p = subprocess.Popen([absSwiftPath, "--voices"], stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        tbl, errors = p.communicate()
        lines = tbl.splitlines();
        
        # Throw out header lines:
        for lineNum, line in enumerate(lines):
            if not line.startswith("-"):
                continue;
            break
        voiceNames = []
        for voiceLine in lines[lineNum:]:
            if not voiceLine.startswith("-"):
                voiceNames.append(str(voiceLine.split()[0]))
                
        # Remember the list for any subsequent calls:
        self.voiceList = voiceNames;
        self.defaultVoice = "David";
        return voiceNames    

class MacTextToSpeech(TextToSpeechEngine):
    '''
    Wrapper for the Mac text to speech engine. 
    The command line t2s command on the Mac works like this:
          say [-v <voice>] [-f <inputFile>] [-o <aiffOutputFile>]
    '''
    
    def __init__(self):
        super(MacTextToSpeech, self).__init__();
        
        self.fullPathToExecutable = TextToSpeechProvider.which("say");
        if self.fullPathToExecutable is None:
            raise NotImplementedError("Mac text-to-speech engine is not implemented/found.")
        
        self.t2sDestFilename = tempfile.NamedTemporaryFile(prefix='speakeasy', suffix='.aiff')
        self.voiceList = None;
        self.getVoiceList();        
        self.ttsEngineName = "mact2s";

    def say(self, text, voice=None):
        voice = self.checkVoiceValid(voice, self.defaultVoice);
        commandLine = 'say -v ' + str(voice) + ' "' + str(text) + '" &';
        os.system(commandLine);

    def sayToFile(self, text, voice=None, destFileName=None):
        
        voice = self.checkVoiceValid(voice, self.defaultVoice);
        if destFileName is None:
            (destFile, destFileName) = tempfile.mkstemp(prefix='speakeasy', suffix='.wav')
        else:
            destFile = os.open(destFileName, 'w')
        
        #commandLine = 'say -v ' + str(voice) + ' "' + str(text) + '"'; 
        commandLine = 'say -v ' + str(voice) + '-o ' + str(destFileName) + ' "' + str(text) + '" &';
        os.system(commandLine);
        os.close(destFile);
        if os.stat(destFileName).st_size == 0:
            raise OSError(failureMsg);
        return destFileName;

    def stop(self):
        os.system("killall --quiet " + str("say"))

    def busy(self):
        for line in os.popen("ps xa"):
            if  line.find("say") > -1:
                return True;
        return False;

    def getVoiceList(self):
        #TODO: find all Mac voices automatically
        TextToSpeechProvider
        if self.voiceList is not None:
            return self.voiceList;
        
        self.voiceList = ["Alex"];
        self.defaultVoice = "Alex";
        return self.voiceList; 
        
if __name__ == "__main__":
    
    tte = TextToSpeechProvider();
    print "Test defaulting t2s engine and voice"
    tte.say("This is a test.")
    print "Done testing defaulting t2s engine and voice"
    print "---------------"
    
    print "Test default t2s engine, ask for particular voice"
#    tte.say("This is a test.", voiceName='David');
    print "Done testing default t2s engine, ask for particular voice"
    print "---------------"
    

    print "Test deliberate voice name-unknown error."
#    try:
#        tte.say("This is a test.", voiceName='Alex');
#    except ValueError:
#        pass # Expected.
#
    print "Done testing deliberate voice name-unknown error."
    print "---------------"
    
    print "Test ask for specific t2s engine"    
#    tte.say("This is a test", t2sEngineName="festival");
    print "Done testing ask for specific t2s engine"    
    print "---------------"

    print "Test tolerance to wrong case in speech engine name:"
#    tte.say("This is a test", t2sEngineName="Festival");
    print "Done testing tolerance to wrong case in speech engine name:"
    print "---------------"

    soundPlayer = SoundPlayer();

    print "Test say-to-file Cepstral"
#    fileName = tte.sayToFile("Testing Cepstral say to file.");
#    print "Cepstral printed to: " + str(fileName);
#    soundPlayer.play(fileName, blockTillDone=True);
#    os.remove(fileName);
    print "Done testing say-to-file Cepstral"
    print "---------------"

    print "Test say-to-file Festival"
#    fileName = tte.sayToFile("Testing Festival say to file.", t2sEngineName="Festival");
#    print "Festival printed to: " + str(fileName);
#    soundPlayer.play(fileName, blockTillDone=True);
#    os.remove(fileName);
    print "Done testing say-to-file Festival"
    print "---------------"

    print "Done";

