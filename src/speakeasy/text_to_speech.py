#!/usr/bin/env python

import os
import subprocess
import tempfile

from sound_player import SoundPlayer;

class TextToSpeechProvider(object):
    
    def __init__(self):
        self.t2sEngines = {};
        self.defaultEngine = self.findAvailableTTSEngines();
        
    # -----------------------------------------------  Public Methods ---------------------------------
    
    def sayToFile(self, text, voiceName=None, t2sEngineName=None, destFileName=None):
        '''
        Create a sound file with the result of turning the
        string passed in parameter 'text' too sound. The 
        given voice engine and voice are used.
        @param text: String to convert into sound.
        @type text: string
        @param t2sEngineName: Name of text-to-speech engine: "festival", "cepstral", "mact2s"
        @type t2sEngineName: string
        @param voiceName: Name of voice to use. Must be a voice supported by the given t2s engine.
        @type voiceName: string
        @raise NotImplementedError: if voice is not supported by the given engine.
        @raise ValueError: if given engine name does not correspond to any know text-to-speech engine. 
        '''
        try:
            if t2sEngineName is None:
                engine = self.defaultEngine;
            else:
                engine = self.t2sEngines[str(t2sEngineName).lower()];
        except KeyError:
            raise ValueError("Unknown text-to-speech engine: " + str(t2sEngineName));
        
        return engine.sayToFile(text, voiceName, destFileName);

    def say(self, text, voiceName=None, t2sEngineName=None):

        try:
            if t2sEngineName is None:
                engine = self.defaultEngine;
            else:
                engine = self.t2sEngines[str(t2sEngineName).lower()];
        except KeyError:
            raise ValueError("Unknown text-to-speech engine: " + str(t2sEngineName));
        engine.say(text, voiceName);

    # -----------------------------------------------  Private Methods ---------------------------------
    def findAvailableTTSEngines(self):
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
    
    def __init__(self):
        pass

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
    
    def __init__(self):
        super(Festival, self).__init__()
        self.ttsEngineName = "festival";
        self.voiceList = None;
        self.getVoiceList();
        self.txtfile = tempfile.NamedTemporaryFile(prefix='speakeasy', suffix='.txt')
        self.txtfilename = self.txtfile.name

    def say(self, text, voice=None):
        # Too complicated to set a voice for now. Ignore that parameter.
        commandLine = 'echo "' + str(text) + '" | festival --tts';
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
        commandLine = "text2wave -eval '(" + str(voice) + ")' " + self.txtfilename + " -o " + str(destFileName);      
        os.system(commandLine);
        self.txtfile.close();
        if os.stat(destFileName).st_size == 0:
            raise OSError(failureMsg);
        return destFileName;
    
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
    
    def __init__(self):
        super(Cepstral, self).__init__()
        self.t2sDestFilename = tempfile.NamedTemporaryFile(prefix='speakeasy', suffix='.wav')
        self.voiceList = None;
        self.getVoiceList();
        self.ttsEngineName = "cepstral";

    def say(self, text, voice=None):
        voice = self.checkVoiceValid(voice, self.defaultVoice);
        failureMsg = "Sound synthesis failed. Is Cepstral's swift installed? Is a Cepstral voice installed? Try running 'rosdep satisfy speakeasy|sh'. Refer to http://pr.willowgarage.com/wiki/sound_play/Troubleshooting"
        commandLine = "padsp swift -n " + str(voice) + " '" + str(text) + "'";
        os.system(commandLine);
        
        
    def sayToFile(self, text, voice=None, destFileName=None):
        
        voice = self.checkVoiceValid(voice, self.defaultVoice);
        if destFileName is None:
            (destFile, destFileName) = tempfile.mkstemp(prefix='speakeasy', suffix='.wav')
        else:
            destFile = os.open(destFileName, 'w')
        
        failureMsg = "Sound synthesis failed. Is Cepstral's swift installed? Is a Cepstral voice installed? Try running 'rosdep satisfy speakeasy|sh'. Refer to http://pr.willowgarage.com/wiki/sound_play/Troubleshooting"
        commandLine = 'padsp swift -n ' + str(voice) + ' "' + str(text) + '"' + ' -o ' + str(destFileName);
        os.system(commandLine);
        os.close(destFile);
        if os.stat(destFileName).st_size == 0:
            raise OSError(failureMsg);
        return destFileName;

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
        self.t2sDestFilename = tempfile.NamedTemporaryFile(prefix='speakeasy', suffix='.aiff')
        self.voiceList = None;
        self.getVoiceList();        
        self.ttsEngineName = "mact2s";

    def say(self, text, voice=None):
        voice = self.checkVoiceValid(voice, self.defaultVoice);
        commandLine = 'say -v ' + str(voice) + ' "' + str(text) + '"';
        os.system(commandLine); 

    def sayToFile(self, text, voice=None, destFileName=None):
        
        voice = self.checkVoiceValid(voice, self.defaultVoice);
        if destFileName is None:
            (destFile, destFileName) = tempfile.mkstemp(prefix='speakeasy', suffix='.wav')
        else:
            destFile = os.open(destFileName, 'w')
        
        #commandLine = 'say -v ' + str(voice) + ' "' + str(text) + '"'; 
        commandLine = 'say -v ' + str(voice) + '-o ' + str(destFileName) + ' "' + str(text) + '"'; 
        os.system(commandLine);
        os.close(destFile);
        if os.stat(destFileName).st_size == 0:
            raise OSError(failureMsg);
        return destFileName;


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
#    tte.say("This is a test.")
#    tte.say("This is a test.", voiceName='David');
#    try:
#        tte.say("This is a test.", voiceName='Alex');
#    except ValueError:
#        pass # Expected.
#    
#    tte.say("This is a test", t2sEngineName="festival");
#    tte.say("This is a test", t2sEngineName="Festival");

    soundPlayer = SoundPlayer();
    
#    fileName = tte.sayToFile("Testing Cepstral say to file.");
#    print "Cepstral printed to: " + str(fileName);
#    soundPlayer.play(fileName, blockTillDone=True);
#    os.remove(fileName);

    fileName = tte.sayToFile("Testing Festival say to file.", t2sEngineName="Festival");
    print "Festival printed to: " + str(fileName);
    soundPlayer.play(fileName, blockTillDone=True);
    os.remove(fileName);

    print "Done";

