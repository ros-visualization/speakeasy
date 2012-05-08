#!/usr/bin/env python

import os

class TextToSpeechProvider(object):
    
    def __init__(self):
        self.t2sEngine = {};
        self.findAvailableTTSEngines();
    
    def createVoiceFile(self, text, t2sEngineName, voiceName):
        
    
    
    def findAvailableTTSEngines(self):
        if os.uname().lower().find('Linux') > -1:
            self.linuxTTSEngines();
        if os.uname().lower().find('Cygwin') > -1:
            self.windowsTTSEngines();
        if os.uname().lower().find('Mac') > -1:
            self.macTTSEngines();
        
    def linuxTTSEngines(self):
        
        festivalPath = self.which("text2wave");
        if (festivalPath is not None):
            self.t2sEngine["festival"] = Festival();
            
        cepstralSwiftPath = self.which("swift");
        if (cepstralSwiftPath is not None):
            self.t2sEngine["cepstral"] = Cepstral();

    def macTTSEngines(self):
        self.t2sEngine["mact2s"] = MacTextToSpeech();
        
    def windowsTTSEngines(self):
        self.soundEngines = [];
        
    def which(self, program):
        '''
        Implements the Unix 'which' shell command. 
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
    
        return None

# ----------------------------------  Text to Speech Engine Classes and Subclasses ---------------

class TextToSpeechEngine(object):
    
    def __init__(self):
        self.txtfile = tempfile.NamedTemporaryFile(prefix='speakeasy', suffix='.txt')
        self.txtfilename = self.txtfile.name
        self.t2sDestFilename = self.wavfile.name

    def getTextToSpeechEngineName(self):
        return self.ttsEngineName;

    def initTextFile(self, text):
        # Just in case client calls getCommandString() twice on the same
        # TTS instance:
        self.txtfile.truncate();
        
    def runTextToSpeechHelper(self, text, commandLine, failureMsg):
        self.txtfile.write(text)
        self.txtfile.flush()
        os.system(commandLine);
        if os.stat(self.t2sDestFilename).st_size == 0:
            raise OSError(failureMsg);

    def getT2SDestFilename(self):
        return self.t2sDestFilename;
        
    def cleanup(self):
        self.txtfile.close();
        #self.wavfile.close(); # Maybe race condition?
        
class Festival(TextToSpeechEngine):
    
    def __init__(self):
        super(Festival, self).__init__()
        self.t2sDestFilename = tempfile.NamedTemporaryFile(prefix='speakeasy', suffix='.wav')
        self.ttsEngineName = "festival";
        
    def runTextToSpeech(self, voice, text):
        self.initTextFile(text)
        failureMsg = "Sound synthesis failed. Is the Festival engine installed? Is a festival voice installed? Try running 'rosdep satisfy speakeasy|sh'. Refer to http://pr.willowgarage.com/wiki/sound_play/Troubleshooting"
        self.runTextToSpeechHelper(text,
                                   "text2wave -eval '(" + str(voice) + ")' " + self.txtfilename + " -o " + self.t2sDestFilename,
                                   failureMsg);
        return self.t2sDestFilename
    
    def getFestivalVoices(self, absFestivalPath):
        #TODO: run festival, and issue Scheme command (voice.list)
        #      to get the real list of voices.
        
        if self.voiceList is not None:
            return self.voiceList;
        
        self.voiceList = ["voice_kal_diphone"];
        return self.voiceList;
    
class Cepstral(TextToSpeechEngine):
    
    def __init__(self):
        super(Cepstral, self).__init__()
        self.t2sDestFilename = tempfile.NamedTemporaryFile(prefix='speakeasy', suffix='.wav')        
        self.ttsEngineName = "cepstral";
        
    def runTextToSpeech(self, voice, text):
        self.initTextFile(text)        
        failureMsg = "Sound synthesis failed. Is Cepstral's swift installed? Is a Cepstral voice installed? Try running 'rosdep satisfy speakeasy|sh'. Refer to http://pr.willowgarage.com/wiki/sound_play/Troubleshooting"
        self.runTextToSpeechHelper(text,
                                   "swift -n " + str(voice) + " -f " + self.txtfilename + " -o " + self.t2sDestFilename,
                                   failureMsg);
        return self.t2sDestFilename

    def getVoices(self, absSwiftPath):
        
        if self.voiceList is not None:
            return self.voiceList;
        
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
        self.ttsEngineName = "mact2s";

    def runTextToSpeech(self, voice, text):
        #commandLine = 'say -v ' + str(voice) + ' "' + str(text) + '"'; 
        commandLine = 'say -v ' + str(voice) + '-o ' + self.t2sDestFilename + ' "' + str(text) + '"'; 
        os.system(commandLine);
        return None;

    def getVoiceList(self):
        #TODO: find all Mac voices automatically
        
        if self.voiceList is not None:
            return self.voiceList;
        
        self.voiceList = ["Alex"];
        return self.voiceList; 
        
