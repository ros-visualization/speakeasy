#!/usr/bin/env python

import subprocess
import threading
import os

class Talker(threading.Thread):
    
    def __init__(self, text):
        super(Talker, self).__init__();
        
        #self.fullPathToExecutable = self.which("audsp");
        self.fullPathToExecutable = self.which("festival");
        if self.fullPathToExecutable is None:
            raise NotImplementedError("Festival text-to-speech engine is not implemented.")
        
        #Thread.__init__();
        self.text = text;
        #self.ttsPipe = subprocess.Popen(["padsp", "festival", "--tts"], stdin=subprocess.PIPE);
        #self.ttsPipe = subprocess.Popen(["padsp swift", "-n " + str("David ") + text]);
        self.busy = False

    
    def run(self):
        self.busy = True
        #commandLine = 'echo "' + str(self.text) + '" | padsp festival --tts';
        commandLine = 'padsp swift -n David "' + str(self.text) + '"';
        os.system(commandLine);
        
        #self.ttsPipe.communicate(self.text);
        self.busy = False
      
    def stop(self):
        print "Called stop"
        #os.system("killall " + str(self.fullPathToExecutable))
        #os.system("killall " + str(self.fullPathToExecutable))
        #os.system("killall --quiet " + str("audsp"))
        #os.system("killall --quiet " + str("padsp"))
        #os.system("killall --quiet " + str("swift"))
        os.system("killall --quiet " + str("swift.bin"))
        #self.ttsPipe.terminate();
        #self.ttsPipe.kill();
        #self.ttsPipe.send_signal(9);
        print "Stop done"
        self.busy = False;
        
    def doneTalking(self):
        return self.busy;
        
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
        
                    
if __name__ == "__main__":
    
    import time;
    
    talkThread = Talker("This is a longer test. Let's see whether we can stop the chatter");
    talkThread.start()
    
    print str(talkThread.doneTalking())
    #time.sleep(1);
    print str(talkThread.doneTalking())
    time.sleep(1);
    print str(talkThread.doneTalking())
    time.sleep(1);
    talkThread.stop();
    time.sleep(1);
    print str(talkThread.doneTalking())
    time.sleep(1);
    

        
