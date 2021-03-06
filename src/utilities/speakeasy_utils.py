#!/usr/bin/env python

import os;
import subprocess;
import re;
import time;
import socket;
import string;
import xmlrpclib;
import unittest;

class SpeakeasyUtils(object):
    
    #----------------------------------
    # fileExtension 
    #--------------
    
    @staticmethod
    def fileExtension(filePath):
        '''
        Return extension of the given file name, without the 'dot'.
        @param filePath: full path or filename.
        @type filePath: string
        @return: Extension only. Example: foo.txt --> 'txt'
        @rtype: string
        '''
        return os.path.splitext(filePath)[1][1:].strip()

    #----------------------------------
    # uriPointsToLocalhost
    #--------------

    @staticmethod
    def uriPointsToLocalhost(theURI):
        '''
        Examines whether given URI is a pointer to localhost. Checks for
        'http://localhost:', 'http://127.0.1.1:', 'http://shortHostname:', and
        'http://hostnameWithFQDN:'. Will miss 'http://<ownOutsideIPAddress>'   
        @param theURI: URI to test. Example: contents of ROS_MASTER_URI
        @type theURI: string
        '''
        if string.find(theURI, 'http://localhost') > -1:
            return True;
        if string.find(theURI, 'http://127.0.1.1') > -1:
            return True;
        if string.find(theURI, 'http://' + socket.gethostname() + ':') > -1:
             return True;
        if string.find(theURI, 'http://' + socket.gethostbyaddr(socket.gethostname())[0]) > -1:
             return True;
        return False;


    #----------------------------------
    # processRunning
    #--------------

    @staticmethod
    def processRunning(proc_name):
        '''
        Returns true if process of given name is running. 
        @param proc_name: Process name (no need for full path)
        @type proc_name: string
        @return: True if process is currently running, else false.
        '''
        ps = subprocess.Popen("ps ax -o pid= -o args= ", shell=True, stdout=subprocess.PIPE)
        ps_pid = ps.pid
        output = ps.stdout.read()
        ps.stdout.close()
        ps.wait()
    
        for line in output.split("\n"):
            res = re.findall("(\d+) (.*)", line)
            if res:
                pid = int(res[0][0])
                if proc_name in res[0][1] and pid != os.getpid() and pid != ps_pid:
                    return True
        return False

    #----------------------------------
    #  rosMasterRunning
    #--------------
    
    @staticmethod
    def rosMasterRunning():
        masterURI = os.getenv("ROS_MASTER_URI");
        if len(masterURI) == 0:
            return False;
        try:
            proxy = xmlrpclib.ServerProxy( masterURI )
            state = proxy.getSystemState("")
            if state[0] != 1:
                return False
        except:
            return False;
        return True;     

    #----------------------------------
    #  rosNodeRunning
    #--------------
    
    @staticmethod
    def rosNodeRunning(nodeName):
        '''
        Returns True/False depending on whether a ROS node of the given name 
        is currently running.
        @param nodeName: ROS node name as listed in rosnode list
        @type nodeName: string.
        '''
        rosnodes = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        output = rosnodes.stdout.read()
        rosnodes.stdout.close()
        rosnodes.wait()
    
        for line in output.split("\n"):
            res = re.findall(nodeName + "$", line)
            if res:
                return True
        return False

    #----------------------------------
    # waitForRosNode 
    #--------------

    @staticmethod
    def waitForRosNode(nodeName, timeout=None, waitMessage=None, provideTimeInfo=None):
        '''
        Wait for a named ROS node to start up. Wait can just hang, or a given message may be
        provided that is printed every second. Additionally, a count-down in seconds may be
        added automatically to the given message, if a timeout is provided. Or, if no timeout
        is provided, a count-up maybe provided to keep track of how long the (indefinite) wait
        has been going on. 
        @param nodeName: entire, or partial name of ROS node as shown in 'rosnode list'
        @type nodeName: string
        @param timeout: Number of seconds after which a ValueError is to be raised.
        @type timeout: int
        @param waitMessage: any message to be printed every second during the wait.
        @type waitMessage: string
        @param provideTimeInfo: provide time info in the 1-second status messages. If a timeout is provided,
                                the time info will be a count-down, else a count-up. If a waitMessage
                                is provided, the timing info is appended to that message.
        @type provideTimeInfo: boolean.
        @raises NotImplementedError: if service is not running before timeout elapses.
        '''
        startTime = time.time();
        while True:
            if SpeakeasyUtils.rosNodeRunning(nodeName):
                return True;
            if timeout == -1:
                return False;
            timeNow = time.time();
            elapsedTime = int(timeNow - startTime);
            if timeout is not None and (elapsedTime >= timeout):
                raise NotImplementedError("Node %s not running." % nodeName);
            if waitMessage is not None:
                msg = waitMessage;
            else:
                msg = None;
            if provideTimeInfo is not None:
                if timeout is not None:
                    # Wants countdown in status message:
                    if waitMessage is not None:
                        msg = waitMessage + str(timeout - elapsedTime);
                    else:
                        msg = str(timeout - elapsedTime);
                else:
                    # Wants count-up in status message:
                    if waitMessage is not None:
                        msg = waitMessage + str(elapsedTime);
                    else:
                        msg = str(elapsedTime);
                    
            if msg is not None:
                print msg; 
            time.sleep(1.0);

    #----------------------------------
    # findPackage  
    #--------------
        
    @staticmethod
    def findPackage(packageName):
        '''
        Find package's absolute path. None if path not found.
        @param packageName: name of package to find
        @type packageName: string
        @return: absolute path to package. None if not found.
        @rtype: {string | None}
        '''
        try:
            # In Python 2.7 we would simply do this:
            #path = subprocess.check_output(["rospack", "find", packageName]);
            path = subprocess.Popen("rospack find " + packageName, shell=True, stdout=subprocess.PIPE)
            output = path.stdout.read()
            path.stdout.close()
            path.wait()
        except subprocess.CalledProcessError:
            return None;
        return output.strip()

    #----------------------------------
    # ensureType
    #--------------
    
    @staticmethod
    def ensureType(item, type):
        '''
        Given a data item and a Python built-in type name,
        return True if item is of the given type. Else return False. 
        @param item: arbitrary, Python build-in datdum
        @type item: {int | float | bool | basestring | dict | list}
        @param type: Type to check.
        @type type: type object.
        @return: True/False depending on whether or not item is of the given type.
        @rtype: bool
        '''
        if type == int:
            return isinstance(item, int);
        elif type == float:
            return isinstance(item, float);
        elif type == bool:
            return (item == True) or (item == False);
        elif type == dict:
            return isinstance(item, dict);
        elif type == basestring:
            return isinstance(item, basestring);
        elif type == list:
            return isinstance(item, list);
                
    #----------------------------------
    # findPackage 
    #--------------

    @staticmethod
    def findPackage(packName, default=None):
        '''
        Find full path of given package. Return None if package is not installed
        on the local machine.
        @param packName: Name of package whose path is to be found.
        @type packName: Full path, or None is the package is not installed.
        @rtype: {string | None}
        '''
        
        # Python 2.7 introduces subprocess.check_output, which is 
        # exactly what we want. For versions < 2.7 we need to use
        # the older pipe mechanism:
        try:
            return subprocess.check_output(["rospack", "find", packName]).strip();
        except AttributeError:
            python26Result = subprocess.Popen(['rospack', 'find', packName], stdout=subprocess.PIPE).communicate()
            if len(python26Result[0]) == 0:
                return default;
            else:
                return python26Result[0].strip();
        except subprocess.CalledProcessError as err:
            return default;
        
    #----------------------------------
    # bashQuotify
    #--------------
        
    @staticmethod
    def bashQuotify(str):
        '''
        Bash does not tolerate single quotes within single-quoted strings.
        Backslashing the embedded single quote does *not* work. Thus:
        echo 'That's it' will lead to error as expected, but so will
        echo 'That\'s it'. The solution is to replace all single quotes
        with '\'' (all quotes are single quotes here).
        
        @param str: String in which to make single quotes safe. Ok not to 
                    have any single quotes.
        @type str: string
        '''
        
        return str.replace("'", "'" + "\\" + "'" + "'");
        
        
# ----------------------------  Testing ----------------------

class UtilsTest(unittest.TestCase):
    
#    def testFileExtension(self):
#        self.assertEqual(SpeakeasyUtils.fileExtension("foo.txt"), 'txt', "Simple extension failed");
#        self.assertEqual(SpeakeasyUtils.fileExtension("foo"), '', "No-extension failed");
#        self.assertEqual(SpeakeasyUtils.fileExtension("/foo/bar.ogg"), 'ogg', "Full name failed");
    
#    def testProcessRunning(self):
#        self.assertTrue(SpeakeasyUtils.processRunning('bash'), "No bash process is seen to be running.");
        
#    def testRosNodeRunning(self):
#        self.assertTrue(SpeakeasyUtils.rosNodeRunning('speakeasy_node'), "No speakeasy_node is seen to be running.");
#            
#    def testWaitForRosNode(self):
#        print str(SpeakeasyUtils.waitForRosNode('speakeasy_node'));
#        #print str(SpeakeasyUtils.waitForRosNode('foo_node'));
#        
#        # Test timeout functionality:
#        startTime = time.time();
#        with self.assertRaises(NotImplementedError) as timeoutError:
#            SpeakeasyUtils.waitForRosNode('foo_node', waitMessage="Waiting for foo_node.", timeout=5);
#        endTime = time.time();
#        self.assertGreater(endTime - startTime, 5, "Timeout was too short.");
#        self.assertLess(endTime - startTime, 6, "Timeout was too long.");
#        
#        # Counting down with message:
#        self.assertRaises(NotImplementedError, SpeakeasyUtils.waitForRosNode, 'bar_node', waitMessage="Waiting for bar_node.", timeout=5, provideTimeInfo=True);
#        # Counting up in message indefinitely:
#        #self.assertRaises(NotImplementedError, SpeakeasyUtils.waitForRosNode, 'fum_node', waitMessage="Waiting indefinitely for fum_node.", provideTimeInfo=True);

#    def testEnsureType(self):
#        self.assertTrue(SpeakeasyUtils.ensureType(4, int), "Int not recognized.")
#        self.assertFalse(SpeakeasyUtils.ensureType(4.0, int), "Float recognized as int.")
#        self.assertFalse(SpeakeasyUtils.ensureType([], dict), "List recognized as dict.")
#        self.assertFalse(SpeakeasyUtils.ensureType('foo', list), "String recognized as list.")
#        self.assertTrue(SpeakeasyUtils.ensureType('foo', basestring), "String not recognized.")
#        self.assertTrue(SpeakeasyUtils.ensureType([], list), "List not recognized.")
#        self.assertTrue(SpeakeasyUtils.ensureType({}, dict), "Dict not recognized.")
    
#    def testFindPackage(self):
#        path = SpeakeasyUtils.findPackage("rospy")
#        print path
#        realPath = os.path.abspath(os.path.join(os.getenv("ROS_ROOT") + "/..", "rospy"));
#        self.assertEqual(SpeakeasyUtils.findPackage("rospy"),
#                         realPath,
#                         "Could not find rospy package. findPackage returns '%s' instead of '%s'" % 
#                         (str(SpeakeasyUtils.findPackage("rospy")), realPath));
    
#    def testUriPointsToLocalhost(self):
#        self.assertTrue(SpeakeasyUtils.uriPointsToLocalhost('http://localhost:113111'), "Failed to recognize 'http://localhost:113111' as localhost.");
#        self.assertTrue(SpeakeasyUtils.uriPointsToLocalhost('http://127.0.1.1:113111'), "Failed to recognize 'http://127.0.1.1:113111' as localhost.");
#        self.assertTrue(SpeakeasyUtils.uriPointsToLocalhost('http://' + socket.gethostname() + ':113111'), "Failed to recognize 'http://<hostname>:113111' as localhost.");
#        self.assertFalse(SpeakeasyUtils.uriPointsToLocalhost('http://foo.bar.com:113111'), "Erroneously recognized 'http://foo.bar.com:113111' as localhost.");
        
    def testBashQuotify(self):
        self.assertEqual(SpeakeasyUtils.bashQuotify("Foo"), "Foo", "Failed string without embedded single quote.");
        self.assertEqual(SpeakeasyUtils.bashQuotify("Foo's bar"), "Foo'" + r"\'" + "'s bar", "Failed single embedded quote.");
        
        
        
if __name__ == '__main__':
    

#    import time
#    while True:
#        time.sleep(2);
    unittest.main();
    
    