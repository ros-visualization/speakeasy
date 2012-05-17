# Request that SpeakEasy node perform text-to-speech
# conversion, and to speak the result over the node's
# speakers.
#
# Command API:
#	-  say(text, voiceName=, engineName=)
#	-  stop()

# Command Codes:
int8 SAY = 0
int8 STOP= 1

# Parameters:
int8 command  	    # One of the above Command codes
string text		    # mandatory
string voiceName    # optional. Use default voice if unspecified
string engineName   # optional. Use default engine if unspecified
