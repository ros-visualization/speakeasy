# Request that SpeakEasy node play a named sound, which
# is stored as a .wav file on the node's machine. 
# The sound will play on the speakers that are connected
# to the node's machine. Use this message for short
# sounds. For music, use MusicPlay.msg. Many sounds can
# play at the same time; only one piece of music can play
# at once.
#
# Command API:
#  play(soundName, volume=)
#  stop(soundName=)
#  pause(soundName=)
#  unpause(soundName=)
#  setVol(newVol, soundName)

# Commands
int8 PLAY=0
int8 STOP=1
int8 PAUSE=2
int8 UNPAUSE=3
int8 SET_VOL=4

int8 command              # One of the above Command codes
string sound_name         # Sound file name with or without file extension.
						  # Required for play(). Optional for others
float32 volume            # Values that cause change: 0.0 to 1.0. Set - -1.0 to use currently set vol.

