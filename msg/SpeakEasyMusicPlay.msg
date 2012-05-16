# Request that SpeakEasy node play a named sound file, which
# is stored as a .wav or .ogg file on the node's machine. 
# The sound will play on the speakers that are connected
# to the node's machine. Use this message for sounds that 
# are longer than very brief sound effects, like songs.
# Use SoundPlay.msg for short sounds. Only one song 
# will play, but many sounds can play at the same time.
#
# Command API:
#
#  play(songName, repeats=, startTime=, volume=)
#  stop()
#  pause()
#  unpause()
#  setVol(volume)
#  setPlayhead(time, timeReference=)

# Command Codes:
int8 PLAY = 0
int8 STOP = 1
int8 PAUSE = 2
int8 UNPAUSE = 3
int8 SET_VOL = 4
int8 SET_PLAYHEAD = 5

# For setPlayhead(): whether time spec is intended relative to
# the current playhead position, or absolute from the start:
int8 RELATIVE = 0
int8 ABSOLUTE = 1

# Parameters:
int8 command  			  # One of the above Command codes

string sound_name         # Sound file name with or without file extension.
int8   repeats            # Optional, and only relevant for play(). Default is to play once.
						  # Required for play(). Optional for others
float32 volume            # Value: 0.0 to 1.0. Current vol if unspecified. Required for setVol()
float32 time              # Optional for play(): How far into a song to start playing 
					      # Mandatory for setPlayhead().
int8   timeReference	  # One of RELATIVE or ABSOLUTE					 
