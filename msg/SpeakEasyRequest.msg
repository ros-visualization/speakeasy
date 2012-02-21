# IMPORTANT: You should never have to generate this message yourself.
# Use the sound_play::SoundClient C++ helper or the
# speakeasy.libsoundplay.SoundClient Python helper.
#
# This message type is derived from an older SoundRequest.msg
# in audio_common.sound_play. This type is backward compatible
# with SoundRequest.msg, but can additionally carry a choice
# of sound engine to use.

# Sounds
int8 BACKINGUP = 1
int8 NEEDS_UNPLUGGING = 2
int8 NEEDS_PLUGGING = 3
int8 NEEDS_UNPLUGGING_BADLY = 4
int8 NEEDS_PLUGGING_BADLY = 5

# Sound identifiers that have special meaning
int8 ALL = -1 # Only legal with PLAY_STOP
int8 PLAY_FILE = -2
int8 SAY = -3

int8 sound # Selects which sound to play (see above)

# Commands
int8 PLAY_STOP = 0 # Stop this sound from playing
int8 PLAY_ONCE = 1 # Play the sound once
int8 PLAY_START = 2 # Play the sound in a loop until a stop request occurs

int8 command # Indicates what to do with the sound

string arg # file name or text to say
string arg2 # other arguments

string text_to_speech_engine # e.g. "festival", "cepstral"