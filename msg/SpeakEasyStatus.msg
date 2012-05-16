# Latched message from SpeakEasy service to communicate
# its available:
#    - Text-to-speech engine names (e.g. ['festival', 'cepstral'])
#    - Voices for each text-to-speech engine (e.g. ['David'])
#    - Sound files (e.g. ['drill.wav', 'bomb.ogg'])
#    - Song files
#    - Number of sounds that may play simultaneously
#    - Music play status: Stopped, paused, or playing.
#    - Current sound effects volume
#    - Current music volume

# Play Status codes:
int8   STOPPED = 0
int8   PAUSED  = 1
int8   PLAYING = 2
 

string[]    ttsEngines
TtsVoices[] voices
string[]    sounds
string[]    songs
int8	 	numSoundChannels
int8        musicStatus
float32     soundVolume
float32     musicVolume

