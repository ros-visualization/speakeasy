# Latched message from SpeakEasy service to communicate
# its available:
#    - Text-to-speech engine names (e.g. ['festival', 'cepstral'])
#    - Voices for each text-to-speech engine (e.g. ['David'])
#    - Sound files (e.g. ['drill.wav', 'bomb.ogg'])
#    - Song files
#    - Number of sounds that may play simultaneously
#    - Current sound effects volume
#    - Current music volume

string[]    ttsEngines
TtsVoices[] voices
string[] 	sounds
string[] 	songs
int8	 	numSoundChannels
float32		soundVolume
float32		musicVolume

