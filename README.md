speakeasy
=========

SpeakEasy provides a sound node to run on a robot, and a GUI to run on a client. GUI users can type text for conversion to speech by the sound node. Utterances can be saved to buttons, and sound effects are available via buttons. Two operation modes are available: ROS based, and stand-alone. The ROS mode is started via launch/speakeasy_ros.launch, and separates the control GUI from the low level speech engine control. This mode can run the GUI and the produced speech in different ROS nodes. Use this mode, for example, if the sound is to be generated on the robot, and the GUI on a laptop.  The second operating mode is started with launch/speakeasy_standalone.launch. This running mode combines all speech engine interaction with the GUI process. This mode will work with or without ROS on your system.

Below the GUI a small library of music, sound, and text-to-speech classes provide an API for other applications that wish to utilize sound on ROS. This package is at a higher level of abstraction than the ROS sound_play package, which is an alternative option for text-to-speech. Sound_play does not provide the client side convenience of an OOP based abstraction.

