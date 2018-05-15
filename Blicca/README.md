# Blicca

### Second version of the ROV's C code (atMega code)

The current code is RovCore_Blicca.

We are working to fix some timing issues, the test code is RovCore_Blicca.rev. In order to use it the pressure sensor's library must be modified: inside the library folder the file MS55837.cpp has to be repaced with the modified one present here (you can find it in <arduino_libraries>/BlueRobotics_MS5837_Library/MS55837.cpp, make a copy of the original one in order to be able to reset it in case of necessity).
