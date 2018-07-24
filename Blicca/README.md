# Blicca

## Second version of the ROV's code (atMega & Raspberry code)

### politocean

The ROS package needed by the Raspberry.

### RovCore_Blicca

The current atMega code.

### RovCore_Blicca.rev

Test code, we are working to fix some timing issues.

In order to use it the pressure sensor's library must be modified: inside the library folder the file MS55837.cpp has to be repaced with the modified one present here (you can find it in <arduino_libraries>/BlueRobotics_MS5837_Library/MS55837.cpp, make a copy of the original one in order to be able to reset it in case of necessity).
