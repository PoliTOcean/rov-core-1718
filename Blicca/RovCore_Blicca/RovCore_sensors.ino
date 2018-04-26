//sensors functions
int readPress(){
  pressure.read();            // say to sensor to read
  return pressure.pressure(); //return read value
}

//save the requested pressure
void saveRequestedPressure(){
  reqPress = readPress();
  savePressure = 0;
}

//read temperature function. Same as readPressure() above
int readTemp(){
  temperature.read();
  return temperature.temperature();
}
