//sensors functions
int readPress(){
  pressure.read();            // say to sensor to read
  return pressure.pressure(); //return read value
//  return 0;
}

//save the requested pressure
void saveRequestedPressure(){
  reqPress = readPress();
  savePressure = 0;
}

void dataRead(){
  imuRead();                              //read IMU data
  curPress = readPress();                 //read pressure
  if(curTemp < 20)
    curTemp = 20;
  else if(curTemp > 51)
    curTemp = 51;
  op = (curTemp-20); // tempertaure from 20°C to 51°C -> 5 bit
  bitWrite(op,5,1);
  bitWrite(op,6,0);
  bitWrite(op,7,1);

  buf[0] = roll;
  buf[1] = pitch;
  buf[2] = curPress;
  buf[3] = op;

  cmdstop = bitRead(ref1,4);
  cmdstart = bitRead(ref1,3);
  pinkie = bitRead(ref1,2);
  trigger = bitRead(ref1,1);
  trigger2 = bitRead(ref1,0);

  if(cmdstart)
    startRov();
  if(cmdstop)
    stopRov();
}
