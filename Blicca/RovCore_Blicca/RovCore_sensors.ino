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
  else if(curTemp > 83)
    curTemp = 83;
  op = (curTemp-20)/2; // tempertaure from 20°C to 83°C -> 5 bit with 2°C of resolution
  bitWrite(op,5,1);
  bitWrite(op,6,0);
  bitWrite(op,7,1);

// don't work... don't know...
//  if(curPress < 230) //230 is the ambient pressure
//    prSpi = 0;
//  else if(curPress > 485)
//    prSpi = 255;
//  else
    prSpi = int(curPress - 200);

  buf[0] = int(roll*180/3.14+127);
  buf[1] = int(pitch*180/3.14+127);
  buf[2] = prSpi;
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

