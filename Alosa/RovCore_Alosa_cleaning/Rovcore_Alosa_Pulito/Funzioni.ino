void Wireinit(){
// Initialise I2C communication as Master
  Wire.begin();
  
  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Select gyroscope configuration register
  Wire.write(0x1B);
  // Full scale range = 2000 dps
  Wire.write(0x18);
  // Stop I2C transmission
  Wire.endTransmission();
  
  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Select accelerometer configuration register
  Wire.write(0x1C);
  // Full scale range = +/-16g
  Wire.write(0x18);
  // Stop I2C transmission
  Wire.endTransmission();
  
  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Select power management register
  Wire.write(0x6B);
  // PLL with xGyro reference
  Wire.write(0x01);
  // Stop I2C transmission
  Wire.endTransmission();
}

void Servoinit(){
 //Servos init
  T200_1.attach(escPin_1);
  setServoMicroseconds(T200_1, 0, MAX_IMU); // send "stop" signal to ESC.
  T200_2.attach(escPin_2);
  setServoMicroseconds(T200_2, 0, MAX_IMU); // send "stop" signal to ESC.
  T200_3.attach(escPin_3);
  setServoMicroseconds(T200_3, 0, MAX_IMU); // send "stop" signal to ESC.
  T200_4.attach(escPin_4);
  setServoMicroseconds(T200_4, 0, MAX_IMU); // send "stop" signal to ESC.
  T200_5.attach(escPin_5);
  setServoMicroseconds(T200_5, 0, MAX_IMU); // send "stop" signal to ESC.
  T200_6.attach(escPin_6);
  setServoMicroseconds(T200_6, 0, MAX_IMU); // send "stop" signal to ESC.

}
//function to calculate pitch and roll given IMU data
void imuRead() {
  uint8_t data[6];


  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Select data register
  Wire.write(0x3B);
  // Stop I2C transmission
  Wire.endTransmission();
  
  // Request 6 bytes of data
  Wire.requestFrom(Addr, 6);
  
  // Read 6 byte of data 
  if(Wire.available() == 6)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
    data[4] = Wire.read();
    data[5] = Wire.read(); 
  }
  
  // Convert the data
  float xAccl = float(data[0] * 256 + data[1] - 34)/2058.85;
  float yAccl = float(data[2] * 256 + data[3] + 67.93)/2059.3;
  float zAccl = float(data[4] * 256 + data[5] + 334.69)/2071.13;

  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Select data register 
  Wire.write(0x43);
  // Stop I2C transmission
  Wire.endTransmission();
  
  // Request 6 bytes of data
  Wire.requestFrom(Addr, 6);
  
  // Read 6 byte of data 
  if(Wire.available() == 6)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
    data[4] = Wire.read();
    data[5] = Wire.read(); 
  }
  // Convert the data
  float xGyro = data[0] * 256 + data[1] + 50;
  float yGyro = data[2] * 256 + data[3] - 28.5;
  float zGyro = data[4] * 256 + data[5] + 5.5;

  //save data to global arrays
  accData[0]={xAccl};
  accData[1]={yAccl};
  accData[2]={zAccl};
  gyroData[0]={xGyro};
  gyroData[1]={yGyro};
  gyroData[2]={zGyro};
}

void ComplementaryFilter()
{
    float pitchAcc, rollAcc;               
 
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    pitch += (gyroData[0] * GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
    roll -= (gyroData[1] * GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis
 
    // Compensate for drift with accelerometer data if !bullshit
    float forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
    if (forceMagnitudeApprox > 0.9 && forceMagnitudeApprox < 1.1)
    {
  // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f(accData[1], (float)accData[2]);
        pitch = pitch * 0.95 + pitchAcc * 0.05;
 
  // Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f(accData[0], (float)accData[2]);
        roll = roll * 0.95 + rollAcc * 0.05;
    }
}

//function to read joystick values from Serial
void joystickRead() {
  char dato;
  int segno=1, i=0, j=0;

  //if there isn't anything, go out. We can't wait
  if(!Serial.available())
    return;

  //reset joystickValue
  for(i=0; i<sizeof(joystickValue)/sizeof(joystickValue[0]); i++)
    joystickValue[i]=0;

  i=0;
  while(Serial.available() && (dato=Serial.read())==0); //discard "0"

  //reading loop. It has to read float numbers
  while(Serial.available() && dato!='\n') {
    if(dato=='-') {
      segno=-1;
      i--;
    }
    else if(dato==' ') {
      i=-1;
      joystickValue[j]=segno*joystickValue[j];
      segno=1;
      j++;
    }
    else if(dato=='.') {
      i--;
    }
    else {
        joystickValue[j]+=(float)(dato-'0')/pow(10,i);
    }
    i++;
    
    while(Serial.available()&&(dato=Serial.read())==0);
  }
}

//saturation value. If the value saturates MAX_VAL, it takes its value
int linSaturation(int value, int MAX_VAL) {
  int command;
  
  if(value>MAX_VAL)
    command = MAX_VAL;
  else if(value<-MAX_VAL)
    command = -MAX_VAL;
  else
    command=value;

  return command;
}

//set value to Servos using above function
void setServoMicroseconds(Servo s, int value, int MAX_VAL) {
  s.writeMicroseconds(STOP+linSaturation(value, MAX_VAL));
}
void SrtStp(float J, float j, int& s,int& f){
  if(J=0){ //press "start" -> start
    s=1; 
  }
  if(j=0){ //press "select" -> stop
    s=0;
    f=1; //we have to save current pressure for autoquote
  }
}

  void PressRS(float ppF, int& ftpF, float& rpF)
 {
  float cp = ppF; //read current pressure
  if(ftpF)
  { //if it is the first time after the START
    ftpF=0;
    rpF= cp; //save current pressure for autoquote
  }
 }
 void AlosaON(float& paF, float& raF, int kagF, float ptcF, float rollF, float jVF[], float AMF, int vrt[], float ptcAF, int MxIF, int& vlF, int& vlLF, int& vlRF, int hmlF, int& sngF, int pvSF, int pivLmF, float fpvSF, int VmlF, int& SvPrF, int& vlUDF, int rPF, int kDpF, int& MxJF)
{
    //values of pitch and roll angle, meant as the difference between the current angle and the ange we want to have

    //calculate actual pitch and roll angles, meant as the difference between
    //the two read from the IMU and the ones we want to reach from the joystick.
    //That difference is than multiplied by a multiplication constant to make it the power for servos
    paF  = kagF*(ptcF+jVF[14]*AMF); //(+ is because of the inversion of y-axis on the joystick)
    raF   = kagF*(rollF-jVF[15]*AMF);

    //adding above "powers" to the values for vertical-acting motors
    vrt[0]+=linSaturation(paF, MxIF);
    vrt[1]+=linSaturation(paF, MxIF);
    vrt[2]-=linSaturation(paF, MxIF);
    vrt[3]-=linSaturation(paF, MxIF);
    
    vrt[0]+=linSaturation(raF, MxIF);
    vrt[1]-=linSaturation(raF, MxIF);
    vrt[2]-=linSaturation(raF, MxIF);
    vrt[3]+=linSaturation(raF, MxIF);

  /* Movimento orizzontale */
    //signals from joystick
    vlF = vlLF = vlRF = -jVF[13]*hmlF; //values for front-back movement
        
    //check movement for rotation inversion
    if(-jVF[13] > 0)
    sngF=1;  //front
    else 
    sngF=-1;                       //back
    //values for "simple" rotation, with inversion when going back
    vlRF -= jVF[12]*hmlF*sngF;
    vlLF += jVF[12]*hmlF*sngF;
 
    /* Now follow the throttle and pivot calculations for a better turning */
    //throttle
    vlRF = vlRF*((float)vlF/((abs(jVF[12])+1)*hmlF))*sgF;
    vlLF = vlLF*((float)vlF/((abs(jVF[12])+1)*hmlF))*sgF;

    //pivot
   pvSF = jVF[12]*hmlF;
    if(abs(vlF)>pivLmF) fpvSF = 0;
    else fpvSF = 1.0 - (float)abs(vlF)/PIV_LIM;

    vlLF = (1.0-fpvSF)*vlLF + fpvSF*pvSF;
    vlRF = (1.0-fpvSF)*vlRF + fpvSF*(pvSF);
    
    //value for up-down movement
    if(jVF[2] || jVF[1]){ //controlled up-down from joystick
      SvPrF=1;   //we have to save pressure when finished
      vlUDF = (jVF[2]-jVF[1])*VmlF; //fixed value depending on buttons pressed
    }else if(SvPrF){
      rPF = cPF; //save pressure for autoquote
      SvPrF=0;
    }
    if(!SvPrF) //if we don't have to save it, make the ROV to reach requested quote
      vlUDF = (rPF-cPF)*kDpF;
        
    //front/back
    setServoMicroseconds(T200_4, vlRF, MxJF);
    setServoMicroseconds(T200_5, vlLF, MxJF);

    //up/down
    setServoMicroseconds(T200_1, vrt[0]+vlUDF, MxJF);
    setServoMicroseconds(T200_2, vrt[1]+vlUDF, MxJF);
    setServoMicroseconds(T200_3, vrt[2]+vlUDF, MxJF);
    setServoMicroseconds(T200_6, vrt[3]+vlUDF, MxJF);
  }
  }
