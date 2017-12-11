/*
 
 void AlosaON(float& paF, float& raF, int kagF, float ptcF, float rollF, float jVF[], float AMF, int vrt[], float ptcAF, int MxIF, int strF, int vlF, int vlLF, int vlRF, int hmlF, int& sngF,
  int pvSF, int pivLmF, float fpvSF, int VmlF, int& SvPrF, int& vlUDF, int rPF, int kDpF, int MxJF){
  /*Pitch and Roll*/
if(strF==1){ 
  //if the ROV is started
  
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



/----//----//----//----//----//----//----//----//----//----//----//----//----//----//----//----/
void setup(){
 
  Serial.begin(9600);
}
int cont;
float T=0.3;
void loop(){
  
  TempCheck(T , cont);
  }
void TempCheck(float& tF, int& CntF){
//print after "cont" times of timer timeout
  if(cont>=tF){
    cont=1; //reset counter
    temperature.read(); //say to the sensor to read temperature
   digitalWrite(RE_n_enable, HIGH); //set enable pin to HIGH
 /*//   Serial.println(temperature.temperature()); //print temperature
    Serial.print(curPress, 0); //print current pressure
    Serial.print(" ");
    Serial.print(reqPress, 0); //print requested pressure
    Serial.print(" ");
    Serial.println(valUD); //print up-down power for UD motors
   */
    Serial.println(tF); //print counter limit
    delay(13);          //delay to send it to the serial. Increase it if we have more data to send
    digitalWrite(RE_n_enable, LOW); //set enable pin to LOW
  }else
    cont++;

    T=tF;
    cont=CntF;
}





  
