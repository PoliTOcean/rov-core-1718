// global variables
int savePressure; //flag to know if we have to save pressure or not
int valUD;        //value for Up and Down motors powers

//Servos variables
Servo T200_1;
Servo T200_2;
Servo T200_3;
Servo T200_4;
Servo T200_5;
Servo T200_6;
/* SERVO MAP
 *  
 *    FRONT
 *    6   3     -> up/down
 *    5   4     -> front/back
 *    1   2     -> up/down
 *    BACK
 *  
 */

//variables for joystick values
int up, down, fastH, fastV, rx, ry, ly, lx;
char cmd[3];
int pos, sign, cmdFlag, begRead;
/* COMMANDS MAP
 *  
 *  CMD   VALUES      DESCRIPTION             JOYSTICK BUTTON
 *  ---------------------------------------------------------
 *  /     -           Begin/end transmission  -
 *  U     -           Up                      Triangle
 *  D     -           Down                    X
 *  V+    -           Fast VERTICAL           L2
 *  V-    -           Slow VERTICAL           L1
 *  H+    -           Fast HORIZONTAL         R2
 *  H-    -           Slow HORIZONTAL         R1
 *  RX    000-100     Right analog x-axis     Rx
 *  RY    000-100     Right analog y-axis     Ry
 *  LY    000-100     Left analog y-axis      Ly
 *  LX    000-100     Left analog x-axis      Lx
 *  G     -           Go: start the ROV       Start
 *  S     -           STOP the ROV            Select
 *  C     -           Calibrate sensors       -
 *  W     -           Is the ROV awake?       -
 */
void joystickRead(){ //function to read from joystick, when it's available, one byte a time
  char data=0; //set data to 0
  
  if(Serial.available())  //if there is something to read
    data = Serial.read();   //read the character

  if(data==0){ //if data is still 0, go out
    if(safeTimer.isExpired()){ //if we haven't received any command for too long, stop ROV
      stopRov();
      safeTimer.restart();
    }
    return;
  }
  safeTimer.restart();


  if(data=='/'){                            //if it is the start one
    up=down=fastH=fastV=pos=rx=ry=ly=lx=0;  //then reset variables
    sign=cmdFlag=1;
    begRead=1;    //change flag state
    return;
  }else if(data=='\\')                      //else, if it is the end one,
    begRead=0;                              //then reset begRead
  
  if(!begRead)  //if I read something, but I'm not reading,
    return;     //then go out
    
  if(cmdFlag){            //if I'm still reading the command
    cmd[pos] = data;        //save it
    pos++;                  //increment pos
    if(pos>=2)              //if I've certainly finished (there is no command longer than 2)
      cmdFlag=pos=cmd[2]=0;   //reset position and cmdFlag, in order to read following values
    else                    //else, if I'm on the first character
      switch(cmd[0]){         //let's see which command is (see COMMAND MAP above)
        case 'S': //stop
          stopRov();
          pos=0;
          break;
        case 'G': //go
          startRov();
          pos=0;
          break;
        case 'U': //up
          up=1;
          pos=0;
          break;
        case 'D': //down
          down=1;
          pos=0;
          break;
        case 'H': //H+/H-
          cmdFlag=pos=cmd[2]=0;
          break;
        case 'V': //V+/V-
          cmdFlag=pos=cmd[2]=0;
          break;
        case 'R': //right x/y
          break;
        case 'L': //left x/y
          break;
        default:
          pos=0;  //reset pos=0, in order to read the following command
          break;
      }
      
    return;
  }else{                //else, if I'm reading the value
    if(pos==0 && (data=='-'||data=='+'||data==' ')){
      if(data=='-')  //check for the sign
        sign=-1;
      if(!strcmp(cmd, "H")){
        fastH=sign;
        cmdFlag=1;  //we've finished here
      }else if(!strcmp(cmd, "V")){
        fastV=sign;
        cmdFlag=1;
      }
      return;
    }
    if('0'<=data&&data<='9'){  //if it is a digit character
      data-='0';  //convert to an integer
    
      //check commands
      if(!strcmp(cmd, "RX"))
        rx = 10*rx + sign*data; //decimal conversion => new_x = old_x*10 + new_digit;
      else if(!strcmp(cmd, "RY"))
        ry = 10*ry + sign*data;
      else if(!strcmp(cmd, "LY"))
        ly = 10*ly + sign*data;
      else if(!strcmp(cmd, "LX"))
        lx = 10*lx + sign*data;
      else                     //else, if it isn't any known command,
        pos=3;                  //then  reach the finish condition
    }else      //else, if it isn't a digit character, either for some error or for ' ' character,
      pos=3;   //then  reach the finish condition
    
    if(pos>=3){ //if it finished
      cmdFlag=1;    //reset command flag to 1
      pos=0;        //reset position
    }else       //else, if it hasn't finished yet,
      pos++;        //then increment pos
    
  }
}

//function for value saturation. If value is in modulus bigger than MAX_VAL, it takes its value
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

//function to set power (int value) to Servos, using the above function linSaturation(int,int)
void setServoMicroseconds(Servo s, int value, int MAX_VAL) {
  s.writeMicroseconds(STOP + linSaturation(value, MAX_VAL));
}

//function to set right pins to the ESC Servos
void initEscServos(byte pin1, byte pin2, byte pin3, byte pin4, byte pin5, byte pin6){
  T200_1.attach(pin1);
  setServoMicroseconds(T200_1, 0, MAX_IMU); // send "stop" signal to ESC.
  T200_2.attach(pin2);
  setServoMicroseconds(T200_2, 0, MAX_IMU); // send "stop" signal to ESC.
  T200_3.attach(pin3);
  setServoMicroseconds(T200_3, 0, MAX_IMU); // send "stop" signal to ESC.
  T200_4.attach(pin4);
  setServoMicroseconds(T200_4, 0, MAX_IMU); // send "stop" signal to ESC.
  T200_5.attach(pin5);
  setServoMicroseconds(T200_5, 0, MAX_IMU); // send "stop" signal to ESC.
  T200_6.attach(pin6);
  setServoMicroseconds(T200_6, 0, MAX_IMU); // send "stop" signal to ESC.
}

//function for pitch power calculation
float calcPitchPower(float kAng){
  /* it takes the difference between current pitch and the requested one from the joystick
   * and multiplicates it for a multiplication constant, passed as parameter */
  return kAng*(pitch+ly*ANG_MUL); //(+ is because of the inversion of y-axis on the joystick)
}

//function for roll power calculation. Same as above, without sign inversion
float calcRollPower(float kAng){
  return kAng*(roll-lx*ANG_MUL);
}

//function to evaluate vertical motors values
void evaluateVertical(float kAng, float kDep, int vertical[4]){
  float pitchPower, rollPower;
  //call above functions for calculations
  pitchPower = calcPitchPower(kAng);
  rollPower = calcRollPower(kAng);

  //set values for pitch
  vertical[0] =  linSaturation(pitchPower, MAX_IMU);
  vertical[1] =  linSaturation(pitchPower, MAX_IMU);
  vertical[2] = -linSaturation(pitchPower, MAX_IMU);
  vertical[3] = -linSaturation(pitchPower, MAX_IMU);

  //adding values for roll
  vertical[0] += linSaturation(rollPower, MAX_IMU);
  vertical[1] -= linSaturation(rollPower, MAX_IMU);
  vertical[2] -= linSaturation(rollPower, MAX_IMU);
  vertical[3] += linSaturation(rollPower, MAX_IMU);

  //value for up-down movement
  if(!begRead){     //if it is not reading, can change valUD based on joystick value
    valUD=0;            //reset valUD
    if(down || up){         //controlled up-down from joystick
      savePressure=1;                           //it has to save pressure when finished
      valUD = -(up-down)*(V_MUL+fastV*FAST_V);  //fixed value depending on buttons pressed
    }else if(savePressure) //else, if it is not (still) pressing up/down buttons
      saveRequestedPressure(); //save pressure for autoquote
  }
  
  if(!savePressure) //change value for autoquote
    valUD = (reqPress-curPress)*kDep;

  //adding values for UD movement/autoquote
  vertical[0] += linSaturation(valUD, MAX_UD);
  vertical[1] += linSaturation(valUD, MAX_UD);
  vertical[2] += linSaturation(valUD, MAX_UD);
  vertical[3] += linSaturation(valUD, MAX_UD);
}

/* function to evaluate powers for horizontal movement.
 * leftValue and rightValue are the powers of horizontal servos */
void evaluateHorizontal(int *leftValue, int *rightValue){
  int val, sign=1, valR, valL, pivSpeed;
  float fPivScale=0.0;

  if(begRead) //if it's reading from the serial, don't change anything
    return;   //and return
  
  //take the front/back value from joystick
  val = valL = valR = -ry*(H_MUL+fastH*FAST_H);
      
  //check movement for rotation inversion
  if(-ry > 0) sign=1;  //front
  else sign=-1;        //back (so invert when turning)
  
  //values for "simple" rotation
  valR -= rx*H_MUL*sign;
  valL += rx*H_MUL*sign;

  /* Now follow throttle and pivot calculations for a more clever turning algorithm. */
  //throttle
  valR = valR*((float)val/((abs(rx)+100)*H_MUL))*sign;
  valL = valL*((float)val/((abs(rx)+100)*H_MUL))*sign;
  //pivot
  pivSpeed = rx*H_MUL;
  if(abs(val)<=PIV_LIM)
    fPivScale = 1.0 - (float)abs(val)/PIV_LIM;

  //save final values
  *leftValue  = (1.0-fPivScale)*valL + fPivScale*pivSpeed;
  *rightValue = (1.0-fPivScale)*valR + fPivScale*(-pivSpeed);
}

//function to set all servos values
void setServosValues(int valL, int valR, int v0, int v1, int v2, int v3, int MAX_VALUE){
  //front/back
  setServoMicroseconds(T200_4, valR, MAX_VALUE);
  setServoMicroseconds(T200_5, valL, MAX_VALUE);

  //up/down
  setServoMicroseconds(T200_1, v0, MAX_VALUE);
  setServoMicroseconds(T200_2, v1, MAX_VALUE);
  setServoMicroseconds(T200_3, v2, MAX_VALUE);
  setServoMicroseconds(T200_6, v3, MAX_VALUE);
}

