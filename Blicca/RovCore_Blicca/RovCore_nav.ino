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
int up, down, fastV, rx, ry, ly, lx;

// variables needed by SPI code
volatile char buf[4];
volatile char ver[3]; //bit check (1 0 1)
volatile byte pos_spi = 1, ref1, op;
volatile float y, x, rz;
volatile bool trigger2, trigger, pinkie, cmdstart, cmdstop;

// SPI interrupt routine
ISR (SPI_STC_vect)
{
  byte c = SPDR;

 if (pos_spi < sizeof buf)
    {
    SPDR = buf [pos_spi];
    }
 switch (pos_spi) 
 {

 case 0:
 y = float(c-127)/127;
 break;

 case 1:
 x = float(c-127)/127;
 break;

 case 2:
 rz = float(c-127)/127;
 break;

 case 3:
 ref1 = c;
 
 ver[0] = bitRead(c,7);
 ver[1] = bitRead(c,6);
 ver[2] = bitRead(c,5);
 cmdstop = bitRead(c,4);
 cmdstart = bitRead(c,3);
 pinkie = bitRead(c,2);
 trigger = bitRead(c,1);
 trigger2 = bitRead(c,0);

 if(cmdstart)
  startRov();
 if(cmdstop)
  stopRov();

 if(trigger)
 {
  up = 1;
  down = 0;
  fastV = 0;
 }
 else if(trigger2)
 {
  up = 1;
  down = 0;
  fastV = 1;
 }
 else if(pinkie)
 {
  up = 0;
  down = 1;
  fastV = 0;
 }
 
 if (ver[0] ==1 && ver[1]==0 && ver[2]==1)
 {
  ;
 }
 break;

 }

 pos_spi++;
 if ( pos_spi >= sizeof (buf))
 pos_spi=0;
  
}  // end of interrupt service routine (ISR) SPI_STC_vect

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
  valUD=0;            //reset valUD
  if(down || up){         //controlled up-down from joystick
    savePressure=1;                           //it has to save pressure when finished
    valUD = -(up-down)*(V_MUL+fastV*FAST_V);  //fixed value depending on buttons pressed
  }else if(savePressure) //else, if it is not (still) pressing up/down buttons
    saveRequestedPressure(); //save pressure for autoquote
  
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
  
  //take the front/back value from joystick
  val = valL = valR = -ry*H_MUL;
      
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

