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
Servo T200_7;
Servo T200_8;
/* SERVO MAP
 *  
 *    FRONT
 *    7   4     -> up/down
 *    5   2     -> front/back
 *    3   6     -> front/back
 *    1   8     -> up/down
 *    BACK
 *  
 */

//variables for joystick values
int up, down, fastV;

// variables needed by SPI code
volatile char buf[4];
volatile byte pos_spi = 1, op, ref1;
volatile float y, x, rz;
volatile bool trigger2, trigger, pinkie, cmdstart, cmdstop;

// SPI interrupt routine
ISR (SPI_STC_vect)
{
  byte c = SPDR;
  volatile char ver[3]; //bit check (1 0 1)

 if (pos_spi < sizeof buf)
    {
    SPDR = buf [pos_spi];
    }
 switch (pos_spi) 
 {

 case 1:
 y = float(c-127)/127;
 break;

 case 2:
 x = float(c-127)/127;
 break;

 case 3:
 rz = float(c-127)/127;
 break;

 case 0:
 ref1 = c;
 ver[0] = bitRead(ref1,7);
 ver[1] = bitRead(ref1,6);
 ver[2] = bitRead(ref1,5);
 
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
void initEscServos(byte pin1, byte pin2, byte pin3, byte pin4, byte pin5, byte pin6,  byte pin7, byte pin8){
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
  T200_7.attach(pin7);
  setServoMicroseconds(T200_7, 0, MAX_IMU); // send "stop" signal to ESC.
  T200_8.attach(pin8);
  setServoMicroseconds(T200_8, 0, MAX_IMU); // send "stop" signal to ESC.
}

//function for pitch power calculation
float calcPitchPower(float kAng){
  /* it takes the difference between current pitch and the requested one from the joystick
   * and multiplicates it for a multiplication constant, passed as parameter */
  return kAng*(pitch); //(+ is because of the inversion of y-axis on the joystick)
}

//function for roll power calculation. Same as above, without sign inversion
float calcRollPower(float kAng){
  return kAng*(roll);
}

//function to evaluate vertical motors values
void evaluateVertical(float kAng, float kDep, int vertical[4]){
 if(trigger)
 {
  up = 1;
  down = 0;
  fastV = 0;
 }
 if(trigger2)
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
 else if(!trigger && !trigger2 && !pinkie)
 {
  up = 0;
  down = 0;
  fastV = 0;
 }
 
  float pitchPower, rollPower;
  //call above functions for calculations
  pitchPower = calcPitchPower(kAng);
  rollPower = calcRollPower(kAng);

  //set values for pitch
  vertical[0] =  -linSaturation(pitchPower, MAX_IMU);
  vertical[1] =  -linSaturation(pitchPower, MAX_IMU);
  vertical[2] = linSaturation(pitchPower, MAX_IMU);
  vertical[3] = linSaturation(pitchPower, MAX_IMU);

  //adding values for roll
  vertical[0] -= linSaturation(rollPower, MAX_IMU);
  vertical[1] += linSaturation(rollPower, MAX_IMU);
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

/* function to evaluate powers for horizontal movement.*/
void evaluateHorizontal(int *leftFront,int  *rightFront,int  *leftBack,int  *rightBack) {
  { // I puntatori si riferiscono ai motori
    int signLF = -1; int signRF = 1; int signLB = -1; int signRB = 1;
    *leftFront = H_MUL * signLF * (-y+x+rz);
    *rightFront = H_MUL * signRF* (-y-x-rz);
    *leftBack = H_MUL * signLB * (-y-x+rz);
    *rightBack = H_MUL * signRB * (-y+x-rz);
  }
}

//function to set all servos values
void setServosValues(int valLF, int valRF, int valLB, int valRB, int v0, int v1, int v2, int v3, int MAX_VALUE){
  //front/back
  setServoMicroseconds(T200_5, valLF, MAX_VALUE);
  setServoMicroseconds(T200_2, valRF, MAX_VALUE);
  setServoMicroseconds(T200_3, -valLB, MAX_VALUE);
  setServoMicroseconds(T200_6, -valRB, MAX_VALUE);

  //up/down
  setServoMicroseconds(T200_7, -v0, MAX_VALUE);
  setServoMicroseconds(T200_4, v1, MAX_VALUE);
  setServoMicroseconds(T200_1, v2, MAX_VALUE);
  setServoMicroseconds(T200_8, -v3, MAX_VALUE);
}

