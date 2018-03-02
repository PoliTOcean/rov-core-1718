#include <RBD_Timer.h>
#include "TSYS01.h"
#include "MS5837.h"
#include <Wire.h>
#include <Servo.h>

//timer variable. It tells us when a certain timeout is expired. We need it for the IMU
RBD::Timer timer;
//sensors
TSYS01 temperature;
MS5837 pressure;

#define RE_n_enable 3 // serial communication enable pin
#define Addr 0x69     // IMU address
#define GYROSCOPE_SENSITIVITY PI/1900.0
#define dt 0.02       // 20 ms sample rate
#define kAng 0        // P control constant (800)
#define STOP 1500     // frequency for a still rotor
#define MAX_JOY 150   //300   // max speed from joystick command
#define MAX_IMU 50    // max speed from imu command
#define H_MUL 300     // front-back movement
#define V_MUL 200     // up-down m.ovement
#define ANG_MUL PI/4  // roll, pitch from joystick
#define PIV_LIM H_MUL/2    // pivot limit
#define T 2/dt        //counter limit for printing (see below, last part of loop)
#define kDep 50       //multiplication constant for autoquote

//esc servos pins
byte escPin_1 = 10;
byte escPin_2 = 9;
byte escPin_3 = 8;
byte escPin_4 = 7;
byte escPin_5 = 6;
byte escPin_6 = 5;

//Servos declaration
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

float roll, pitch;   // pitch, roll
//variables for pitch and roll calculation given IMU data
float accData[3];       
float gyroData[3];

int start; //start variable
int vertical[4];  //array for vertical motors values
float joystickValue[20];  //array for joystick values (see KEY MAP below)

int cont; //counter for printing

// sensors values variables
int savePressure, firstTimePres=1;
float reqPress=0.0, curPress=0.0;

/* KEY MAP
 *  0  -> select / tasto PS         left button (square)
 *  1  -> L3                        up button   (triangle)
 *  2  -> R3                        down button (X)
 *  3  -> start                     right button (O)
 *  4  -> up                        L1
 *  5  -> right                     L2
 *  6  -> down                      R1
 *  7  -> left                      R2
 *  8  -> L2                        left central button (select)
 *  9  -> R2                        right central button (start)
 *  10 -> L1                        L3
 *  11 -> R1                        R3
 *  12 -> triangle                  rx
 *  13 -> O                         ry
 *  14 -> X                         ly  up/down
 *  15 -> square                    lx  left/right
 *  16 -> ry (positiva basso)       FREE
 *  17 -> rx (positiva dx)          FREE
 *  18 -> ly (positiva basso)       FREE
 *  19 -> lx (positiva dx?)         FREE
 *  20 -> FREE                      FREE
 */

void setup(){ //setup => leave it here
  Serial.begin(9600); //Serial init
  
  pinMode(RE_n_enable , OUTPUT); //enable PIN init
  
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

  //Sensors init
  temperature.init();
  pressure.init();
  
  delay(1000); //delay to make actions complete
  digitalWrite(RE_n_enable, LOW); //setting LOW. We don't have to communicate

  timer.setTimeout(dt*1000);   //set timer timeout. dt*1000 => from seconds to milliseconds
  timer.restart();             //start the timer
}

//function to read IMU data
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

//function to calculate pitch and roll given IMU data
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

//loop function. Leave it here!
void loop() {
  int val, valUD=0, sign=1, valR=0, valL=0, pivSpeed=0;
  float fPivScale=0.0, pitchAngle=0.0, rollAngle=0.0;
  
  if(timer.isExpired()) { //if it is time to do something
  timer.restart(); //restart timer
  
  imuRead(); //read from IMU
  ComplementaryFilter(); //calculate pitch and roll
  pressure.read(); //say to the sensor to read pressure

  //read joystick values
  joystickRead();

  //reset vertical
  for(int i=0;i<4;i++)
    vertical[i]=0;
  if(joystickValue[9]!=0){ //press "start" -> start
    start=1; 
  }
  if(joystickValue[8]!=0){ //press "select" -> stop
    start=0;
    firstTimePres=1; //we have to save current pressure for autoquote
  }

  curPress = pressure.pressure(); //read current pressure
  if(firstTimePres){ //if it is the first time after the START
    firstTimePres=0;
    reqPress = curPress; //save current pressure for autoquote
  }

  if(start==1){ //if the ROV is started
    /* Pitch and Roll */
  
    //values of pitch and roll angle, meant as the difference between the current angle and the ange we want to have

    //calculate actual pitch and roll angles, meant as the difference between
    //the two read from the IMU and the ones we want to reach from the joystick.
    //That difference is than multiplied by a multiplication constant to make it the power for servos
    pitchAngle  = kAng*(pitch+joystickValue[14]*ANG_MUL); //(+ is because of the inversion of y-axis on the joystick)
    rollAngle   = kAng*(roll-joystickValue[15]*ANG_MUL);

    //adding above "powers" to the values for vertical-acting motors
    vertical[0]+=linSaturation(pitchAngle, MAX_IMU);
    vertical[1]+=linSaturation(pitchAngle, MAX_IMU);
    vertical[2]-=linSaturation(pitchAngle, MAX_IMU);
    vertical[3]-=linSaturation(pitchAngle, MAX_IMU);
    
    vertical[0]+=linSaturation(rollAngle, MAX_IMU);
    vertical[1]-=linSaturation(rollAngle, MAX_IMU);
    vertical[2]-=linSaturation(rollAngle, MAX_IMU);
    vertical[3]+=linSaturation(rollAngle, MAX_IMU);

    //signals from joystick
    val = valL = valR = -joystickValue[13]*H_MUL; //values for front-back movement
        
    //check movement for rotation inversion
    if(-joystickValue[13] > 0) sign=1;  //front
    else sign=-1;                       //back
    
    //values for "simple" rotation, with inversion when going back
    valR -= joystickValue[12]*H_MUL*sign;
    valL += joystickValue[12]*H_MUL*sign;
 
    /* Now follow the throttle and pivot calculations for a better turning */
    //throttle
    valR = valR*((float)val/((abs(joystickValue[12])+1)*H_MUL))*sign;
    valL = valL*((float)val/((abs(joystickValue[12])+1)*H_MUL))*sign;

    //pivot
    pivSpeed = joystickValue[12]*H_MUL;
    if(abs(val)>PIV_LIM) fPivScale = 0;
    else fPivScale = 1.0 - (float)abs(val)/PIV_LIM;

    valL = (1.0-fPivScale)*valL + fPivScale*pivSpeed;
    valR = (1.0-fPivScale)*valR + fPivScale*(-pivSpeed);
    
    //value for up-down movement
    if(joystickValue[2] || joystickValue[1]){ //controlled up-down from joystick
      savePressure=1;   //we have to save pressure when finished
      valUD = (joystickValue[2]-joystickValue[1])*V_MUL; //fixed value depending on buttons pressed
    }else if(savePressure){
      reqPress = curPress; //save pressure for autoquote
      savePressure=0;
    }
    if(!savePressure) //if we don't have to save it, make the ROV to reach requested quote
      valUD = (reqPress-curPress)*kDep;
        
    //front/back
    setServoMicroseconds(T200_4, valR, MAX_JOY);
    setServoMicroseconds(T200_5, valL, MAX_JOY);

    //up/down
    setServoMicroseconds(T200_1, vertical[0]+valUD, MAX_JOY);
    setServoMicroseconds(T200_2, vertical[1]+valUD, MAX_JOY);
    setServoMicroseconds(T200_3, vertical[2]+valUD, MAX_JOY);
    setServoMicroseconds(T200_6, vertical[3]+valUD, MAX_JOY);
  }
  else {
    setServoMicroseconds(T200_1, 0, MAX_IMU); // send "stop" signal to ESC.
    setServoMicroseconds(T200_2, 0, MAX_IMU); // send "stop" signal to ESC.
    setServoMicroseconds(T200_3, 0, MAX_IMU); // send "stop" signal to ESC.
    setServoMicroseconds(T200_4, 0, MAX_IMU); // send "stop" signal to ESC.
    setServoMicroseconds(T200_5, 0, MAX_IMU); // send "stop" signal to ESC.
    setServoMicroseconds(T200_6, 0, MAX_IMU); // send "stop" signal to ESC.
  }

  //print after "cont" times of timer timeout
  if(cont>=T){
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
    Serial.println(T); //print counter limit
    delay(13);          //delay to send it to the serial. Increase it if we have more data to send
    digitalWrite(RE_n_enable, LOW); //set enable pin to LOW
  }else
    cont++;
  }
}
