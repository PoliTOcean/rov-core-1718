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
  Wireinit();
  Servoinit();
  //Sensors init
  temperature.init();
  pressure.init();
  delay(1000); //delay to make actions complete
  digitalWrite(RE_n_enable, LOW); //setting LOW. We don't have to communicate
  timer.setTimeout(dt*1000);   //set timer timeout. dt*1000 => from seconds to milliseconds
  timer.restart();             //start the timer
}


//loop function. Leave it here!
void loop() {
  
//Variables Reset
  
  int val, valUD=0, sign=1, valR=0, valL=0, pivSpeed=0;
  float fPivScale=0.0, pitchAngle=0.0, rollAngle=0.0;
  
//----//----//----//----//----//----//----//----//----//----/
  
//if it is time to do something
  
  if(timer.isExpired()) {
  timer.restart(); //restart timer
//----//----//----//----//----//----//----//----//----//----/

//read from IMU

  imuRead();
  ComplementaryFilter(); //calculate pitch and roll
  pressure.read(); //say to the sensor to read pressure
  
//----//----//----//----//----//----//----//----//----//----/

//read joystick values

  joystickRead();
  
//----//----//----//----//----//----//----//----//----//----/

//reset vertical:

  for(int i=0;i<4;i++)
    vertical[i]=0;
    
//----//----//----//----//----//----//----//----//----//----/

//Start and Select Reading:

SrtStp(joystickValue[9],joystickValue[8], start, firstTimePres);


//----//----//----//----//----//----//----//----//----//----/

//Pressure Reading and Saving

PressRS(pressure.pressure(), firstTimePres, 0.0);
firstTimePres=ftpF;
reqPress=rpF;

//----//----//----//----//----//----//----//----//----//----/

if(start==1){
  
//If Rov is on

void AlosaON(pitchAngle , rollAngle, kAng, pitch , roll, joystickValue[], ANG_MUL, vertical[], MAX_IMU, val, valL, valR, H_MUL, sign, pivSpeed, PIV_LIM, fPivScale, savePressure, valUD, reqPress, curPress, kDep, MAX_JOY);
    vertical[]=vrt[];
    MAX_JOY=MxJF;
    valUD=vlUDF;
    valL=vlLF;
    valR=vlRF;
    setServoMicroseconds(T200_4, valR, MAX_JOY);
    setServoMicroseconds(T200_5, valL, MAX_JOY);
    //up/down
    setServoMicroseconds(T200_1, vertical[0]+valUD, MAX_JOY);
    setServoMicroseconds(T200_2, vertical[1]+valUD, MAX_JOY);
    setServoMicroseconds(T200_3, vertical[2]+valUD, MAX_JOY);
    setServoMicroseconds(T200_6, vertical[3]+valUD, MAX_JOY);
    
  //----//----//----//----//----//----//----//----//----//----/
  
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
    Serial.println(T); //print counter limit
    delay(13);          //delay to send it to the serial. Increase it if we have more data to send
    digitalWrite(RE_n_enable, LOW); //set enable pin to LOW
  }else
    cont++;
  }
}
