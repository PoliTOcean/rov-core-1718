#include <RBD_Timer.h>  //library for custom timer
#include "TSYS01.h"     //library for temperature sensor
#include "MS5837.h"     //library for pressure sensor
#include <Servo.h>      //library for servos

//uncomment following row to test in water. It enables autoquote and autostabilization (see below how)
#define TEST 1

/*Important values initialization*/
#define RE_n_enable 3         // serial communication enable pin
#define IMU_ADDR 0x69         // IMU IMU_ADDRess
#define STOP 1500             // frequency for a still rotor
#define GYRO_SENS PI/1400.0   // gyroscope sensitivity
#define dt 0.01               // IMU timer timeout in seconds  (10ms)
#define dST 2                 // serial timer timeout in seconds  (2")
#define safedT 30              // safe timer timeout in seconds (3")   ->  consider this if we want to be able \
                                                                          to command it using command line

//esc servos pins
#define escPin_1 10
#define escPin_2 9
#define escPin_3 8
#define escPin_4 7
#define escPin_5 6
#define escPin_6 5

/*Multiplicative constants*/
#define K_ANG 1200            // P control constant
#define H_MUL 2               // front-back movement
#define V_MUL 200             // up-down movement
#define FAST_V 70             // up-down turbo
#define ANG_MUL PI/400        // roll, pitch from joystick
#define K_DEP 50              // autoquote

/*Saturation contanst*/
#define MAX_IMU 80            // max speed from imu command (it will be counted twice: once for roll and once for pitch)
#define MAX_UD  300           // max speed for Up/Down movement
#define MAX_SRV 400           // max speed of ESC servos
#define MAX_TEMP 45           // max functioning temperature
#define PIV_LIM H_MUL/2       // pivot limit

/*Global variables definition*/
RBD::Timer timer, safeTimer;         //needed for the IMU reading process: it tells us when a certain timeout is expired 
//sensors variables
MS5837 pressure;
TSYS01 temperature;
int start;                            //ROV start/stop flag
int valR, valL;                       //values of horizontal movement
float reqPress, curPress, curTemp, pitch, roll;    //sensors values

//setup function
void setup(){
  serialInit(dST*1000);   //serial initialization (timeout passed as parameter)

  initI2C();              //IMU communication protocol initialization
  
  //set the ESC Servos pins in the right way
  initEscServos(escPin_1, escPin_2, escPin_3, escPin_4, escPin_5, escPin_6);
  
  //sensors initialization
  temperature.init();
  pressure.init();
  
  delay(1000);    //delay of 1 second to make actions complete
  
  //set the IMU timer. dt*1000 => seconds -> milliseconds
  timer.setTimeout(dt*1000);
  timer.restart();

  safeTimer.setTimeout(safedT*1000);
  safeTimer.restart();

  while(!Serial.available());    //wait for first command from Serial
}

//loop function
void loop(){
  //array for vertical motors values
  int vertical[4] = {0, 0, 0, 0};

 /* if(curTemp>=MAX_TEMP) //safe check on temperature. We hope we'll never need that
    stopRov();*/
  
  joystickRead();       //read data from joystick, if available
  
  if(isTime()) {                            //if the IMU timer is expired
    imuRead();                              //read IMU data
    curPress = readPress();                 //read pressure
    curTemp =  readTemp();                  //read temperature
    //print over Serial (function internally checks if it is time to do that)
    printOverSerial(0, 13, "%d %d %d %d\n", curPress, curTemp, pitch, roll);
  }
  
  if(start){                            //if the ROV is started
#ifdef TEST                                     //if we are in water
    evaluateVertical(K_ANG, K_DEP, vertical);   //then evaluate vertical values for autostabilization and autoquote
#else                                           //else, if we aren't,
    evaluateVertical(0, 0, vertical);           //then evaluate them just for joystick up/down
#endif

    evaluateHorizontal(&valL, &valR);           //evaluate values for horizontal movement

    //set new motors powers
    setServosValues(valL, valR, vertical[0], vertical[1], vertical[2], vertical[3], MAX_SRV);
    printOverSerial(1, 10, "L: %d\tR: %d\tV1: %d\tV2: %d\tV3: %d\tV4: %d\n", valL, valR, vertical[0], vertical[1], vertical[2], vertical[3]);
  }else                                 //else, if the ROV is stopped,
    setServosValues(0, 0, 0, 0, 0, 0, 0);       //then send STOP signal to motors
}
