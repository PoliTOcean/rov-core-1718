#include <RBD_Timer.h>  //library for custom timer
#include "MS5837.h"     //library for pressure sensor
#include <Servo.h>      //library for servos

//uncomment following row to test in water. It enables autoquote and autostabilization (see below how)
#define TEST 1

/*Important values initialization*/
#define IMU_ADDR 0x68         // IMU IMU_ADDRess
#define STOP 1500             // frequency for a still rotor
#define dt 0.01               // IMU timer timeout in seconds  (10ms)

//esc servos pins
#define escPin_1 2
#define escPin_2 3
#define escPin_3 4
#define escPin_4 5
#define escPin_5 6
#define escPin_6 7
#define escPin_7 8
#define escPin_8 9

/*Multiplicative constants*/
#define K_ANG 200             // P control constant
#define H_MUL 200             // front-back movement
#define V_MUL 100             // up-down movement
#define FAST_V 70             // up-down turbo
#define K_DEP 50              // autoquote

/*Saturation contanst*/
#define MAX_IMU 80            // max speed from imu command (it will be counted twice: once for roll and once for pitch)
#define MAX_UD  300           // max speed for Up/Down movement
#define MAX_SRV 400           // max speed of ESC servos
#define MAX_TEMP 45           // max functioning temperature

/*Global variables definition*/
RBD::Timer timer;         //needed for the IMU reading process: it tells us when a certain timeout is expired 
//sensors variables
MS5837 pressure;
int start;  //start-stop variable
float reqPress, curPress, curTemp, pitch, roll;    //sensors values

//setup function
void setup(){
  initI2C();              //IMU communication protocol initialization
  
  //set the ESC Servos pins in the right way
  initEscServos(escPin_1, escPin_2, escPin_3, escPin_4, escPin_5, escPin_6, escPin_7, escPin_8);
  
  //sensors initialization
  pressure.init();

  pinMode(MISO, OUTPUT);
  
  delay(200);    //delay of 0.2 second to make actions complete
  
  //set the IMU timer. dt*1000 => seconds -> milliseconds
  timer.setTimeout(dt*1000);
  timer.restart();

  // turn on SPI interrupt
  cli();
  SPCR = 0b11000000;
  sei();
}

//loop function
void loop(){
  //array for vertical motors values
  static int vertical[4] = {0, 0, 0, 0};
  //values of horizontal movement
  static int valLF, valRF, valLB, valRB;

 /* if(curTemp>=MAX_TEMP) //safe check on temperature. We hope we'll never need that
    stopRov();*/
    
  if(isTime()) {                            //if the IMU timer is expired
    dataRead();     // read angles, pressure, temperature and prepare it for spi
#ifdef TEST                                     //if we are in water
    evaluateVertical(K_ANG, K_DEP, vertical);   //then evaluate vertical values for autostabilization and autoquote
#else                                           //else, if we aren't,
    evaluateVertical(0, 0, vertical);           //then evaluate them just for joystick up/down
#endif
  }
  
  if(start){                            //if the ROV is started
    evaluateHorizontal(&valLF, &valRF, &valLB, &valRB);           //evaluate values for horizontal movement

    //set new motors powers
    setServosValues(valLF, valRF, valLB, valRB, vertical[0], vertical[1], vertical[2], vertical[3], MAX_SRV);
  }else                                 //else, if the ROV is stopped,
    setServosValues(0, 0, 0, 0, 0, 0, 0, 0, 0);       //then send STOP signal to motors
}
