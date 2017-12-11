#include <RBD_Timer.h>
#include <Wire.h>
#include <Servo.h>

RBD::Timer timer;

#define RE_n_enable 3 // serial communication enable pin
#define Addr 0x69     // IMU address
#define GYROSCOPE_SENSITIVITY PI/1900.0
#define dt 0.05       // 10 ms sample rate!
int kAng=800, tempKAng;         // P control constant
#define STOP 1500     // frequency for a still rotor
#define MAX_IMU 150    // max speed from imu command

byte escPin_1 = 10;
byte escPin_2 = 9;
byte escPin_3 = 8;
byte escPin_4 = 7;
byte escPin_5 = 6;
byte escPin_6 = 5;

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

float angles[]={0,0};   // pich, roll
float accData[3];
float gyroData[3];
int start=0;
int vertical[4];

void setup()
{
  Serial.begin(9600);
  
  pinMode(RE_n_enable , OUTPUT);
    
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
  
  delay(1000);
  digitalWrite(RE_n_enable, LOW);

  timer.setTimeout(dt*1000);   //10 ms
  timer.restart();
}

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

  accData[0]={xAccl};
  accData[1]={yAccl};
  accData[2]={zAccl};
  gyroData[0]={xGyro};
  gyroData[1]={yGyro};
  gyroData[2]={zGyro};
}

void ComplementaryFilter(float accData[3], float gyrData[3], float *pitch, float *roll)
{
    float pitchAcc, rollAcc;               
 
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *pitch += (gyrData[0] * GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
    *roll -= (gyrData[1] * GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis
 
    // Compensate for drift with accelerometer data if !bullshit
    float forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
    if (forceMagnitudeApprox > 0.9 && forceMagnitudeApprox < 1.1)
    {
  // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f(accData[1], (float)accData[2]);
        *pitch = *pitch * 0.95 + pitchAcc * 0.05;
 
  // Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f(accData[0], (float)accData[2]);
        *roll = *roll * 0.95 + rollAcc * 0.05;
    }
}

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

void setServoMicroseconds(Servo s, int value, int MAX_VAL) {
  if(value>MAX_VAL)
    value = MAX_VAL;
  else if(value<-MAX_VAL)
    value = -MAX_VAL;
  s.writeMicroseconds(STOP+value);
}

void loop() {
  float pitchAngle=0.0, rollAngle=0.0;
  char dato=0;
  
  if(timer.isExpired()) {
  timer.restart();
  
  imuRead();
  ComplementaryFilter(accData, gyroData, &angles[0], &angles[1]);
  
  if(Serial.available())
    dato=Serial.read();
  
  
  for(int i=0;i<4;i++)
    vertical[i]=0;
  if(dato=='a') //press "start" -> start
    start=1;
  else if(dato=='s') //press "select" -> stop
    start=0;
  else if(dato=='.'){
    kAng = tempKAng;
    tempKAng = 0;
  }else if(dato>='0' && dato<='9')
    tempKAng = tempKAng*10 + dato-'0';
  else
    tempKAng = 0;
  
  digitalWrite(RE_n_enable, HIGH);
  Serial.print(kAng);
  Serial.print(" ");
  Serial.print(tempKAng);
  Serial.print(" ");
  Serial.println(dato);
  delay(20);
  digitalWrite(RE_n_enable, LOW);
  
  
  if(start==1){
    /* Pitch and Roll */
  
    //values of pitch and roll angle, meant as the difference between the current angle and the ange we want to have
    
    pitchAngle  = kAng*angles[0]; //(+ is because of the inversion of y-axis on the joystick)
    rollAngle   = kAng*angles[1];
    
    vertical[0]+=linSaturation(pitchAngle, MAX_IMU);
    vertical[1]+=linSaturation(pitchAngle, MAX_IMU);
    vertical[2]-=linSaturation(pitchAngle, MAX_IMU);
    vertical[3]-=linSaturation(pitchAngle, MAX_IMU);
    
    vertical[0]+=linSaturation(rollAngle, MAX_IMU);
    vertical[1]-=linSaturation(rollAngle, MAX_IMU);
    vertical[2]-=linSaturation(rollAngle, MAX_IMU);
    vertical[3]+=linSaturation(rollAngle, MAX_IMU);
        
    //front/back
    setServoMicroseconds(T200_4, 0, MAX_IMU);
    setServoMicroseconds(T200_5, 0, MAX_IMU);

    //up/down
    setServoMicroseconds(T200_1, vertical[0], MAX_IMU);
    setServoMicroseconds(T200_2, vertical[1], MAX_IMU);
    setServoMicroseconds(T200_3, vertical[2], MAX_IMU);
    setServoMicroseconds(T200_6, vertical[3], MAX_IMU);
  }
  else {
    setServoMicroseconds(T200_1, 0, MAX_IMU); // send "stop" signal to ESC.
    setServoMicroseconds(T200_2, 0, MAX_IMU); // send "stop" signal to ESC.
    setServoMicroseconds(T200_3, 0, MAX_IMU); // send "stop" signal to ESC.
    setServoMicroseconds(T200_4, 0, MAX_IMU); // send "stop" signal to ESC.
    setServoMicroseconds(T200_5, 0, MAX_IMU); // send "stop" signal to ESC.
    setServoMicroseconds(T200_6, 0, MAX_IMU); // send "stop" signal to ESC.
  }
  }
}
