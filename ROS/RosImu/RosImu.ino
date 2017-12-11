#include <RBD_Timer.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

RBD::Timer timer;

#define Addr 0x69
#define GYROSCOPE_SENSITIVITY PI/1900.0
#define dt 0.01         // 10 ms sample rate!

ros::NodeHandle nh; // ROS handler

std_msgs::Float32MultiArray imu_msg;
ros::Publisher p("imu", &imu_msg);

void setup()
{ 
  nh.initNode();
  imu_msg.data_length = 6;
  nh.advertise(p);

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
  delay(200);

  timer.setTimeout(dt*1000);   //10 ms
  timer.restart();
}

void loop()
{
  if(timer.isExpired()) {
  timer.restart();

  uint8_t data[6];
  float valori[6];
  
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
  valori[0] = float(data[0] * 256 + data[1] - 31.5)/2060.5;
  valori[1] = float(data[2] * 256 + data[3] + 61)/2061.5;
  valori[2] = float(data[4] * 256 + data[5] + 319.1)/2092.8;

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
  valori[3] = data[0] * 256 + data[1] + 50;
  valori[4] = data[2] * 256 + data[3] - 28.5;
  valori[5] = data[4] * 256 + data[5] + 5.5;

  imu_msg.data = valori;

  p.publish(&imu_msg);

  nh.spinOnce();
  }
}

