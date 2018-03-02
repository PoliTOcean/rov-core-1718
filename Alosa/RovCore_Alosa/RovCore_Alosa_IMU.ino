#include <Wire.h> //library for I2C communication

//I2C initialization to communicate with IMU
void initI2C(){
  // Initialise I2C communication as Master
  Wire.begin();
  
  // Start I2C transmission
  Wire.beginTransmission(IMU_ADDR);
  // Select gyroscope configuration register
  Wire.write(0x1B);
  // Full scale range = 2000 dps
  Wire.write(0x18);
  // Stop I2C transmission
  Wire.endTransmission();
  
  // Start I2C transmission
  Wire.beginTransmission(IMU_ADDR);
  // Select accelerometer configuration register
  Wire.write(0x1C);
  // Full scale range = +/-16g
  Wire.write(0x18);
  // Stop I2C transmission
  Wire.endTransmission();
  
  // Start I2C transmission
  Wire.beginTransmission(IMU_ADDR);
  // Select power management register
  Wire.write(0x6B);
  // PLL with xGyro reference
  Wire.write(0x01);
  // Stop I2C transmission
  Wire.endTransmission();
}

//function for IMU data calculations
void ComplementaryFilter( float accData[],  //accelerometer data read from IMU
                          float gyrData[]   //gyroscope data read from IMU
                          ){
  float pitchAcc, rollAcc;

  // Integrate the gyroscope data -> int(angularSpeed) = angle
  pitch += (gyrData[0] * GYRO_SENS) * dt; // Angle around the X-axis
  roll -= (gyrData[1] * GYRO_SENS) * dt;    // Angle around the Y-axis

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

//function to read IMU data
void imuRead() {
  uint8_t data[6];
  float accData[3], gyroData[3];
  
  // Start I2C transmission
  Wire.beginTransmission(IMU_ADDR);
  // Select data register
  Wire.write(0x3B);
  // Stop I2C transmission
  Wire.endTransmission();
  
  // Request 6 bytes of data
  Wire.requestFrom(IMU_ADDR, 6);
  
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
  Wire.beginTransmission(IMU_ADDR);
  // Select data register 
  Wire.write(0x43);
  // Stop I2C transmission
  Wire.endTransmission();
  
  // Request 6 bytes of data
  Wire.requestFrom(IMU_ADDR, 6);
  
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

  //save data for further calculations
  accData[0] = xAccl;
  accData[1] = yAccl;
  accData[2] = zAccl;
  gyroData[0] = xGyro;
  gyroData[1] = yGyro;
  gyroData[2] = zGyro;
    
  ComplementaryFilter(accData, gyroData); //call calculation function
}
