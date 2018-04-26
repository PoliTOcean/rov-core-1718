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
void ComplementaryFilter(float accData[3], float gyrData[3])
{
    float pitchAcc, rollAcc, droll, dpitch, dyaw;
    float cdr, cdp, cdy, cr, cp;
    float sdr, sdp, sdy, sr, sp;
    float accTot;

    droll = (gyrData[0] * GYRO_SENS) * dt;   // Angle around the X-axis
    dpitch = (gyrData[1] * GYRO_SENS) * dt;  // Angle around the Y-axis
    dyaw = (gyrData[2] * GYRO_SENS) * dt;    // Angle around the Z-axis

    cdr=cos(droll);
    cdp=cos(dpitch);
    cdy=cos(dyaw);

    cr=cos(roll);
    cp=cos(pitch);

    sdr=sin(droll);
    sdp=sin(dpitch);
    sdy=sin(dyaw);

    sr=sin(roll);
    sp=sin(pitch);

    roll=atan2(cdr*(sdy*sp + cdy*cp*sr) - sdr*(sdp*(cdy*sp - cp*sdy*sr) - cdp*cp*cr), - sdr*(sdy*sp + cdy*cp*sr) - cdr*(sdp*(cdy*sp - cp*sdy*sr) - cdp*cp*cr));
    pitch=atan2(cdp*(cdy*sp - cp*sdy*sr) + cp*cr*sdp, sqrt(pow(cdr*(sdy*sp + cdy*cp*sr) - sdr*(sdp*(cdy*sp - cp*sdy*sr) - cdp*cp*cr),2)+pow(- sdr*(sdy*sp + cdy*cp*sr) - cdr*(sdp*(cdy*sp - cp*sdy*sr) - cdp*cp*cr),2)));
 
    accTot = sqrt(pow(abs(accData[0]),2) + pow(abs(accData[1]),2) + pow(abs(accData[2]),2));
    if (accTot > 0.9 && accTot < 1.1)
    {
        rollAcc = atan2(accData[1], accData[2]);
        pitchAcc = asin(-accData[0]/fabs(accTot));

        roll = roll * 0.9 + rollAcc * 0.1;
        pitch = pitch * 0.9 + pitchAcc * 0.1;   
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
  float xAccl = float(data[0] * 256 + data[1] - 31.5)/2060.5;
  float yAccl = float(data[2] * 256 + data[3] + 61)/2061.5;
  float zAccl = float(data[4] * 256 + data[5] + 319.1)/2092.8;

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
  float xGyro = (data[0] * 256 + data[1] + 50)*1.285;
  float yGyro = (data[2] * 256 + data[3] - 28.5)*1.2;
  float zGyro = (data[4] * 256 + data[5] + 5.5)*0.86;

  //save data for further calculations
  accData[0] = xAccl;
  accData[1] = yAccl;
  accData[2] = zAccl;
  gyroData[0] = xGyro;
  gyroData[1] = yGyro;
  gyroData[2] = zGyro;
    
  ComplementaryFilter(accData, gyroData); //call calculation function
}
