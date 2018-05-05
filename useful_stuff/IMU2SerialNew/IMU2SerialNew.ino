#include <RBD_Timer.h>
#include <Wire.h>

RBD::Timer timer;

#define Addr 0x68
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
#define dt 0.01         // 10 ms sample rate!

float angles[]={0,0,0};   // pich, roll

void setup()
{
  Serial.begin(9600);
    
  // Initialise I2C communication as Master
  Wire.begin();
  
  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Select gyroscope configuration register
  Wire.write(0x6B);
  
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(200);

  timer.setTimeout(dt*1000);   //10 ms
  timer.restart();
}
    float accTot;

void ComplementaryFilter(float accData[3], float gyrData[3], float *pitch, float *roll, float *yaw)
{
    float pitchAcc, rollAcc, droll, dpitch, dyaw;
    float cdr, cdp, cdy, cr, cp;
    float sdr, sdp, sdy, sr, sp;

    droll = (gyrData[0]) * dt;   // Angle around the X-axis
    dpitch = (gyrData[1]) * dt;  // Angle around the Y-axis
    dyaw = (gyrData[2]) * dt;    // Angle around the Z-axis

    cdr=cos(droll);
    cdp=cos(dpitch);
    cdy=cos(dyaw);

    cr=cos(*roll);
    cp=cos(*pitch);

    sdr=sin(droll);
    sdp=sin(dpitch);
    sdy=sin(dyaw);

    sr=sin(*roll);
    sp=sin(*pitch);

    *roll=atan2(cdr*(sdy*sp + cdy*cp*sr) - sdr*(sdp*(cdy*sp - cp*sdy*sr) - cdp*cp*cr), - sdr*(sdy*sp + cdy*cp*sr) - cdr*(sdp*(cdy*sp - cp*sdy*sr) - cdp*cp*cr));
    *pitch=atan2(cdp*(cdy*sp - cp*sdy*sr) + cp*cr*sdp, sqrt(pow(cdr*(sdy*sp + cdy*cp*sr) - sdr*(sdp*(cdy*sp - cp*sdy*sr) - cdp*cp*cr),2)+pow(- sdr*(sdy*sp + cdy*cp*sr) - cdr*(sdp*(cdy*sp - cp*sdy*sr) - cdp*cp*cr),2)));

    accTot = sqrt(pow(abs(accData[0]),2) + pow(abs(accData[1]),2) + pow(abs(accData[2]),2));
    if (accTot > 0.9 && accTot < 1.1)
    {
        rollAcc = atan2(accData[1], accData[2]);
        pitchAcc = asin(-accData[0]/fabs(accTot));

        *roll = *roll * 0.9 + rollAcc * 0.1;
        *pitch = *pitch * 0.9 + pitchAcc * 0.1;   
    }
}


void loop() {
  if(timer.isExpired()) {
  timer.restart();

  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Select data register
  Wire.write(0x3B);
  // Stop I2C transmission
  Wire.endTransmission();
  
  Wire.requestFrom(Addr,14,true);  // request a total of 14 registers
 AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
 AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
 AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
 Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
 GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
 GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
 GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  // Convert the data
  float xAccl = float(AcX - 1089.4)/16436;
  float yAccl = float(AcY + 496.4)/16357;
  float zAccl = -float(AcZ + 1396.8)/16802.6;

  // Convert the data
  float xGyro = (GyX + 159.07)/2700;
  float yGyro = (GyY - 115.9)/2500;
  float zGyro = (GyZ + 141.44)/2500;

  float accData[]={xAccl,yAccl,zAccl};
  float gyroData[]={xGyro,yGyro,zGyro};
  ComplementaryFilter(accData, gyroData, &angles[0], &angles[1], &angles[2]);

  Serial.print("Roll: ");
  Serial.print(angles[0]*180/PI);
  Serial.print("  Pitch: ");
  Serial.println(angles[1]*180/PI);
  }
}
