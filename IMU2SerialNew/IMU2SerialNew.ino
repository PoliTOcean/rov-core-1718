#include <RBD_Timer.h>
#include <Wire.h>

RBD::Timer timer;

#define Addr 0x69
#define GYROSCOPE_SENSITIVITY PI/1400.0
#define dt 0.01         // 10 ms sample rate!

#define RE_n_enable 3

float angles[]={0,0};   // pich, roll
uint8_t data[6];

void setup()
{
  pinMode(RE_n_enable , OUTPUT);
  Serial.begin(9600);
  digitalWrite(RE_n_enable, HIGH);
    
  // Initialise I2C communication as Master
  Wire.begin();
  
  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Select gyroscope configuration register
  Wire.write(0x1B);
  
  uint8_t data[6];
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

void ComplementaryFilter(float accData[3], float gyrData[3], float *pitch, float *roll)
{
    float pitchAcc, rollAcc, droll, dpitch, dyaw;
    float cdr, cdp, cdy, cr, cp;
    float sdr, sdp, sdy, sr, sp;
    float accTot;

    droll = (gyrData[0] * GYROSCOPE_SENSITIVITY) * dt;   // Angle around the X-axis
    dpitch = (gyrData[1] * GYROSCOPE_SENSITIVITY) * dt;  // Angle around the Y-axis
    dyaw = (gyrData[2] * GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Z-axis

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
  float xAccl = float(data[0] * 256 + data[1] - 31.5)/2060.5;
  float yAccl = float(data[2] * 256 + data[3] + 61)/2061.5;
  float zAccl = float(data[4] * 256 + data[5] + 319.1)/2092.8;

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

  float accData[]={xAccl,yAccl,zAccl};
  float gyroData[]={xGyro,yGyro,zGyro};
  ComplementaryFilter(accData, gyroData, &angles[0], &angles[1]);

  Serial.print("Roll: ");
  Serial.print(angles[0]*180/PI);
  Serial.print("  Pitch: ");
  Serial.println(angles[1]*180/PI);
  }
}
