#include <Servo.h>

byte escPin_1 = 2;
Servo T200_1;
byte escPin_2 = 3;
Servo T200_2;
byte escPin_3 = 4;
Servo T200_3;
byte escPin_4 = 5;
Servo T200_4;
byte escPin_5 = 6;
Servo T200_5;
byte escPin_6 = 7;
Servo T200_6;
byte escPin_7 = 8;
Servo T200_7;
byte escPin_8 = 9;
Servo T200_8;

int indice=1;

void setup()
{
  Serial.begin(9600);
  
  T200_1.attach(escPin_1);
  T200_2.attach(escPin_2);
  T200_3.attach(escPin_3);
  T200_4.attach(escPin_4);
  T200_5.attach(escPin_5);
  T200_6.attach(escPin_6);
  T200_7.attach(escPin_7);
  T200_8.attach(escPin_8);
 
  T200_1.writeMicroseconds(1500); // send "stop" signal to ESC.
  T200_2.writeMicroseconds(1500);
  T200_3.writeMicroseconds(1500);
  T200_4.writeMicroseconds(1500);
  T200_5.writeMicroseconds(1500);
  T200_6.writeMicroseconds(1500);
  T200_7.writeMicroseconds(1500);
  T200_8.writeMicroseconds(1500);
  
  delay(1000);
}

void loop() {

  while(!Serial.available());
  indice=Serial.read()-'0';
  
  switch(indice) {
    case 1:
      T200_1.writeMicroseconds(1550);
      break;
    case 2:
      T200_2.writeMicroseconds(1550);
      break;
    case 3:
      T200_3.writeMicroseconds(1550);
      break;
    case 4:
      T200_4.writeMicroseconds(1550);
      break;
      case 5:
      T200_5.writeMicroseconds(1550);
      break;
    case 6:
      T200_6.writeMicroseconds(1550);
      break;
    case 7:
      T200_7.writeMicroseconds(1550);
      break;
    case 8:
      T200_8.writeMicroseconds(1550);
      break;
    default:
      Serial.println("Error indexing motors");
  }
  delay(5000);
  
  T200_1.writeMicroseconds(1500);
  T200_2.writeMicroseconds(1500);
  T200_3.writeMicroseconds(1500);
  T200_4.writeMicroseconds(1500);
  T200_5.writeMicroseconds(1500);
  T200_6.writeMicroseconds(1500);
  T200_7.writeMicroseconds(1500);
  T200_8.writeMicroseconds(1500);

}


