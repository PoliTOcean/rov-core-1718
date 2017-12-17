 #include <Servo.h>

#define RE_n_enable 3

byte escPin_1 = 10;
Servo T200_1;
byte escPin_2 = 9;
Servo T200_2;
byte escPin_3 = 8;
Servo T200_3;
byte escPin_4 = 7;
Servo T200_4;
byte escPin_5 = 6;
Servo T200_5;
byte escPin_6 = 5;
Servo T200_6;

int test, indice=1;

void setup()
{
  Serial.begin(9600);

  pinMode(RE_n_enable , OUTPUT);
  
  T200_1.attach(escPin_1);
  T200_2.attach(escPin_2);
  T200_3.attach(escPin_3);
  T200_4.attach(escPin_4);
  T200_5.attach(escPin_5);
  T200_6.attach(escPin_6);
 
  T200_1.writeMicroseconds(1500); // send "stop" signal to ESC.
  T200_2.writeMicroseconds(1500);
  T200_3.writeMicroseconds(1500);
  T200_4.writeMicroseconds(1500);
  T200_5.writeMicroseconds(1500);
  T200_6.writeMicroseconds(1500);
  
  delay(1000);
  
  digitalWrite(RE_n_enable, LOW);
}

void loop() {
  while(!Serial.available());
  
  indice=Serial.read();
  
  switch(indice) {
    case '1':
      T200_1.writeMicroseconds(1600);
      delay(2000);
      break;
    case '2':
      T200_2.writeMicroseconds(1600);
      delay(2000);
      break;
    case '3':
      T200_3.writeMicroseconds(1600);
      delay(2000);
      break;
    case '4':
      T200_4.writeMicroseconds(1600);
      delay(2000);
      break;
    case '5':
      T200_5.writeMicroseconds(1600);
      delay(2000);
      break;
    case '6':
      T200_6.writeMicroseconds(1600);
      delay(2000);
      break;
    case 0:
      break;
    default:
      digitalWrite(RE_n_enable, HIGH);
      Serial.println((int)indice);
      delay(50);
      digitalWrite(RE_n_enable, LOW);
  }
  
  T200_1.writeMicroseconds(1500);
  T200_2.writeMicroseconds(1500);
  T200_3.writeMicroseconds(1500);
  T200_4.writeMicroseconds(1500);
  T200_5.writeMicroseconds(1500);
  T200_6.writeMicroseconds(1500);
 
}


