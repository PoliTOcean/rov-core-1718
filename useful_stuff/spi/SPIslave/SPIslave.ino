void setup (void)
{
  pinMode(MISO, OUTPUT);

  // turn on SPI interrupt
  cli();
  SPCR = 0b11000000;
  sei();
}

volatile char buf[4];
volatile char ver[3]; //new temp
volatile byte pos, ref1,op;
volatile float y, x, rz;
volatile bool trigger2, trigger, pinkie, cmdstart, cmdstop;
volatile float Roll,Pitch,Pressure,Temperature;

// SPI interrupt routine
ISR (SPI_STC_vect)
{
  byte c = SPDR;

 if (pos < sizeof buf)
    {
    SPDR = buf [pos];
    }
 switch (pos) 
 {

 case 0:
 y = (c-127)/127;
 break;

 case 1:
 x = (c-127)/127;
 break;

 case 2:
 rz = (c-127)/127;
 break;

 case 3:
 ref1 = c;
 
 ver[0] = bitRead(c,7);
 ver[1] = bitRead(c,6);
 ver[2] = bitRead(c,5);
 cmdstop = bitRead(c,4);
 cmdstart = bitRead(c,3);
 pinkie = bitRead(c,2);
 trigger = bitRead(c,1);
 trigger2 = bitRead(c,0);
 
 if (ver[0] ==1 && ver[1]==0 && ver[2]==1)
 {
  Serial.println("Condition Verified");
 }
 break;

 //new part output

ISR (SPI_STC_vect)
{
  byte op = SPCR;

 for (byte i = 0; i<6; i++)
 {
 op[i] = Temperature;
 }

bitWrite(op,5,1);
bitWrite(op,6,0);
bitWrite(op,7,1);


}

 } 
  pos++;
  if ( pos >= sizeof (buf))
  pos=0;
  
}  // end of interrupt service routine (ISR) SPI_STC_vect

void loop (void)
{
  Roll = 0;
  Pitch = 1;
  Pressure = 0;
  Temperature = 25;


  buf[0] = Roll;
  buf[1] = Pitch;
  buf[2] = Pressure;
  buf[3] = Temperature;
} 


