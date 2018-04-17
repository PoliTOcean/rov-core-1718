void setup (void)
{
  pinMode(MISO, OUTPUT);

  // turn on SPI interrupt
  cli();
  SPCR = 0b11000000;
  sei();
}

volatile char buf [4];
volatile byte pos ref1;
volatile float y x rz;
volatile bool trigger2 trigger pinkie start stop;


// SPI interrupt routine
ISR (SPI_STC_vect)
{
  byte c = SPDR;

 if (pos < sizeof buf)
    {
    SPDR = buf [pos];
    }
 switch pos 
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

 bitSet(c,7);
 bitClear(c,6);
 bitSet(c,5);
 bitWrite(c,4,stop);
 bitWrite(c,3,start);
 bitWrite(c,2,pinkie);
 bitWrite(c,1,trigger);
 bitWrite(c,0,trigger2);
 
 break;

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


