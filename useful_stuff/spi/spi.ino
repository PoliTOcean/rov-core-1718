void setup (void)
{
  pinMode(MISO, OUTPUT);

  // turn on SPI interrupt
  cli();
  SPCR = 0b11000000;
  sei();
}


// SPI interrupt routine
ISR (SPI_STC_vect)
{
  byte c = SPDR;

  SPDR = c + 1;  // add 1
}


void loop (void)
{

}

