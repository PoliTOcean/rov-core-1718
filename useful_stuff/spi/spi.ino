// what to do with incoming data

void setup (void)
{

  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);

  // turn on SPI in slave mode
  cli();
  SPCR = 0b11000000;
  sei();

}  // end of setup


// SPI interrupt routine
ISR (SPI_STC_vect)
{
  byte c = SPDR;

  SPDR = c + 1;  // add 1
}  // end of interrupt service routine (ISR) SPI_STC_vect


void loop (void)
{

}  // end of loop

