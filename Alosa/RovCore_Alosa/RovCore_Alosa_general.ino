//ROV start function
void startRov(){
  start=1; //set start flag to 1

  //Actions when started
  curPress = readPress();     //read pressure
  saveRequestedPressure();    //and save it as the requested one

#ifdef DEBUG
  printOverSerial(1, 15, "Starting ROV...\n");
#endif
}

//ROV stop function
void stopRov(){
  start=0;

  //Actions when stopped
  //...
#ifdef DEBUG
  printOverSerial(1, 15, "Stopping ROV...\n");
#endif
}

//function for timer expiration
int isTime(){
  if(timer.isExpired()){  //if it is expired
    timer.restart();      //restart it
    return 1;             //and return TRUE
  }
  return 0;               //else, return FALSE
}

//Serial functions
RBD::Timer serialTimer; //timer for serial printing
void serialInit(int timeout){     /*int timeout = timeout in ms to print out*/
  Serial.begin(9600);             //Serial init
  pinMode(RE_n_enable , OUTPUT);  //communication enable pin 
  serialTimer.setTimeout(timeout);    //set timeout
  serialTimer.restart();          //start timer
  delay(100);                     //delay for RE_n_enable
  digitalWrite(RE_n_enable, LOW); //set output enable LOW. We don't have to communicate anything, for now
}

void printOverSerial(int printAnyway, int del, const char *format, ...){ /* int del =  delay to wait before setting RE_n_enable to LOW.
                                                         *            longer is the string, higher it has to be.   */
  va_list arg;
  char str[strlen(format)+40];    //final string that has to be printed
  
  if(printAnyway || serialTimer.isExpired()){    //if it is time to print
    serialTimer.restart();            //restart timer
    //parse formatted string
    va_start(arg, format);            //initialize arguments
    vsprintf(str, format, arg);       //print formatted string to str
    va_end(arg);                      //end arguments
    //Send it on the Serial
    digitalWrite(RE_n_enable, HIGH);  //set output enable to HIGH
    Serial.print(str);                //print formatted string
    delay(del);                       //delay for communication
    digitalWrite(RE_n_enable, LOW);   //set output enable to LOW
  }
}

