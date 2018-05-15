//ROV start function
void startRov(){
  start=1; //set start flag to 1

  //Actions when started
  saveRequestedPressure();
}

//ROV stop function
void stopRov(){
  start=0;

  //Actions when stopped
  //...
}

//function for timer expiration
int isTime(){
  if(timer.isExpired()){  //if it is expired
    timer.restart();      //restart it
    return 1;             //and return TRUE
  }
  return 0;               //else, return FALSE
}
