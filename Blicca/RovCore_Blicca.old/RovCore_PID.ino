#define epsilon 0.01
#define MAX (4) //Saturation
#define MIN (-4)
#define Kp 0.1
#define Kd 0
#define Ki 0

void PIDcal(float setpoint,float actual_value)
{
  static float pre_error = 0;         // global
  static float integral = 0;          // global
  float error;
  float derivative;
  float output;                       // global

  //CaculateP,I,D
   
  error = setpoint - actual_value;
  
  
  if(abs(error)> epsilon)
  {
     integral = integral + error*dt;
  }
  derivative = (error - pre_error)/dt;
  output = Kp*error + Ki*integral + Kd*derivative;
  
  //Saturation Filter
  if(output> MAX)
  {
    output= MAX;
  }
  else if(output< MIN);
  {
    output= MIN;
  }
  //Update error
  pre_error= error;
}
