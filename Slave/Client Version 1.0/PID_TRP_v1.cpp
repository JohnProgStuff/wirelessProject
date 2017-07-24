/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "PID_TRP_v1.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(int* Input, int* Output, int* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)
{
	
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
	  inAuto = false;
	
	  PID::SetOutputLimits(0, 255);				//default output limit corresponds to 
												//the arduino pwm limits
    SampleTime = 10;							//default Controller Sample Time is 0.1 seconds

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd);
    lastTime = millis()-SampleTime;				
}
 
 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
bool PID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   double output2;
   signed int lastError;
   if(timeChange>=SampleTime)
   {
	  
	  /*Serial.print("\nin: ");
	  Serial.print(*myInput);
	  Serial.print(",SP: ");
	  Serial.print(*mySetpoint);
	  */
	  /*Serial.print(", ");
	  Serial.print(kp); Serial.print(", ");
	  Serial.print(ki); Serial.print(", ");
	  Serial.print(kd); Serial.print(", ");
	  */
      /*Compute all the working error variables*/
	  int input = *myInput;
    signed int error = *mySetpoint - input;   // 285-295 = -10
    lastError = error;
	 // if (error * lastError < 0){ //error *= (-1);
	 //   Serial.println("OS");
	 // }
	 // Serial.print("\nerr: ");	  Serial.println(error);
	 // Serial.print(" , ki: ");	  Serial.print(ki*10000); Serial.print(",\t");
     if (error < 0 ){error *= (-1);}
	 ITerm += (ki * error);
      //ITerm += (error);
      //Serial.print("err: ");  Serial.print(error);
	  Serial.print(", I: ");  Serial.print(ITerm);Serial.print(",\t");
	  
	  if(ITerm > outMax) ITerm= outMax; 
    else if(ITerm < outMin) ITerm= outMin;
    int dInput = (input - lastInput);
    
    /*Compute PID Output*/
	  //Serial.print(", ");     Serial.print(dInput);
      	  
	 // problem with the Iterm. If we don't want a integral 
	 // amount to be included in the calculation the motor speed needs to start at zero
	 // I had the motor speed set at 128 and when the compute function runs the 1st time/
	 // it adds 128 to (ki *error) => 128+(0*200)=128 so the Iterm is then always included 
	 // in the output equation even if the ki is set at zero
	  //if (error < 0 ){error *= (-1);}
    double output2 = kp * error + ITerm- kd * dInput;
    int output = (int)output2;
	  
    if(output > outMax) output = outMax;
	  else if(output < outMin) output = outMin;
	  *myOutput = output;
	  
    /*Remember some variables for next time*/
    lastInput = input;
    lastTime = now;
  	// Serial.print("\n,out: "); Serial.println(output2);

	  return true;
   }
   else return false;
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void PID::SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
 
   dispKp = Kp; dispKi = Ki; dispKd = Kd;
   
   double SampleTimeInSec = ((double)SampleTime)/1000;  
   //Serial.println("/samtime");
   kp = Kp;
   ki = Ki * SampleTimeInSec; // ki will most likely always be zero if it is a int
   kd = Kd / SampleTimeInSec; // kd will most likely always be zero if it is a int
 
  if(controllerDirection ==REVERSE)
   {
	  //Serial.println("0-kd");
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (int)NewSampleTime
                      / (int)SampleTime;
	  Serial.println("/ratio");
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
 
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(int Min, int Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
 
   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;
	 
	   if(ITerm > outMax) ITerm= outMax;
	   else if(ITerm < outMin) ITerm= outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}
 
/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
void PID::Initialize()
{
   ITerm = *myOutput;
   lastInput = *myInput;
   if(ITerm > outMax) ITerm = outMax;
   else if(ITerm < outMin) ITerm = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	  Serial.println("0-kd");
	  kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }   
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp(){ return  dispKp; }
double PID::GetKi(){ return  dispKi;}
double PID::GetKd(){ return  dispKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}

