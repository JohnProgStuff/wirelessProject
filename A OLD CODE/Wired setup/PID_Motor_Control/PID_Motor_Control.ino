/*
John Peterson and Russell Mueller
Electrical Engineering Senior Project
Motor Control Program
*/

#include <SoftwareSerial.h>
#include <PID_v1.h>

/* Use Arduino pins 4 and 5 on the UNO for the Serial connection to the BLE modules,
*  RX -> D4 and TX -> D5*/
// Cal-eng board try D9 & D10 so that INT0/INT1 can be used with encoder
SoftwareSerial ble113(4,5); 

// Clockwise and counter-clockwise definitions.
// Depending on how you wired your motors, you may need to swap.**************
#define CW 0
#define CCW 1

// Motor definitions to make life easier:*************************************
#define MOTOR_A 0
#define MOTOR_B 1

// PID Constants:*************************************************************
#define AUTOMATIC 1
#define MANUAL    0
#define DIRECT    0
#define REVERSE   1

// COMMONLY USED PID FUNCTIONS DEFINITIONS*********************************************

//PID::PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int ControllerDirection); 
//void SetMode(int Mode);  // * sets PID to manual(0) or Auto (non-0)
//bool Compute();  /* * performs the PID calculation. it should be called 
//                    *   every time loop() cycles. ON/OFF and calculation frequency 
//                   *   can be set using SetMode/SetSampleTime respectively */
//void SetOutputLimits(double, double); /* clamps output to a specific range. 0-255 
//                                       *   by default, but it's likely the user will 
//                                       *   want to change this depending on application*/
// NOT COMMONLY USED PID FUNCTION DEFINITIONS******************************************
//void SetTunings(double, double, double);/* While most users will set the tunings once 
//                                         * in constructor, this gives user option of 
//                                         * changing during runtime for Adaptive control*/
//void SetControllerDirection(int); /* Sets Direction, or "Action" of controller. 
//                                   * DIRECT: output increases when error is positive
//                                   * REVERSE: output decreases when error is positive
//                                   * function isn't used much, Direction set in constructor*/
//void SetSampleTime(int)   /* Sets the frequency, in Milliseconds, which PID calculation is done. default is 100*/
// OTHER PID FUNCTIONS:****************************************************************
//double GetKp();     // 
//double GetKp();
//double GetKp();
//int GetMode();
//int GetDirection();
//void Initialize();

// Pin Assignments // byte = unsigned char ***********************************
// Don't change these! These pins are statically defined by shield layout
const byte PWMA = 3;  // PWM control (speed) for motor A
const byte PWMB = 11; // PWM control (speed) for motor B
const byte DIRA = 12; // Direction control for motor A
const byte DIRB = 13; // Direction control for motor B
const int button_1_Pin = 6;     // the number of the shift UP   pushbutton pin
const int button_2_Pin = 7;     // the number of the shift DOWN pushbutton pin
const int pin8 = 8;

// Commands received from BUp signal from master
String ShiftUP = "UP";    // Up signal from master
String ShiftDN = "DN";    // Down signal from master
String startTune = "ST";  // start Tune signal from master
String endTune = "ET";    // end Tune signal from master
boolean tuneON = 0;       // tune mode on/off variable 

// Serial variables **********************************************************
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

// Button debouncing variables: chars are signed values
char button_1_read, button_2_read;
char BTNS, BTNS_OLD, BTNS_temp, BTNS_STATE, BTNS_STATE_CHANGED;
char BTNS_Still_Down, OLD_STATE;

// Motor controls ************************************************************
int RunTime = 5; // the amount of milliseconds to run motor
int gear = 1; // the cog position starts at 1 goes to 11
int numCogs = 11;
int motorFadeValue = 5;
int motorDelay = 1;
double motorspeed = 120;
// PID variables ******************************************************************
double myKp=2; // (P)roportional Tuning Parameter
double myKi=5; // (I)ntegral Tuning Parameter
double myKd=1; // (D)erivative Tuning Parameter
double dispKp; // saves user-entered format for display purposes
//double dispKp; // saves user-entered format for display purposes
//double dispKp; // saves user-entered format for display purposes

int controllerDirection;
/* Pointers to the Input, Output, and Setpoint variables
 * This creates hard link between the variables and the PID, freeing user from having 
 * to constantly tell us what these values are. with pointers we'll just know.
 */
double *myInput; 
double *myOutput;
double *mySetpoint;

unsigned long lastTime;
double ITerm, lastInput;
unsigned long SampleTime;
double outMin, outMax;
bool inAuto;
int ratio;


// Encoder signals*****************************************************************
char encoderStateA = 0x00;
char encoderStateB = 0x00;
double Setpoint = 100;
volatile int EncoderA = LOW; // INT0
volatile int EncoderB = LOW; // INT1
double encoderstate = 0;


/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 *    * constructor. links the PID to the Input, Output, and Setpoint. 
 *      Initial tuning parameters are also set here.
 ***************************************************************************/
/*PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;
    PID::SetOutputLimits(0, 255);  // default output limit corresponds to 
                                    //  the arduino pwm limits
    SampleTime = 100;    //default Controller Sample Time is 0.1 seconds
    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd);
    lastTime = millis()-SampleTime;
}
*/
PID myPID(&encoderstate, &motorspeed, &Setpoint, myKp, myKi, myKd, DIRECT);

/**********************************************************************************
* START FUNCTION
**********************************************************************************/
void setup()
{
    Serial.begin(9600); //This pipes to the serial monitor
    Serial.print("BLE-Slave Sketch setup...");
    ble113.begin(9600); // SoftwareSerial
    setupArdumoto(); // Set all pins as outputs
    // initialize the pushbutton pins as an input: Don't need for wireless
    //pinMode(button_1_Pin, INPUT);
    //pinMode(button_2_Pin, INPUT);
    // digitalWrite(button_1_Pin, HIGH); // Pull the button high so it goes low when pressed
    digitalWrite(pin8, OUTPUT); // has min/max cog been reached
    // reserve 200 bytes for the inputString:
    inputString.reserve(2);
    //pinMode(button_pin, INPUT); 
    pinMode(EncoderA, INPUT);
    pinMode(EncoderB, INPUT);
    // attachInterrupt(interrupt,ISR,mode)
    attachInterrupt(0, encoderISR, RISING); //INT0=0 is pin D2
    //attachInterrupt(1, encoderISR, RISING); //INT1=1 is pin D1
// SETTINGS FROM PID_v1 library
//    Input = analogRead(PIN_INPUT);
//    Setpoint = 100;
    //Turn the PID on
    myPID.SetMode(AUTOMATIC);
    Serial.println(" done.");
}

/**********************************************************************************
* PROGRAM LOOP
**********************************************************************************/
void loop()
{
  // get the state of the Shift, tune, reset buttons with each loop.
  // IF BOTH BUTTONS are not pressed continue on
        //getBTNS(); // Gets Shifts if wired, else reset, tune or pair.
        //RunMotorWired(); // comment out if wireless is used.
     //SerialFromPC(); // Comment out if not using to shave a tiny bit of time.
     SerialFromBLE();
     // IF tuneON then change Delay to 1 ms else 5 ms
     if (inputString == startTune) { // set Tune mode settings.
        inputString = "";
	tuneON = 1;
        Serial.println("gotST"); delay(10);
        ble113.print("gotST"); delay(10);
        motorDelay = 1;
        motorFadeValue = 5;
        //encCount = 5
     }  
     if (inputString == endTune){ // reset to default
        inputString = "";
        Serial.println("gotET"); delay(10);
        ble113.print("gotET"); delay(10);
        motorDelay = 5; 
        motorFadeValue = 5;
        tuneON = 0; // turn off Tune mode.        
        //encCount = 50;
     }
     RunMotorWireless(); // comment out if wired is used.
     inputString = "";
     // LIGHT an LED if the end cogs have ben reached. maxUP/maxDN
     if (gear == 1 || gear == numCogs){
         digitalWrite(pin8, HIGH);
     }
     else{
         digitalWrite(pin8, LOW);
     }

}// end main loop function

/***************************************************************************************
* RunMotorWireless() is the new way of running the motor for a specific amount of time and
*               checking BTNS. This replaced by the wired method. This runs the motor if
*               a specific char is received.
****************************************************************************************/
void RunMotorWireless(void){
 // if inputString() == commands -> perform command
    //SHIFT_UP -> Counter Clock-Wise Direction##############
    //Serial.print("inputString = ");
    //Serial.println(inputString);
    if (inputString == ShiftUP) { // Button 1 pressed 1st sample  
        inputString = "";   
        if (gear != numCogs || tuneON == 1){
            //  PID Control of the motor
            encoderstate = 0;
            // turn on motor and the Encoder Interrupt will recalculate the speed etc.
            while(encoderstate < 25) {
            digitalWrite(DIRB, CW);
            analogWrite(PWMB, motorspeed); }   
            stopArdumoto(MOTOR_B);
            if (tuneON == 1){
              Serial.println("tunUP"); delay(5);  ble113.print("tunUP"); delay(5); }
            else { // tuneON = 0, normal mode.
              Serial.println("gotUP"); delay(5);  ble113.print("gotUP"); delay(5);  
              gear = gear +1;
            }
        }// end gear if
        else{ Serial.print("maxUP"); delay(10);  ble113.print("maxUP"); delay(10);  }
    } // END IF UP
    // IF SHIFT_DN -> Clock-Wise Direction##############
    if (inputString == ShiftDN) {
        inputString = "";
        if (gear != 1) {
        // PID Control of the motor
              encoderstate = 0;
              // turn on motor and the Encoder Interrupt will recalculate the speed etc.
              digitalWrite(DIRA, CCW);
              analogWrite(PWMB, motorspeed);   
              
			  if (tuneON == 1){
                  Serial.println("tunDN"); delay(5);  ble113.print("tunDN"); delay(5); }
              else { // tuneON = 0, normal mode.
                  Serial.println("gotDN"); delay(5);  ble113.print("gotDN"); delay(5);
				  gear = gear - 1;
			  }              
        } // end if
        else{ Serial.print("maxDN"); delay(10);   ble113.print("maxDN"); delay(10);  }       
    } // END IF DOWN
}

/*****************************************************************************
* ISR for Encoder 
*     The following code needs to be in the Setup loop:
*     // attachInterrupt(interrupt,ISR,mode)
*     attachInterrupt(0, encoderISR, RISING) //INT0=0 is pin D2x
*     attachInterrupt(1, encoderISR, RISING) //INT1=1 is pin D1
******************************************************************************/
void encoderISR(void){
        encoderstate++;
        myPID.Compute();
        
        Serial.println(encoderstate);
        //Serial.println(error);
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
/*bool PID::Compute()
{
   if(!inAuto) return false; // if inAUTO=0 return false
   unsigned long now = millis(); //save current time in milliseconds
   unsigned long timeChange = (now - lastTime); // calc time from last Compute() to this Compute()
    //if that timeChange is greater than the SampleTime 
    //compute next error variables, else don't compute. 
    //returns true if new output was calculated,
    //returns false if new output wasn't calculated.
   if(timeChange>=SampleTime) 
   {
      //Compute all the working error variables
      double input = *myInput;
      double error = *mySetpoint - input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      double dInput = (input - lastInput);
 
      //Compute PID Output
      double output = kp * error + ITerm- kd * dInput;
      
	  if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	  *myOutput = output;
	  
      //Remember some variables for next time
      lastInput = input;
      lastTime = now;
	  return true;
   }
   else return false;
}*/
/* SetTunings(...)*********************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also 
 * be adjusted on the fly during normal operation.
 *  While most users will set the tunings once 
 *  in constructor, this gives user option of 
 *  changing during runtime for Adaptive control
 *************************************************************************************/
/*void PID::SetTunings(double Kp, double Ki, double Kd)
{
    if (Kp<0 || Ki<0 || Kd<0) return;

	dispKp = Kp; dispKi = Ki; dispKd = Kd;
	
	double SampleTimeInSec = ((double)SampleTime)/1000;
	kp = Kp;
	ki = Ki * SampleTimeInSec;
	kd = Kd / SampleTimeInSec;
	if(controllerDirection == REVERSE)
	{
		kp = (0-kp);
		ki = (0-ki);
		kd = (0-kd);
	}
     
}*/

/* SetSampleTime(...)**********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
/*void PID::SetSampleTime(int NewSampleTime){
	if (NewSampleTime > 0){
		double ration = (double)NewSampleTime/(double)SampleTime;
		
		ki *= ratio;
		kd /= ratio;
		SampleTime = (unsigned long)NewSampleTime;
	}
}*/

/* SetOutputLimits(...)********************************************************
 * This function will be used far more often than SetInputLimits. while
 * the input to the controller will generally be in the 0-1023 range (which is 
 * the default already,) the output will be a little different. maybe they'll 
 * be doing a time window and will need 0-8000 or something. or maybe they'll 
 * want to clamp it from 0-125. who knows. at any rate, that can all be done here.
 ******************************************************************************/
/*void PID::SetOutputLimits(double Min, double Max){
	if(Min >=Max) return;
	outMin = Min;
	outMax = Max;
	if(inAuto){
		if(*myOutput > outMax) *myOutput = outMax;
		else if(*myOutput < outMin) *myOutput = outMin;
		
		if(ITerm > outMax) ITerm = outMax;
		else if (ITerm < outMin) ITerm = outMin;
	}
}*/
/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
/*void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC); // if Mode is AUTO than newAuto=1
    if(newAuto == !inAuto)
    {  //we just went from manual to auto
        PID::Initialize();
    }
    inAuto = newAuto;
}*/

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
/*void PID::Initialize()
{
   ITerm = *myOutput;
   lastInput = *myInput;
   if(ITerm > outMax) ITerm = outMax;
   else if(ITerm < outMin) ITerm = outMin;
}*/

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
/*void PID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction != controllerDirection)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }   
   controllerDirection = Direction;
}*/

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
//double PID::GetKp(){ return  dispKp;}
//double PID::GetKi(){ return  dispKi;}
//double PID::GetKd(){ return  dispKd;}
//int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}// returns AUTO if inAUTO=1,MANUAL if inAUTO=0
//int PID::GetDirection(){ return controllerDirection;}

/*********************************************************************
* driveArdumoto drives 'motor' in 'dir' direction at 'spd' speed     *
**********************************************************************/
void driveArdumoto(byte motor, byte dir, byte spd)
{
  if (motor == MOTOR_A)
  {
    digitalWrite(DIRA, dir);
    analogWrite(PWMA, spd);
  }
  else if (motor == MOTOR_B)
  {
    digitalWrite(DIRB, dir);
    analogWrite(PWMB, spd);
  }  
}

/*********************************************************************
* stopArdumoto - stops motor (BRAKE)                                 *
**********************************************************************/
void stopArdumoto(byte motor)
{
  driveArdumoto(motor, 0, 0);
}

/*********************************************************************
* setupArdumoto initialize all pins                                  *
**********************************************************************/
void setupArdumoto()
{
  // All pins should be setup as outputs:
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);

  // Initialize all pins as low:
  digitalWrite(PWMA, LOW);
  digitalWrite(PWMB, LOW);
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, LOW);
}
/*********************************************************************

* SerialFromPC - used only to test sending from one pc to another to *
*                test BLE connectivity                               *
**********************************************************************/
void SerialFromPC(){
  // If Data from the PC serial console, submit it to the
  // BLE113 byte per byte. (1 byte at a time... no buffer)
  // this if loop cycles through once for each byte received from PC console.
  if (Serial.available()) { // if Serial buffer has stuff
      int chs = Serial.read();
      char ch = char(chs);
      Serial.print(ch); // echo each char back to PC
      ble113.write(ch); // send to module
      //Serial1.write(ch);  
      if (!Serial.available()){
          //Serial.println();
          //Serial.print("sent\n");  // done returning string to PC
      }
  }
}

/*********************************************************************
* SerialFromBLE                                                      *
**********************************************************************/
void SerialFromBLE(){
  // if BLE module has sent info. send it to the serial console.
  if (ble113.available()) {
      while(inputString.length() < 2){
        if(ble113.available()) {
            char inChar = (char)ble113.read();
            inputString += inChar;
            //Serial.write(inChar);
        }
      }
      Serial.print(inputString);
      inputString.trim(); // take off the '\n' if there is one
      if (inputString.length() > 2){ inputString = "";} // this line won't be needed if the master has a good debounce.
  }
}

/*********************************************************************
* SerialFromBLE OLD                                                  *
**********************************************************************/
/*void SerialFromBLE(){
  // if BLE module has sent info. send it to the serial console.
  if (ble113.available()) {
      while(ble113.available()){
        char inChar = (char)ble113.read();
        inputString += inChar;
          //Serial.write(ble113.read()); // use if UNO
        Serial.write(inChar);
      }
      inputString.trim(); // take off the '\n'
      if (inputString.length() > 2){ inputString = "";} // this line won't be needed if the master has a good debounce.
  //if (Serial1.available()) {
      // Serial.print("ble113: "); 
      // Serial.write(Serial1.read()); // use if ProMicro
      // Serial.write(ble113.read()); // use if UNO
  }
}
*/
/***************************************************************************************
* RunMotorWired() is the old way of running the motor for a specific amount of time and
*               checking BTNS. This is replaced by the wireless method.
****************************************************************************************/
/*void RunMotorWired(void){
    // IF BOTH BUTTONS are not pressed continue on
    // IF BTN1 -> Counter Clock-Wise Direction
    if (BTNS_STATE == 1 & BTNS_STATE_CHANGED == 1) { // Button 1 pressed 1st sample      
        
	// Gradually increases the speed of the motor up to max
	for (int fadevalue = 0 ; fadevalue <= 255; fadevalue += 5) {
	    driveArdumoto(MOTOR_B, CCW, fadevalue); // Set motor B to CCW at max
	    delay(5); // This is where we would implement the proportional control system
            stopArdumoto(MOTOR_B);
	} 
    }
    // If up BTN2 is pushed SHIFT_DOWN
    if (BTNS_STATE == 2 & BTNS_STATE_CHANGED == 2) {
      //digitalWrite(ledPin6, HIGH);   // Serial.println("SHIFT_Dn_Sent");  
	  // Gradually increases the speed of the motor up to max in Clock-Wise Direction
	for (int fadevalue = 0 ; fadevalue <= 255; fadevalue += 5) {
	    driveArdumoto(MOTOR_B, 0, fadevalue); // Set motor B to CCW at max
	    delay(5); // This is where we would implement the proportional control system
            stopArdumoto(MOTOR_B);
	}     
    }
} // end RunMotorWired
*/
/********************************************************************************
* getBTNS gets the state of the BTNS - not needed in the wireless motor driver. *
*********************************************************************************/
/*void getBTNS(void){
  // read the state of the both pushbuttons:
  button_1_read = !digitalRead(button_1_Pin); // read into bit 0
  button_2_read = (!digitalRead(button_2_Pin) << 1); // read into bit 1
  BTNS = button_1_read + button_2_read; // This is the 2 bit status of the buttons
  // XCHANGE OLD and Current BTNS with 3 operations
  BTNS_temp = BTNS;   BTNS = BTNS_OLD;   BTNS_OLD = BTNS_temp;
  BTNS = BTNS ^ BTNS_OLD;
//  CHANGES = BTNS; 
  BTNS = BTNS & BTNS_OLD;
  OLD_STATE = BTNS_STATE;
  BTNS_STATE_CHANGED = BTNS_STATE ^ ( BTNS_OLD);
  BTNS_STATE = (BTNS ^ BTNS_OLD);  
  // If we only want the event to happen once, and not repetitively than we use the BTNS_STILL_DOWN.
  BTNS_Still_Down = (BTNS_STATE & BTNS_OLD) & (BTNS_STATE != B00000000); 
}

*/
