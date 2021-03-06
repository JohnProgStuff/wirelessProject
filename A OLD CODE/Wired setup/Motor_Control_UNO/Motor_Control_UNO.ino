/*
John Peterson and Russell Mueller
Electrical Engineering Senior Project
Motor Control Program
*/

#include <SoftwareSerial.h>
//#include <PID_v1.h>

/* Use Arduino pins 4 and 5 on the UNO for the Serial connection to the BLE modules,
*  RX -> D4 and TX -> D5*/
SoftwareSerial ble113(4,5); 

// Clockwise and counter-clockwise definitions.
// Depending on how you wired your motors, you may need to swap.**************
#define CW  0
#define CCW 1

// Motor definitions to make life easier:*************************************
#define MOTOR_A 0
#define MOTOR_B 1

// Pin Assignments // byte = unsigned char ***********************************
// Don't change these! These pins are statically defined by shield layout
const byte PWMA = 3;  // PWM control (speed) for motor A
const byte PWMB = 11; // PWM control (speed) for motor B
const byte DIRA = 12; // Direction control for motor A
const byte DIRB = 13; // Direction control for motor B
const int button_1_Pin = 6;     // the number of the shift UP   pushbutton pin
const int button_2_Pin = 7;     // the number of the shift DOWN pushbutton pin

// Commands received from BLE module (from Master device)*********************
String ShiftUP = "UP";    // Up signal from master
String ShiftDN = "DN";    // Down signal from master
String startTune = "TN";  // start Tune signal from master
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
int motorDelay = 5;

// Encoder signals*****************************************************************
char encoderState = 0x00;
int EncoderA = 2; // INT0
int EncoderB = 3; // INT1

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
    pinMode(button_1_Pin, INPUT);
    pinMode(button_2_Pin, INPUT);
    // digitalWrite(button_1_Pin, HIGH); // Pull the button high so it goes low when pressed
    // reserve 200 bytes for the inputString:
    inputString.reserve(2);
    //pinMode(button_pin, INPUT); 
    pinMode(EncoderA, INPUT);
    pinMode(EncoderB, INPUT);
    // attachInterrupt(interrupt,ISR,mode)
    attachInterrupt(0, encoderISR, RISING); //INT0=0 is pin D2
    attachInterrupt(1, encoderISR, RISING); //INT1=1 is pin D1
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
     SerialFromPC(); // Comment out if not using to shave a tiny bit of time.
     SerialFromBLE();
     // IF tuneON then change Delay to 1 ms else 5 ms
     if (inputString == startTune) { // set Tune mode settings.
        inputString = "";
	tuneON = 1;
        Serial.println("gotTN"); delay(10);
        ble113.print("gotTN"); delay(10);
        motorDelay = 1;
        motorFadeValue = 5;
        
     }  
     if (inputString == endTune){ // reset to default
        inputString = "";
        Serial.println("gotET"); delay(10);
        ble113.print("gotET"); delay(10);
        motorDelay = 5; 
        motorFadeValue = 5;
        tuneON = 0; // turn off Tune mode.        
     }
     RunMotorWireless(); // comment out if wired is used.
}// end main loop function

/***************************************************************************************
* RunMotorWireless() is the new way of running the motor for a specific amount of time and
*               checking BTNS. This replaced by the wired method. This runs the motor if
*               a specific char is received.
****************************************************************************************/
void RunMotorWireless(void){
 // if inputString() == commands -> perform command
    // SHIFT_UP -> Counter Clock-Wise Direction##############
    //Serial.print("inputString = ");
    //Serial.println(inputString);
    if (inputString == ShiftUP) { // Button 1 pressed 1st sample  
        inputString = "";    
	if (tuneON == 1){
            Serial.println("tunUP"); delay(5);
            ble113.print("tunUP"); delay(5);
        }
        else { // tuneON = 0, normal mode.
            Serial.println("gotUP"); delay(5);
            ble113.print("gotUP"); delay(5);  
        }
        if (gear != numCogs){
            // Gradually increases the speed of the motor up to max
            for (int fadevalue = 0 ; fadevalue <= 255; fadevalue += motorFadeValue) {
	        driveArdumoto(MOTOR_B, CCW, fadevalue); // Set motor B to CCW at max
	        delay(motorDelay); // This is where we would implement the proportional control system
                stopArdumoto(MOTOR_B);
	    }
            gear = gear +1;
        }// end gear if
        else{
          //Serial.print("error maxed out"); delay(10);
          //ble113.print("error maxed out"); delay(10);
        }// end else   
    }
    // IF SHIFT_DN -> Clock-Wise Direction##############
    if (inputString == ShiftDN) {
        inputString = "";
	if (tuneON == 1){
            Serial.println("tunDN"); delay(5);
            ble113.print("tunDN"); delay(5);
        }
        else { // tuneON = 0, normal mode.
            Serial.println("gotDN"); delay(5);
            ble113.print("gotDN"); delay(5);  
        }
        if (gear != 1) {
        // Gradually increases the speed of the motor up to max
	  for (int fadevalue = 0 ; fadevalue <= 255; fadevalue += motorFadeValue) {
	      driveArdumoto(MOTOR_B, CW, fadevalue); // Set motor B to CCW at max
	      delay(motorDelay); // This is where we would implement the proportional control system
              stopArdumoto(MOTOR_B);
          }
          gear = gear - 1;
        } // end if
        else{
            //Serial.print("error maxed out"); delay(10);
            //ble113.print("error maxed out"); delay(10);
        }       

    }
    /*
    // IF BTN3 -> Tune_ON ##################
    if (inputString == TuneON) { // TN
        inputString = "";
	Serial.println("gotTN"); delay(10);
        ble113.print("gotTN"); delay(10);
        //digitalWrite(ledPin6, HIGH);   
        // Gradually increases the speed of the motor up to max
	for (int fadevalue = 0 ; fadevalue <= 255; fadevalue += 5) {
	    driveArdumoto(MOTOR_B, CW, fadevalue); // Set motor B to CCW at max
	    delay(); // This is where we would implement the proportional control system
            stopArdumoto(MOTOR_B);
	}     
    }
    // IF BTN3 -> Tune_OFF #################
    if (inputString == TuneOFF) {// ET
        inputString = "";
	Serial.println("gotET"); delay(10);
        ble113.print("gotET"); delay(10);
        //digitalWrite(ledPin6, HIGH);   
        // Gradually increases the speed of the motor up to max
	for (int fadevalue = 0 ; fadevalue <= 255; fadevalue += 5) {
	    driveArdumoto(MOTOR_B, CW, fadevalue); // Set motor B to CCW at max
	    delay(5); // This is where we would implement the proportional control system
            stopArdumoto(MOTOR_B);
	}     
    }
    */
}

/*****************************************************************************
* ISR for Encoder 
*     The following code needs to be in the Setup loop:
*     // attachInterrupt(interrupt,ISR,mode)
*     attachInterrupt(0, encoderISR, RISING) //INT0=0 is pin D2
*     attachInterrupt(1, encoderISR, RISING) //INT1=1 is pin D1
******************************************************************************/
void encoderISR(void){
	encoderState = digitalRead(EncoderA);		 // read in the A signal
	encoderState = (digitalRead(EncoderB) << 1);     // read in the B signal
	//encoderState = (digitalRead(EncoderI) << 2);     // read in the Index line
	
	
}

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

/***************************************************************************************
* RunMotorWired() is the old way of running the motor for a specific amount of time and
*               checking BTNS. This is replaced by the wireless method.
****************************************************************************************/
void RunMotorWired(void){
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
	    driveArdumoto(MOTOR_B, CW, fadevalue); // Set motor B to CCW at max
	    delay(5); // This is where we would implement the proportional control system
            stopArdumoto(MOTOR_B);
	}     
    }
} // end RunMotorWired

/********************************************************************************
* getBTNS gets the state of the BTNS - not needed in the wireless motor driver. *
*********************************************************************************/
void getBTNS(void){
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

