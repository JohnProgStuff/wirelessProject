/*
John Peterson and Russell Mueller
Electrical Engineering Senior Project
Motor Control Program
*/
#include "PID_TRP_v1.h"
#include <SoftwareSerial.h>

/* Use Arduino pins 4 and 5 on the UNO for the Serial connection to the BLE modules,
*  RX -> D4 and TX -> D5*/
// Cal-eng board try D9 & D10 so that INT0/INT1 can be used with encoder
SoftwareSerial ble113(4,5); 
//SoftwareSerial ble113(9,10);
// Clockwise and counter-clockwise definitions.
// Depending on how you wired your motors, you may need to swap.**************
#define CW 0
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
//String pidUpdate = "";

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
int encCount = 600;
int encTarget = 600;
int encTuneTarget = 5;
int motorspeed = 10;

// PID variables ******************************************************************
int myKp=1; // (P)roportional Tuning Parameter
int myKi=1; // (I)ntegral Tuning Parameter
int myKd=1; // (D)erivative Tuning Parameter


// Encoder signals*****************************************************************
char encoderStateA = 0x00;
char encoderStateB = 0x00;
int SetTarget = 260;
volatile int EncoderA = HIGH; // INT0
volatile int EncoderB = HIGH; // INT1
int encoderstate = 0;
// volatile unsigned int encoder Counter = 0;
int encoderCounter = 0;
bool dir = 0;
byte currEncState = 0x00;
byte lastEncState = 0x00;

PID myPID(&encoderCounter, &motorspeed, &SetTarget, myKp, myKi, myKd, DIRECT);

/**********************************************************************************
* START FUNCTION
**********************************************************************************/
void setup()
{
    Serial.begin(115200); //This pipes to the serial monitor
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
    pinMode(7, OUTPUT);
    attachInterrupt(0, enc_A_ISR, CHANGE); //INT0=0 is pin D2
    attachInterrupt(1, enc_B_ISR, CHANGE); //INT1=1 is pin D3
    lastEncState = (PIND & B00001100);    
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0,255);
    Serial.println(" done.");
}

/**********************************************************************************
* PROGRAM MAIN LOOP
**********************************************************************************/
byte lastA, lastB;
void loop()
{
     SerialFromPC(); // Comment out if not using to shave a tiny bit of time.
     SerialFromBLE();
     // IF tuneON then change Delay to 1 ms else 5 ms
     if (inputString == startTune) { // set Tune mode settings.
        inputString = "";
	tuneON = 1;
        Serial.println("gotST"); delay(10);
        ble113.print("gotST"); delay(10);
        motorDelay = 1;
        motorFadeValue = 5;
        encCount = encTuneTarget;
     }  
     if (inputString == endTune){ // reset to default
        inputString = "";
        Serial.println("gotET"); delay(10);
        ble113.print("gotET"); delay(10);
        motorDelay = 5; 
        motorFadeValue = 5;
        tuneON = 0; // turn off Tune mode.        
        encCount = encTarget;
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
	// IF SHIFT_UP -> Counter-Clock-Wise Direction##############
    if (inputString == ShiftUP) { // Button 1 pressed 1st sample  
        inputString = "";   
        if (gear != numCogs || tuneON == 1){
            dir=CW;
            //  PID Control of the motor
            encoderstate = 0;
            encoderCounter = 0;
            // turn on motor and the Encoder Interrupt will recalculate the speed etc.
           while(encoderCounter <= SetTarget){
                myPID.Compute();
                driveArdumoto(MOTOR_B, CW, motorspeed); // Set motor B to CCW at max                      
                Serial.print(encoderCounter);   Serial.print(" ");  Serial.print(motorspeed); Serial.print(" ");  Serial.println(SetTarget);                   
           }//end while
        }//end while
        stopArdumoto(MOTOR_B);
        Serial.println("e");            
            if (tuneON == 1){
              Serial.println("tunUP"); delay(5);  ble113.print("tunUP"); delay(5); }
            else { // tuneON = 0, normal mode.
              Serial.println("gotUP"); delay(5);  ble113.print("gotUP"); delay(5);  
              gear = gear +1;
            }
            delay(1000);
            Serial.print("error= ");            //Serial.println(error);
            Serial.print("mspeed= ");            Serial.println(motorspeed);            Serial.print("Target= ");            Serial.println(encCount);            Serial.print("encoderCounter= ");            Serial.println(encoderCounter);
            encoderstate = 0;
    } // end shift up
    // IF SHIFT_DN -> Clock-Wise Direction##############
    if (inputString == ShiftDN) {
        inputString = "";
        if (gear != 1) {
            dir=CCW;
        // PID Control of the motor
            encoderstate = 0;
            encoderCounter = 0;       
            while(encoderCounter < encCount){
              // let motor run till it stops
              driveArdumoto(MOTOR_B, CCW, motorspeed);                    
            }//end while  
            stopArdumoto(MOTOR_B); 
              
	   if (tuneON == 1){
              Serial.println("tunDN"); delay(5);  ble113.print("tunDN"); delay(5); }
           else { // tuneON = 0, normal mode.
              Serial.println("gotDN"); delay(5);  ble113.print("gotDN"); delay(5);
   	      gear = gear - 1;
	   }
            delay(1000);
            Serial.print("error= ");
            //Serial.println(error);
            Serial.print("mspeed= ");            Serial.println(motorspeed);            Serial.print("Target= ");            Serial.println(encCount);            Serial.print("encoderCounter= ");            Serial.println(encoderCounter);

           encoderstate = 0;
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

// Encoder A interrupt (Is ran on every rising and falling edge of encoder channel A)
void enc_A_ISR(void){
    int tc=0;
    currEncState = ((PIND & B00001100)>>2) | lastEncState;
    enc_case((int)currEncState);     
    lastEncState = (currEncState<<2) & 0x0C;   
}

// Encoder B interrupt (Is ran on every rising and falling edge of encoder channel B)
void enc_B_ISR(void){
    int tc = 0;
	currEncState = ((PIND & B00001100)>>2) | lastEncState;
    enc_case((int)currEncState);    
    lastEncState = (currEncState<<2) & 0x0C;         
}

// Encoder State Table
void enc_case(int state) {
  switch (state) {
        case 0:   //do nothing
          {  }
          break;
        case 1: 
          {encoderCounter++;
           }
          break;
        case 2: 
          {encoderCounter--;
           
          }break;
        case 3: 
          {
            if (dir = 1) {encoderCounter++;}
            else {encoderCounter--;} 
            
          }break;
        case 4: 
          {encoderCounter--;
           }break;
        case 5:   //do nothing
          {}break;
        case 6: 
          { 
            if (dir = 1) {encoderCounter++;}
            else {encoderCounter--;} 
          }break;
        case 7: 
          {encoderCounter++;
           }break;
        case 8: 
          {encoderCounter++;
           }break;
        case 9: 
          { 
            if (dir = 1) {encoderCounter++;}
            else {encoderCounter--;} 
          }break;
        case 10:   //do nothing
          { }break;
        case 11: 
          {encoderCounter--;
           }break;
        case 12: 
          { 
            if (dir = 1) {encoderCounter++;}
            else {encoderCounter--;}          
          }break;
        case 13: 
          {encoderCounter--;
           }break;
        case 14: 
          {encoderCounter++;
           }break;
        case 15:   //do nothing
          break;
        default:
          {}break;
      }
}

/*********************************************************************
* driveArdumoto drives 'motor' in 'dir' direction at 'spd' speed     *
**********************************************************************/
void driveArdumoto(byte motor, byte dir, byte spd)
{
    digitalWrite(DIRB, dir);
    analogWrite(PWMB, spd); 
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
  pinMode(PWMB, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);

  // Initialize all pins as low:
  digitalWrite(PWMB, LOW);
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, LOW);
}
/*********************************************************************
* SerialFromPC - used only to test sending from one pc to another to *
*                test BLE connectivity                               *
**********************************************************************/
String inPcString = "";
void SerialFromPC(void){
  // If Data from the PC serial console, submit it to the
  // BLE113 byte per byte. (1 byte at a time... no buffer)
  // this if loop cycles through once for each byte received from PC console.
  if (Serial.available()) { // if Serial buffer has stuff
      int chs = Serial.read();
      char ch = char(chs);
      inPcString += ch;

      if (ch == '\n'){ // signifies end of user entry
          Serial.println("received: "); 
          //Serial.print(inPcString);
          inPcString.trim();
          updateVariables(inPcString);
      }
  }
}
/**********************************************************************
* updateVariables - Allows updating variables from a string input.
***********************************************************************/
void updateVariables(String pidUpdate){
     Serial.println(pidUpdate);
     String whatVar = inPcString.substring(0,inPcString.indexOf('='));
     String sVal = (inPcString.substring(inPcString.indexOf('=')+1));
     int Value = sVal.toInt();
     if (whatVar == "Kp"){myKp = Value; myPID.SetTunings(myKp, myKi, myKd);}
     else if (whatVar == "Ki") {myKi = Value; myPID.SetTunings(myKp, myKi, myKd);}
     else if (whatVar == "Kd") {myKd = Value; myPID.SetTunings(myKp, myKi, myKd);}
     else if (whatVar == "encTarget") {encTarget = Value; encCount = Value;} 
     else if (whatVar == "encTuneTarget") {encTuneTarget = Value; encCount = Value;}
     else {Serial.println("Not a Variable");}
     inPcString="";
     Serial.print("GetKp: "); Serial.println(myPID.GetKp());
     Serial.print("GetKi: "); Serial.println(myPID.GetKi());
     Serial.print("GetKd: "); Serial.println(myPID.GetKd());
     Serial.print("encTarget: "); Serial.println(encTarget);
     Serial.print("encTuneTarget: "); Serial.println(encTuneTarget);
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
