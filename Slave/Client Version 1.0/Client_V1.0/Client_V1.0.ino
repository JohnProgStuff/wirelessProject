/*
John Peterson and Russell Mueller
Electrical Engineering Senior Project
Motor Control Program
*/
#include "PID_TRP_v2.h"
#include <SoftwareSerial.h>
//#include <avr/pgmspace.h>
#include <EEPROM.h>

/* Use Arduino pins 4 and 5 on the UNO for the Serial connection to the BLE modules,
*  RX -> D4 and TX -> D5*/
// Cal-eng board try D9 & D10 so that INT0/INT1 can be used with encoder
//SoftwareSerial ble113(4,5); 
SoftwareSerial ble113(9,10);
// Clockwise and counter-clockwise definitions.
// Depending on how you wired your motors, you may need to swap.**************
#define NUMBERofGEARS 11
#define CW 0
#define CCW 1

// Motor definitions to make life easier:*************************************
#define MOTOR_A 0
#define MOTOR_B 1





byte BoardUsed = 1; // 0 for Cal-Eng board, 1 for Ardumoto board






// Pin Assignments // byte = unsigned char ***********************************
// Don't change these! These pins are statically defined by shield layout
//const byte PWMA = 3;  // PWM control (speed) for motor A
const byte PWMB = 11; // PWM control (speed) for motor B
//const byte DIRA = 12; // Direction control for motor A
const byte DIRB = 13; // Direction control for motor B
const int pin8 = 8;

// Commands received from BUp signal from master
String startTune = "ST";  // start Tune signal from master
String endTune = "ET";    // end Tune signal from master
boolean tuneON = 0;       // tune mode on/off variable 
boolean command = 0;   // disable command received flag.

// Serial variables **********************************************************
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
//String pidUpdate = "";
double extra=0;

// Button debouncing variables: chars are signed values
char button_1_read, button_2_read;
char BTNS, BTNS_OLD, BTNS_temp, BTNS_STATE, BTNS_STATE_CHANGED;
char BTNS_Still_Down, OLD_STATE;

// Motor controls ************************************************************
int RunTime = 5; // the amount of milliseconds to run motor
int gear = 5; // the cog position starts at 1 goes to 11

//int numCogs = 11;
int motorFadeValue = 5;
int motorDelay = 1;
int encCount = 200;
int encTarget = 200;
int TuneCount = 15;
double motorspeed = 1;

// PID variables ******************************************************************
double myKp=2; // (P)roportional Tuning Parameter
double myKi=0; // (I)ntegral Tuning Parameter
double myKd=0; // (D)erivative Tuning Parameter


// Encoder signals*****************************************************************
double SetTarget = 10000;
volatile int EncoderA = HIGH; // INT0
volatile int EncoderB = HIGH; // INT1

// volatile unsigned int encoder Counter = 0;
double encoderCounter = 0;
bool dir = 0;
byte currEncState = 0x00;
byte lastEncState = 0x00;
// Array to save encoder position for each gear.
int shiftCount[NUMBERofGEARS] = {9579,9850,10100,10332,10590,10810,11100,11350,11550,11734,12065};
// all of these can be changed to chars?????? to save space
int eeprom_myKp_addr = 0;
int eeprom_myKi_addr = 1;
int eeprom_myKd_addr = 2;
int eeprom_gear = 3;
int eeprom_shiftCount_addr = 4;
int eeprom_encoderCounterHighByte_addr = 5;
int eeprom_encoderCounterLowByte_addr = 6;
int eeprom_Board_addr = 7;
                           
PID myPID(&encoderCounter, &motorspeed, &SetTarget, myKp, myKi, myKd, DIRECT);
//PID (INPUT, OUTPUT, SETTARGET,KP,KI,KD, DIRECT)
/**********************************************************************************
* START FUNCTION
**********************************************************************************/
void setup()
{
    Serial.begin(9600); //This pipes to the serial monitor
    Serial.print("Motor-Board Set As: ");
    if(BoardUsed == 0){Serial.println("0:Cal-Eng");}
    else if(BoardUsed == 1){Serial.println("1:Ardumoto");}
    Serial.print(F("BLE-Slave Sketch setup..."));
    //encoderCounter = EEPROM.read(eeprom_encoderCounterHighByte_addr);
    //Serial.println(encoderCounter);
    //encoderCounter = encoderCounter*256;
    //Serial.println(encoderCounter);
    //encoderCounter += EEPROM.read(eeprom_encoderCounterLowByte_addr);
    
    myKp = EEPROM.read(eeprom_myKp_addr)/100;
    myKi = EEPROM.read(eeprom_myKi_addr)/100;
    myKd = EEPROM.read(eeprom_myKd_addr)/100;
	  gear = EEPROM.read(eeprom_gear);
    //encoderCounter = 10800; // delete after testing
    //gear = 5; // delete after testing
    
    char i =0;
    Serial.println("In Gear: "); Serial.println(gear);
    encoderCounter = shiftCount[gear-1];
    Serial.println(encoderCounter);
    
    Serial.println(F("Current Gear values"));
    for (i=0; i < NUMBERofGEARS; i++){
      //shiftCount[i] = EEPROM.read(eeprom_shiftCount[i]);
      Serial.print("\rGear ");
      Serial.print(i+1); Serial.print(": ");
      Serial.println(shiftCount[i]);
    }
    ble113.begin(9600); // SoftwareSerial
    setupArdumoto(); // Set all pins as outputs
    digitalWrite(pin8, OUTPUT); // has min/max cog been reached LED
    // reserve 2 bytes for the inputString:
    inputString.reserve(2);
    pinMode(EncoderA, INPUT);
    pinMode(EncoderB, INPUT);

    //CAL-ENG Board Setup
      pinMode(7, OUTPUT);
      pinMode(A2, OUTPUT);
      pinMode(A3, OUTPUT);
      //pinMode(13, OUTPUT);
      //pinMode(5, OUTPUT); don't need
      digitalWrite(A2, LOW);
      digitalWrite(A3, LOW);
      //NEED TO SET PIN 7 HIGH FOR THE CALENG MOTOR DRIVER TO BE ENABLED
      digitalWrite(7, HIGH);

    // attachInterrupt(interrupt,ISR,mode)
    attachInterrupt(0, enc_A_ISR, CHANGE); //INT0=0 is pin D2
    attachInterrupt(1, enc_B_ISR, CHANGE); //INT1=1 is pin D3
    lastEncState = (PIND & B00001100);    
  //**SETTINGS FROM PID_v1 library**
  //  Input = analogRead(PIN_INPUT); //EncoderA testing
  // Turn the PID on
    myPID.SetMode(AUTOMATIC);
    Serial.println(" done.");
    Serial.println(F("\n**********************************************************************"
                   "\n* System Variables can be changed through the USB Serial input from  *"
                   "\n* a PC terminal COM port window (ex. \"Putty.exe\") The format to    *"
                   "\n* change the numeral value of variables is as follows.               *"
                   "\n* kp=1          or             ki=1.5            or            kd=3  *"
                   "\n*         shiftCount=231         or         TuneCount= 200           *"
                   "\n**********************************************************************"));
    Serial.print(F("encoderCounter: ")); Serial.println(encoderCounter);
   // Serial.print("Shift Count: "); Serial.println(shiftCount);
}

/**********************************************************************************
* PROGRAM LOOP
*_________________________________________________________________________________*/
void loop()
{

  
     //driveArdumoto(MOTOR_B, CW, motorspeed); // Set motor B to CCW at max  
     //motorspeed = 255;
     //while(1) { driveCalEng(1,0,200);  delay(1000);  driveCalEng(1,1,100);  delay(1000);                }//driveArdumoto(MOTOR_B, CW, motorspeed);} // Set motor B to CCW at max 
     SerialFromPC(); // Comment out if not using to shave a tiny bit of time.
     SerialFromBLE();
     
     if(isTuneCheck()){ // if so change variables - returns 1 if tuneOn=1
     }
     RunMotorWireless(); // comment out if wired is used.
     //LIGHT an LED if the end cogs have ben reached. maxUP/maxDN
     //if (gear == 1 || gear == numCogs){  digitalWrite(pin8, HIGH);     }
     //else{  digitalWrite(pin8, LOW);  }
     inputString = ""; // clear it here just in case it hasn't been cleared yet.
}// end main loop function
  
/*************************************************************************************
 * isTuneCheck() checks to see if the inputString from the SerialFromBLE() is
 *               instructing to enter/exit Tune Mode. And to change values accordingly
 *___________________________________________________________________________________*/
bool isTuneCheck(void){
  if ((inputString == "ST") || (inputString == "TS")) { // set Tune mode settings.
    inputString = "";
    tuneON = 1;
    Serial.println("gotST"); delay(10);
    ble113.print("gotST"); delay(10);
    //motorDelay = 1;   //motorFadeValue = 5;
    //shiftCount = TuneCount;
    return 1;
  }  
  if ((inputString == "ET") || (inputString == "TE")) { // reset to default
    inputString = "";
    Serial.println("gotET"); delay(10);
    ble113.print("gotET"); delay(10);
    //motorDelay = 5;   //motorFadeValue = 5;
   // if(encoderCounter < shiftCount)
    shiftCount[gear-1] = encoderCounter;
    EEPROM.write(eeprom_shiftCount_addr+gear,shiftCount[gear-1]);
    tuneON = 0; // turn off Tune mode.        
    //shiftCount = 200; encTarget;
    //motorspeed = 25;
    return 0;
  }
  else {return 0;}
}// end isTuneCheck

long lastTime, runTime;
char timeout = 0;
/***************************************************************************************
* RunMotorWireless() is the new way of running the motor for a specific amount of time and
*               checking BTNS. This replaced by the wired method. This runs the motor if
*               a specific char is received.
****************************************************************************************/
void RunMotorWireless(void){
    int dirCount = 0;
    if ((inputString == "UP") || (inputString == "PU")) { 
        inputString = "";  
        if (gear != NUMBERofGEARS || tuneON == 1){//if current gear isn't maxed and tunemode is off
            dir=CCW;//*********************************************************************
            
            if(gear > 14){ extra = 40;} else { extra = 0;}
            Serial.print("\nm_speed: "); Serial.println(motorspeed); 
            Serial.print("\nsetTarget: "); Serial.println(SetTarget); 
            //SetTarget = encoderCounter + shiftCount + extra;
            if (tuneON == 1){
                Serial.println("tunUP"); delay(5);  ble113.print("tunUP"); delay(5); 
                SetTarget = encoderCounter + TuneCount + extra;
            }
            else { // tuneON = 0, normal mode.
                Serial.println("gotUP"); delay(5);  ble113.print("gotUP"); delay(5);  
                gear++;
                Serial.print("goToGear: "); Serial.println(gear);
                if (gear > NUMBERofGEARS){gear=NUMBERofGEARS;}
                SetTarget = shiftCount[gear-1]; // current position + amount to shift    
            }
            bool dirM = dir;
		        int lastErr = 0;
            dirCount = 0;
            Serial.print("SetTarget: ");  Serial.println(SetTarget);
            runTime = millis();
            timeout=0;
            signed int error=SetTarget - encoderCounter;
            Serial.print("error: "); Serial.println(error);
            Serial.print("encoderCounter= ");            Serial.println(encoderCounter);
            Serial.println("\tRUN");
		        //while((!((encoderCounter <= SetTarget+3) & (encoderCounter >= SetTarget-3))) & timeout == 0) {
            while((error < -5 || error > 5) && timeout == 0){
                error = SetTarget - encoderCounter;
			          if ((lastErr > 0) && (error < 0)) {dirM = !dirM; dirCount++;}
			          if ((lastErr < 0) && (error > 0)) {dirM = !dirM; dirCount++;}
			          lastErr = error;
			          if (myPID.Compute() == 1){
                    //stopArdumoto(MOTOR_B); delay(10);
				            driveArdumoto(MOTOR_B, dirM, motorspeed);
                    // Serial.print(encoderCounter);   Serial.print("\t");  Serial.print(motorspeed);    
                    //Serial.print("\t");  Serial.println(dirM);
                    //Serial.println("C");
			          }
               //Serial.println(encoderCounter);
               lastTime = millis();
               if (((lastTime-runTime) > 1000) && (error > 5 || error < -5)){
                  Serial.println("timeout");
                  timeout = 1;
                  if(error > 100 || error < -100){ // our error is 
                      gear--; // put gear back where it was if still far away.  
                  }
               }
               
            }//end while
            stopArdumoto(MOTOR_B);
            EEPROM.write(eeprom_gear, gear);
            //EEPROM.write(eeprom_encoderCounterLowByte_addr, encoderCounter);
            //EEPROM.write(eeprom_encoderCounterHighByte_addr, encoderCounter/256);

            delay(500);
            Serial.print("mspeed= ");            Serial.println(motorspeed);  
            Serial.print("Target= ");            Serial.println(SetTarget);     
            Serial.print("encoderCounter= ");            Serial.println(encoderCounter);
            Serial.print("C_Gear: "); Serial.println(gear);
            Serial.print("dirM: "); Serial.println(dirCount, DEC);

            
        }// end gear if
        if (gear == NUMBERofGEARS){ 
            Serial.print("maxUP"); delay(10);  ble113.print("maxUP"); delay(10);  
        }
    } // END IF UP
    // IF SHIFT_DN -> Clock-Wise Direction##############
    if ((inputString == "DN")||(inputString == "ND")) {
        inputString = "";
        if (gear != 1 || tuneON == 1) {
            dir=CW;
            if(gear <10){ extra = 30;} else { extra = 0;}
			      //SetTarget = encoderCounter - shiftCount-extra; // current position + amount to shift    
            
            if (tuneON == 1){
                Serial.println("tunDN"); delay(5);  ble113.print("tunDN"); delay(5); 
			          SetTarget = encoderCounter - TuneCount-extra;
            }
			      else { // tuneON = 0, normal mode.
                Serial.println("gotDN"); delay(5);  ble113.print("gotDN"); delay(5);
                gear--;
                if (gear <= 0){gear=1;}
                Serial.print(F("goToGear: ")); Serial.println(gear);
                SetTarget = shiftCount[gear-1]; // current position + amount to shift    
			      }
            bool dirM = dir;
            int lastErr = 0;
            dirCount = 0;
            runTime = millis();
            timeout = 0;
            lastTime = 0;
            Serial.print(F("SetTarget: "));  Serial.println(SetTarget);
            signed int error = SetTarget - encoderCounter;
            Serial.print("error: "); Serial.println(error);
            Serial.print("encoderCounter= ");            Serial.println(encoderCounter);
            Serial.println("\tRUN");
            //while((!((encoderCounter <= SetTarget+3) & (encoderCounter >= SetTarget-3))) & (timeout == 0)) {
     	      while((error < -3 || error > 3) && timeout == 0){
     	          error = SetTarget - encoderCounter;
     	          
                if ((lastErr > 0) && (error < 0)) {dirM = !dirM; dirCount++;}
                if ((lastErr < 0) && (error > 0)) {dirM = !dirM; dirCount++;}
                lastErr = error;
                if (myPID.Compute() == 1){
                    driveArdumoto(MOTOR_B, dirM, motorspeed);
                    //Serial.print(encoderCounter);   Serial.print("\t");  Serial.print(motorspeed); Serial.print("\t");  Serial.print(SetTarget);   
                    //Serial.print("\t");  Serial.println(dirM);
                }
                lastTime = millis();
                //Serial.println(lastTime-runTime);
                if (((lastTime-runTime) > 400) && (error > 3 || error < -3)){
                    Serial.println("timeout");
                    timeout = 1;
                    if(error > 100 || error < -100){
                        if (gear != 1) {gear++;} // put gear back where it was if still far away.  
                    }
               }

            }//end while  
            stopArdumoto(MOTOR_B); 
            EEPROM.write(eeprom_gear, gear);
            //EEPROM.write(eeprom_encoderCounterLowByte_addr, encoderCounter);
            //EEPROM.write(eeprom_encoderCounterHighByte_addr, (encoderCounter)/256);
            
            delay(500);
            Serial.print("mspeed= ");            Serial.println(motorspeed);            Serial.print("Target= ");            Serial.println(SetTarget);            Serial.print("encoderCounter= ");            Serial.println(encoderCounter);
            Serial.print("C_Gear: "); Serial.println(gear);
            Serial.print("dirM: "); Serial.println(dirCount, DEC);
        } // end if
        if(gear == 1 && tuneON != 1){ Serial.print("maxDN"); delay(10);   ble113.print("maxDN"); delay(10);  }       
    } // END IF DOWN
}//end function

/*****************************************************************************
* ISR A & B for Encoder 
*     The following code needs to be in the Setup loop:
*     // attachInterrupt(interrupt,ISR,mode)
*     attachInterrupt(0, encoderISR, RISING) //INT0=0 is pin D2x
*     attachInterrupt(1, encoderISR, RISING) //INT1=1 is pin D1
******************************************************************************/
void enc_A_ISR(void){
    currEncState = ((PIND & B00001100)>>2) | lastEncState;
    //Serial.print((int)currEncState);
    enc_case((int)currEncState);     
    lastEncState = (currEncState<<2) & 0x0C;
//  Serial.println(encoderCounter);
}
void enc_B_ISR(void){
    currEncState = ((PIND & B00001100)>>2) | lastEncState;
    //Serial.print((int)currEncState);
    enc_case((int)currEncState);       
    lastEncState = (currEncState<<2) & 0x0C;
//  Serial.println(encoderCounter);
}
void enc_case(int state) {
    int wCase = 50;
    switch (state) {
        case 0:   //do nothing
            wCase = 0;
            break;
        case 1: 
            {encoderCounter++; wCase = 1;
            break;}
        case 2: 
            {encoderCounter--;   wCase = 2;  
            break;}
        case 3: 
            {if (dir = 1) {encoderCounter++;encoderCounter++;}
            else {encoderCounter--;encoderCounter--;} wCase = 3;
            break;}
        case 4: 
            {encoderCounter--; wCase = 4;
            break;}
        case 5:   //do nothing
            wCase = 5;
            break;
        case 6: 
            {if (dir = 1) {encoderCounter++;encoderCounter++;}
            else {encoderCounter--;encoderCounter--;} wCase = 6;
            break;}
        case 7: 
            {encoderCounter++; wCase = 7;
            break;}
        case 8: 
            {encoderCounter++; wCase = 8;
            break;}
        case 9: 
            {if (dir = 1) {encoderCounter++;encoderCounter++;}
            else {encoderCounter--;encoderCounter--;} wCase = 9;
            break;}
        case 10:   //do nothing
            wCase = 10;break;
        case 11: 
            {encoderCounter--; wCase = 11;
            break;}
        case 12: 
            {if (dir = 1) {encoderCounter++;encoderCounter++;}
            else {encoderCounter--;encoderCounter--;} wCase = 12;          
            break;}
        case 13: 
            {encoderCounter--; wCase = 13;
            break;}
        case 14: 
            {encoderCounter++; wCase = 14;
            break;}
        case 15:   //do nothing
            wCase = 15;
            break;
        default:
            break;
        }
//      Serial.print("\t");
//      Serial.println(wCase);//encoderCounter);
}

/*********************************************************************
* driveArdumoto drives 'motor' in 'dir' direction at 'spd' speed     *
**********************************************************************/
void driveArdumoto(byte motor, byte dir, byte spd)
{
  if (BoardUsed == 0){driveCalEng(motor, dir, spd);} // using CalEng board
  else if (BoardUsed == 1){
    //if (motor == MOTOR_A){ digitalWrite(DIRA, dir);  analogWrite(PWMA, spd);}
    // else if (motor == MOTOR_B){
    digitalWrite(DIRB, dir); //pin13=DIRB
    analogWrite(PWMB, spd); //pin11=PWMB  //}  
  }
  else{
    while(1){ Serial.println("BoardUsed != 0 or 1");}
  }
}
void driveCalEng(byte motor, byte dir, byte spd){ //dir is 0 or 1 //spd=0-255
    digitalWrite(A2, dir); //pin13=DIRB
    digitalWrite(A3, !dir); //pin13=DIRB
    analogWrite(5, spd); //pin5=PWMB
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
{// All pins should be setup as outputs:
  pinMode(PWMB, OUTPUT);
  pinMode(DIRB, OUTPUT);
  // Initialize all pins as low:
  digitalWrite(PWMB, LOW);
  digitalWrite(DIRB, LOW);
  //  pinMode(PWMA, OUTPUT);  //pinMode(DIRA, OUTPUT);  //digitalWrite(PWMA, LOW); //digitalWrite(DIRA, LOW);
}
/*********************************************************************
* SerialFromPC - used only to test sending from one pc to another to *
*                test BLE connectivity                               *
**********************************************************************/
String inPcString = "";
int chs;
char ch;
void SerialFromPC(void){
  // If Data from the PC serial console, submit it to the
  // BLE113 byte per byte. (1 byte at a time... no buffer)
  // this if loop cycles through once for each byte received from PC console.
  //if
  while(Serial.available()) { // if Serial buffer has stuff
      chs = Serial.read();
      ch = char(chs);
      //ble113.write(ch); // send to module
      //Serial1.write(ch);  
      inPcString += ch;
      //Serial.print(inPcString); // echo each char back to PC
      Serial.print(ch); // echo each char back to PC
      Serial.println();
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
int Value=0;
void updateVariables(String pidUpdate){
     //Serial.println(pidUpdate);
     String whatVar = inPcString.substring(0,inPcString.indexOf('='));
     String sVal = (inPcString.substring(inPcString.indexOf('=')+1));
     //String whole;// = (sVal.substring(0,sVal.indexOf('.')));
     //String dec = "";
     //double dvalue = 0;
     Value = 0;
     /*if (sVal.indexOf('.') > 0){ 
        whole = (sVal.substring(0,sVal.indexOf('.')));
        dec = (sVal.substring(sVal.indexOf('.')+1));
        int idec = dec.toInt();
        if(idec > 1000){dvalue = idec/1000;}
        else if(idec > 100){dvalue = idec/100;}
        else if(idec > 10){dvalue = idec/10;}
        else dvalue = idec;
        dvalue += whole.toInt();
        Serial.print("\r\ndvalue: "); Serial.println(dvalue); 
     }
     else{
        Value = sVal.toInt();
     }*/
     Value = sVal.toInt();
     Serial.print(whatVar);
       
//     Serial.print(" = ");
//     Serial.println(Value);
     if ((whatVar == "Kp") || (whatVar == "kp")){myKp = Value; myPID.SetTunings(myKp, myKi, myKd);}
     else if ((whatVar == "Ki") || (whatVar == "ki")) {myKi = Value; myPID.SetTunings(myKp, myKi, myKd);}
     else if ((whatVar == "Kd") || (whatVar == "kd")) {myKd = Value; myPID.SetTunings(myKp, myKi, myKd);}
     else if ((whatVar == "TuneCount") || (whatVar == "tunecount")) {TuneCount = Value; if(tuneON==1){/*shiftCount = Value;*/ Serial.println("Tuning Mode ON");}} //shiftCount = Value;}
     else if ((whatVar == "gear") || (whatVar == "Gear")){gear = Value; encoderCounter = shiftCount[gear-1]; EEPROM.write(eeprom_gear, gear);}
     else if ((whatVar == "g1") || (whatVar == "G1")){shiftCount[0] = Value;}
     else if ((whatVar == "g2") || (whatVar == "G2")){shiftCount[1] = Value;}
     else if ((whatVar == "g3") || (whatVar == "G3")){shiftCount[2] = Value;}
     else if ((whatVar == "g4") || (whatVar == "G4")){shiftCount[3] = Value;}
     else if ((whatVar == "g5") || (whatVar == "G5")){shiftCount[4] = Value;}
     else if ((whatVar == "g6") || (whatVar == "G6")){shiftCount[5] = Value;}
     else if ((whatVar == "g7") || (whatVar == "G7")){shiftCount[6] = Value;}
     else if ((whatVar == "g8") || (whatVar == "G8")){shiftCount[7] = Value;}
     else if ((whatVar == "g9") || (whatVar == "G9")){shiftCount[8] = Value;}
     else if ((whatVar == "g10") || (whatVar == "G10")){shiftCount[9] = Value;}
     else if ((whatVar == "g11") || (whatVar == "G11")){shiftCount[10] = Value;}
     else if ((whatVar == "Board") || (whatVar == "board")){BoardUsed = Value;}
     else if ((whatVar == "enc") || (whatVar == "Enc"))
     {  encoderCounter = Value;             
        EEPROM.write(eeprom_encoderCounterLowByte_addr, encoderCounter);
        EEPROM.write(eeprom_encoderCounterHighByte_addr, (encoderCounter)/256);
     }
     
     else {Serial.println(F("Not a Variable"));}
     //kvalues[0] = myKp;
     //kvalues[1] = myKi;
     //kvalues[2] = myKd;
     inPcString="";
     Serial.println(F("****** VARIABLE VALUES ***********"));
     Serial.print("GetKp: "); Serial.println(myPID.GetKp());
     Serial.print("GetKi: "); Serial.println(myPID.GetKi());
     Serial.print("GetKd: "); Serial.println(myPID.GetKd());
     //Serial.print("shiftCount: "); Serial.println(shiftCount);
     Serial.print("TuneCount: "); Serial.println(TuneCount);
	   Serial.print("Gear: "); Serial.println(gear);
         char i =0;
    Serial.println(F("Current Gear values"));
    for (i=0; i < NUMBERofGEARS; i++){
      //shiftCount[i] = EEPROM.read(eeprom_shiftCount[i]);
      Serial.print("\rGear ");
      Serial.print(i+1); Serial.print(": ");
      Serial.println(shiftCount[i]);
    }
    i=0;
    Serial.print(F("encoderCounter: ")); Serial.println(encoderCounter);
    // Load each new encoder value to eeprom
    for (i=0;i<NUMBERofGEARS;i++){
     EEPROM.write(eeprom_shiftCount_addr+i, shiftCount[i]);
    }
     EEPROM.write(eeprom_myKp_addr, myKp*100);
     EEPROM.write(eeprom_myKi_addr, myKi*100);
     EEPROM.write(eeprom_myKd_addr, myKd*100);
     
     
	 
     
}
char count=0;
long ptime = 0;
/*********************************************************************
* SerialFromBLE                                                      *
**********************************************************************/
/*
 * void SerialFromBLE(){
  // if BLE module has sent info. send it to the serial console.
  //if ((millis()-ptime) > 200)
  {inputString = "\n"; inputString.trim();}
  if (ble113.available()) {
      count = 0;
        while(count != 2){
          if(ble113.available()) {
            char inChar = (char)ble113.read();
            inputString += inChar;
            count++;
            delay(10);
            //Serial.write(inChar);
          }
        }
        inputString.trim();
      if(inputString == "ZZ"){
        command = 1;
        Serial.print("COMMAND: ");
        count = 0;
        inputString = "\n"; inputString.trim();
        //Serial.print(inputString.length());
        while(count != 2){
          if(ble113.available()) {
            char inChar = (char)ble113.read();
            inputString += inChar;
            count++;
            delay(10);
          }
        }
        
        Serial.print(inputString);
      }
      else{
        Serial.print(inputString);
      }
      count = 0;      
  }
}*/
void SerialFromBLE(){
  // if BLE module has sent info. send it to the serial console.
  if (ble113.available()) {
      while(inputString.length() < 2){
        if(ble113.available()) {
            char inChar = (char)ble113.read();
            inputString += inChar;
        }
      }
      Serial.print(inputString);
      inputString.trim(); // take off any '\n' and whitespace trailing string.
      if (inputString.length() > 2){ inputString = "";}  //
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
