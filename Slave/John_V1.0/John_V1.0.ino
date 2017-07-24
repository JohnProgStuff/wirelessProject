/* TRP & WSU Concept Center
 *  Version 1.1
 *  with V1.1 we have switched completely to the Purple board
 *  that was designed by John and Russ for their senior project
 * 
*/
#include "PID_TRP_v2.h" // "" for same folder as arduino 
#include <SoftwareSerial.h> // < > in compiler
#include <EEPROM.h> // enable writing to non-volatile memory

SoftwareSerial ble113(9,10); // pin 9 and 10 are serial communication between ble113 and atmel chip
// Depending on how you wired your motors, you may need to swap.**************
#define NUMBERofGEARS 11
#define CW 0     // Clockwise  
#define CCW 1    // Counter-Clockwise
// Motor definitions to make life easier:*************************************
#define MOTOR_B 1

// Values that may change regularly
byte first_RUN = 1; // 1 to overwrite flash on 1st run, 0 to leave eeprom alone.
int gear = 6; // the cog position starts at 1 goes to 11
int timeoutValue = 1000; // time in milliseconds motor has to reach position.


// Pin Assignments // byte = unsigned char ***********************************
const byte PWMB = 11; // PWM pin to control (speed) for motor B , arduino pin
const byte DIRB = 13; // Direction control for motor B , pin 13 control direction

// Commands received from BLE signal from master
boolean tuneON = 0;       // tune mode on/off variable 

// Serial variables **********************************************************
String inputString = "";         // a string to hold incoming data from the ble.

byte dontTune = 0; // 0=yes to shift/tune 1=do not Tune, skips motor while loop
int TuneCount = 15; // the amount of pulse motor move during tune mode
double motorspeed = 1; // speed of motor change by PID libary 


// PID variables for PID libary**************************************************
double myKp=2; // (P)roportional Tuning Parameter
double myKi=1; // (I)ntegral Tuning Parameter
double myKd=0; // (D)erivative Tuning Parameter

// Encoder signals*****************************************************************
double SetTarget = 10000; // encoder target position
volatile int EncoderA = HIGH; // INT0 aka pin 2
volatile int EncoderB = HIGH; // INT1 aka pin 3

double encoderPosition = 0; // position of motor
bool dir = 0; // direction of motor
byte currEncState = 0x00;  // use by encoder interupt routine
byte lastEncState = 0x00; // use by encoder interupt routine

// Array to save encoder position for each gear 1 - 11
int shiftCount[NUMBERofGEARS] = {9579,9850,10100,10332,10590,10810,11100,11350,11550,11734,12065};

//saving variables in EEPROM requires that we remember the address of each variable
//so these variables store the address of the variable in EEPROM
int eeprom_myKp_addr = 0;
int eeprom_myKi_addr = 1;
int eeprom_myKd_addr = 2;
int eeprom_gear = 3;
int eeprom_firstrun_addr = 4;
int eeprom_Board_addr = 7;
int eeprom_gear_offset = 8; // offset for whole gears
int eeprom_shiftCount_addr = 10; // uses 2 bytes for every number(11) so 22 bytes
// next available byte will be 32


int offset = 0; // sram variable for offseting all shift gear postions.
                           
PID myPID(&encoderPosition, &motorspeed, &SetTarget, myKp, myKi, myKd, DIRECT);
//PID (INPUT, OUTPUT, SETTARGET,KP,KI,KD, DIRECT)

/**********************************************************************************
* START FUNCTION
**********************************************************************************/
void setup()
{	//set up baud rate for ble and comp
    ble113.begin(9600); // SoftwareSerial for ble
    Serial.begin(9600); //This pipes to the pc serial monitor 
              
    Serial.println(F("Derailer setup...")); // F() stores in Flash memory
    myKp = EEPROM.read(eeprom_myKp_addr)/100; // divide by 100 to convert back to double.
    myKi = EEPROM.read(eeprom_myKi_addr)/100; // might not be working, leave it in.
    myKd = EEPROM.read(eeprom_myKd_addr)/100;

    Serial.println(F("Current Gear values")); 
    firstRun(); // check if gear position are in EEPROM 

    gear = EEPROM.read(eeprom_gear);
    Serial.print("In Gear: "); Serial.println(gear, DEC);
    encoderPosition = shiftCount[gear-1];
    Serial.print(F("encoderPosition: ")); Serial.println(encoderPosition);
   
    setupArdumoto(); // Set all pins as outputs

    // reserve 2 bytes for the inputString:
    inputString.reserve(2);
    pinMode(EncoderA, INPUT);  
    pinMode(EncoderB, INPUT);

    // attachInterrupt(interrupt,ISR,mode)
    attachInterrupt(0, enc_A_ISR, CHANGE); //INT0=0 is pin D2
    attachInterrupt(1, enc_B_ISR, CHANGE); //INT1=1 is pin D3
    lastEncState = (PIND & B00001100);    
  //**SETTINGS FROM PID_v1 library**
    myPID.SetMode(AUTOMATIC); // Turn the PID on
    Serial.println(" setup done.");
    Serial.println(F("\n**********************************************************************"
                   "\n* System Variables can be changed through the USB Serial input from  *"
                   "\n* a PC terminal COM port window (ex. \"Putty.exe\") The format to    *"
                   "\n* change the numeral value of variables is as follows. lowercase     *"
                   "\n* kp=1          or             ki=1.5            or            kd=3  *"
                   "\n*               gear=5         or         tunecount= 200             *"
                   "\n**********************************************************************"));
} // end setup

/**********************************************************************************
* PROGRAM LOOP
*_________________________________________________________________________________*/
void loop()
{
     SerialFromPC(); // Comment out if not using to shave a tiny bit of time.
     SerialFromBLE();   // check for serial communication  
     isTuneCheck(); // if so change variables - returns 1 if tuneOn=1
     MotorUPorDN(); // comment out if wired is used.
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
    Serial.print("Tuning gear: "); Serial.println(gear);
    return 1;
  }  
  if ((inputString == "ET") || (inputString == "TE")) { // reset to default
    inputString = "";
    Serial.println("gotET"); delay(10);
    ble113.print("gotET"); delay(10);
    shiftCount[gear-1] = encoderPosition;
	// EEPROM.write(address to write to,value to write)
    EEPROM.write(eeprom_shiftCount_addr+(gear-1)*2,shiftCount[gear-1]); // 2 bytes for each gear position (2*11 = 22)
    tuneON = 0; // turn off Tune mode.        
    Serial.print("Gear "); Serial.print(gear); Serial.print(" Tuned to "); Serial.println(shiftCount[gear-1]);
    return 0;
  }
  else {return 0;}
}// end isTuneCheck

// variables for MotorUPorDN
long lastTime, runTime;
char timeout = 0;
int dirCount = 0;
int error = 0;
int firstError = 0;
int lastErr = 0;
/***************************************************************************************
* MotorUPorDN() is the way of running the motor for a specific amount of time and
*         checking BTNS. This replaced by the wired method. This runs the motor if
*         a specific char is received.
****************************************************************************************/
void MotorUPorDN(void){

    if ((inputString == "UP") || (inputString == "PU")) { 
        inputString = "";  
        if (gear != NUMBERofGEARS || tuneON == 1){//if current gear isn't maxed and tunemode is off
            dir=CCW;
            if (tuneON == 1){
                Serial.println("tunUP"); delay(5);  ble113.print("tunUP"); delay(5); 
                SetTarget = encoderPosition + TuneCount;
                if (!tunePreventCheck(SetTarget)){ // if it returns 1 its good to tune else not
                  dontTune = 1;// we don't want to run the motor
                }
            }
            else { // tuneON = 0, normal mode.
                Serial.println("gotUP"); delay(5);  ble113.print("gotUP"); delay(5);  
                gear++;
                Serial.print("goToGear: "); Serial.println(gear);
                if (gear > NUMBERofGEARS){gear=NUMBERofGEARS;}
                SetTarget = shiftCount[gear-1]; // current position + amount to shift    
            }
          if (!dontTune){ // Run motor if dontTune wasn't flagged.
            RUN_Motor();
            if(error > 100 || error < -100){ // our error is 
                gear--; // put gear back where it was if still far away.  
            }
            EEPROM.write(eeprom_gear, gear);
          }
		  else {
			  dontTune = 0; // skipped running motor, now clear flag
		  }
            afterRUNprint();
        }// end gear if
        if (gear == NUMBERofGEARS){ Serial.print("maxUP"); delay(10);  ble113.print("maxUP"); delay(10);  
        }
    } // END IF UP
    if ((inputString == "DN")||(inputString == "ND")) {
        inputString = "";
        if (gear != 1 || tuneON == 1) {
            dir=CW;          
            if (tuneON == 1){
                Serial.println("tunDN"); delay(5);  ble113.print("tunDN"); delay(5); 
                    SetTarget = encoderPosition - TuneCount;
                if (!tunePreventCheck(SetTarget)){ // if it returns 1 its good to tune else not
                    dontTune = 1;// we don't want to run the motor
                }
            }
			else { // tuneON = 0, normal mode.
                Serial.println("gotDN"); delay(5);  ble113.print("gotDN"); delay(5);
                gear--;
                if (gear <= 0){ gear=1; }
                Serial.print(F("goToGear: ")); Serial.println(gear);
                SetTarget = shiftCount[gear-1]; // current position + amount to shift    
			}
            if (!dontTune){ // if it returns 1 its good to tune else not
                RUN_Motor();			
                if(error > 100 || error < -100){
                    // if (gear != 1) 
                    {gear++;} // put gear back where it was if still far away.  
                }
                EEPROM.write(eeprom_gear, gear);
            }// end if     
            else {
                dontTune = 0; // skipped running motor, now clear flag
            }    
            afterRUNprint();
        } // end if
        if(gear == 1 && tuneON != 1){ Serial.print("maxDN"); delay(10);   ble113.print("maxDN"); delay(10);  }       
    } // END IF DOWN
}//end function

void RUN_Motor(void){
    bool dirM = dir;
    lastErr = 0;
    dirCount = 0;
    runTime = millis();
    timeout=0;
    lastTime = 0;
    Serial.print(F("SetTarget: "));  Serial.println(SetTarget);
    error = SetTarget - encoderPosition;
    firstError = error;
    Serial.print("encoderPosition= ");            Serial.println(encoderPosition);
    Serial.print("error: "); Serial.println(error);
    Serial.print("L.error: "); Serial.println(lastErr);
  
    Serial.print("\tRUN");
	if (encoderPosition > SetTarget){
		dirM =  CW;
	}
	else dirM = CCW;
	while((error < -5 || error > 5) && timeout == 0){
		error = SetTarget - encoderPosition;
    //Serial.print("CE "); Serial.print(error);
    //Serial.print("\tLe "); Serial.println(lastErr);
        if ((lastErr > 0) && (error < 0)) {dirM = !dirM; dirCount++; Serial.println(dirM);}
        if ((lastErr < 0) && (error > 0)) {dirM = !dirM; dirCount++; Serial.println(dirM);}
		lastErr = error;
        if (myPID.Compute() == 1){
            driveArdumoto(MOTOR_B, dirM, motorspeed);
        }
		lastTime = millis();
        //Serial.println(lastTime-runTime);
        if (((lastTime-runTime) > timeoutValue) && (error > 5 || error < -5)){
            Serial.print(" timeout");
            timeout = 1;
            break; // exit while loop
        }
	}
	driveArdumoto(MOTOR_B, 0, 0); // slightly quicker than stopArdumoto(MOTOR_B);
} // end RUN_Motor

/*************************************************************************************
* afterRUNprint -  prints stuff to PC after the motor runs. We can elimate this delay
*                  when we are comfortable the motor is going to the right location 
*                  consistently.
**************************************************************************************/
void afterRUNprint(void){
	delay(500);
    Serial.print("\n\rm.speed= ");            Serial.println(motorspeed);
    Serial.print("Target= ");                 Serial.println(SetTarget);
    Serial.print("encoderPosition= ");        Serial.println(encoderPosition);
    Serial.print("C_Gear: "); Serial.println(gear);
    Serial.print("dirChange: "); Serial.println(dirCount);
}


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
    short_enc_case((int)currEncState);     
    lastEncState = (currEncState<<2) & 0x0C;
//  Serial.println(encoderPosition);
}
void enc_B_ISR(void){
    currEncState = ((PIND & B00001100)>>2) | lastEncState;
    //Serial.print((int)currEncState);
    short_enc_case((int)currEncState);       
    lastEncState = (currEncState<<2) & 0x0C;
//  Serial.println(encoderPosition);
}
/****************************************************************
This funtion keep track of the postion of our encoder within the state machine
There are 4 stages (00 -> 01 -> 11 -> 10) 
******************************************************************/
void short_enc_case(int state){
  switch (state){
    // do nothing cases - state never changed // 0000, 0101, 1010, 1111
       case 0: case 5: case 10: case 15: 
            // DO NOTHING
            break;
    // add 1 cases // 0001, 0111, 1000, 1110
       case 1: case 7: case 8: case 14:
            {encoderPosition++;}
            break;
    // subtract 1 cases // 0010, 0100, 1011, 1101
       case 2: case 4: case 11: case 13: 
            {encoderPosition--;}
            break;
    // add 2 or subtract 2 cases // 0011, 0110, 1001, 1100
       case 3: case 6: case 9: case 12:  
            {if (dir == 1) {encoderPosition++;encoderPosition++;}
            else {encoderPosition--;encoderPosition--;}
            break;}
       default:
            break;
  }
}


/*********************************************************************
* driveArdumoto drives 'motor' in 'dir' direction at 'spd' speed     *
**********************************************************************/
void driveArdumoto(byte motor, byte dir, byte spd)
{
    digitalWrite(DIRB, dir); //pin13=DIRB
    analogWrite(PWMB, spd); //pin11=PWMB  //}  
}

/*********************************************************************
* stopArdumoto - stops motor (BRAKE)                                 *
**********************************************************************/
void stopArdumoto(byte motor){  driveArdumoto(motor, 0, 0); }

/*********************************************************************
* setupArdumoto initialize all pins                                  *
**********************************************************************/

void setupArdumoto()
{// All pins should be setup as outputs:
  pinMode(PWMB, OUTPUT);  //pin 11
  pinMode(DIRB, OUTPUT); //pin 13
  // Initialize all pins as low:
  digitalWrite(PWMB, LOW); // pin 11
  digitalWrite(DIRB, LOW); //pin 13
  //  pinMode(PWMA, OUTPUT);  //pinMode(DIRA, OUTPUT);  //digitalWrite(PWMA, LOW); //digitalWrite(DIRA, LOW);
}

/*********************************************************************
* SerialFromPC - used only to test sending from one pc to another to *
*                test BLE connectivity                               *
**********************************************************************/
String inPcString = "";
int chs;
char ch;
// pull in G then E , then A .. (Gear) as long as there are serial available
void SerialFromPC(void){
  while(Serial.available()) { // if Serial buffer has stuff (from pc)
      chs = Serial.read();
      ch = char(chs); // // convert to char from int
      inPcString += ch; 
      //Serial.print(inPcString); // echo each char back to PC
      Serial.print(ch); // echo each char back to PC
      //Serial.println();
      if (ch == '\n'){ // signifies end of user entry (enter key)
          Serial.println("received: "); 
          //Serial.print(inPcString);
          inPcString.trim(); // trim white space
          updateVariables(inPcString); // check to see if string is a variable
      }
  }
}

/**********************************************************************
* updateVariables - Allows updating variables from a string input.
***********************************************************************/
int Value=0;
void updateVariables(String varUpdate){ // not sure why this is using inpcstring instead of pidUpdate
     char i =0;
     
     String whatVar = inPcString.substring(0,inPcString.indexOf('='));  // look for = sign
     String sVal = (inPcString.substring(inPcString.indexOf('=')+1));
     Value = 0;
     
     Value = sVal.toInt(); // toint covert string value to int
     Serial.print(whatVar);
       
     if ((whatVar == "Kp") || (whatVar == "kp")){
        myKp = Value;     myPID.SetTunings(myKp, myKi, myKd); 
        EEPROM.write(eeprom_myKp_addr, myKp*100);}
     else if ((whatVar == "Ki") || (whatVar == "ki")) {
        myKi = Value;     myPID.SetTunings(myKp, myKi, myKd); 
        EEPROM.write(eeprom_myKi_addr, myKi*100);}
     else if ((whatVar == "Kd") || (whatVar == "kd")) {
        myKd = Value;     myPID.SetTunings(myKp, myKi, myKd); 
        EEPROM.write(eeprom_myKd_addr, myKd*100);}
     else if ((whatVar == "TuneCount") || (whatVar == "tunecount")) {TuneCount = Value; if(tuneON==1){ Serial.println("Tuning Mode ON");}} //shiftCount = Value;}
     else if ((whatVar == "gear") || (whatVar == "Gear")){gear = Value;    encoderPosition = shiftCount[gear-1];    EEPROM.write(eeprom_gear, gear);}
     else if ((whatVar == "g1") || (whatVar == "G1"))   {shiftCount[0]  = Value;    EEPROM.put(eeprom_shiftCount_addr+0,  Value);}
     else if ((whatVar == "g2") || (whatVar == "G2"))   {shiftCount[1]  = Value;    EEPROM.put(eeprom_shiftCount_addr+2,  Value);}
     else if ((whatVar == "g3") || (whatVar == "G3"))   {shiftCount[2]  = Value;    EEPROM.put(eeprom_shiftCount_addr+4,  Value);}
     else if ((whatVar == "g4") || (whatVar == "G4"))   {shiftCount[3]  = Value;    EEPROM.put(eeprom_shiftCount_addr+6,  Value);}
     else if ((whatVar == "g5") || (whatVar == "G5"))   {shiftCount[4]  = Value;    EEPROM.put(eeprom_shiftCount_addr+8,  Value);}
     else if ((whatVar == "g6") || (whatVar == "G6"))   {shiftCount[5]  = Value;    EEPROM.put(eeprom_shiftCount_addr+10, Value);}
     else if ((whatVar == "g7") || (whatVar == "G7"))   {shiftCount[6]  = Value;    EEPROM.put(eeprom_shiftCount_addr+12, Value);}
     else if ((whatVar == "g8") || (whatVar == "G8"))   {shiftCount[7]  = Value;    EEPROM.put(eeprom_shiftCount_addr+14, Value);}
     else if ((whatVar == "g9") || (whatVar == "G9"))   {shiftCount[8]  = Value;    EEPROM.put(eeprom_shiftCount_addr+16, Value);}
     else if ((whatVar == "g10") || (whatVar == "G10")) {shiftCount[9]  = Value;    EEPROM.put(eeprom_shiftCount_addr+18, Value);}
     else if ((whatVar == "g11") || (whatVar == "G11")) {shiftCount[10] = Value;    EEPROM.put(eeprom_shiftCount_addr+20, Value);}
     else if ((whatVar == "enc") || (whatVar == "Enc")) {  encoderPosition = Value;}
     else if (whatVar == "offset")  {   offset = Value;    EEPROM.write(eeprom_gear_offset, offset);
      i=0; // reset for next loop to use
      for (i=0; i < NUMBERofGEARS; i++){        
        shiftCount[i] += offset;
        save_shiftCount(); // save the whole table to EEPROM
      }
    }
     else {Serial.println(F("Not a Variable"));}
     inPcString="";
     print_settings();
}


void print_settings(void){
    Serial.println(F("****** VARIABLE VALUES ***********"));
    Serial.print("GetKp: "); Serial.println(myPID.GetKp());
    Serial.print("GetKi: "); Serial.println(myPID.GetKi());
    Serial.print("GetKd: "); Serial.println(myPID.GetKd());
    Serial.println(F("Current Gear values")); //*************************************
    char i = 0;
    for(i=0; i < NUMBERofGEARS; i++)
    {
       Serial.print("\rGear "); Serial.print(i+1); Serial.print(": \t");
       Serial.println(EEPROM.get(eeprom_shiftCount_addr+i*2, shiftCount[i]));
       //eeprom commands average 3.3 milliseconds
    }
    Serial.print("Current Gear: "); Serial.println(gear);
    Serial.print(F("encoderPosition: ")); Serial.println(encoderPosition);
    Serial.print("TuneCount: "); Serial.println(TuneCount);
}

void SerialFromBLE(){
  // if BLE module has sent info. send it to the serial console.
  if (ble113.available()) {
      int lastTime = millis();
      int now = 0;
      while(inputString.length() < 2){ //problem is this holds serialFromPC from running.
        if(ble113.available()) {
            char inChar = (char)ble113.read();
            inputString += inChar;
            now = millis(); // take a closer look
        }
		// now = millis(); might need to be here instead of above ***********************
        // if its been 1 second and no more chars received
        if (now-lastTime > 1000 && inputString.length() < 2){ 
          break; // exit current loop.
        }
      }
      Serial.print(inputString);
      inputString.trim(); // take off any '\n' and whitespace trailing string.
      if (inputString.length() > 2){ inputString = "";}  // might not ever run 
  }
}



void firstRun(void){
    if (first_RUN == 1){
      Serial.println("First Run, saving developers gear positions to EEPROM");
      EEPROM.write(eeprom_firstrun_addr, 1); // flag that shiftCount has been loaded once.
      save_shiftCount(); // save the whole table to EEPROM
    }
    // else it is 255 and we want the shiftCount data saved in memory.
    else {
      Serial.println("NOT FIRST RUN, Loading gear positions from EEPROM");
      char i=0;
      for(i=0; i < NUMBERofGEARS; i++)
      {
        Serial.print("\rGear "); Serial.print(i+1); Serial.print(": \t");
        Serial.println(EEPROM.get(eeprom_shiftCount_addr+i*2, shiftCount[i]));
        //eeprom commands average 3.3 milliseconds
      }
    }
}

void save_shiftCount(void){
    char i=0;
    for(i=0; i < NUMBERofGEARS; i++)
    {  // SAVE the developer's shiftCount table to memory
        EEPROM.put(eeprom_shiftCount_addr+i*2, shiftCount[i]);
        //Serial.print("\t");
        //Serial.print(EEPROM.get(eeprom_shiftCount_addr+i*2, shiftCount[i]));
        //eeprom commands average 3.3 milliseconds
    }
    i=0;
    for(i=0; i < NUMBERofGEARS; i++)
    {
        //EEPROM.put(eeprom_shiftCount_addr+i*2, shiftCount[i]);
        //Serial.print("\t");
        Serial.print("\rGear "); Serial.print(i+1); Serial.print(": \t");
        Serial.println(EEPROM.get(eeprom_shiftCount_addr+i*2, shiftCount[i]));
        //eeprom commands average 3.3 milliseconds
    }
    EEPROM.write(eeprom_gear, gear);
    encoderPosition = shiftCount[gear-1];
    
}

/* we need to check if the new tune value of going up/down tuneCount+10 will put 
* us too close to the next position, if it will then it won't tune this gear 
* until the other gear moves further away first. 
* returns true if it won't be too close
* returns false if it will put the motor too close.
* rule of thumb: we don't want to be within 50 of the next gear.
*/
bool tunePreventCheck(unsigned int Target){
  if (shiftCount[gear+1]-(Target) < 50 && (gear != NUMBERofGEARS)) { //(10332 - (10100+15+10)) < 
    Serial.print("Thats too close to gear "); Serial.println(gear+1);
    return false;
  }
  if (((Target)-shiftCount[gear-2]) < 50 && (gear != 1)) {  //((10332-15-10)-10100) < 50
    Serial.print("Thats too close to gear "); Serial.println(gear-1);
    return false;
  }
  return true;
}

