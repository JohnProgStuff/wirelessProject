/*
BUTTON DEBOUNCING USING Dr. Browns technique
It might not be 100% accurate but it seems to be working for me.
In this example LedPin7 is mapped  
 */

// constants won't change. These set pin numbers
const int button_1_Pin = 8;     // the number of the shift UP   pushbutton pin
const int button_2_Pin = 9;     // the number of the shift DOWN pushbutton pin
const int ledPin7 =  7;         // the number of the shiftup LED pin
const int ledPin6 =  6;         // the number of the shiftdown LED pin

// Button debouncing variables:
char button_1_read;
char button_2_read;
char BTNS;
char BTNS_OLD;
char BTNS_temp;
char BTNS_STATE;
char BTNS_STATE_CHANGED;
char BTNS_Still_Down;
char OLD_STATE;


//**********************************************************************************
// SETUP 
//**********************************************************************************
void setup() {
  // initialize the pushbutton pins as an input:
  pinMode(button_1_Pin, INPUT);
  pinMode(button_2_Pin, INPUT);
      //  digitalWrite(button_1_Pin, HIGH); // Pull the button high so it goes low when pressed
  // initialize the LED pins as outputs:
  pinMode(ledPin7, OUTPUT);
  pinMode(ledPin6, OUTPUT);

  Serial.begin(9600); //This pipes to the serial monitor
}

//**********************************************************************************
// PROGRAM LOOP
//**********************************************************************************
void loop() {
  // get the state of the buttons with each loop.
  getBTNS();
  // IF BOTH BUTTONS are not pressed continue on
    // If up BTN1 is pushed turn on pin7 LED, send SHIFT_UP
    if (BTNS_STATE == 1 & BTNS_STATE_CHANGED == 1) { // Button 1 pressed 1st sample      
      digitalWrite(ledPin7, HIGH);  // Serial.println("SHIFT_Up_Sent");
      //******************************************************************
      // START MOTOR 1 direction 1 ***************************************
      
    }
    // If up BTN1 is released turn off pin7 LED, send Up_released
    else if ((OLD_STATE == 1 & BTNS_STATE_CHANGED == 1) & BTNS_STATE == 0) {
      digitalWrite(ledPin7, LOW);   //  Serial.println("Up_released");
       
    }
    // If up BTN2 is pushed turn on pin6 LED, send SHIFT_DOWN, set flagDn.
    if (BTNS_STATE == 2 & BTNS_STATE_CHANGED == 2) {
      digitalWrite(ledPin6, HIGH);   // Serial.println("SHIFT_Dn_Sent");  
      //******************************************************************
      // START MOTOR 1 direction 2 ***************************************
      
    }
    // If up BTN2 is released turn off pin7 LED, send Dn_released, clr flagDn.
    else if ((OLD_STATE == 2 & BTNS_STATE_CHANGED == 2) & BTNS_STATE == 0) {
      digitalWrite(ledPin6, LOW);    // Serial.println("Dn_released");
      
    } 
    delay(20); // delay for 20ms before you sample btn again
  
}// end main loop function

//**********************************************************************************
// getBTNS finds the state of the buttons according to Dr. Browns C language method
// Button 1 is connected to pin 8 above and button 2 is on pin 9 above
//**********************************************************************************


void getBTNS(void){

  // read the state of the both pushbuttons:
  button_1_read = !digitalRead(button_1_Pin); // read into bit 0
  button_2_read = (!digitalRead(button_2_Pin) << 1); // read into bit 1
  BTNS = button_1_read + button_2_read; // This is the 2 bit status of the buttons
//       Serial.print(BTNS,BIN); Serial.print(", "); Serial.print(BTNS_OLD,BIN); Serial.print(", "); Serial.print(BTNS_STATE,BIN); Serial.println(", ");
  // XCHANGE OLD and Current BTNS with 3 operations
     BTNS_temp = BTNS;   BTNS = BTNS_OLD;   BTNS_OLD = BTNS_temp;  //  Serial.print(BTNS,BIN); Serial.print(", "); Serial.print(BTNS_OLD,BIN); Serial.print(", "); Serial.print(BTNS_STATE,BIN); Serial.println(", ");
  
  BTNS = BTNS ^ BTNS_OLD;           //   Serial.print(BTNS,BIN); Serial.print(", "); Serial.print(BTNS_OLD,BIN); Serial.print(", "); Serial.print(BTNS_STATE,BIN); Serial.println(", ");
//  CHANGES = BTNS; 
  BTNS = BTNS & BTNS_OLD;           //   Serial.print(BTNS,BIN); Serial.print(", "); Serial.print(BTNS_OLD,BIN); Serial.print(", "); Serial.print(BTNS_STATE,BIN); Serial.println(", ");
  OLD_STATE = BTNS_STATE;
  BTNS_STATE_CHANGED = BTNS_STATE ^ ( BTNS_OLD);// Serial.print("State Change = "); Serial.println(BTNS_STATE_CHANGED,BIN);
  BTNS_STATE = (BTNS ^ BTNS_OLD);     // Serial.print(BTNS,BIN); Serial.print(", "); Serial.print(BTNS_OLD,BIN); Serial.print(", "); Serial.print(BTNS_STATE,BIN); Serial.println(", ");
  
  // If we only want the event to happen once, and not repetitively than we use the BTNS_STILL_DOWN.
  BTNS_Still_Down = (BTNS_STATE & BTNS_OLD) & (BTNS_STATE != B00000000); // Serial.print("BTNS_STILL_DOWN = "); Serial.println(BTNS_Still_Down,BIN); Serial.println(" \n");

} // end getBTNS function



