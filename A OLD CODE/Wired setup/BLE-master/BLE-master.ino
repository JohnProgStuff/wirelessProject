/*
 * BLE113 SoftSerial Sketch for Arduino
 */
#include <SoftwareSerial.h>

// In order to not interfere with the USB serial console
// we configure the BLE-113/UNO serial lines to be available
// through SoftwareSerial on pins D4 and D5. Please set the jumpers
// to RX -> D4 and TX -> D5
SoftwareSerial ble113(4, 5);
boolean connected;
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int ledPinUP = 9;
int ledPinDN = 8;      // the number of the DOWN LED pin

const int button_1_Pin = 2;     // the number of the shift UP   pushbutton pin
const int button_2_Pin = 3;     // the number of the shift DOWN pushbutton pin

// Button debouncing variables:
char button_1_read, button_2_read;
char BTNS, BTNS_OLD, BTNS_temp, BTNS_STATE, BTNS_STATE_CHANGED;
char BTNS_Still_Down, OLD_STATE;
  
// Commands received from BLE module (from Master device)
String ShiftUP = "UP"; // could be any value
String ShiftDN = "DN" ; // could be any value

//**********************************************************************************
// START FUNCTION
//**********************************************************************************
void setup() {
  // On the Arduino Uno, the Serial port is mapped to the USB Cable and available in pins D0D1.
  Serial.begin(9600);
  Serial.print("BLE-Master Sketch setup...");
  //Serial1.begin(9600);// for pro micro 3.3V arduino instead of SoftwareSerial
  ble113.begin(9600); // software serial
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  // The int1 is on pin2 on the Pro micro
  // attachInterrupt(1, connection, CHANGE); //INT1 
  //connected = false;
  //  pinMode(tune_button_pin, INPUT); 
  pinMode(ledPinUP, OUTPUT); // LED on pin 9- 'motor'
  pinMode(ledPinDN, OUTPUT); // LED on pin 8- 'motor'
  // initialize the pushbutton pins as an input:
  
  pinMode(button_1_Pin, INPUT);
  pinMode(button_2_Pin, INPUT);  
  Serial.println(" done.");

}


/**********************************************************************************
* PROGRAM LOOP
**********************************************************************************/
void loop() // run over and over
{
  // gets info from one serial device and sends it to the other
  SerialFromPC(); // send from USB to module
  //getBTNS();
  if (BTNS_STATE == 1 & BTNS_STATE_CHANGED == 1) { // Button 1 pressed 1st sample      
      //Serial1.println("UP");  // send to module pro micro
      ble113.println("UP");  // send to module
      Serial.println("UPbt");
      digitalWrite(ledPinUP, HIGH);
      delay(10);
  }
  else { digitalWrite(ledPinUP,LOW);digitalWrite(ledPinDN,LOW);}
  // if(BTNS_STATE == 0 ){Serial.print(BTNS_STATE);Serial.println(" " + BTNS_STATE_CHANGED);}
  if (BTNS_STATE == 2 & BTNS_STATE_CHANGED == 2) { // Button 1 pressed 1st sample      
      //Serial1.println("DN");  // send to module pro micro
      ble113.println("DN");  // send to module
      Serial.println("DNbtn");
      digitalWrite(ledPinDN, HIGH);
      delay(10);
  }
  else { digitalWrite(ledPinUP,LOW);digitalWrite(ledPinDN,LOW);}
  SerialFromBLE(); // get anything from the BLE and MOVE it to the USB 
  
} // end of main loop function

/********************************************************************
* SerialFromPC
********************************************************************/
void SerialFromPC(){
  // If Data from the PC serial console, submit it to the
  // BLE113 byte per byte. (1 byte at a time... no buffer)
  // this if loop cycles through once for each byte received from PC console.
  if (Serial.available()) { // if Serial buffer has stuff
      int chs = Serial.read();
      char ch = char(chs);
      Serial.print(ch); // echo each char back to PC
      ble113.write(ch);
      //Serial1.write(ch);  // send to module pro micro
      if (!Serial.available()){
          //Serial.println();
          //Serial.print("sent\n");  // done returning string to PC
      }
  }
}

/********************************************************************
* SerialFromBLE
********************************************************************/
void SerialFromBLE(){
  // if BLE module has sent info. send it to the serial console.
  if (ble113.available()) { // if UNO
  //if (Serial1.available()) { // if pro micro
      //Serial.print("ble113: "); 
      //Serial.write(Serial1.read()); // use if ProMicro sends to PC
      Serial.write(ble113.read()); // use if UNO
  }
}


/**********************************************************************************
* connection()
* This method is called if the connection state changes. 
* When a connection is established, the Interrupt will be set to high, and
* set to low, once the BLE-Shield is disconnected.  I don't think we need this 
* because the BLE sends a serial string saying it is connected or disconnected.
**********************************************************************************/
/*
void connection() {
  connected = !connected;
  if (connected) {
    Serial.println();
    Serial.println("BLE-113 Connected = yes");
  }
  else {
    Serial.println();
    Serial.println("BLE-113 Connected = no");  
  }
}
*/

/**********************************************************************************
* getBTNS gets the state of the button presses for the 2 shift buttons.
**********************************************************************************/
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
