/*
 * Date Created:    August 5th, 2023
 * Last Modified:   August 11th, 2023
 * Filename:        controller_firmware.ino
 * Purpose:         Process input data from two joystics and send them using an RF communication module to Thunderbird.
 * Microcontroller: Arduino Uno R3
 * Connections:     See controller_schematics located in the schematics folder
 * Notes:           Disconnect the RF module if there are issues uploading the sketch.
 */

//#define DEBUG // uncomment this line when trying to debug the code with a computer

// Pin definitions
  #define RF_SERIAL_RATE     9600    //change this to the baud rate that the RF communication module is set at
  #define DEBUG_SERIAL_RATE  9600
  #define RUDR               A4
  #define SAIL               A3

// RF definition
  #define SENDING_INTERVAL   750

// Setup
void setup() {
  // Pin Setup for joystick
  pinMode(RUDR, INPUT);
  pinMode(SAIL, INPUT);
  
  #ifdef DEBUG
    //Begin serial communication to computer
    Serial.begin(DEBUG_SERIAL_RATE);
  #else
    //Begin serial communication to RF Module
    Serial.begin(RF_SERIAL_RATE);
  #endif
}

//store last rf message sent time
uint32_t last_send_time;

void loop() {
  // initialize joystic status variables
  uint16_t rudr_val = 0;
  uint16_t sail_val = 0;

  //get joystick status
  getJoystick(&rudr_val, &sail_val);

  #ifdef DEBUG
    // if debugging mode is on, send raw values
    sendSerialDebug(rudr_val, sail_val);
  #else
    if(millis() >= last_send_time + SENDING_INTERVAL){
      // Send message via RF if debugging is off and sending interval has expired
      sendSeralRF(rudr_val, sail_val);
      last_send_time = millis();
    }
  #endif
}

// get joystick status
void getJoystick(uint16_t* rudr_val, uint16_t* sail_val) {
  // initialize raw value variables and obtain raw values from analog ports
  *rudr_val = analogRead(RUDR);
  *sail_val = analogRead(SAIL);
}

// send serial message to RF module to instruct what to send
void sendSeralRF(uint16_t rudr_val, uint16_t sail_val) {
  Serial.print(rudr_val);
  Serial.print(";");
  Serial.println(sail_val);
}

// send serial debug message containing raw values from each analog port
void sendSerialDebug(uint16_t rudr_val, uint16_t sail_val) {
  Serial.print("RUDDER: ");
  Serial.print(rudr_val);
  Serial.print("; SAIL: ");
  Serial.println(sail_val);
}