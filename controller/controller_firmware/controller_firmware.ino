/*
 * Date Created:    August 5th, 2023
 * Last Modified:   August 11th, 2023
 * Filename:        controller_firmware.ino
 * Purpose:         Process input data from two joystics and send them using an RF communication module to Thunderbird.
 * Microcontroller: Arduino Uno R3
 * Connections:     See controller_schematics located in the schematics folder
 * Notes:           Disconnect the RF module if there are issues uploading the sketch.
 */

#include <SoftwareSerial.h>

// For debugging use only
  #define DEBUG                           // uncomment this line when trying to debug the code with a computer
  #define DEBUG_SERIAL_RATE  115200       // Debug Serial Baud rate

// Potentiometer and Joystick pin definitions
  #define RUDR               A1
  #define SAIL               A4

// RF definition
  #define RF_SERIAL_RATE     115200       // change this to the baud rate that the RF communication module is set at
  #define SENDING_INTERVAL   150
  SoftwareSerial rfSerial(11, 10);        // RX, TX Pins

// Setup
void setup() {
  // Pin Setup for joystick
  pinMode(RUDR, INPUT);
  pinMode(SAIL, INPUT);
  
  #ifdef DEBUG
    //Begin serial communication to computer
    Serial.begin(DEBUG_SERIAL_RATE);
  #endif

  //Begin serial communication to RF Module
  rfSerial.begin(RF_SERIAL_RATE);
}

//store last rf message sent time
uint32_t last_send_time;

void loop() {
  // initialize joystic status variables
  uint16_t rudr_val = 0;
  uint16_t sail_val = 0;
  uint8_t rudr_pos = 0;
  int32_t sail_pos = 0;

  //get joystick status
  getJoystick(&rudr_val, &sail_val);

  //Read returning position data from the boat
  getPosition(&rudr_pos, &sail_pos);

  #ifdef DEBUG
    // if debugging mode is on, send raw values
    sendSerialDebug(rudr_pos, sail_pos, rudr_val, sail_val);
  #endif

  if(millis() >= last_send_time + SENDING_INTERVAL){
    // Send message via RF when sending interval has expired
    sendSeralRF(rudr_val, sail_val);
    last_send_time = millis();
  }
}

// get joystick status
void getJoystick(uint16_t* rudr_val, uint16_t* sail_val) {
  // initialize raw value variables and obtain raw values from analog ports
  *rudr_val = analogRead(RUDR);
  *sail_val = analogRead(SAIL);
}

// read RF serial
void getPosition(uint8_t* rudr_pos, int32_t* sail_pos) {
  // store the first integer in rudder value
  *rudr_pos = rfSerial.parseInt();

  // if the separation semicolon is detected (for validation), store the second integer as sail value
  if(rfSerial.read() == ';') {
    *sail_pos = rfSerial.parseInt();
  }

  // flush remaining data in serial buffer to prevent errors
  rfSerial.flush();
}

// send serial message to RF module to instruct what to send
void sendSeralRF(uint16_t rudr_val, uint16_t sail_val) {
  rfSerial.print(rudr_val);
  rfSerial.print(";");
  rfSerial.println(sail_val);
}

// send serial debug message containing raw values from each analog port
void sendSerialDebug(uint8_t rudr_pos, int32_t sail_pos, uint16_t rudr_val, uint16_t sail_val) {
  Serial.print(rudr_pos);
  Serial.print(";");
  Serial.print(sail_pos);
  Serial.print(";");
  Serial.print(rudr_val);
  Serial.print(";");
  Serial.println(sail_val);
}