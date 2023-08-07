/*
 * Date Created:    August 5th, 2023
 * Last Modified:   August 7th, 2023
 * Filename:        controller_firmware.ino
 * Purpose:         Process input data from two joystics and send them using an RF communication module to Thunderbird.
 * Microcontroller: Arduino Uno R3
 * Connections:     See controller_schematics located in the schematics folder
 * Notes:           Disconnect the RF module if there are issues uploading the sketch.
 */

//#define DEBUG // uncomment this line when trying to debug the code with a computer

// Pin definitions
  #define RF_SERIAL_RATE     9600    //change this to the baud rate that the RF communication module is set at
  #define DEBUG_SERIAL_RATE  115200
  #define RUDR_BUTTON        2
  #define RUDR_VERT          A1
  #define RUDR_HORZ          A2
  #define SAIL_BUTTON        3
  #define SAIL_VERT          A3
  #define SAIL_HORZ          A4

// Function declarations
  void getJoystick(int8_t* rudr_stat, int8_t* sail_stat);
  void sendSeralRF(int8_t rudr_stat, int8_t sail_stat);
  void sendSerialDebug(uint16_t rudr_val, uint16_t sail_val);

// Setup
void setup() {
  // Pin Setup for RUDDER joystick (BUTTON and Horizontal Axis disabled for this application)
  // pinMode(RUDR_BUTTON, INPUT);
  pinMode(RUDR_VERT, INPUT);
  // pinMode(RUDR_HORZ, INPUT);

  // Pin Setup for SAIL joystick (BUTTON and Horizontal Axis disabled for this application)
  // pinMode(SAIL_BUTTON, INPUT);
  pinMode(SAIL_VERT, INPUT);
  // pinMode(SAIL_HORZ, INPUT);
  
  #ifdef DEBUG
    //Begin serial communication to computer
    Serial.begin(DEBUG_SERIAL_RATE);
  #else
    //Begin serial communication to RF Module
    Serial.begin(RF_SERIAL_RATE);
  #endif
}

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
    // Send message via RF if debugging is off
    sendSeralRF(rudr_val, sail_val);
  #endif
}

// get joystick status
void getJoystick(uint16_t* rudr_val, uint16_t* sail_val) {
  // initialize raw value variables and obtain raw values from analog ports
  *rudr_val = analogRead(RUDR_VERT);
  *sail_val = analogRead(SAIL_VERT);
}

// send serial message to RF module to instruct what to send
void sendSeralRF(uint16_t rudr_val, uint16_t sail_val) {
  Serial.print(rudr_val);
  Serial.println(sail_val);
}

// send serial debug message containing raw values from each analog port
void sendSerialDebug(uint16_t rudr_val, uint16_t sail_val) {
  Serial.print("RUDDER: ");
  Serial.print(rudr_val);
  Serial.print("; SAIL: ");
  Serial.println(sail_val);
}