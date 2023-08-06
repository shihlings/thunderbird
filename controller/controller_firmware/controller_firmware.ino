/*
 * Date Created:    August 5th, 2023
 * Last Modified:   August 6th, 2023
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
  
// RF Message definitions
  #define RUDR_RIGHT_LOW  "RR1"
  #define RUDR_RIGHT_HIGH "RR2"
  #define RUDR_LEFT_LOW   "RL1"
  #define RUDR_LEFT_HIGH  "RL2"
  #define SAIL_DOWN_LOW   "SD1"
  #define SAIL_DOWN_HIGH  "SD2"
  #define SAIL_UP_LOW     "SU1"
  #define SAIL_UP_HIGH    "SU2"

// Threshold Values (Needs to be adjusted once the controller is assembled)
  #define OFF_LOWER_THRESHOLD     450
  #define OFF_HIGHER_THRESHOLD    550
  #define BOTTOM_THRESHOLD        200
  #define TOP_THRESHOLD           800

// Status Values
  #define OFF         -1
  #define TOP_HIGH    4
  #define TOP_LOW     3
  #define BOTTOM_HIGH 2
  #define BOTTOM_LOW  1

// Function declarations
  void getJoystick(int8_t* rudr_stat, int8_t* sail_stat);
  int8_t determineStatus(uint16_t value);
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
  int8_t rudr_stat = OFF;
  int8_t sail_stat = OFF;

  //get joystick status
  getJoystick(&rudr_stat, &sail_stat);

  // Send message via RF
  #ifndef DEBUG
    if (!(rudr_stat == OFF && sail_stat == OFF)) {
      sendSeralRF(rudr_stat, sail_stat);
    }
  #endif
}

// get joystick status
void getJoystick(int8_t* rudr_stat, int8_t* sail_stat) {
  // initialize raw value variables and obtain raw values from analog ports
  uint16_t rudr_val = analogRead(RUDR_VERT);
  uint16_t sail_val = analogRead(SAIL_VERT);

  // if debugging mode is on, send raw values
  #ifdef DEBUG
    sendSerialDebug(rudr_val, sail_val);
  #endif

  // determine status of each joystick
  *rudr_stat = determineStatus(rudr_val);
  *sail_stat = determineStatus(sail_val);
}

// determine joystick status according to the raw values obtained
// see threshold value definitions for specific values
int8_t determineStatus(uint16_t value) {
  if(value < OFF_LOWER_THRESHOLD) {
    if(value < BOTTOM_THRESHOLD) {
      return BOTTOM_HIGH;
    }
    else {
      return BOTTOM_LOW;
    }
  }
  else if(value > OFF_HIGHER_THRESHOLD) {
    if(value > TOP_THRESHOLD) {
      return TOP_HIGH;
    }
    return TOP_LOW;
  }
  else {
    return OFF;
  }
}

// send serial message to RF module to instruct what to send
void sendSeralRF(int8_t rudr_stat, int8_t sail_stat) {
  Serial.print(rudr_stat);
  Serial.println(sail_stat);
}

// send serial debug message containing raw values from each analog port
void sendSerialDebug(uint16_t rudr_val, uint16_t sail_val) {
  Serial.print("RUDDER: ");
  Serial.print(rudr_val);
  Serial.print("; SAIL: ");
  Serial.println(sail_val);
}