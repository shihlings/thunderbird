/*
 * Date Created:    August 5th, 2023
 * Last Modified:   August 21th, 2023
 * Filename:        controller_firmware.ino
 * Purpose:         Process input data from two joystics and send them using an RF communication module to Thunderbird.
 * Microcontroller: Arduino Uno R3
 * Connections:     See controller_schematics located in the schematics folder
 * Notes:           This is the final production firmware.
 */

#include <SoftwareSerial.h>

// For debugging use only
  //#define DEBUG                           // uncomment this line when trying to debug the code with a computer
  #define DEBUG_SERIAL_RATE  115200       // Debug Serial Baud rate

// Potentiometer and Joystick pin definitions
  #define RUDR               A1
  #define SAIL               A2

// RF definition
  #define RF_SERIAL_RATE     115200       // change this to the baud rate that the RF communication module is set at
  #define SENDING_INTERVAL   150
  SoftwareSerial rfSerial(11, 10);        // RX, TX Pins

// Variables 
  #define  MEAN_NUM             5         // Defines the number of past values to average
  uint32_t last_send_time;                // Store last RF send time
  uint16_t rudr_val[MEAN_NUM];            // Store an array of past rudder values
  uint16_t sail_val[MEAN_NUM];            // Store an array of past sail values
  uint8_t  val_index = 0;                 // Current position of the index for the two arrays

// Setup
void setup() {
  // Pin Setup for joystick
  pinMode(RUDR, INPUT);
  pinMode(SAIL, INPUT);
  
  #ifdef DEBUG
    // Begin serial communication to computer
    Serial.begin(DEBUG_SERIAL_RATE);
  #endif

  // Begin serial communication to RF Module
  rfSerial.begin(RF_SERIAL_RATE);

  // fill the rudr_val and sail_val variables with default values (500 is the magic number, because the joystick nominal value sits at around 490 - 510).
  for(int index = 0; index < MEAN_NUM; ++index) {
    rudr_val[index] = 500;
    sail_val[index] = 500;
  }
}

// main loop
void loop() {
  // initialize mean joystick value variables
  uint16_t mean_rudr_val = 0;
  uint16_t mean_sail_val = 0;

  // get joystick status
  getJoystick(&mean_rudr_val, &mean_sail_val);

  // initialize position variables to 0
  uint8_t rudr_pos = 0;
  int32_t sail_pos = 0;

  // Read returning position data from the boat
  getPosition(&rudr_pos, &sail_pos);

  #ifdef DEBUG
    // if debugging mode is on, send raw values
    sendSerialDebug(rudr_pos, sail_pos, mean_rudr_val, mean_sail_val);
  #endif

  if(millis() >= last_send_time + SENDING_INTERVAL){
    // Send message via RF when sending interval has expired
    sendSeralRF(mean_rudr_val, mean_sail_val);
    last_send_time = millis();
  }
}

// get joystick status
void getJoystick(uint16_t* mean_rudr_val, uint16_t* mean_sail_val) {
  // initialize raw value variables and obtain raw values from analog ports
  rudr_val[val_index] = analogRead(RUDR);
  sail_val[val_index] = analogRead(SAIL);

  // if index is overflowing, reset to 0
  if(++val_index == MEAN_NUM) {
    val_index = 0;
  }
  
  // initialize variable to calculate sum of the array
  uint32_t rudr_total = 0;
  uint32_t sail_total = 0;

  // calculate sum for both arrays
  for(uint8_t index = 0; index < MEAN_NUM; ++index) {
    rudr_total += rudr_val[index];
    sail_total += sail_val[index];
  }

  // Calculate the mean (average) for both arrays
  *mean_rudr_val = rudr_total / MEAN_NUM;
  *mean_sail_val = sail_total / MEAN_NUM;
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
void sendSeralRF(uint16_t mean_rudr_val, uint16_t mean_sail_val) {
  rfSerial.print(mean_rudr_val);
  rfSerial.print(";");                                    // error checking symbol for signal integrity
  rfSerial.print(mean_sail_val);
  if (mean_rudr_val == 0 && mean_sail_val == 0) {
    rfSerial.println("?");                                // error checking symbol for signal integrity
  }
  else {
    rfSerial.print(":");
    rfSerial.println(mean_sail_val + mean_rudr_val);      // error checking value for signal integrity
  }
}

// send serial debug message containing raw values from each analog port
void sendSerialDebug(uint8_t rudr_pos, int32_t sail_pos, uint16_t mean_rudr_val, uint16_t mean_sail_val) {
  Serial.print(rudr_pos);
  Serial.print(";");
  Serial.print(sail_pos);
  Serial.print(";");
  Serial.print(mean_rudr_val);
  Serial.print(";");
  Serial.println(mean_sail_val);
}