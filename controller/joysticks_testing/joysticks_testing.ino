/*
 * Date Created:    August 5th, 2023
 * Last Modified:   August 5th, 2023
 * Filename:        joysticks_testing.ino
 * Purpose:         Print the vertical readings from the joysticks directly on the serial output.
 * Microcontroller: Arduino Uno R3
 * Connections:     See controller_schematics located in the schematics folder
 */

// Pin definitions
  #define RUDR_BUTTON     2
  #define RUDR_VERT       A1
  #define RUDR_HORZ       A2
  #define SAIL_BUTTON     3
  #define SAIL_VERT       A3
  #define SAIL_HORZ       A4

// Setup
void setup() {
  // Pin Setup for RUDDER joystick
  pinMode(RUDR_BUTTON, INPUT);
  pinMode(RUDR_VERT, INPUT);
  pinMode(RUDR_HORZ, INPUT);

  // Pin Setup for SAIL joystick
  pinMode(SAIL_BUTTON, INPUT);
  pinMode(SAIL_VERT, INPUT);
  pinMode(SAIL_HORZ, INPUT);

  //Begin serial communication to RF Module
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int rudr_vert_val = analogRead(RUDR_VERT);
  int sail_vert_val = analogRead(SAIL_VERT);
  Serial.print(rudr_vert_val);
  Serial.print(";");
  Serial.println(sail_vert_val);
}
