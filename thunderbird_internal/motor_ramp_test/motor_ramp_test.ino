/*
 * Date Created:    August 10th, 2023
 * Last Modified:   August 11th, 2023
 * Filename:        motor_ramp_test.ino
 * Purpose:         Ramp up then down (both clockwise and counterclockwise) the DC Motor (FIT0521) using PWM.
 * Microcontroller: Arduino Uno R3
 * Connections:     See internal_schematics located in the schematics folder
 */

#include <Encoder.h>
Encoder myEnc(2,3);

// define constants and pins 
  #define PWM1          5
  #define PWM2          6
  #define RAMP_INTERVAL 100
  #define RAMP_PAUSE    1000

long prev_pos  = -999;

// setup
void setup() {
  // setup pins for motor pwm
  Serial.begin(9600);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
}

// main loop
void loop() {
  // ramp up counterclockwise
  digitalWrite(PWM2, 0);
  for (int index = 0; index <= 255; index++) {
    analogWrite(PWM1, index);
    readEncoder();
    delay(RAMP_INTERVAL);
  }

  // ramp down counterclockwise
  for (int index = 255; index >= 0; index--) {
    analogWrite(PWM1, index);
    readEncoder();
    delay(RAMP_INTERVAL);
  }
  delay(RAMP_PAUSE);

  // ramp up clockwise
  digitalWrite(PWM1, 0);
  for (int index = 0; index <= 255; index++) {
    analogWrite(PWM2, index);
    readEncoder();
    delay(RAMP_INTERVAL);
  }

  // ramp down clockwise
  for (int index = 255; index >= 0; index--) {
    analogWrite(PWM2, index);
    readEncoder();
    delay(RAMP_INTERVAL);
  }
  delay(RAMP_PAUSE);
}

// read current encoder value, if there is a change, store the change and print on serial port
void readEncoder () {
  long pos = myEnc.read();

  // store and print on serial port if change is detected
  if (pos != prev_pos) {
    prev_pos = pos;
    Serial.println(pos);
  }
}