/*
 * Date Created:    August 11th, 2023
 * Last Modified:   August 11th, 2023
 * Filename:        internals_firmware.ino
 * Purpose:         Receive controller commands and turn the sail and rudder accordingly.
 * Microcontroller: Arduino Uno R3
 * Connections:     See internal_schematics located in the schematics folder
 */
 
//#define DEBUG                           // Uncomment to see raw data
#include <Servo.h>

// Servo definitions
  Servo rudder;                           // create servo object to control the rudder
  uint8_t pos = 90;                       // variable to store the servo position
  #define MAX_SERVO_INTERVAL    10.0      // must be a float, defines how much the servo turns at max speed (must not exceed 180)

// define constants and pins 
  #define RF_SERIAL         9600
  #define PWM1              5
  #define PWM2              6
  #define MOTOR_MAX_SPEED   255.0         // must be a float, defines how fast the dc motor turns at max speed (limit is 255)

//define thresholds
  #define OFF_UPPER     550               // defines the joystick value upper bound for the motors to stay off
  #define OFF_LOWER     450               // defines the joystick value lower bound for the motors to stay off
  #define MAX           1023              // defines the max joystick value
  #define MIN           0                 // defines the min joystick value

// setup
void setup() {
  // setup pins for dc motor pwm
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);

  // attach rudder to pin 4
  rudder.attach(4);

  // begin receiving RF data via Serial 
  Serial.begin(RF_SERIAL);
}

// main loop
void loop() {
  //initialize rudr and sail values
  uint16_t rudr_val = 0;
  uint16_t sail_val = 0;

  //Get RF values and print on Serial port if DEBUG is on
  readRF(&rudr_val, &sail_val);
  #ifdef DEBUG
    Serial.print(rudr_val);
    Serial.print(";");
    Serial.println(sail_val);
  #endif

  // Adjust the two motor's speed, direction, and position
  sailAdjust(sail_val);
  rudrAdjust(rudr_val);
}


// read RF serial
void readRF(uint16_t* rudr_val, uint16_t* sail_val) {
  // store the first integer in rudder value
  *rudr_val = Serial.parseInt();

  // if the separation semicolon is detected (for validation), store the second integer as sail value
  if (Serial.read() == ';'){
    *sail_val = Serial.parseInt();
  }

  // flush remaining data in serial buffer to prevent errors
  Serial.flush();
}

// Adjust Sail motor speed and direction
void sailAdjust(uint16_t sail_val) {
  if (sail_val > OFF_UPPER) {
    digitalWrite(PWM1, LOW);
    uint8_t power = (uint8_t) ((float) (sail_val - OFF_UPPER)/(float) (MAX - OFF_UPPER) * MOTOR_MAX_SPEED);
    analogWrite(PWM2, power);
  }
  else if (sail_val < OFF_LOWER){
    digitalWrite(PWM2, LOW);
    uint8_t power = (uint8_t) ((float) (OFF_LOWER - sail_val)/(float) (OFF_LOWER - MIN) * MOTOR_MAX_SPEED);
    analogWrite(PWM1, power);
  }
  else {
    // motor off
    digitalWrite(PWM1, LOW);
    digitalWrite(PWM2, LOW);
  }
}

// Adjust Ruddr motor position
void rudrAdjust(uint16_t rudr_val){
  if (rudr_val > OFF_UPPER) {
    uint8_t delta = (uint8_t) ((float) (rudr_val - OFF_UPPER)/(float) (MAX - OFF_UPPER) * MAX_SERVO_INTERVAL);

    // prevent angle going above 180 degrees
    if (pos + delta < 180) {
      pos += delta;
    }
    else {
      pos = 180;
    }
  }
  else if (rudr_val < OFF_LOWER){
    uint8_t delta = (uint8_t) ((float) (OFF_LOWER - rudr_val)/(float) (OFF_LOWER - MIN) * MAX_SERVO_INTERVAL);

    //prevent overflowing for unsigned int
    if (delta > pos) {
      pos = 0;
    }
    else{
      pos -= delta;
    }
  }
  rudder.write(pos);
}
