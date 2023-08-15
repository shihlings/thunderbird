/*
 * Date Created:    August 11th, 2023
 * Last Modified:   August 14th, 2023
 * Filename:        internals_firmware.ino
 * Purpose:         Receive controller commands and turn the sail and rudder accordingly.
 * Microcontroller: Arduino Uno R3
 * Connections:     See internal_schematics located in the schematics folder
 */

// For debug use only 
  //#define DEBUG                           // Uncomment to see raw data
  #define DEBUG_SERIAL          9600      // Debug Serial Baud rate

#include <Servo.h>
#include <Encoder.h>
#include <SoftwareSerial.h>

// Servo definitions
  Servo rudder;                           // create servo object to control the rudder
  uint8_t rudr_pos = 90;                  // variable to store the servo position
  #define MAX_SERVO_INTERVAL    10.0      // must be a float, defines how much the servo turns at max speed (must not exceed 180)
  #define MAX_RUDR_POS          180       // the maximum angle the rudder servo can turn
  #define MIN_RUDR_POS          0         // the minimum angle the rudder servo can turn

// define RF 
  #define RF_SERIAL             115200    // RF Serial Baud rate
  SoftwareSerial rfSerial(10, 11);        // RX, TX Pins

// define Sail motor & encoder
  #define PWM1                  5
  #define PWM2                  6
  #define MOTOR_MAX_SPEED       255.0     // must be a float, defines how fast the dc motor turns at max speed (limit is 255)
  #define MAX_SAIL_POS          10000     // the maximum amount the sail motor can turn
  #define MIN_SAIL_POS          -10000    // the minimum amount the sail motor can turn
  int32_t sail_pos = 0;                   // variable to store sail encoder value
  Encoder myEnc(2,3);

//define thresholds
  #define OFF_UPPER             550       // defines the joystick value upper bound for the motors to stay off
  #define OFF_LOWER             450       // defines the joystick value lower bound for the motors to stay off
  #define MAX                   1023      // defines the max joystick value
  #define MIN                   0         // defines the min joystick value

// setup
void setup() {
  // setup pins for dc motor pwm
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);

  // attach rudder to pin 4
  rudder.attach(4);

  // begin debug serial if defined
  #ifdef DEBUG
    Serial.begin(DEBUG_SERIAL);
  #endif

  // begin receiving RF data via Serial 
  rfSerial.begin(RF_SERIAL);
}

// main loop
void loop() {
  // initialize rudr and sail values
  uint16_t rudr_val = 0;
  uint16_t sail_val = 0;

  // update sail position & send it through RF and print on Serial port if DEBUG is on
  sailEncoderRead();
  sendRF();

  // Get RF values and print on Serial port if DEBUG is on
  readRF(&rudr_val, &sail_val);

  // Adjust the two motor's speed, direction, and position
  sailAdjust(sail_val);
  rudrAdjust(rudr_val);
}

// read RF serial
void readRF(uint16_t* rudr_val, uint16_t* sail_val) {
  // store the first integer in rudder value
  *rudr_val = rfSerial.parseInt();

  // if the separation semicolon is detected (for validation), store the second integer as sail value
  if(rfSerial.read() == ';') {
    *sail_val = rfSerial.parseInt();
  }

  // flush remaining data in serial buffer to prevent errors
  rfSerial.flush();

  // send to computer if debug is defined
  #ifdef DEBUG
    Serial.print(rudr_val);
    Serial.print(";");
    Serial.println(sail_val);
  #endif
}

// send sail and rudr position through serial
void sendRF() {
  rfSerial.print(rudr_pos);
  rfSerial.print(";");
  rfSerial.println(sail_pos);

  // send to computer if debug is defined
  #ifdef DEBUG
    Serial.print(rudr_pos);
    Serial.print(";");
    Serial.print(sail_pos);
    Serial.print(";");
  #endif
}

// Adjust Sail motor speed and direction
void sailAdjust(uint16_t sail_val) {
  if(sail_val > OFF_UPPER) {
    // check if sail is going above limits
    if(sail_pos >= MAX_SAIL_POS) {
      // if at limit, turn off motor
      digitalWrite(PWM1, LOW);
      digitalWrite(PWM2, LOW);
    }
    else {
      digitalWrite(PWM1, LOW);
      uint8_t power = (uint8_t) ((float) (sail_val - OFF_UPPER)/(float) (MAX - OFF_UPPER) * MOTOR_MAX_SPEED);
      analogWrite(PWM2, power);
    }
  }
  else if(sail_val < OFF_LOWER) {
    // check if sail is going below limits
    if(sail_pos <= MIN_SAIL_POS) {
      // if at limit, turn off motor
      digitalWrite(PWM1, LOW);
      digitalWrite(PWM2, LOW);
    }
    else {
      digitalWrite(PWM2, LOW);
      uint8_t power = (uint8_t) ((float) (OFF_LOWER - sail_val)/(float) (OFF_LOWER - MIN) * MOTOR_MAX_SPEED);
      analogWrite(PWM1, power);
    }
  }
  else {
    // motor off
    digitalWrite(PWM1, LOW);
    digitalWrite(PWM2, LOW);
  }
}

// Adjust Ruddr motor position
void rudrAdjust(uint16_t rudr_val) {
  if(rudr_val > OFF_UPPER) {
    uint8_t delta = (uint8_t) ((float) (rudr_val - OFF_UPPER)/(float) (MAX - OFF_UPPER) * MAX_SERVO_INTERVAL);

    // prevent angle going above 180 degrees
    if(rudr_pos + delta < 180) {
      rudr_pos += delta;
    }
    else {
      rudr_pos = 180;
    }
  }
  else if(rudr_val < OFF_LOWER) {
    uint8_t delta = (uint8_t) ((float) (OFF_LOWER - rudr_val)/(float) (OFF_LOWER - MIN) * MAX_SERVO_INTERVAL);

    // prevent overflowing for unsigned int
    if(delta > rudr_pos) {
      rudr_pos = 0;
    }
    else {
      rudr_pos -= delta;
    }
  }
  rudder.write(rudr_pos);
}

// Read the current sail position from the sail encoder
void sailEncoderRead() {
  int32_t new_sail_pos = myEnc.read();

  // store and print on serial port if change is detected
  if(new_sail_pos != sail_pos) {
    sail_pos = new_sail_pos;
    Serial.println(new_sail_pos);
  }
}