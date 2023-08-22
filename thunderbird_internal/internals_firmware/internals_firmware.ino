/*
 * Date Created:    August 11th, 2023
 * Last Modified:   August 21th, 2023
 * Filename:        internals_firmware.ino
 * Purpose:         Receive controller commands and turn the sail and rudder accordingly.
 * Microcontroller: Arduino Uno R3
 * Connections:     See internal_schematics located in the schematics folder
 * Notes:           This is the final production firmware.
 */

#define ENCODER_OPTIMIZE_INTERRUPTS       // optional setting causes Encoder to use more optimized code (must be defined before Encoder.h)
#include <Servo.h>
#include <Encoder.h>
#include <SoftwareSerial.h>

// For debug use only 
  //#define DEBUG                           // Uncomment to see raw data
  #define DEBUG_SERIAL          115200    // Debug Serial Baud rate

// Servo definitions
  Servo rudder;                           // create servo object to control the rudder
  uint8_t rudr_pos = 90;                  // variable to store the servo position
  #define MAX_SERVO_INTERVAL    10.0      // must be a float, defines how much the servo turns at max speed (must not exceed 180)
  #define MAX_RUDR_POS          180.0     // the maximum angle the rudder servo can turn (must be a float)
  #define MIN_RUDR_POS          0.0       // the minimum angle the rudder servo can turn (must be a float)
  #define SLOW_DOWN_THRESHOLD   255.0     // must be a float, defines the encoder value difference (to the MAX or MIN position) at which the motor starts to slow down

// define RF 
  #define RF_SERIAL             115200    // RF Serial Baud rate
  SoftwareSerial rfSerial(11, 10);        // RX, TX Pins

// define Sail motor & encoder
  #define PWM1                  5
  #define PWM2                  6
  #define MOTOR_MAX_SPEED       255.0     // must be a float, defines how fast the dc motor turns at max speed (limit is 255)
  #define MAX_SAIL_POS          10000     // the maximum amount the sail motor can turn
  #define MIN_SAIL_POS          -10000    // the minimum amount the sail motor can turn
  int32_t sail_pos = 0;                   // variable to store sail encoder value
  Encoder sailEncoder(2,3);

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

  // initialize a checksum and error value
  uint16_t rf_sum = 0;
  bool error = false;

  // update sail position
  sailEncoderRead();

  // Get RF data from the controller
  readRF(&rudr_val, &sail_val, &rf_sum, &error);

  // if error check passed, continue to adjust motor speed/direction/position
  if(!error) {
    // Adjust the two motor's speed, direction, and position
    sailAdjust(sail_val);
    rudrAdjust(rudr_val);
  }
  else {
    // stop sail motor immediately and keep rudder position when error check fails
    digitalWrite(PWM1, LOW);
    digitalWrite(PWM2, LOW);
  }

  // send sail position and rudder position through RF
  sendRF();

  // if debugging mode is on, send debugging messages through serial port
  #ifdef DEBUG
    sendDebug(rudr_val, sail_val, rf_sum, error);
  #endif
}

// read RF serial
void readRF(uint16_t* rudr_val, uint16_t* sail_val, uint16_t* rf_sum, bool* error) {
  // read in the rudr value first (regardless of signal integrity)
  *rudr_val = rfSerial.parseInt();

  // if the separation semicolon is detected (for validation), store the second integer as sail value
  if(rfSerial.read() == ';') {
    *sail_val = rfSerial.parseInt();
  }
  else {
    // if signal integrity could not be validated through the semicolon, report it via the error boolean
    *error = true;
  }

  // if both values are zero, check for the following ?, if not present, report it via error boolean (signal integrity problem)
  if(*rudr_val == 0 && *sail_val == 0 && rfSerial.read() != '?') {
    *error = true;
  }
  else  {
    // if one of the values isn't zero, read in the sum of the two values (only sent when both values are non-zero)
    *rf_sum = rfSerial.parseInt();
    
    // if the summation does not match, report signal integrity problem via error boolean
    if(*rudr_val + *sail_val != *rf_sum) {
      *error = true;
    }
  }

  // flush remaining data in serial buffer to prevent errors
  rfSerial.flush();
}

// send sail and rudr position through serial
void sendRF() {
  rfSerial.print(rudr_pos);
  rfSerial.print(";");
  rfSerial.println(sail_pos);
}

// send all statistics through serial port
void sendDebug(uint16_t rudr_val, uint16_t sail_val, uint16_t rf_sum, bool error) {
  Serial.print(rudr_pos);
  Serial.print(";");
  Serial.print(sail_pos);
  Serial.print(";");
  Serial.print(rudr_val);
  Serial.print(";");
  Serial.print(sail_val);
  Serial.print(";");
  Serial.print(rf_sum);
  Serial.print(";");
  Serial.print(sail_val+rudr_val);
  Serial.print(";");
  Serial.println(error);
}

 // Adjust Sail motor speed and direction
void sailAdjust(uint16_t sail_val) {
  if (sail_val > OFF_UPPER) {
    // check if sail is going above limits
    if(sail_pos >= MAX_SAIL_POS) {
      // if at limit, turn off motor
      digitalWrite(PWM1, LOW);
      digitalWrite(PWM2, LOW);
    }
    else if ((MAX_SAIL_POS - sail_pos) <= SLOW_DOWN_THRESHOLD) {
      // reduce the speed by a "power reduction ratio" if the sail value is approaching the limit
      digitalWrite(PWM1, LOW);
      float reduction_ratio = (float) (MAX_SAIL_POS - sail_pos) / SLOW_DOWN_THRESHOLD;
      float power = (float) (sail_val - OFF_UPPER) / (float) (MAX - OFF_UPPER) * MOTOR_MAX_SPEED;
      uint8_t reduced_power = (uint8_t) (power * reduction_ratio);
      analogWrite(PWM2, reduced_power);
    }
    else {
      digitalWrite(PWM1, LOW);
      uint8_t power = (uint8_t) ((float) (sail_val - OFF_UPPER) / (float) (MAX - OFF_UPPER) * MOTOR_MAX_SPEED);
      analogWrite(PWM2, power);
    }
  }
  else if (sail_val < OFF_LOWER){
    // check if sail is going below limits
    if(sail_pos <= MIN_SAIL_POS) {
      // if at limit, turn off motor
      digitalWrite(PWM1, LOW);
      digitalWrite(PWM2, LOW);
    }
    else if ((sail_pos - MIN_SAIL_POS) <= SLOW_DOWN_THRESHOLD) {
      // reduce the speed by a "power reduction ratio" if the sail value is approaching the limit
      digitalWrite(PWM1, LOW);
      float reduction_ratio = (float) (sail_pos - MIN_SAIL_POS) / SLOW_DOWN_THRESHOLD;
      float power = (float) (OFF_LOWER - sail_val) / (float) (OFF_LOWER - MIN) * MOTOR_MAX_SPEED;
      uint8_t reduced_power = (uint8_t) (power * reduction_ratio);
      analogWrite(PWM2, reduced_power);
    }
    else {
      digitalWrite(PWM2, LOW);
      uint8_t power = (uint8_t) ((float) (OFF_LOWER - sail_val) / (float) (OFF_LOWER - MIN) * MOTOR_MAX_SPEED);
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
  rudr_pos = (uint16_t) ((float) rudr_val / (float) MAX * (MAX_RUDR_POS - MIN_RUDR_POS) + MIN_RUDR_POS);
  rudder.write(rudr_pos);
}

// Read the current sail position from the sail encoder
void sailEncoderRead() {
  int32_t new_sail_pos = sailEncoder.read();

  // store and print on serial port if change is detected
  if(new_sail_pos != sail_pos) {
    sail_pos = new_sail_pos;
    Serial.println(new_sail_pos);
  }
}