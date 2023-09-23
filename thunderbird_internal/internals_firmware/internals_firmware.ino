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
  #define MIN_RUDR_POS          67.0       // the minimum angle the rudder servo can turn (must be a float)
  #define SLOW_DOWN_THRESHOLD   12500.0    // must be a float, defines the encoder value difference (to the MAX or MIN position) at which the motor starts to slow down

// define RF 
  #define RF_SERIAL             115200    // RF Serial Baud rate
  #define SENDING_INTERVAL      200
  SoftwareSerial rfSerial(10, 11);        // RX, TX Pins

// define Sail motor & encoder
  #define PWM1                  5
  #define PWM2                  6
  #define MOTOR_MAX_SPEED       255.0     // must be a float, defines how fast the dc motor turns at max speed (limit is 255)
  #define MAX_SAIL_POS          100000     // the maximum amount the sail motor can turn
  #define MIN_SAIL_POS          -100000    // the minimum amount the sail motor can turn
  int32_t sail_pos = 0;                   // variable to store sail encoder value
  Encoder sailEncoder(2,3);

//define thresholds
  #define OFF_UPPER             550       // defines the joystick value upper bound for the motors to stay off
  #define OFF_LOWER             450       // defines the joystick value lower bound for the motors to stay off
  #define MAX                   1023      // defines the max joystick value
  #define MIN                   0         // defines the min joystick value

// Variables 
  #define  MEAN_NUM             10        // Defines the number of past values to average
  uint32_t last_send_time;                // Store last RF send time
  uint16_t rudr_val[MEAN_NUM];            // Store an array of past rudder values
  uint16_t sail_val[MEAN_NUM];            // Store an array of past sail values
  uint16_t old_rudr_val = 500;
  uint16_t old_sail_val = 500;
  uint8_t  val_index = 0;                 // Current position of the index for the two arrays
  uint16_t error_count = 0;

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

  // fill the rudr_val and sail_val variables with default values (500 is the magic number, because the joystick nominal value sits at around 490 - 510).
  for(int index = 0; index < MEAN_NUM; ++index) {
    rudr_val[index] = 500;
    sail_val[index] = 500;
  }
}

// main loop
void loop() {
  // initialize temporary storage for rudr and sail values
  uint16_t rudr_val_temp = 500;
  uint16_t sail_val_temp = 500;
  uint16_t mean_rudr_val = 500;
  uint16_t mean_sail_val = 500;

  // initialize a checksum and error value
  uint16_t rf_sum = 0;
  bool error = false;

  // update sail position
  sailEncoderRead();

  // Get RF data from the controller
  readRF(&rudr_val_temp, &sail_val_temp, &rf_sum, &error);

  // if error check passed, continue to adjust motor speed/direction/position
  if(!error) {
    old_rudr_val = rudr_val_temp;
    old_sail_val = sail_val_temp;
    error_count = 0;
  }
  else {
    if(error_count++ > 3) {
      rudr_val_temp = old_rudr_val;
      sail_val_temp = 500;
      old_sail_val = 500;
    }
    else {
      rudr_val_temp = old_rudr_val;
      sail_val_temp = old_sail_val;
    }
  }

  valAveraging(rudr_val_temp, sail_val_temp, &mean_rudr_val, &mean_sail_val);

  // Adjust the two motor's speed, direction, and position
  sailAdjust(mean_sail_val);
  rudrAdjust(mean_rudr_val);

  if(millis() >= last_send_time + SENDING_INTERVAL){
    // Send message via RF when sending interval has expired
    sendRF();
    last_send_time = millis();
  }

  // if debugging mode is on, send debugging messages through serial port
  #ifdef DEBUG
    sendDebug(rudr_val_temp, sail_val_temp, rf_sum, error);
  #endif
}

// read RF serial
void readRF(uint16_t* rudr_val_temp, uint16_t* sail_val_temp, uint16_t* rf_sum, bool* error) {
  // read in the rudr value first (regardless of signal integrity)
  *rudr_val_temp = rfSerial.parseInt();

  // if the separation semicolon is detected (for validation), store the second integer as sail value
  if(rfSerial.read() == ';') {
    *sail_val_temp = rfSerial.parseInt();
  }
  else {
    // if signal integrity could not be validated through the semicolon, report it via the error boolean
    *error = true;
  }

  // if both values are zero, check for the following ?, if not present, report it via error boolean (signal integrity problem)
  if(*rudr_val_temp == 0 && *sail_val_temp == 0 && rfSerial.read() != '?') {
    *error = true;
  }
  else  {
    // if one of the values isn't zero, read in the sum of the two values (only sent when both values are non-zero)
    *rf_sum = rfSerial.parseInt();
    
    // if the summation does not match, report signal integrity problem via error boolean
    if(*rudr_val_temp + *sail_val_temp != *rf_sum) {
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
      digitalWrite(PWM2, LOW);
      float reduction_ratio = (float) (sail_pos - MIN_SAIL_POS) / SLOW_DOWN_THRESHOLD;
      float power = (float) (OFF_LOWER - sail_val) / (float) (OFF_LOWER - MIN) * MOTOR_MAX_SPEED;
      uint8_t reduced_power = (uint8_t) (power * reduction_ratio);
      analogWrite(PWM1, reduced_power);
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

void valAveraging(uint16_t rudr_val_temp, uint16_t sail_val_temp, uint16_t* mean_rudr_val, uint16_t* mean_sail_val) {
  // if index is overflowing, reset to 0
  if(++val_index == MEAN_NUM) {
    val_index = 0;
  }

  // add new values to the array
  rudr_val[val_index] = rudr_val_temp;
  sail_val[val_index] = sail_val_temp;

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