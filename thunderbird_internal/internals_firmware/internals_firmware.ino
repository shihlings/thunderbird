// define constants and pins 
  #define PWM1          5
  #define PWM2          6
  #define RAMP_INTERVAL 100
  #define RAMP_PAUSE    1000

// setup
void setup() {
  // setup pins for motor pwm
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
}

// main loop
void loop() {
  // ramp up counterclockwise
  digitalWrite(PWM2, 0);
  for (int index = 0; index <= 255; index++) {
    analogWrite(PWM1, index);
    delay(RAMP_INTERVAL);
  }
  delay(RAMP_PAUSE);

  // ramp down counterclockwise
  for (int index = 255; index >= 0; index--) {
    analogWrite(PWM1, index);
    delay(RAMP_INTERVAL);
  }
  delay(RAMP_PAUSE);

  // ramp up clockwise
  digitalWrite(PWM1, 0);
  for (int index = 0; index <= 255; index++) {
    analogWrite(PWM2, index);
    delay(RAMP_INTERVAL);
  }
  delay(RAMP_PAUSE);

  // ramp down clockwise
  for (int index = 255; index >= 0; index--) {
    analogWrite(PWM2, index);
    delay(RAMP_INTERVAL);
  }
  delay(RAMP_PAUSE);
}
