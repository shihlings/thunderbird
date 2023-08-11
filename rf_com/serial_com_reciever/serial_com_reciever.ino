/*
 * Date Created:    August 10th, 2023
 * Last Modified:   August 11th, 2023
 * Filename:        serial_com_receiver.ino
 * Purpose:         Receive messages from the RF module, then print it on the screen.
 * Microcontroller: Arduino Uno R3
 * Connections:     See internal_schematics located in the schematics folder
 */
 
 // Setup
 void setup() {
  // Begin the Serial at 9600 Baud
  Serial.begin(9600);
}

// Main loop
void loop() {
  // read all messages coming from the RF module
  int receive = Serial.read();

  // if there is any valid message (not -1), translate it into ASCII character and print on screen.
  if(receive != -1) {
    char print = receive;
    Serial.print(print);
  }
}
