/*
 * Date Created:    August 10th, 2023
 * Last Modified:   August 11th, 2023
 * Filename:        serial_com_receiver.ino
 * Purpose:         Read messages from Serial port then send the messages using the RF module.
 * Microcontroller: Arduino Uno R3
 * Connections:     See controller_schematics located in the schematics folder
 */
 
 // Setup
 void setup() {
  // Begin the Serial at 9600 Baud
  Serial.begin(9600);
}

// Main loop
void loop() {
  // Read all messages from the serial port
  char read = Serial.read();

  // If the message is valid (not -1), send it to the RF module.
  if (read != -1){
    Serial.print(read);
  }
}