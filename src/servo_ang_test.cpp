#include <Servo.h>
#include <Arduino.h>
// This script is used to validate that a single MG996R servo is able to turn effectively within its specified angle range without issue
// Create servo object
Servo servo;

void setup() {
  // Set servo to pin 3 on arduino
  servo.attach(3);
}

void loop() {
  // Increment in 45 degree steps, one second per increment.
  servo.write(0);
  delay(1000);
  servo.write(45);
  delay(1000);
  servo.write(90);
  delay(1000);
  servo.write(135);
  delay(1000);
  servo.write(180);
  delay(1000);
}
