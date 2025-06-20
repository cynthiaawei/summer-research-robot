#define MOTOR_PIN 5 // PWM-capable pin (D5 on Feather nRF52840)

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  analogWrite(MOTOR_PIN, 100); // Set PWM to 50% (0-255 range)
  delay(500); // Run motor for 0.5 seconds
  analogWrite(MOTOR_PIN, 0); // Stop motor
}

void loop() {
  // Do nothing, motor remains off
}
