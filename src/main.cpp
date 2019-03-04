#include <Arduino.h>

#include <BLDCControl.h>

// Pot value and duty cycle
int pot_val;
const int max_duty_cycle = 250;

// Create BLDCControl instance
BLDCControl bldc_control;

void setup() {
  // Initialize bldc control
  bldc_control.initialize();

  // // Initialize serial port
  // Serial.begin(9600);
}

void loop() {
  pot_val = analogRead(POT_PIN);
  bldc_control.set_duty_cycle(map(pot_val, 0, 1023, 0, max_duty_cycle));

  delay(20);

  // // Print debug info
  // Serial.println(OCR4A, HEX);
  // Serial.println(OCR4C, HEX);
}
