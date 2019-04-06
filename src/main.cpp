#include <Arduino.h>

#include <BLDCControl.h>

// Pot value and duty cycle
uint8_t duty_cycle = 0;
const uint max_duty_cycle = 250;

// Timer variables
uint current_time = 0;
uint previous_time = 0;
uint counter = 0;

// Create BLDCControl instance
BLDCControl bldc_control;

void setup() {
  // Initialize bldc control
  bldc_control.initialize();

  bldc_control.rate_command(127, 0);

  // Initialize serial port
  Serial.begin(9600);
}

void loop() {
  current_time = millis();

  if (current_time - previous_time > 10) {
    // bldc_control.rate_command(duty_cycle, 0);
    if (duty_cycle < 50) {
      duty_cycle++;
    } else {
      duty_cycle = 0;
    }
    previous_time = current_time;
    counter += 1;
  }

  if (counter >= 100) {
    // duty_cycle += 64;
    counter = 0;
    Serial.println(bldc_control.pwm_seq[0]);
    Serial.println(bldc_control.pwm_seq[1]);
    Serial.println(bldc_control.pwm_seq[2]);
    Serial.println(bldc_control.pwm_seq[3]);
  }
}
