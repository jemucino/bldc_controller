#include <Arduino.h>

// Hold hall sensor states
bool hall_a;
bool hall_b;
bool hall_c;

void setup() {
  // put your setup code here, to run once:
  attachInterrupt(digitalPinToInterrupt(pin), hall_sensor_state, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin), hall_sensor_state, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin), hall_sensor_state, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void update_hall_sensor_state() {
  // update hall sensor states
  
}
