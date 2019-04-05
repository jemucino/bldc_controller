/*
  BLDCControlConfig.h - Library for controlling a three phase bridge to drive a BLDC motor.
  Created by J. Eduardo Mucino on 01 April 2019.
*/
#ifndef BLDCControlConfig_h
#define BLDCControlConfig_h

#include <Arduino.h>

// Pin definitions
#define U_HI 21
#define U_LO 20
#define V_HI 19
#define V_LO 29
#define W_HI 25
#define W_LO 30

// Interrupt assignments for Feather 32u4
// pin 11 -> PCINT7
// pin 10 -> PCINT6
// pin 9 -> PCINT5
// pin 2 -> INT1
// pin 0 -> INT2
// pin 1 -> INT3

// Interrupt assignments for Feather nrf52
// The interrupt number follows the peripheral ID

#define HALL_U 18
#define HALL_V 5
#define HALL_W 6

// #define U_PUMP
// #define V_PUMP
// #define W_PUMP

#define U_I 15
#define V_I 16
#define W_I 17

// Define nrf52832 peripheral channels
#define GPIOTE_CH_A 0
#define PIN_A       7

#define PPI_CH_A    0

#define PPI_CH_B    1

#endif
