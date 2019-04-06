/*
  BLDCControlConfig.h - Library for controlling a three phase bridge to drive a BLDC motor.
  Created by J. Eduardo Mucino on 01 April 2019.
*/
#ifndef BLDCControlConfig_h
#define BLDCControlConfig_h

#include <Arduino.h>

// Pin definitions
#define U_HI          2
#define U_LO          3
#define V_HI          4
#define V_LO          5
#define W_HI          15
#define W_LO          16

#define HALL_U        25
#define HALL_V        26
#define HALL_W        27
#define HALL_UVW_IN   (((NRF_GPIO->IN)&0x0E000000)>>25)

#define U_I           15
#define V_I           16
#define W_I           17

/*
NOTES:
- When Nordic SoftDecice is enabled:
  - Only PPI channels 0-16 are available
  - All GPIOTE channels available
  - TIMER1 and TIMER2 are available, TIMER0 access is blocked
  - PWM1 and PWM2 are available, PWM0 access is unknown
  - EGU0 and EGU3 are available; EGU1 is restricted; and EGU2, EGU4, and EGU5 are blocked
- ...
*/

// Define which peripherals to use for bridge control
#define BRIDGE_TIMER  NRF_TIMER4 // Must be TIMER3 or TIMER4
#define BRIDGE_PWM    NRF_PWM1
#define BRIDGE_EGU    NRF_EGU0

#endif
