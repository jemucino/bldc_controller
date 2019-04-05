/*
  BLDCControl.h - Library for controlling a three phase bridge to drive a BLDC motor.
  Created by J. Eduardo Mucino on 03 March 2019.
*/
#ifndef BLDCControl_h
#define BLDCControl_h

#include <Arduino.h>

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

// Define which timer to use for bridge control
#define BRIDGE_TIMER NRF_TIMER4

// Define GPIOTE and PPI channels to use for bridge control
#define U_LO_GPIOTE_CH 0
#define V_LO_GPIOTE_CH 1
#define W_LO_GPIOTE_CH 2
#define PPI_CH_A    0
#define PPI_CH_B    1
#define PPI_CH_C    2
#define PPI_CH_D    3

// Define HALL sensor register read
#define U_HALL_GPIOTE_CH 3
#define U_HALL_GPIOTE_CH 4
#define U_HALL_GPIOTE_CH 5

// Declare BLDCControl class
class BLDCControl {
  public:
    static bool direction;

    BLDCControl();
    void initialize();
    void rate_command(int duty_cycle, bool direction);
  private:
    void set_duty_cycle(int duty_cycle);
    void set_direction(bool direction);

    static void update_commutation();
    static void charge_pump_u();
    static void charge_pump_v();
    static void charge_pump_w();
};

#endif
