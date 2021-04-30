/*
  BLDCControl.h - Library for controlling a three phase bridge to drive a BLDC motor.
  Created by J. Eduardo Mucino on 03 March 2019.
*/
#ifndef BLDCControl_h
#define BLDCControl_h

#include <Arduino.h>
#include "BLDCControlConfig.h"

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

// Define GPIOTE and PPI channels to use for bridge control
#define U_LO_GPIOTE_CH      0
#define V_LO_GPIOTE_CH      1
#define W_LO_GPIOTE_CH      2
#define PPI_CH_A            0
#define PPI_CH_B            1
#define PPI_CH_C            2
#define PPI_CH_D            3

#define PPI_CH_E            4
#define PPI_CH_F            5
#define PPI_CH_G            6
#define PPI_CH_H            7
#define PPI_CH_I            8
#define PPI_CH_J            9

// Define HALL sensor register read
#define U_HALL_GPIOTE_CH    3
#define V_HALL_GPIOTE_CH    4
#define W_HALL_GPIOTE_CH    5

// Define PWM channels for high-side switches
#define U_HI_PWM_CH         1
#define V_HI_PWM_CH         2
#define W_HI_PWM_CH         3

// Define EGU channels for low-side switches
#define UVW_LO_DIS_EGU_CH   0
#define U_LO_ENA_EGU_CH     1
#define V_LO_ENA_EGU_CH     2
#define W_LO_ENA_EGU_CH     3

// Declare BLDCControl class
class BLDCControl {
  public:
    static bool direction;
    static uint8_t duty_cycle;
    static uint16_t pwm_seq[4];

    BLDCControl();
    void initialize();
    void rate_command(uint duty_cycle, bool direction_cmd);
  private:
    static void update_commutation();
    static void initialize_high_side_switches();
    static void initialize_low_side_switches();
    static void initialize_three_phase_bridge();
    static void initialize_hall_sensor_inputs()
};

#endif
