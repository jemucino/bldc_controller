/*
  BLDCControl.h - Library for controlling a three phase bridge to drive a BLDC motor.
  Created by J. Eduardo Mucino on 03 March 2019.
*/
#ifndef BLDCControl_h
#define BLDCControl_h

#include <Arduino.h>

// Pin definitions
#define U_HI 6
#define U_LO A0
#define V_HI 12
#define V_LO A1
#define W_HI 13
#define W_LO A2

#define HALL_U 11 // pin 11 -> PCINT7
#define HALL_V 10 // pin 10 -> PCINT6
#define HALL_W 9  // pin 9 -> PCINT5

#define U_PUMP 2  // pin 2 -> INT1
#define V_PUMP 0  // pin 0 -> INT2
#define W_PUMP 1  // pin 1 -> INT3

#define POT_PIN A5

// Define register set/clear
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Define HALL sensor register read
#define hall_u ((_SFR_BYTE(PINB) & _BV(PB7)) > 0) // PCINT7 -> PB7
#define hall_v ((_SFR_BYTE(PINB) & _BV(PD6)) > 0) // PCINT6 -> PB6
#define hall_w ((_SFR_BYTE(PINB) & _BV(PB5)) > 0) // PCINT5 -> PB5

class BLDCControl {
  public:
    BLDCControl();
    void initialize();
    void set_duty_cycle(int value);
  private:
    static void update_commutation();
    static void charge_pump_u();
    static void charge_pump_v();
    static void charge_pump_w();
};

#endif
