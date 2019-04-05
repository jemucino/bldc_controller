/*
  BLDCControl.h - Library for controlling a three phase bridge to drive a BLDC motor.
  Created by J. Eduardo Mucino on 03 March 2019.
*/
// #include <Arduino.h>
//
// #include <EnableInterrupt.h>

#include "BLDCControl.h"

// #include <EnableInterrupt.h>

// Initialize direction
static bool BLDCControl::direction = 0;

// BLDC commutation functions
void uv() {
    // cbi(TCCR4E,OC4OE5);
    cbi(PORTF, PF7);
    cbi(TCCR4E,OC4OE4);
    // cbi(PORTF, PF6);
    cbi(TCCR4E,OC4OE1);
    cbi(PORTF, PF5);

    sbi(PORTF, PF6);
    sbi(TCCR4E,OC4OE5);
}

void uw() {
  // cbi(TCCR4E,OC4OE5);
  cbi(PORTF, PF7);
  cbi(TCCR4E,OC4OE4);
  cbi(PORTF, PF6);
  cbi(TCCR4E,OC4OE1);
  // cbi(PORTF, PF5);

  sbi(PORTF, PF5);
  sbi(TCCR4E,OC4OE5);
}

void vw() {
  cbi(TCCR4E,OC4OE5);
  cbi(PORTF, PF7);
  // cbi(TCCR4E,OC4OE4);
  cbi(PORTF, PF6);
  cbi(TCCR4E,OC4OE1);
  // cbi(PORTF, PF5);

  sbi(PORTF, PF5);
  sbi(TCCR4E,OC4OE4);
}

void vu() {
  cbi(TCCR4E,OC4OE5);
  // cbi(PORTF, PF7);
  // cbi(TCCR4E,OC4OE4);
  cbi(PORTF, PF6);
  cbi(TCCR4E,OC4OE1);
  cbi(PORTF, PF5);

  sbi(PORTF, PF7);
  sbi(TCCR4E,OC4OE4);
}

void wu() {
  cbi(TCCR4E,OC4OE5);
  // cbi(PORTF, PF7);
  cbi(TCCR4E,OC4OE4);
  cbi(PORTF, PF6);
  // cbi(TCCR4E,OC4OE1);
  cbi(PORTF, PF5);

  sbi(PORTF, PF7);
  sbi(TCCR4E,OC4OE1);
}

void wv() {
  cbi(TCCR4E,OC4OE5);
  cbi(PORTF, PF7);
  cbi(TCCR4E,OC4OE4);
  // cbi(PORTF, PF6);
  // cbi(TCCR4E,OC4OE1);
  cbi(PORTF, PF5);

  sbi(PORTF, PF6);
  sbi(TCCR4E,OC4OE1);
}

void coast() {
  // Coast
  cbi(TCCR4E,OC4OE5);
  cbi(PORTF, PF7);
  cbi(TCCR4E,OC4OE4);
  cbi(PORTF, PF6);
  cbi(TCCR4E,OC4OE1);
  cbi(PORTF, PF5);
}

// BLDC commutation lookup table
void  (*commutation_functions[])() = {coast, vw, uv, uw, wu, vu, wv, coast, coast, wv, vu, wu, uw, uv, vw, coast};

BLDCControl::BLDCControl() {
  // Set low side switch pins to outputs
  pinMode(U_LO, OUTPUT);  // sets the pin as output
  pinMode(V_LO, OUTPUT);  // sets the pin as output
  pinMode(W_LO, OUTPUT);  // sets the pin as output

  // Assign high side switch pins to PWM hw
  HwPWM0.addPin(U_HI);
  HwPWM0.addPin(V_HI);
  HwPWM0.addPin(W_HI);

  // Set hall pins to inputs with pullups and initialize
  pinMode(HALL_U, INPUT_PULLUP);
  pinMode(HALL_V, INPUT_PULLUP);
  pinMode(HALL_W, INPUT_PULLUP);

  // Attach/enable interrupts
  // enableInterrupt(U_PUMP, charge_pump_u, FALLING); // pin 2 -> INT1
  // enableInterrupt(V_PUMP, charge_pump_v, FALLING); // pin 0 -> INT2
  // enableInterrupt(W_PUMP, charge_pump_w, FALLING); // pin 1 -> INT3
  attachInterrupt(HALL_U, update_commutation, CHANGE); // pin 11 -> PCINT7
  attachInterrupt(HALL_V, update_commutation, CHANGE); // pin 10 -> PCINT6
  attachInterrupt(HALL_W, update_commutation, CHANGE); // pin 9 -> PCINT5
}

void BLDCControl::initialize() {
  // Configure for PWM6 on PD6, PD7, and PC7
  TCCR4A |= _BV(PWM4A);   // Enables PWM mode based on comparator OCR4A

  TCCR4A |= _BV(COM4A1);  // Set OC4A mode
  TCCR4C |= _BV(COM4D1);  // Set OC4D and _OC4D_ mode

  TCCR4B &= (B11110000);  // Clear the existing prescaler bits
  TCCR4B |= _BV(CS40);    // Set the new prescaler value (1:1)

  TCCR4D |= _BV(WGM41);   // Set the WGM41 bit to set PWM6 mode
  TCCR4D &= ~_BV(WGM40);  // Clear the WGM40 bit to set single-slope mode

  // Initialize hall sensors
  update_commutation();
}

void BLDCControl::rate_command(int duty_cycle, bool direction) {
  set_duty_cycle(duty_cycle);
  set_direction(direction);
  update_commutation();
}

void BLDCControl::set_duty_cycle(int value) {
  // In PWM6 mode the duty cycle of all pins is control by the OCR4A register
  OCR4A = value;  // Write to OCR4A register in 8-bit mode
}

void BLDCControl::set_direction(bool value) {
  // In PWM6 mode the duty cycle of all pins is control by the OCR4A register
  direction = value;  // Write to OCR4A register in 8-bit mode
}

// ISR definitions

void BLDCControl::update_commutation() {
  // BLDC commutation lookup table
  (*commutation_functions[((direction<<3)|(hall_u<<2)|(hall_v<<1)|(hall_w))])();
}

void BLDCControl::charge_pump_u() {
  // U phase charge pump
  sbi(PORTD, 1);  // INT1 -> PD1
  cbi(PORTD, 1);
}

void BLDCControl::charge_pump_v() {
  // V phase charge pump
  sbi(PORTD, 2); // INT2 -> PD2
  cbi(PORTD, 2);
}

void BLDCControl::charge_pump_w() {
  // W phase charge pump
  sbi(PORTD, 3);  // INT3 -> PD3
  cbi(PORTD, 3);
}

void BLDCControl::initialize_blank() {
  // Set up pin A in toggle mode, inital value low
  NRF_GPIOTE->CONFIG[GPIOTE_CH_A] = GPIOTE_CONFIG_MODE_Task       << GPIOTE_CONFIG_MODE_Pos |
                                    GPIOTE_CONFIG_OUTINIT_Low     << GPIOTE_CONFIG_OUTINIT_Pos |
                                    GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
                                    PIN_A                         << GPIOTE_CONFIG_PSEL_Pos;
  NRF_GPIOTE->CONFIG[GPIOTE_CH_B] = GPIOTE_CONFIG_MODE_Task       << GPIOTE_CONFIG_MODE_Pos |
                                    GPIOTE_CONFIG_OUTINIT_Low     << GPIOTE_CONFIG_OUTINIT_Pos |
                                    GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
                                    PIN_B                         << GPIOTE_CONFIG_PSEL_Pos;
  NRF_GPIOTE->CONFIG[GPIOTE_CH_C] = GPIOTE_CONFIG_MODE_Task       << GPIOTE_CONFIG_MODE_Pos |
                                    GPIOTE_CONFIG_OUTINIT_Low     << GPIOTE_CONFIG_OUTINIT_Pos |
                                    GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
                                    PIN_C                         << GPIOTE_CONFIG_PSEL_Pos;
  // Set up TIMER4
  NRF_TIMER4->BITMODE                 = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
  NRF_TIMER4->PRESCALER               = 0;
  NRF_TIMER4->SHORTS                  = TIMER_SHORTS_COMPARE5_CLEAR_Msk;
  NRF_TIMER4->MODE                    = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
  NRF_TIMER4->CC[3]                   = 2;
  NRF_TIMER4->CC[4]                   = 4;
  NRF_TIMER4->CC[5]                   = 6;

  // Have CC[0] in the timer toggle both pin A and pin B (using the FORK feature to control two tasks)
  NRF_PPI->CH[PPI_CH_A].EEP   = (uint32_t)&NRF_PWM0->EVENTS_PWMPERIODEND;
  NRF_PPI->CH[PPI_CH_A].TEP   = (uint32_t)&NRF_GPIOTE->TASKS_OUT[GPIOTE_CH_A];
  NRF_PPI->FORK[PPI_CH_A].TEP = (uint32_t)&NRF_TIMER4->TASKS_START;
  NRF_PPI->CH[PPI_CH_B].EEP   = (uint32_t)&NRF_TIMER4->EVENTS_COMPARE[3];
  NRF_PPI->CH[PPI_CH_B].TEP   = (uint32_t)&NRF_GPIOTE->TASKS_OUT[GPIOTE_CH_A];
  NRF_PPI->FORK[PPI_CH_B].TEP   = (uint32_t)&NRF_GPIOTE->TASKS_OUT[GPIOTE_CH_B];
  NRF_PPI->CH[PPI_CH_C].EEP   = (uint32_t)&NRF_TIMER4->EVENTS_COMPARE[4];
  NRF_PPI->CH[PPI_CH_C].TEP   = (uint32_t)&NRF_GPIOTE->TASKS_OUT[GPIOTE_CH_B];
  NRF_PPI->FORK[PPI_CH_C].TEP   = (uint32_t)&NRF_GPIOTE->TASKS_OUT[GPIOTE_CH_C];
  NRF_PPI->CH[PPI_CH_D].EEP   = (uint32_t)&NRF_TIMER4->EVENTS_COMPARE[5];
  NRF_PPI->CH[PPI_CH_D].TEP   = (uint32_t)&NRF_GPIOTE->TASKS_OUT[GPIOTE_CH_C];
  NRF_PPI->FORK[PPI_CH_D].TEP = (uint32_t)&NRF_TIMER4->TASKS_STOP;

  // Enable the appropriate PPI channels and timer interrupts
  NRF_PPI->CHENSET = (1 << PPI_CH_A) | (1 << PPI_CH_B) | (1 << PPI_CH_C) | (1 << PPI_CH_D);
}
