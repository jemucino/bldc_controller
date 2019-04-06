/*
  BLDCControl.h - Library for controlling a three phase bridge to drive a BLDC motor.
  Created by J. Eduardo Mucino on 03 March 2019.
*/
// #include <Arduino.h>
//
// #include <EnableInterrupt.h>

#include "BLDCControl.h"

// #include <EnableInterrupt.h>

// Initialize direction and duty cycle
bool BLDCControl::direction = 0;
uint8_t BLDCControl::duty_cycle = 0;
uint16_t BLDCControl::pwm_seq[4] = {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF};
const uint16_t pwm_seq_zero[4] = {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF};

// BLDC commutation functions
void uv() {
  // Open all bridge switches
  memcpy(BLDCControl::pwm_seq, pwm_seq_zero, sizeof(BLDCControl::pwm_seq));
  BRIDGE_EGU->TASKS_TRIGGER[UVW_LO_DIS_EGU_CH] = 1;

  // Close U_HI and V_LO switches
  BLDCControl::pwm_seq[U_HI_PWM_CH] = (uint8_t) ~BLDCControl::duty_cycle;
  BRIDGE_EGU->TASKS_TRIGGER[V_LO_ENA_EGU_CH] = 1;
}

void uw() {
  // Open all bridge switches
  memcpy(BLDCControl::pwm_seq, pwm_seq_zero, sizeof(BLDCControl::pwm_seq));
  BRIDGE_EGU->TASKS_TRIGGER[UVW_LO_DIS_EGU_CH] = 1;

  // Close U_HI and W_LO switches
  BLDCControl::pwm_seq[U_HI_PWM_CH] = (uint8_t) ~BLDCControl::duty_cycle;
  BRIDGE_EGU->TASKS_TRIGGER[W_LO_ENA_EGU_CH] = 1;
}

void vw() {
  // Open all bridge switches
  memcpy(BLDCControl::pwm_seq, pwm_seq_zero, sizeof(BLDCControl::pwm_seq));
  BRIDGE_EGU->TASKS_TRIGGER[UVW_LO_DIS_EGU_CH] = 1;

  // Close V_HI and W_LO switches
  BLDCControl::pwm_seq[V_HI_PWM_CH] = (uint8_t) ~BLDCControl::duty_cycle;
  BRIDGE_EGU->TASKS_TRIGGER[W_LO_ENA_EGU_CH] = 1;
}

void vu() {
  // Open all bridge switches
  memcpy(BLDCControl::pwm_seq, pwm_seq_zero, sizeof(BLDCControl::pwm_seq));
  BRIDGE_EGU->TASKS_TRIGGER[UVW_LO_DIS_EGU_CH] = 1;

  // Close V_HI and U_LO switches
  BLDCControl::pwm_seq[V_HI_PWM_CH] = (uint8_t) ~BLDCControl::duty_cycle;
  BRIDGE_EGU->TASKS_TRIGGER[U_LO_ENA_EGU_CH] = 1;
}

void wu() {
  // Open all bridge switches
  memcpy(BLDCControl::pwm_seq, pwm_seq_zero, sizeof(BLDCControl::pwm_seq));
  BRIDGE_EGU->TASKS_TRIGGER[UVW_LO_DIS_EGU_CH] = 1;

  // Close W_HI and U_LO switches
  BLDCControl::pwm_seq[W_HI_PWM_CH] = (uint8_t) ~BLDCControl::duty_cycle;
  BRIDGE_EGU->TASKS_TRIGGER[U_LO_ENA_EGU_CH] = 1;
}

void wv() {
  // Open all bridge switches
  memcpy(BLDCControl::pwm_seq, pwm_seq_zero, sizeof(BLDCControl::pwm_seq));
  BRIDGE_EGU->TASKS_TRIGGER[UVW_LO_DIS_EGU_CH] = 1;

  // Close W_HI and V_LO switches
  BLDCControl::pwm_seq[W_HI_PWM_CH] = (uint8_t) ~BLDCControl::duty_cycle;
  BRIDGE_EGU->TASKS_TRIGGER[V_LO_ENA_EGU_CH] = 1;
}

void coast() {
  // Open all bridge switches
  memcpy(BLDCControl::pwm_seq, pwm_seq_zero, sizeof(BLDCControl::pwm_seq));
  BRIDGE_EGU->TASKS_TRIGGER[UVW_LO_DIS_EGU_CH] = 1;
}

void brake() {
  // Open all high-side switches
  memcpy(BLDCControl::pwm_seq, pwm_seq_zero, sizeof(BLDCControl::pwm_seq));

  // Close all low-side switches
  BRIDGE_EGU->TASKS_TRIGGER[U_LO_ENA_EGU_CH] = 1;
  BRIDGE_EGU->TASKS_TRIGGER[V_LO_ENA_EGU_CH] = 1;
  BRIDGE_EGU->TASKS_TRIGGER[W_LO_ENA_EGU_CH] = 1;
}

// BLDC commutation lookup table
void  (*commutation_functions[])() = {coast, vw, uv, uw, wu, vu, wv, coast, coast, wv, vu, wu, uw, uv, vw, coast};

BLDCControl::BLDCControl() {
  // Set hall sensor pins to inputs with pullups and initialize
  pinMode(U_HALL, INPUT_PULLUP);
  pinMode(V_HALL, INPUT_PULLUP);
  pinMode(W_HALL, INPUT_PULLUP);

  // Attach interrupts to hall sensor pins
 attachInterrupt(U_HALL, update_commutation, CHANGE);
 attachInterrupt(V_HALL, update_commutation, CHANGE);
 attachInterrupt(W_HALL, update_commutation, CHANGE);
}

void BLDCControl::initialize() {
  // Initialize the three phase bridge
  initialize_three_phase_bridge();

  // // Attach interrupts to hall sensor pins
  // attachInterrupt(U_HALL, update_commutation, CHANGE);
  // attachInterrupt(V_HALL, update_commutation, CHANGE);
  // attachInterrupt(W_HALL, update_commutation, CHANGE);

  // Initialize hall sensors
  update_commutation();
}

void BLDCControl::rate_command(uint duty_cycle_cmd, bool direction_cmd) {
  // Set commutation direction and duty cycle
  duty_cycle = duty_cycle_cmd;
  direction = direction_cmd;

  // Update commutation
  update_commutation();
}

void BLDCControl::update_commutation() {
  // BLDC commutation lookup table
  (*commutation_functions[((direction<<3) | UVW_HALL_IN)])();
}

void BLDCControl::initialize_high_side_switches() {
  // Configure pins in GPIO peripheral
  // TODO: Set registers directly
  pinMode(U_HI, OUTPUT);
  digitalWrite(U_HI, LOW);
  pinMode(V_HI, OUTPUT);
  digitalWrite(V_HI, LOW);
  pinMode(W_HI, OUTPUT);
  digitalWrite(W_HI, LOW);

  // Configure pins in PWM peripheral
  BRIDGE_PWM->PSEL.OUT[U_HI_PWM_CH] = (U_HI << PWM_PSEL_OUT_PIN_Pos) |
                                      (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
  BRIDGE_PWM->PSEL.OUT[V_HI_PWM_CH] = (V_HI << PWM_PSEL_OUT_PIN_Pos) |
                                      (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
  BRIDGE_PWM->PSEL.OUT[W_HI_PWM_CH] = (W_HI << PWM_PSEL_OUT_PIN_Pos) |
                                      (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);

  // Enable PWMn peripheral
  BRIDGE_PWM->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);

  // Configure PWM mode and frequency
  BRIDGE_PWM->MODE              = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
  BRIDGE_PWM->PRESCALER         = (PWM_PRESCALER_PRESCALER_DIV_2 << PWM_PRESCALER_PRESCALER_Pos);
  BRIDGE_PWM->COUNTERTOP        = (255 << PWM_COUNTERTOP_COUNTERTOP_Pos);

  // Define PWM sequencing
  BRIDGE_PWM->LOOP              = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);
  BRIDGE_PWM->DECODER           = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) |
                                  (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
  BRIDGE_PWM->SEQ[0].PTR        = ((uint32_t)(pwm_seq) << PWM_SEQ_PTR_PTR_Pos);
  BRIDGE_PWM->SEQ[0].CNT        = ((sizeof(pwm_seq) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
  BRIDGE_PWM->SEQ[0].REFRESH    = 0;
  BRIDGE_PWM->SEQ[0].ENDDELAY   = 0;

  // Begin PWM sequence
  BRIDGE_PWM->TASKS_SEQSTART[0] = 1;
}

void BLDCControl::initialize_low_side_switches() {
  // Set up the low-side bridge switches in toggle mode, inital value low
  NRF_GPIOTE->CONFIG[U_LO_GPIOTE_CH] = GPIOTE_CONFIG_MODE_Task       << GPIOTE_CONFIG_MODE_Pos |
                                       GPIOTE_CONFIG_OUTINIT_Low     << GPIOTE_CONFIG_OUTINIT_Pos |
                                       GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
                                       U_LO                          << GPIOTE_CONFIG_PSEL_Pos;
  NRF_GPIOTE->CONFIG[V_LO_GPIOTE_CH] = GPIOTE_CONFIG_MODE_Task       << GPIOTE_CONFIG_MODE_Pos |
                                       GPIOTE_CONFIG_OUTINIT_Low     << GPIOTE_CONFIG_OUTINIT_Pos |
                                       GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
                                       V_LO                          << GPIOTE_CONFIG_PSEL_Pos;
  NRF_GPIOTE->CONFIG[W_LO_GPIOTE_CH] = GPIOTE_CONFIG_MODE_Task       << GPIOTE_CONFIG_MODE_Pos |
                                       GPIOTE_CONFIG_OUTINIT_Low     << GPIOTE_CONFIG_OUTINIT_Pos |
                                       GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
                                       W_LO                          << GPIOTE_CONFIG_PSEL_Pos;

   // Configure PPI to enable/disable low-side switches
   NRF_PPI->CH[PPI_CH_E].EEP = (uint32_t)&BRIDGE_EGU->EVENTS_TRIGGERED[U_LO_ENA_EGU_CH];
   NRF_PPI->CH[PPI_CH_E].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[U_LO_GPIOTE_CH];
   NRF_PPI->FORK[PPI_CH_E].TEP = (uint32_t)&BRIDGE_PWM->TASKS_SEQSTART[0];
   NRF_PPI->CH[PPI_CH_F].EEP = (uint32_t)&BRIDGE_EGU->EVENTS_TRIGGERED[UVW_LO_DIS_EGU_CH];
   NRF_PPI->CH[PPI_CH_F].TEP = (uint32_t)&NRF_GPIOTE->TASKS_CLR[U_LO_GPIOTE_CH];
   NRF_PPI->FORK[PPI_CH_F].TEP = (uint32_t)&BRIDGE_PWM->TASKS_SEQSTART[0];

   NRF_PPI->CH[PPI_CH_G].EEP = (uint32_t)&BRIDGE_EGU->EVENTS_TRIGGERED[V_LO_ENA_EGU_CH];
   NRF_PPI->CH[PPI_CH_G].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[V_LO_GPIOTE_CH];
   NRF_PPI->FORK[PPI_CH_G].TEP = (uint32_t)&BRIDGE_PWM->TASKS_SEQSTART[0];
   NRF_PPI->CH[PPI_CH_H].EEP = (uint32_t)&BRIDGE_EGU->EVENTS_TRIGGERED[UVW_LO_DIS_EGU_CH];
   NRF_PPI->CH[PPI_CH_H].TEP = (uint32_t)&NRF_GPIOTE->TASKS_CLR[V_LO_GPIOTE_CH];
   NRF_PPI->FORK[PPI_CH_H].TEP = (uint32_t)&BRIDGE_PWM->TASKS_SEQSTART[0];

   NRF_PPI->CH[PPI_CH_I].EEP = (uint32_t)&BRIDGE_EGU->EVENTS_TRIGGERED[W_LO_ENA_EGU_CH];
   NRF_PPI->CH[PPI_CH_I].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[W_LO_GPIOTE_CH];
   NRF_PPI->FORK[PPI_CH_I].TEP = (uint32_t)&BRIDGE_PWM->TASKS_SEQSTART[0];
   NRF_PPI->CH[PPI_CH_J].EEP = (uint32_t)&BRIDGE_EGU->EVENTS_TRIGGERED[UVW_LO_DIS_EGU_CH];
   NRF_PPI->CH[PPI_CH_J].TEP = (uint32_t)&NRF_GPIOTE->TASKS_CLR[W_LO_GPIOTE_CH];
   NRF_PPI->FORK[PPI_CH_J].TEP = (uint32_t)&BRIDGE_PWM->TASKS_SEQSTART[0];

   // Enable the appropriate PPI channels
   NRF_PPI->CHENSET |= (1 << PPI_CH_E) |
                       (1 << PPI_CH_F) |
                       (1 << PPI_CH_G) |
                       (1 << PPI_CH_H) |
                       (1 << PPI_CH_I) |
                       (1 << PPI_CH_J);
}

void BLDCControl::initialize_three_phase_bridge() {
  /*
  This function configures the nrf52832 PPI to close the low-side bridge
  switches once per PWM cycle to ensure the charge pump capacitors are
  charged.

  The low side switches are closed in succession. The timing is controlled
  by TIMERn (see BLDCControlConfig.h). When PWMn (see BLDCControlConfig.h)
  generates the PWMPERIODEND event, the PPI triggers the START task on
  TIMERn. The PPI subsequently pulses the low-side switches in order (U, V,
  W) for two clock cycles each (125 ns). The PPI finally triggers the STOP
  task on TIMERn. TIMERn is cleared at the end of every sequence.
  */

  // Initialize switches
  initialize_high_side_switches();  // Configure high-side switches for PWM
  initialize_low_side_switches();   // Configure low_side switches for GPIOTE

  // Set up TIMERn
  BRIDGE_TIMER->BITMODE       = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
  BRIDGE_TIMER->PRESCALER     = 0;
  BRIDGE_TIMER->SHORTS        = TIMER_SHORTS_COMPARE5_CLEAR_Msk;
  BRIDGE_TIMER->MODE          = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
  BRIDGE_TIMER->CC[3]         = 2;
  BRIDGE_TIMER->CC[4]         = 4;
  BRIDGE_TIMER->CC[5]         = 6;

  // Configure PPI to pulse low-side switches sequentially at the start of each PWM period
  NRF_PPI->CH[PPI_CH_A].EEP   = (uint32_t)&BRIDGE_PWM->EVENTS_PWMPERIODEND;
  NRF_PPI->CH[PPI_CH_A].TEP   = (uint32_t)&NRF_GPIOTE->TASKS_OUT[U_LO_GPIOTE_CH];
  NRF_PPI->FORK[PPI_CH_A].TEP = (uint32_t)&BRIDGE_TIMER->TASKS_START;

  NRF_PPI->CH[PPI_CH_B].EEP   = (uint32_t)&BRIDGE_TIMER->EVENTS_COMPARE[3];
  NRF_PPI->CH[PPI_CH_B].TEP   = (uint32_t)&NRF_GPIOTE->TASKS_OUT[U_LO_GPIOTE_CH];
  NRF_PPI->FORK[PPI_CH_B].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[V_LO_GPIOTE_CH];

  NRF_PPI->CH[PPI_CH_C].EEP   = (uint32_t)&BRIDGE_TIMER->EVENTS_COMPARE[4];
  NRF_PPI->CH[PPI_CH_C].TEP   = (uint32_t)&NRF_GPIOTE->TASKS_OUT[V_LO_GPIOTE_CH];
  NRF_PPI->FORK[PPI_CH_C].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[W_LO_GPIOTE_CH];

  NRF_PPI->CH[PPI_CH_D].EEP   = (uint32_t)&BRIDGE_TIMER->EVENTS_COMPARE[5];
  NRF_PPI->CH[PPI_CH_D].TEP   = (uint32_t)&NRF_GPIOTE->TASKS_OUT[W_LO_GPIOTE_CH];
  NRF_PPI->FORK[PPI_CH_D].TEP = (uint32_t)&BRIDGE_TIMER->TASKS_STOP;

  // Enable the appropriate PPI channels
  NRF_PPI->CHENSET |= (1 << PPI_CH_A) |
                      (1 << PPI_CH_B) |
                      (1 << PPI_CH_C) |
                      (1 << PPI_CH_D);
}
