#include <Arduino.h>

#include <EnableInterrupt.h>

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

// Pot value and duty cycle
int pot_val;
const int max_duty_cycle = 250;

// Function declarations
void update_commutation();

void charge_pump_u();
void charge_pump_v();
void charge_pump_w();

void set_duty_cycle(int value) {
  // In PWM6 mode the duty cycle of all pins is control by the OCR4A register
  OCR4A = value;  // Write to OCR4A register in 8-bit mode
}

void setup() {
  // Set bridge pins to outputs
  pinMode(U_HI, OUTPUT);  // sets the pin as output
  pinMode(U_LO, OUTPUT);  // sets the pin as output
  pinMode(V_HI, OUTPUT);  // sets the pin as output
  pinMode(V_LO, OUTPUT);  // sets the pin as output
  pinMode(W_HI, OUTPUT);  // sets the pin as output
  pinMode(W_LO, OUTPUT);  // sets the pin as output

  // Set hall pins to inputs with pullups and initialize
  pinMode(HALL_U, INPUT_PULLUP);
  pinMode(HALL_V, INPUT_PULLUP);
  pinMode(HALL_W, INPUT_PULLUP);

  // Attach/enable interrupts
  enableInterrupt(U_PUMP, charge_pump_u, FALLING); // pin 2 -> INT1
  enableInterrupt(V_PUMP, charge_pump_v, FALLING); // pin 0 -> INT2
  enableInterrupt(W_PUMP, charge_pump_w, FALLING); // pin 1 -> INT3
  enableInterrupt(HALL_U, update_commutation, CHANGE); // pin 11 -> PCINT7
  enableInterrupt(HALL_V, update_commutation, CHANGE); // pin 10 -> PCINT6
  enableInterrupt(HALL_W, update_commutation, CHANGE); // pin 9 -> PCINT5

  // Configure for PWM6 on PD6, PD7, and PC7
  TCCR4A |= _BV(PWM4A);   // Enables PWM mode based on comparator OCR4A

  TCCR4A |= _BV(COM4A1);  // Set OC4A mode
  TCCR4C |= _BV(COM4D1);  // Set OC4D and _OC4D_ mode

  TCCR4B &= (B11110000);  // Clear the existing prescaler bits
  TCCR4B |= _BV(CS40);    // Set the new prescaler value (1:1)

  TCCR4D |= _BV(WGM41);   // Set the WGM41 bit to set PWM6 mode
  TCCR4D &= ~_BV(WGM40);  // Clear the WGM40 bit to set single-slope mode

  // // Initialize serial port
  // Serial.begin(9600);
}

void loop() {
  pot_val = analogRead(POT_PIN);
  set_duty_cycle(map(pot_val, 0, 1023, 0, max_duty_cycle));

  update_commutation();

  delay(20);

  // // Print debug info
  // Serial.println(OCR4A, HEX);
  // Serial.println(OCR4C, HEX);
}

// ISR definitions

void update_commutation() {
  // BLDC commutation logic
  if ((hall_u && hall_v && hall_w) ||
      (!hall_u && !hall_v && !hall_w)) {
    // Coast
    cbi(TCCR4E,OC4OE5);
    digitalWrite(U_LO, LOW);
    cbi(TCCR4E,OC4OE4);
    digitalWrite(V_LO, LOW);
    cbi(TCCR4E,OC4OE1);
    digitalWrite(W_LO, LOW);

    // Serial.println("Coasting");
  } else if (hall_u && !hall_v && hall_w) {
    // cbi(TCCR4E,OC4OE5);
    digitalWrite(U_LO, LOW);
    cbi(TCCR4E,OC4OE4);
    // digitalWrite(V_LO, LOW);
    cbi(TCCR4E,OC4OE1);
    digitalWrite(W_LO, LOW);

    digitalWrite(V_LO, HIGH);
    sbi(TCCR4E,OC4OE5);

    // Serial.println("Phase I");
  } else if (hall_u && !hall_v && !hall_w) {
    // cbi(TCCR4E,OC4OE5);
    digitalWrite(U_LO, LOW);
    cbi(TCCR4E,OC4OE4);
    digitalWrite(V_LO, LOW);
    cbi(TCCR4E,OC4OE1);
    // digitalWrite(W_LO, LOW);

    digitalWrite(W_LO, HIGH);
    sbi(TCCR4E,OC4OE5);

    // Serial.println("Phase II");
  } else if (hall_u && hall_v && !hall_w) {
    cbi(TCCR4E,OC4OE5);
    digitalWrite(U_LO, LOW);
    // cbi(TCCR4E,OC4OE4);
    digitalWrite(V_LO, LOW);
    cbi(TCCR4E,OC4OE1);
    // digitalWrite(W_LO, LOW);

    digitalWrite(W_LO, HIGH);
    sbi(TCCR4E,OC4OE4);

    // Serial.println("Phase III");
  } else if (!hall_u && hall_v && !hall_w) {
    cbi(TCCR4E,OC4OE5);
    // digitalWrite(U_LO, LOW);
    // cbi(TCCR4E,OC4OE4);
    digitalWrite(V_LO, LOW);
    cbi(TCCR4E,OC4OE1);
    digitalWrite(W_LO, LOW);

    digitalWrite(U_LO, HIGH);
    sbi(TCCR4E,OC4OE4);

    // Serial.println("Phase IV");
  } else if (!hall_u && hall_v && hall_w) {
    cbi(TCCR4E,OC4OE5);
    // digitalWrite(U_LO, LOW);
    cbi(TCCR4E,OC4OE4);
    digitalWrite(V_LO, LOW);
    // cbi(TCCR4E,OC4OE1);
    digitalWrite(W_LO, LOW);

    digitalWrite(U_LO, HIGH);
    sbi(TCCR4E,OC4OE1);

    // Serial.println("Phase V");
  } else if (!hall_u && !hall_v && hall_w) {
    cbi(TCCR4E,OC4OE5);
    digitalWrite(U_LO, LOW);
    cbi(TCCR4E,OC4OE4);
    // digitalWrite(V_LO, LOW);
    // cbi(TCCR4E,OC4OE1);
    digitalWrite(W_LO, LOW);

    digitalWrite(V_LO, HIGH);
    sbi(TCCR4E,OC4OE1);

    // Serial.println("Phase VI");
  }
}

void charge_pump_u() {
  // U phase charge pump
  sbi(PORTD, 1);  // INT1 -> PD1
  cbi(PORTD, 1);
}

void charge_pump_v() {
  // V phase charge pump
  sbi(PORTD, 2); // INT2 -> PD2
  cbi(PORTD, 2);
}

void charge_pump_w() {
  // W phase charge pump
  sbi(PORTD, 3);  // INT3 -> PD3
  cbi(PORTD, 3);
}
