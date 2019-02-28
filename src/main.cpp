#include <Arduino.h>

// #include <EnableInterrupt.h>

// Pin definitions
#define U_HI 6
#define U_LO A0
#define V_HI 10
#define V_LO A1
#define W_HI 13
#define W_LO A2

#define HALL_U 11 // pin 11 -> PCINT7
#define HALL_V 3  // pin 3 -> INT0
#define HALL_W 9  // pin 9 -> PCINT5

#define U_PUMP 2  // pin 2 -> INT1
#define V_PUMP 0  // pin 0 -> INT2
#define W_PUMP 1  // pin 1 -> INT3

#define POT_PIN A5

// Define register read/write operations
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Define HALL sensor read macros
#define hall_v ((_SFR_BYTE(PIND) & _BV(PD0)) > 0) // INT0 -> PD0
#define hall_u ((_SFR_BYTE(PINB) & _BV(PB7)) > 0) // PCINT7 -> PB7
#define hall_w ((_SFR_BYTE(PINB) & _BV(PB5)) > 0) // PCINT5 -> PB5


// // Hall sensor states
// bool hall_u, hall_v, hall_w;

// Pot value and duty cycle
int pot_val, duty_cycle;
const int max_duty_cycle = 250;

// Function declarations
// void update_hall_sensors();

void charge_pump_u();
void charge_pump_v();
void charge_pump_w();

void myAnalogWrite(int pin, int value) {
  // if (pin == 6) {
    // OCR4D=value;
  // } else if (pin == 10) {
    OCR4B=value;
  // } else if (pin == 13) {
  //   OCR4A=value;
  // }

  // OCR4D=value;   // Set PWM value
  // DDRD|=1<<7;    // Set Output Mode D7
  // TCCR4C|=0x09;  // Activate channel D
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

  // update_hall_sensors();

  // Attach/enable interrupts
  attachInterrupt(1, charge_pump_u, FALLING); // pin 2 -> INT1
  attachInterrupt(2, charge_pump_v, FALLING); // pin 0 -> INT2
  attachInterrupt(3, charge_pump_w, FALLING); // pin 1 -> INT3
  // enableInterrupt(HALL_U, update_hall_sensors, CHANGE); // pin 11 -> PCINT7
  // enableInterrupt(HALL_V, update_hall_sensors, CHANGE); // pin 3 -> INT0
  // enableInterrupt(HALL_W, update_hall_sensors, CHANGE); // pin 9 -> PCINT5

  // Configure Timer4 for fast PWM
  TCCR1A &= ~_BV(COM1B1);
  TCCR4A |= _BV(COM4A1) | _BV(COM4B1) | _BV(PWM4A) | _BV(PWM4B);

  TCCR4B &= (B11110000);    // Clear the existing prescaler bits
  TCCR4B |= _BV(CS40);      // Set the new prescaler value (1:1)

  TCCR4C |= _BV(COM4D1) | _BV(PWM4D);

  TCCR4D &= (B11111100);    // Clear the WGM4x bits to set Fast PWM mode

  // // Initialize serial port
  // Serial.begin(9600);
}

void loop() {
  pot_val = analogRead(POT_PIN);
  duty_cycle = map(pot_val, 0, 1023, 0, max_duty_cycle);

  // BLDC commutation logic
  if ((hall_u && hall_v && hall_w) ||
      (!hall_u && !hall_v && !hall_w)) {
    // Coast
    digitalWrite(U_HI, LOW);
    digitalWrite(U_LO, LOW);
    digitalWrite(V_HI, LOW);
    digitalWrite(V_LO, LOW);
    digitalWrite(W_HI, LOW);
    digitalWrite(W_LO, LOW);

    // Serial.println("Coasting");
  } else if (hall_u && !hall_v && hall_w) {
    // digitalWrite(U_HI, LOW);
    digitalWrite(U_LO, LOW);
    myAnalogWrite(V_HI, 0);
    // digitalWrite(V_LO, LOW);
    digitalWrite(W_HI, LOW);
    digitalWrite(W_LO, LOW);

    digitalWrite(V_LO, HIGH);
    analogWrite(U_HI, duty_cycle);

    // Serial.println("Phase I");
  } else if (hall_u && !hall_v && !hall_w) {
    // digitalWrite(U_HI, LOW);
    digitalWrite(U_LO, LOW);
    myAnalogWrite(V_HI, 0);
    digitalWrite(V_LO, LOW);
    digitalWrite(W_HI, LOW);
    // digitalWrite(W_LO, LOW);

    digitalWrite(W_LO, HIGH);
    analogWrite(U_HI, duty_cycle);

    // Serial.println("Phase II");
  } else if (hall_u && hall_v && !hall_w) {
    digitalWrite(U_HI, LOW);
    digitalWrite(U_LO, LOW);
    // digitalWrite(V_HI, LOW);
    digitalWrite(V_LO, LOW);
    digitalWrite(W_HI, LOW);
    // digitalWrite(W_LO, LOW);

    digitalWrite(W_LO, HIGH);
    myAnalogWrite(V_HI, duty_cycle);

    // Serial.println("Phase III");
  } else if (!hall_u && hall_v && !hall_w) {
    digitalWrite(U_HI, LOW);
    // digitalWrite(U_LO, LOW);
    // digitalWrite(V_HI, LOW);
    digitalWrite(V_LO, LOW);
    digitalWrite(W_HI, LOW);
    digitalWrite(W_LO, LOW);

    digitalWrite(U_LO, HIGH);
    myAnalogWrite(V_HI, duty_cycle);

    // Serial.println("Phase IV");
  } else if (!hall_u && hall_v && hall_w) {
    digitalWrite(U_HI, LOW);
    // digitalWrite(U_LO, LOW);
    myAnalogWrite(V_HI, 0);
    digitalWrite(V_LO, LOW);
    // digitalWrite(W_HI, LOW);
    digitalWrite(W_LO, LOW);

    digitalWrite(U_LO, HIGH);
    analogWrite(W_HI, duty_cycle);

    // Serial.println("Phase V");
  } else if (!hall_u && !hall_v && hall_w) {
    digitalWrite(U_HI, LOW);
    digitalWrite(U_LO, LOW);
    myAnalogWrite(V_HI, 0);
    // digitalWrite(V_LO, LOW);
    // digitalWrite(W_HI, LOW);
    digitalWrite(W_LO, LOW);

    digitalWrite(V_LO, HIGH);
    analogWrite(W_HI, duty_cycle);

    // Serial.println("Phase VI");
  }

  // // Print some info
  // Serial.print(PIND, BIN);
  // Serial.print("\t");
  // Serial.print(PINB, BIN);
  // Serial.print("\t");
  // Serial.print(PB7);
  // Serial.print("\t");
  // Serial.print(PD0);
  // Serial.print("\t");
  // Serial.print(PB5);
  // Serial.print("\t");
  // Serial.print(1<<PD3, BIN);
  // Serial.print("\n");
  // Serial.print("\n");

  // // Print the hall sensor state
  // Serial.print(hall_u);
  // Serial.print("\t");
  // Serial.print(hall_v);
  // Serial.print("\t");
  // Serial.print(hall_w);
  // Serial.print("\n");
  //
  // delay(100);
}

// ISR definitions

// void update_hall_sensors() {
//   // Update hall sensor states
//   hall_u = _SFR_BYTE(PINB) & _BV(PB7); // PCINT7 -> PB7
//   hall_v = _SFR_BYTE(PIND) & _BV(PD0); // INT0 -> PD0
//   hall_w = _SFR_BYTE(PINB) & _BV(PB5); // PCINT5 -> PB5
// }

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
