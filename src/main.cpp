#include <Arduino.h>

#include <EnableInterrupt.h>

// Pin definitions
#define U_HI 6
#define U_LO A0
#define V_HI 10
#define V_LO A1
#define W_HI 13
#define W_LO A2

#define HALL_U 11
#define HALL_V 3
#define HALL_W 9

#define U_PUMP 2
#define V_PUMP 0
#define W_PUMP 1

#define POT_PIN A5

// // Define fast read/write operations
// #ifndef CLR
// #define CLR(x,y) (x&=(~(1<<y)))
// #endif
// #ifndef SET
// #define SET(x,y) (x|=(1<<y))
// #endif

// Define register read/write operations
// #ifndef cbi
// #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
// #endif
// #ifndef sbi
// #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
// #endif


// Hall sensor states
bool hall_u, hall_v, hall_w;

// Pot value and duty cycle
int pot_val, duty_cycle;

// Function declarations
void update_hall_sensors();

void charge_pump_u();
void charge_pump_v();
void charge_pump_w();

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

  update_hall_sensors();

  // Attach/enable interrupts
  enableInterrupt(U_PUMP, charge_pump_u, FALLING); // pin 2 -> INT1
  enableInterrupt(V_PUMP, charge_pump_v, FALLING); // pin 0 -> INT2
  enableInterrupt(W_PUMP, charge_pump_w, FALLING); // pin 1 -> INT3
  enableInterrupt(HALL_U, update_hall_sensors, CHANGE); // pin 11 -> PCINT7
  enableInterrupt(HALL_V, update_hall_sensors, CHANGE); // pin 3 -> INT0
  enableInterrupt(HALL_W, update_hall_sensors, CHANGE); // pin 9 -> PCINT5

  // // Initialize serial port
  // Serial.begin(9600);
}

void loop() {
  pot_val = analogRead(POT_PIN);
  duty_cycle = map(pot_val, 0, 1023, 0, 127);

  // BLDC commutation logic
  if ((hall_u == HIGH && hall_v == HIGH && hall_w == HIGH) ||
      (hall_u == LOW && hall_v == LOW && hall_w == LOW)) {
    // set error bit?

    // Coast
    digitalWrite(U_HI, LOW);
    digitalWrite(U_LO, LOW);
    digitalWrite(V_HI, LOW);
    digitalWrite(V_LO, LOW);
    digitalWrite(W_HI, LOW);
    digitalWrite(W_LO, LOW);
  } else if (hall_u == HIGH && hall_v == LOW && hall_w == HIGH) {
    digitalWrite(U_HI, LOW);
    // digitalWrite(U_LO, LOW);
    // digitalWrite(V_HI, LOW);
    digitalWrite(V_LO, LOW);
    digitalWrite(W_HI, LOW);
    digitalWrite(W_LO, LOW);

    digitalWrite(U_LO, HIGH);
    analogWrite(V_HI, duty_cycle);
  } else if (hall_u == HIGH && hall_v == LOW && hall_w == LOW) {
    digitalWrite(U_HI, LOW);
    // digitalWrite(U_LO, LOW);
    digitalWrite(V_HI, LOW);
    digitalWrite(V_LO, LOW);
    // digitalWrite(W_HI, LOW);
    digitalWrite(W_LO, LOW);

    digitalWrite(U_LO, HIGH);
    analogWrite(W_HI, duty_cycle);
  } else if (hall_u == HIGH && hall_v == HIGH && hall_w == LOW) {
    digitalWrite(U_HI, LOW);
    digitalWrite(U_LO, LOW);
    digitalWrite(V_HI, LOW);
    // digitalWrite(V_LO, LOW);
    // digitalWrite(W_HI, LOW);
    digitalWrite(W_LO, LOW);

    digitalWrite(V_LO, HIGH);
    analogWrite(W_HI, duty_cycle);
  } else if (hall_u == LOW && hall_v == HIGH && hall_w == LOW) {
    // digitalWrite(U_HI, LOW);
    digitalWrite(U_LO, LOW);
    digitalWrite(V_HI, LOW);
    // digitalWrite(V_LO, LOW);
    digitalWrite(W_HI, LOW);
    digitalWrite(W_LO, LOW);

    digitalWrite(V_LO, HIGH);
    analogWrite(U_HI, duty_cycle);
  } else if (hall_u == LOW && hall_v == HIGH && hall_w == HIGH) {
    // digitalWrite(U_HI, LOW);
    digitalWrite(U_LO, LOW);
    digitalWrite(V_HI, LOW);
    digitalWrite(V_LO, LOW);
    digitalWrite(W_HI, LOW);
    // digitalWrite(W_LO, LOW);

    digitalWrite(W_LO, HIGH);
    analogWrite(U_HI, duty_cycle);
  } else if (hall_u == LOW && hall_v == LOW && hall_w == HIGH) {
    digitalWrite(U_HI, LOW);
    digitalWrite(U_LO, LOW);
    // digitalWrite(V_HI, LOW);
    digitalWrite(V_LO, LOW);
    digitalWrite(W_HI, LOW);
    // digitalWrite(W_LO, LOW);

    digitalWrite(W_LO, HIGH);
    analogWrite(V_HI, duty_cycle);
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
  //
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

void update_hall_sensors() {
  // Update hall sensor states
  hall_u = (PINB & (1<<PB7)); // PCINT7 -> PB7
  hall_v = (PIND & (1<<PD0)); // INT0 -> PD0
  hall_w = (PINB & (1<<PB5)); // PCINT5 -> PB5
}

void charge_pump_u() {
  // U phase charge pump
  // sbi(PORTD, 1);  // INT1 -> PD1
  // cbi(PORTD, 1);
  digitalWrite(U_LO, HIGH);
  digitalWrite(U_LO, LOW);
}

void charge_pump_v() {
  // V phase charge pump
  // sbi(PORTD, 2); // INT2 -> PD2
  // cbi(PORTD, 2);
  digitalWrite(V_LO, HIGH);
  digitalWrite(V_LO, LOW);
}

void charge_pump_w() {
  // W phase charge pump
  // sbi(PORTD, 3);  // INT3 -> PD3
  // cbi(PORTD, 3);
  digitalWrite(W_LO, HIGH);
  digitalWrite(W_LO, LOW);
}
