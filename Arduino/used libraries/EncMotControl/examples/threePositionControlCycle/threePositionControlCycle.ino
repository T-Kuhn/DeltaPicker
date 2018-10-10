#include<Wire.h>
#include "EncMotControl.h"
#include "PID.h"
#include "Encoder.h"

EncMotControl emc1(22, 24, 2);

Encoder Encoder1(23, 25);

void setup() {
  pinMode(22, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(2, OUTPUT);

  emc1.begin(1000.0f, 2000.0f, 1000.0f);
  Serial.begin(250000); // debug serial

  //debug
  pinMode(52, OUTPUT);
  pinMode(53, OUTPUT);
  
  // - - - - - - - - - - - - - - - - - - -
  // - - - - -  SET UP TIMER 1 - - - - - -
  // - - - - - - - - - - - - - - - - - - - 
  noInterrupts();               // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 3;                   // compare match register 16MHz/256/20kHz
  TCCR1B |= (1 << WGM12);      // CTC mode
  TCCR1B |= (1 << CS12);       // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);     // enable timer compare interrupt

  // - - - - - - - - - - - - - - - - - - -
  // - - - - -  SET UP TIMER 5 - - - - - -
  // - - - - - - - - - - - - - - - - - - - 

  /*
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5  = 0;

  OCR5A = 625;                   // compare match register 16MHz/256/100Hz
  TCCR5B |= (1 << WGM12);      // CTC mode
  TCCR5B |= (1 << CS12);       // 256 prescaler 
  TIMSK5 |= (1 << OCIE1A);     // enable timer compare interrupt
  */

  // - - - - - - - - - - - - - - - - - - - - - - - -
  // - -  SET PIN 5, 3, 2 PWM FREQ. TO 400kHz  - - -
  // - - - - - - - - - - - - - - - - - - - - - - - -
  int myEraser = 7;            // this is 111 in binary and is used as an eraser
  TCCR3B &= ~myEraser;         // this operation (AND plus NOT),  set the three bits in TCCR2B to 0

  int myPrescaler = 2;         // this could be a number in [1 , 6]. In this case, 3 corresponds in binary to 011.   
  TCCR3B |= myPrescaler;       //this operation (OR), replaces the last three bits in TCCR2B with our new value 011

  interrupts();                // enable all interrupts
}

// - - - - - - - - - - - - - - - - - - -
// - - TIMER 1 INTERRUPT FUNCTION  - - -
// - - - - - every 60 us - - - - - - - -
// - - - - - - - - - - - - - - - - - - - 
ISR(TIMER1_COMPA_vect)
{
  Encoder1.update();
  digitalWrite(52, digitalRead(52) ^ 1);   // toggle pin 52
}

// - - - - - - - - - - - - - - - - - - -
// - - TIMER 5 INTERRUPT FUNCTION  - - -
// - - - - - every 10 ms - - - - - - - -
// - - - - - - - - - - - - - - - - - - - 
/*
ISR(TIMER5_COMPA_vect)
{
  Serial.print(Encoder1.currentRot);
  Serial.print("    ");
  Serial.println(Encoder1.count);
  emc1.update(Encoder1);
  
  digitalWrite(53, digitalRead(53) ^ 1);   // toggle pin 52
}
*/

void loop() {
  
  Serial.print(Encoder1.currentRot);
  Serial.print("    ");
  Serial.println(Encoder1.count);
  emc1.update(Encoder1);
  
  digitalWrite(53, digitalRead(53) ^ 1);   // toggle pin 52
  delay(10);

}




