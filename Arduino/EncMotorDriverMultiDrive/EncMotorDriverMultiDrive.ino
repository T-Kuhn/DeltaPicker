#include<Wire.h>
#include "EncMotControl.h"
#include "PID.h"
#include "MPU6050.h"
#include "Encoder.h"
#include "DeltaRobInverseKin.h"
//DeltaRobInverseKin DRIK(0.180, 0.445, 0.1, 0.026, 0.05, 0.09); old parameters
DeltaRobInverseKin DRIK(0.180, 0.445, 0.10, 0.026, 0.052, 0.09);

EncMotControl emc1(22, 24, 6, 36, 4.72, true);
EncMotControl emc2(26, 28, 7, 38, 4.75, false);
EncMotControl emc3(30, 32, 8, 40, 4.75, false);

Encoder Encoder1(23, 25);
Encoder Encoder2(27, 29);
Encoder Encoder3(31, 33);

int initFlag = false;
int initCounter = 0;
int tmpCounter = 0;
int cycleCounter = 0;
float goalpos = 200.0f;
long unsigned startTime;

// - - - - - - - - - - - - - - - - - - -
// - - - - TAKING PICTURE POSE - - - - -
// - - - - - - - - - - - - - - - - - - - 
void takingPicturePose(){
  DRIK.setGoalCoordinates(0.0, 0.0, -0.36, 0);
  DRIK.setGoalCoordinates(0.0, 0.0, -0.30, 0);
  DRIK.setGoalCoordinates(0.0, 0.0, -0.28, 0);
}

// - - - - - - - - - - - - - - - - - - -
// - - - - SET NEW DRIK POS ARR  - - - -
// - - - - - - - - - - - - - - - - - - -
void setNewDRIKposArr(){
  bool endTrans = false;
  float floatArr[4];
  int indexCounter = 0;
  String inputString = "";
  DRIK.resetArr();

  while(!endTrans){
    while(indexCounter < 4 && !endTrans){
      
      if(Serial1.available() > 0 && !endTrans) {
        byte byteData = Serial1.read();
        if (byteData == 'e') {
          Serial.println("got all the data");
          endTrans = true;
        }else if (byteData != '\n') {
          inputString += (char)byteData;
        }else{
          floatArr[indexCounter++] = inputString.toFloat();
          Serial.println(inputString.toFloat());
          inputString = "";
        }
      }
    }
    indexCounter = 0;
    if(!endTrans){
      DRIK.setGoalCoordinates(floatArr[0], floatArr[1], floatArr[2], (int)floatArr[3]);
    }
  }
}

void setup() {
  Serial.begin(19200); 
  Serial1.begin(9600); 
  DRIK.debugFlag = false;
  
  emc1.begin();
  emc2.begin();
  emc3.begin();
  
  //debug
  pinMode(52, OUTPUT);
  
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


  // - - - - - - - - - - - - - - - - - - - - - - - -
  // - -  SET PIN 6, 7, 8 PWM CLOCK DEVIDER  - - - -
  // - - - - - - - - - - - - - - - - - - - - - - - -
  TCCR4B &= ~(1 << CS12);
  TCCR4B  |=   (1 << CS11);
  TCCR4B &= ~(1 << CS10);  
                             

  // - - - - - - - - - - - - - - - - - - - - - - - -
  // - -  SET PIN 6, 7, 8 PWM PRECISION 10BIT  - - -
  // - - - - - - - - - - - - - - - - - - - - - - - -

  TCCR4B &= ~(1 << WGM13);    // Timer B clear bit 4
  TCCR4B |=  (1 << WGM12);    // set bit 3

  TCCR4A |= (1 << WGM11);    //  Timer A set bit 1
  TCCR4A |= (1 << WGM10);    //  set bit 0

  interrupts();                // enable all interrupts
}

// - - - - - - - - - - - - - - - - - - -
// - - TIMER 1 INTERRUPT FUNCTION  - - -
// - - - - - every 60 us - - - - - - - -
// - - - - - - - - - - - - - - - - - - - 
ISR(TIMER1_COMPA_vect)
{
  Encoder1.update();
  Encoder2.update();
  Encoder3.update();
  //digitalWrite(52, digitalRead(52) ^ 1);   // toggle pin 52
}

void loop() {
// - - - - - - - - - - - - - - - - - - -
// - - - - - - - - INIT  - - - - - - - -
// - - - - - - - - - - - - - - - - - - -
  if(!initFlag){
    emc1.initStep1();
    emc2.initStep1();
    emc3.initStep1();
    while(initCounter < 300){
      emc1.initStep1_5(); 
      emc2.initStep1_5();
      emc3.initStep1_5();  
      initCounter++;      
    }
    emc1.initStep2();
    emc2.initStep2();
    emc3.initStep2();
    Encoder1.count = 0;
    Encoder2.count = 0;
    Encoder3.count = 0;

    DRIK.resetArr();
    takingPicturePose();
    cycleCounter = 1;
    initFlag = true;
  }
  

  // - - - - - - - - - - - - - - - - - - -
  // - - - - - - MAIN LOOP - - - - - - - -
  // - - - - - - - - - - - - - - - - - - -

  startTime = millis();
  int tmp;

  if(!emc1.pathFollowing && !emc2.pathFollowing && !emc3.pathFollowing){
    tmpCounter++;
    if(tmpCounter >= 30){
      tmp = 0;
      if(cycleCounter == 0){
        setNewDRIKposArr();
        }
      for(int i = 0; i < 3; i++){
        if(cycleCounter == 0){
          tmp = 800;
        }else{
          if(tmp < abs(DRIK.posArr[cycleCounter - 1][i] - DRIK.posArr[cycleCounter][i])*2 + 200){ //2, 200
            tmp = abs(DRIK.posArr[cycleCounter - 1][i] - DRIK.posArr[cycleCounter][i])*2 + 200; 
          }
        }
      }
      //Serial.println(tmp);
        emc1.move(DRIK.posArr[cycleCounter][0], tmp, 100); //100
        emc2.move(DRIK.posArr[cycleCounter][1], tmp, 100);
        emc3.move(DRIK.posArr[cycleCounter][2], tmp, 100);
        //Vacuum pump!!!
        if(DRIK.posArr[cycleCounter][3] == 1){
          digitalWrite(52, HIGH);
        }else{
          digitalWrite(52, LOW);
        }
        
      
      tmpCounter = 0;
      cycleCounter++;
      if(cycleCounter >= DRIK.maxArrIndex){
      cycleCounter = 0;  
      }
    }
  }
  emc1.update(Encoder1);
  emc2.update(Encoder2);
  emc3.update(Encoder3);  
  
  while(startTime + 5 > millis());
}





