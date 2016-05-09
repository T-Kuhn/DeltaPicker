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

Encoder Encoder1(23, 25, 35, 37);
Encoder Encoder2(27, 29, 39, 41);
Encoder Encoder3(31, 33, 43, 45);

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
  DRIK.setGoalCoordinates(0.0, 0.0, -0.36);
  DRIK.setGoalCoordinates(0.0, 0.0, -0.30);
  DRIK.setGoalCoordinates(0.0, 0.0, -0.28);
}

// - - - - - - - - - - - - - - - - - - -
// - - - - CAMERA CALIBRATION  - - - - -
// - - - - - - - - - - - - - - - - - - - 
void camCalib(float x, float y, float z){
  DRIK.setGoalCoordinates(x, y, -0.36);
}

// - - - - - - - - - - - - - - - - - - -
// - - - - - LIFTING UP THINGS - - - - -
// - - - - - - - - - - - - - - - - - - - 
void lifting(float x, float y, float z){
  double valueUP = -0.43;
  
  double valueDOWN = -0.470;
  DRIK.setGoalCoordinates(0.0, 0.0, -0.28);
  DRIK.setGoalCoordinates(0.0, 0.0, -0.36);
  for(int i = 0; i < 1; i++){
    DRIK.setGoalCoordinates(x, y, valueUP);
    DRIK.setGoalCoordinates(x, y, valueDOWN);
    DRIK.setGoalCoordinates(x, y, valueUP);
  }
  DRIK.setGoalCoordinates(0.0, 0.0, -0.36);
  DRIK.setGoalCoordinates(0.0, 0.0, -0.28);
}

// - - - - - - - - - - - - - - - - - - -
// - - - - - LIFTING UP THINGS - - - - -
// - - - - - - - - - - - - - - - - - - - 
void liftingUpThings(){
  double valueUP = -0.43;
  double valueDOWN = -0.470;
  DRIK.setGoalCoordinates(0.0, 0.0, -0.36);
  DRIK.setGoalCoordinates(0.0, 0.0, -0.30);
  DRIK.setGoalCoordinates(0.0, 0.0, -0.28);
  for(int i = 0; i < 1; i++){
    DRIK.setGoalCoordinates(0.0, 0.0, valueUP);
    DRIK.setGoalCoordinates(0.0, 0.0, valueDOWN);
    DRIK.setGoalCoordinates(0.0, 0.0, valueUP);
    DRIK.setGoalCoordinates(0.0, 0.12, valueUP);
    DRIK.setGoalCoordinates(0.0, 0.12, valueDOWN);
    DRIK.setGoalCoordinates(0.0, 0.12, valueUP);
    DRIK.setGoalCoordinates(0.0, -0.12, valueUP);
    DRIK.setGoalCoordinates(0.0, -0.12, valueUP);
    DRIK.setGoalCoordinates(0.0, -0.12, valueDOWN);
    DRIK.setGoalCoordinates(0.0, -0.12, valueUP);
    DRIK.setGoalCoordinates(0.12, 0.0, valueUP);
    DRIK.setGoalCoordinates(0.12, 0.0, valueDOWN);
    DRIK.setGoalCoordinates(0.12, 0.0, valueUP);
    DRIK.setGoalCoordinates(-0.12, 0.0, valueUP);
    DRIK.setGoalCoordinates(-0.12, 0.0, valueDOWN);
    DRIK.setGoalCoordinates(-0.12, 0.0, valueUP);
    DRIK.setGoalCoordinates(0.12, 0.12, valueUP);
    DRIK.setGoalCoordinates(0.12, 0.12, valueDOWN);
    DRIK.setGoalCoordinates(0.12, 0.12, valueUP);    
  }
  Serial.println(DRIK.maxArrIndex);
}

// - - - - - - - - - - - - - - - - - - -
// - - - - - RANDOM MOVEMENT - - - - - -
// - - - - - - - - - - - - - - - - - - - 
void randomMovement(){
  double value = 0.11;
  DRIK.setGoalCoordinates(0.0, 0.0, -0.36);  
  DRIK.setGoalCoordinates(0.05, -0.05, -0.4);
  for(int i = 0; i < 1000; i++){
    DRIK.setGoalCoordinates((random(1000) - 500) / 500.0 * value, (random(1000) - 500) / 500.0 * value, -0.40 + (random(1000) - 500) / 500.0 * 0.05);
  }
  Serial.println(DRIK.maxArrIndex);
}

// - - - - - - - - - - - - - - - - - - -
// - - - - - - - SQUARES - - - - - - - -
// - - - - - - - - - - - - - - - - - - - 
void squares(){
  double value = 0.12;
  DRIK.setGoalCoordinates(0.0, 0.0, -0.36);  
  DRIK.setGoalCoordinates(0.05, -0.05, -0.4);
  
  for(double d = 1.0; d > 0.0; d -= 0.02){
    DRIK.setGoalCoordinates(d * value, d * -value, -0.43);
    DRIK.setGoalCoordinates(d * value, d * -value, -0.45);
    DRIK.setGoalCoordinates(d * value, d * value, -0.45);
    DRIK.setGoalCoordinates(d * -value, d * value, -0.45);
    DRIK.setGoalCoordinates(d * -value, d * -value, -0.45);
    DRIK.setGoalCoordinates(d * value, d * -value, -0.45); 
  }
  Serial.println(DRIK.maxArrIndex);
}

// - - - - - - - - - - - - - - - - - - -
// - - - - - SQUARES IN AIR  - - - - - -
// - - - - - - - - - - - - - - - - - - - 
void squaresInAir(){
  double value = 0.12;
  double _z = -0.30;
  
  //draw big squares
  DRIK.setGoalCoordinates(-value, value, _z);  
  DRIK.setGoalCoordinates(value, value, _z);  
  DRIK.setGoalCoordinates(value, value, _z - 0.05);
  DRIK.setGoalCoordinates(value, value, _z);  
  DRIK.setGoalCoordinates(value, -value, _z);
  DRIK.setGoalCoordinates(-value, -value, _z);
  DRIK.setGoalCoordinates(-value, -value, _z - 0.05);
  DRIK.setGoalCoordinates(-value, -value, _z);
  DRIK.setGoalCoordinates(-value, value, _z);

  value = 0.06;
  _z = -0.45;
  //draw small square
  DRIK.setGoalCoordinates(-value, value, _z);  
  DRIK.setGoalCoordinates(value, value, _z);  
  DRIK.setGoalCoordinates(value, value, _z + 0.05);
  DRIK.setGoalCoordinates(value, value, _z);  
  DRIK.setGoalCoordinates(value, -value, _z);
  DRIK.setGoalCoordinates(-value, -value, _z);
  DRIK.setGoalCoordinates(-value, -value, _z + 0.05);
  DRIK.setGoalCoordinates(-value, -value, _z);
  DRIK.setGoalCoordinates(-value, value, _z);

  Serial.println(DRIK.maxArrIndex);
}

// - - - - - - - - - - - - - - - - - - -
// - - - - ROTATING GEO THING  - - - - -
// - - - - - - - - - - - - - - - - - - - 
void rotatingGeoThing(){
  double value = 0.10;
  DRIK.setGoalCoordinates(0.0, 0.0, -0.36);  
  DRIK.setGoalCoordinates(0.05, -0.05, -0.4);
  
  for(double d = 0.0; d < 2*PI; d += 0.099887014){
    DRIK.setGoalCoordinates(cos(d) * value, sin(d) * value, -0.43);
    DRIK.setGoalCoordinates(cos(d) * value, sin(d) * value, -0.45);
    DRIK.setGoalCoordinates(cos(d + PI / 2.0) * value, sin(d + PI / 2.0) * value, -0.45);
    DRIK.setGoalCoordinates(cos(d + PI) * value, sin(d + PI) * value, -0.45);
    DRIK.setGoalCoordinates(cos(d + 2.0 * PI / 3.0) * value, sin(d + 2.0 * PI / 3.0) * value, -0.45);
    DRIK.setGoalCoordinates(cos(d) * value, sin(d) * value, -0.45);
  }
  Serial.println(DRIK.maxArrIndex);
}

// - - - - - - - - - - - - - - - - - - -
// - - - - ROTATING SQUARES  - - - - - -
// - - - - - - - - - - - - - - - - - - - 
void rotatingSquares(){
  double value = 0.10;
  DRIK.setGoalCoordinates(0.0, 0.0, -0.36);  
  DRIK.setGoalCoordinates(0.05, -0.05, -0.4);
  
  for(double d = 0.0; d <= 2*PI; d += 0.099887014){
    DRIK.setGoalCoordinates(cos(d) * value, sin(d) * value, -0.43);
    DRIK.setGoalCoordinates(cos(d) * value, sin(d) * value, -0.45);
    DRIK.setGoalCoordinates(cos(d + PI / 2.0) * value, sin(d + PI / 2.0) * value, -0.45);
    DRIK.setGoalCoordinates(cos(d + PI) * value, sin(d + PI) * value, -0.45);
    DRIK.setGoalCoordinates(cos(d + 3.0 * PI / 2.0) * value, sin(d + 3.0 * PI / 2.0) * value, -0.45);
    DRIK.setGoalCoordinates(cos(d) * value, sin(d) * value, -0.45);
  }
  Serial.println(DRIK.maxArrIndex);
}

// - - - - - - - - - - - - - - - - - - -
// - - - - -  RANDOM LINES - - - - - - -
// - - - - - - - - - - - - - - - - - - - 
void randomLines(){
  double value = 0.11;
  DRIK.setGoalCoordinates(0.0, 0.0, -0.36);  
  DRIK.setGoalCoordinates(0.05, -0.05, -0.4);
  for(int i = 0; i < 1000; i++){
    DRIK.setGoalCoordinates((random(1000) - 500) / 500.0 * value, (random(1000) - 500) / 500.0 * value, -0.450);
  }
  Serial.println(DRIK.maxArrIndex);
}

// - - - - - - - - - - - - - - - - - - -
// - - - - - SPECIAL THING - - - - - - -
// - - - - - - - - - - - - - - - - - - - 
void specialThing(){
  double value = 0.06;
  double tmp_x = -0.06;
  double tmp_y = 0.06;
  
  DRIK.setGoalCoordinates(0.0, 0.0, -0.36);  
  DRIK.setGoalCoordinates(0.05, -0.05, -0.4);

  //draw big squares
  DRIK.setGoalCoordinates(-value, value, -0.40);
  DRIK.setGoalCoordinates(-value, value, -0.45);  
  DRIK.setGoalCoordinates(value, value, -0.45);  
  DRIK.setGoalCoordinates(value, -value, -0.45);
  DRIK.setGoalCoordinates(-value, -value, -0.45);
  DRIK.setGoalCoordinates(-value, value, -0.45);
  DRIK.setGoalCoordinates(-value, value, -0.40);

  value = 0.03;
  //draw small square #1
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.40);  
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.45);  
  DRIK.setGoalCoordinates(tmp_x + value, tmp_y + value, -0.45);  
  DRIK.setGoalCoordinates(tmp_x + value, tmp_y - value, -0.45);
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y - value, -0.45);
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.45);
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.40);
  
  value = 0.03;
  tmp_x = 0.06;
  tmp_y = 0.06;
  //draw small square #2
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.40);  
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.45);  
  DRIK.setGoalCoordinates(tmp_x + value, tmp_y + value, -0.45);  
  DRIK.setGoalCoordinates(tmp_x + value, tmp_y - value, -0.45);
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y - value, -0.45);
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.45);
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.40);

  value = 0.03;
  tmp_x = 0.06;
  tmp_y = -0.06;
  //draw small square #3
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.40);  
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.45);  
  DRIK.setGoalCoordinates(tmp_x + value, tmp_y + value, -0.45);  
  DRIK.setGoalCoordinates(tmp_x + value, tmp_y - value, -0.45);
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y - value, -0.45);
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.45);
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.40);

  value = 0.03;
  tmp_x = -0.06;
  tmp_y = -0.06;
  //draw small square #4
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.40);  
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.45);  
  DRIK.setGoalCoordinates(tmp_x + value, tmp_y + value, -0.45);  
  DRIK.setGoalCoordinates(tmp_x + value, tmp_y - value, -0.45);
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y - value, -0.45);
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.45);
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.40);

  //fill square #1 with stripes!
  value = 0.03;
  tmp_x = -0.06;
  tmp_y = 0.06;
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.40); 
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.45); 
  for(double d = 0.0; d <= 0.9; d += 0.1){
    DRIK.setGoalCoordinates(tmp_x + value, tmp_y + value - 2 * value * d, -0.45); 
    DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value - 2 * value * (d + 0.05), -0.45); 
  }
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y - value, -0.40); 

  //fill square #2 with stripes!
  value = 0.03;
  tmp_x = 0.06;
  tmp_y = 0.06;
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.40); 
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.45); 
  for(double d = 0.0; d <= 0.9; d += 0.1){
    DRIK.setGoalCoordinates(tmp_x + value, tmp_y + value - 2 * value * d, -0.45); 
    DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value - 2 * value * (d + 0.05), -0.45); 
  }
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y - value, -0.40);

  //fill square #3 with stripes!
  value = 0.03;
  tmp_x = 0.06;
  tmp_y = -0.06;
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.40); 
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.45); 
  for(double d = 0.0; d <= 0.9; d += 0.1){
    DRIK.setGoalCoordinates(tmp_x + value, tmp_y + value - 2 * value * d, -0.45); 
    DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value - 2 * value * (d + 0.05), -0.45); 
  }
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y - value, -0.40);

  //fill square #4 with stripes!
  value = 0.03;
  tmp_x = -0.06;
  tmp_y = -0.06;
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.40); 
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value, -0.45); 
  for(double d = 0.0; d <= 0.9; d += 0.1){
    DRIK.setGoalCoordinates(tmp_x + value, tmp_y + value - 2 * value * d, -0.45); 
    DRIK.setGoalCoordinates(tmp_x - value, tmp_y + value - 2 * value * (d + 0.05), -0.45); 
  }
  DRIK.setGoalCoordinates(tmp_x - value, tmp_y - value, -0.40);
  
  Serial.println(DRIK.maxArrIndex);
}

// - - - - - - - - - - - - - - - - - - -
// - - - - SET NEW DRIK POS ARR  - - - -
// - - - - - - - - - - - - - - - - - - -
void setNewDRIKposArr(){
  bool endTrans = false;
  float floatArr[3];
  int indexCounter = 0;
  String inputString = "";
  DRIK.resetArr();

  while(!endTrans){
    while(indexCounter < 3 && !endTrans){
      
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
      DRIK.setGoalCoordinates(floatArr[0], floatArr[1], floatArr[2]);
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
  pinMode(53, OUTPUT);
  pinMode(4, OUTPUT);
  
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
    if(tmpCounter >= 100){
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
  
 /*
  pk = 0.86f;  //random(1, 1000)/500.0f;
  pi = 0.005f;  //random(1, 10000)/90000.0f;
  pd = 0.056f;  //random(1, 1000)/1000.0f;
  
  Serial.print("pk: ");
  Serial.println(pk);
  Serial.print("pi: ");
  Serial.println(pi);
  Serial.print("pd: ");
  Serial.println(pd);
  emc1.pid.begin(pk, pi, pd, 3.0f);
  emc2.pid.begin(pk, pi, pd, 3.0f);
  emc3.pid.begin(pk, pi, pd, 3.0f);
  */
  
  //Serial.println(50*sin((float)millis() * 2 * PI / 1000.0f));

  


  //while(tmpCounter < 600){
    //emc1.updatePID_ext(goalpos + 100*sin((float)millis() * 2 * PI / 2000.0f), Encoder1);
    //emc2.updatePID_ext(goalpos + 100*sin(2.0f / 3 * PI + (float)millis() * 2 * PI / 2000.0f), Encoder2);
    //emc3.updatePID_ext(goalpos + 100*sin(4.0f / 3 * PI + (float)millis() * 2 * PI / 2000.0f), Encoder3);

    //delay(5);
    //digitalWrite(53, digitalRead(53) ^ 1);   // toggle pin 52
    //tmpCounter++;
  //}
  //tmpCounter = 0;


  while(startTime + 5 > millis());
}





