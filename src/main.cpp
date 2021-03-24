#include <Arduino.h>
#include <motor.h>

//define rotation 

#define clockwise       0x1
#define cClockwise      0x0

//define button pins
#define cButtonRed      35
#define ccButtonRed     34
#define cButtonGreen    33
#define ccButtonGreen   32
#define cButtonBlue     26
#define ccButtonBlue    25

//define motor output pins
#define motorT1         23
#define motorT2         22
#define motorR1         18
#define motorR2         5
#define motorB1         4
#define motorB2         0

//define encoder pins
#define encoderT1       21
#define encoderT2       19
#define encoderR1       17
#define encoderR2       16
#define encoderB1       2
#define encoderB2       15

//initialize global encoder variables

int translationTicks = 0;
int rotationTicks = 0;
int bendingTicks = 0;

//create motor objects
Motor motorT(500, -500, motorT1, motorT2, encoderT1, encoderT2);
Motor motorR(500, -500, motorR1, motorR2, encoderR1, encoderR2);
Motor motorB(500, -500, motorB1, motorB2, encoderB1, encoderB2);

//translation encoder ISR
void tInt() {

  int direction = digitalRead(encoderT2);

  if (direction == HIGH) {
    translationTicks++;
  } else {
    translationTicks--;
  }


  //Serial.println(translationTicks);
}

//rotation encoder ISR
void rInt() {
  int direction = digitalRead(encoderR2);

  if (direction == HIGH) {
    rotationTicks++;
  } else {
    rotationTicks--;
  }
}

//bending encoder ISR
void bInt() {
  int direction = digitalRead(encoderB2);

  if (direction == HIGH) {
    bendingTicks++;
  } else {
    bendingTicks--;
  }
}


void setup() {

  //start Serial connection
  Serial.begin(9600); //115200 should work because its esp32 but only 9600 works. Weird

  //set button pins to input
  pinMode(cButtonRed, INPUT);
  pinMode(ccButtonRed, INPUT);
  pinMode(cButtonGreen, INPUT);
  pinMode(ccButtonGreen, INPUT);
  pinMode(cButtonBlue, INPUT);
  pinMode(ccButtonBlue, INPUT);

  //set motor pins to output
  pinMode(motorT1, OUTPUT);
  pinMode(motorT2, OUTPUT);
  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  //attach encoder pins to ISR functions
  attachInterrupt(encoderT1, tInt, RISING);
  attachInterrupt(encoderR1,rInt, RISING);
  attachInterrupt(encoderB1, bInt, RISING);
}




void loop() {

  //read all buttons 
  int cButtonRedVal = digitalRead(cButtonRed);
  int ccButtonRedVal = digitalRead(ccButtonRed);
  int cButtonGreenVal = digitalRead(cButtonGreen);
  int ccButtonGreenVal = digitalRead(ccButtonGreen);
  int cButtonBlueVal = digitalRead(cButtonBlue);
  int ccButtonBlueVal = digitalRead(ccButtonBlue);

  if (cButtonRedVal == ccButtonRedVal) {
    
    motorT.setMotor(LOW, clockwise, translationTicks);
  } else if (cButtonRedVal == HIGH) {
    motorT.setMotor(HIGH, clockwise, translationTicks);
  } else if (ccButtonRedVal == HIGH) {
    motorT.setMotor(HIGH, cClockwise, translationTicks);
  }

  if (cButtonGreenVal == ccButtonGreenVal) {
    motorR.setMotor(LOW, clockwise, rotationTicks);
  } else if (cButtonGreenVal == HIGH) {
    motorR.setMotor(HIGH, clockwise, rotationTicks);
  } else if (ccButtonGreenVal == HIGH) {
    motorR.setMotor(HIGH, cClockwise, rotationTicks);
  }

  if (cButtonBlueVal == ccButtonBlueVal) {
    motorB.setMotor(LOW, clockwise, bendingTicks);
  } else if (cButtonBlueVal == HIGH) {
    motorB.setMotor(HIGH, clockwise, bendingTicks);
  } else if (ccButtonBlueVal == HIGH) {
    motorB.setMotor(HIGH, cClockwise, bendingTicks);
  }
  
  //Serial.printf("TranslationTicks: %d\n", translationTicks);
  Serial.printf("Rotation Ticks: %d\n", rotationTicks);
  //Serial.printf("Bending Ticks: %d\n", bendingTicks);
  

  delay(100);
  
}