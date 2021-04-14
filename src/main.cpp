#include <Arduino.h>
#include <motor.h>

//define rotation 

#define clockwise       0x1
#define cClockwise      0x0

//define button pins
#define cButtonRed      32
#define ccButtonRed     33
#define cButtonGreen    25
#define ccButtonGreen   26
#define cButtonBlue     27
#define ccButtonBlue    12

//define motor output pins
#define motorT1         15
#define motorT2         0 
#define motorTE         2 
#define motorB1         4 
#define motorB2         16   
#define motorBE         24
#define motorR1         21
#define motorR2         35
#define motorRE         34

//define encoder pins
#define encoderT1       17
#define encoderT2       5
#define encoderB1       22
#define encoderB2       23
#define encoderR1       18
#define encoderR2       19

//initialize global encoder variables

int translationTicks = 0;
int rotationTicks = 0;
int bendingTicks = 0;

//initialize translation to bending ratio

int bendingRatio = 10;

//create motor objects
Motor motorT(5000, -5000, motorT1, motorT2, encoderT1, encoderT2);
Motor motorR(5000, -5000, motorR1, motorR2, encoderR1, encoderR2);
Motor motorB(5000, -5000, motorB1, motorB2, encoderB1, encoderB2);

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

  pinMode(motorTE, OUTPUT);
  pinMode(motorRE, OUTPUT);
  pinMode(motorBE, OUTPUT);

  //attach encoder pins to ISR functions
  attachInterrupt(encoderT1, tInt, RISING);
  attachInterrupt(encoderR1,rInt, RISING);
  attachInterrupt(encoderB1, bInt, RISING);

  digitalWrite(motorTE, HIGH);
  digitalWrite(motorBE, HIGH);
  digitalWrite(motorRE, HIGH);


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
  //Serial.printf("Rotation Ticks: %d\n", rotationTicks);
  //Serial.printf("Bending Ticks: %d\n", bendingTicks);
  

  delay(100);
  
}