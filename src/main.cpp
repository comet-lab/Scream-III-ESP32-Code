#include <Arduino.h>

//define rotation 

#define clockwise       0x1
#define cClockwise      0x0

//define button pins
#define cButtonRed      34
#define ccButtonRed     35
#define cButtonGreen    32
#define ccButtonGreen   33
#define cButtonBlue     25
#define ccButtonBlue    26

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

//translation encoder ISR
void tInt() {

  int direction = digitalRead(encoderT2);

  if (direction == HIGH) {
    translationTicks++;
  } else {
    translationTicks--;
  }

  Serial.println(translationTicks);
}

//rotation encoder ISR
void rInt() {

  Serial.println(rotationTicks);
}

//bending encoder ISR
void bInt() {

  Serial.println(bendingTicks);
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



void setMotor (int power, int direction, int motorPin1, int motorPin2) {

  // power is set to on set speed to 255 check diretion
  if (power == HIGH) {
    if (direction == LOW) {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
    } else if (direction == HIGH) {
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
    } 
  } else {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
  }
  
}

void loop() {

  // digitalWrite(motorT2,HIGH);
  // Serial.println("on");
  // delay(1000);
  // digitalWrite(motorT2, LOW);
  // Serial.println("off");
  // delay(1000);

  //read all buttons 
  int cButtonRedVal = digitalRead(cButtonRed);
  int ccButtonRedVal = digitalRead(ccButtonRed);
  int cButtonGreenVal = digitalRead(cButtonGreen);
  int ccButtonGreenVal = digitalRead(ccButtonGreen);
  int cButtonBlueVal = digitalRead(cButtonBlue);
  int ccButtonBlueVal = digitalRead(ccButtonBlue);

  if (cButtonRedVal == ccButtonRedVal) {
    setMotor(LOW, clockwise, motorT1, motorT2);
  } else if (cButtonRedVal == HIGH) {
    setMotor(HIGH, clockwise, motorT1, motorT2);
  } else if (ccButtonRedVal == HIGH) {
    setMotor(HIGH, cClockwise, motorT1, motorT2);
  }

  if (cButtonGreenVal == ccButtonGreenVal) {
    setMotor(LOW, clockwise, motorR1, motorR2);
  } else if (cButtonGreenVal == HIGH) {
    setMotor(HIGH, clockwise, motorR1, motorR2);
  } else if (ccButtonGreenVal == HIGH) {
    setMotor(HIGH, cClockwise, motorR1, motorR2);
  }

  if (cButtonBlueVal == ccButtonBlueVal) {
    setMotor(LOW, clockwise, motorB1, motorB2);
  } else if (cButtonBlueVal == HIGH) {
    setMotor(HIGH, clockwise, motorB1, motorB2);
  } else if (ccButtonBlueVal == HIGH) {
    setMotor(HIGH, cClockwise, motorB1, motorB2);
  }
  
  

  delay(100);
  
}