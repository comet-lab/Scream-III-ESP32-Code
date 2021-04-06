#include <Arduino.h>
#include <motor.h>
#include <analogWrite.h>
#include <PID_v1.h>

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
#define motorTEn        27
#define motorR1         18
#define motorR2         5
#define motorREn        14
#define motorB1         4
#define motorB2         0
#define motorBEn        12

//define encoder pins
#define encoderT1       21
#define encoderT2       19
#define encoderR1       17
#define encoderR2       16
#define encoderB1       2
#define encoderB2       15


//initialize global encoder variables
double translationTicks = 0;
double rotationTicks = 0;
double bendingTicks = 0;

double setPointT, inputT, outputT;
double setPointR, inputR, outputR;
double setPointB, inputB, outputB;

double Tkp = 2, Tki = 5, Tkd = 1;
double Rkp = 2, Rki = 5, Rkd = 1;
double Bkp = 2, Bki = 5, Bkd = 1;

//PID
PID pidT(&translationTicks, &outputT, &setPointT, Tkp, Tki, Tkd, DIRECT);
PID pidR(&rotationTicks, &outputR, &setPointR, Rkp, Rki, Rkd, DIRECT);
PID pidB(&bendingTicks, &outputB, &setPointB, Bkp, Bki, Bkd, DIRECT);


int testNum = 0;

//initialize translation to bending ratio

int bendingRatio = 10;

//create motor objects
Motor motorT(&pidT, 5000, -5000, motorT1, motorT2, encoderT1, encoderT2);
Motor motorR(&pidR, 5000, -5000, motorR1, motorR2, encoderR1, encoderR2);
Motor motorB(&pidB, 5000, -5000, motorB1, motorB2, encoderB1, encoderB2);


//translation encoder ISR
void tInt() {
  int direction = digitalRead(encoderT2);

  if (direction == HIGH) {
    translationTicks++;
  } else {
    translationTicks--;
  }
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
  Serial.println("setup completed");
}


void loop() {

  //read all buttons 
  int cButtonRedVal = digitalRead(cButtonRed);
  int ccButtonRedVal = digitalRead(ccButtonRed);
  int cButtonGreenVal = digitalRead(cButtonGreen);
  int ccButtonGreenVal = digitalRead(ccButtonGreen);
  int cButtonBlueVal = digitalRead(cButtonBlue);
  int ccButtonBlueVal = digitalRead(ccButtonBlue);

  // Button Controlled movement 
  // if (cButtonRedVal == ccButtonRedVal) {
  //   motorT.setMotor(LOW, clockwise, translationTicks);
  // } else if (cButtonRedVal == HIGH) {
  //   motorT.setMotor(HIGH, clockwise, translationTicks);
  // } else if (ccButtonRedVal == HIGH) {
  //   motorT.setMotor(HIGH, cClockwise, translationTicks);
  // }

  // if (cButtonGreenVal == ccButtonGreenVal) {
  //   motorR.setMotor(LOW, clockwise, rotationTicks);
  // } else if (cButtonGreenVal == HIGH) {
   
  //   motorR.setMotor(HIGH, clockwise, rotationTicks);
  // } else if (ccButtonGreenVal == HIGH) {
  //   motorR.setMotor(HIGH, cClockwise, rotationTicks);
  // }

  // if (cButtonBlueVal == ccButtonBlueVal) {
  //   motorB.setMotor(LOW, clockwise, bendingTicks);
  // } else if (cButtonBlueVal == HIGH) {
  //   motorB.setMotor(HIGH, clockwise, bendingTicks);
  // } else if (ccButtonBlueVal == HIGH) {
  //   motorB.setMotor(HIGH, cClockwise, bendingTicks);
  // }

  if (cButtonRedVal == ccButtonRedVal) {
    motorT.setMotor(LOW, clockwise, translationTicks);
  } else if (cButtonRedVal == HIGH) { 
    int targetTicks = 1000; // or some other number we decide
    while(1){
      // compute PID and get its setpoint 
      setPointT = targetTicks; //we should only need to change this variable since the address is passed to PID
      pidT.Compute();               //fixed

      // write it to motor? 
      // TODO how do we use PID output to control speed??? 
      //    do we do motor.setMotor(etc) or do we do analogWrite(enable pin)? 
        
      // Print so we can copy to kind of like a csv 
      Serial.printf("TranslationTicks: %lf,\n", translationTicks);
    }    
    // Let PID control motor 


    // Print ticks "csv"
    
    motorT.setMotor(HIGH, clockwise, translationTicks);
  } 

  // If we moved T or B then we need to correct the other one (B or T) using PID 



  //Serial.printf("TranslationTicks: %lf\n", translationTicks);
  // Serial.printf("Rotation Ticks: %lf\n", rotationTicks);
  //Serial.printf("Bending Ticks: %lf\n", bendingTicks);
  

  delay(100);
  
}