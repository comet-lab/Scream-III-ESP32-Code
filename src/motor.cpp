#include <motor.h>
#include<Arduino.h>

Motor::Motor(int max, int min, int p1, int p2, int p3, int p4) {
    maxTicks = max;
    minTicks = min;
    motorPin1 = p1;
    motorPin2 = p2;
    encoderPin1 = p3;
    encoderPin2 = p4;
}

void Motor::setMotor(int power, int direction, int currentTick) {

    // power is set to on set speed to 255 check diretion
    if (power == HIGH) {
        boolean test = (direction == HIGH && currentTick <= maxTicks);
        Serial.println(test);
        if (direction == LOW && currentTick >= minTicks) {
            digitalWrite(motorPin1, LOW);
            digitalWrite(motorPin2, HIGH);
        } else if (direction == HIGH && currentTick <= maxTicks) {
            digitalWrite(motorPin1, HIGH);
            digitalWrite(motorPin2, LOW);
        } else {
            digitalWrite(motorPin1, LOW);
            digitalWrite(motorPin2, LOW);
        }
    } else {
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, LOW);
    }
  
}