#include "Motor.h"
#include "Arduino.h"
#include <digitalWriteFast.h>



Motor::Motor(int directionAMotor, int directionBMoteur, int pwmMotor) {
  c_directionAMotor = directionAMotor;
  c_directionBMotor = directionBMoteur;
  c_pwmMotor = pwmMotor ;
}

/* Fonction  pour initialiser les pins du moteur */
void Motor::init() {
  pinMode(c_directionAMotor, OUTPUT);
  pinMode(c_directionBMotor, OUTPUT);
  pinMode(c_pwmMotor, OUTPUT);
}

void Motor::run(int speed) {
  c_speed = speed ;
  if(c_speed  > 255) {
    c_speed  = 255;
  }
  if(c_speed < -255) {
    c_speed  = -255;
  }
  if (c_speed  >= 0) {
    digitalWrite(c_directionAMotor, HIGH);
    digitalWrite(c_directionBMotor, LOW);
    analogWrite(c_pwmMotor, c_speed);
  }
  if (c_speed  < 0) {
    digitalWrite(c_directionAMotor, LOW);
    digitalWrite(c_directionBMotor, HIGH);
    analogWrite(c_pwmMotor, -c_speed);
  }
}
void Motor::stop(){
  digitalWrite(c_directionAMotor, LOW);
  digitalWrite(c_directionBMotor, LOW);
  analogWrite(c_pwmMotor, 0);
}
