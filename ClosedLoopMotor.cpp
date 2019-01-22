#include "WProgram.h"
#include "ClosedLoopMotor.h"


ClosedLoopMotor::ClosedLoopMotor(int encA, int encB, int motorA, int motorB, int motorEn, float Kp, float Ki, float Kd, int cpm)
{
  //ENCODER A PIN MUST MUST MUST BE AN INTERRUPT PIN!!!!
  _encA = encA;
  _encB = encB;
  _motorA = motorA;
  _motorB = motorB;
  _motorEn = motorEn;
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
  _cpm = cpm
  _encoderCount = 0;
  _vel = 0;
  _velGoal = 0;

  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  pinMode(motorEn, OUTPUT);

  attachInterrupt(_encA, encoderChange, CHANGE);

}

void ClosedLoopMotor::encoderChange()
{
  if (digitalRead(_encA) == HIGH)
  {
    //Rising
    if (digitalRead(_encB) == HIGH)
    {
      _encoderCount ++;
    } else {
      _encoderCount --;
    }
  } else {
    //Falling
    if (digitalRead(_encB) == HIGH)
    {
      _encoderCount --;
    } else {
      _encoderCount ++;
    }
  }
}

void ClosedLoopMotor::compute()
{
  long currentTime = millis();
  _vel = (_encoderCount - _previousEncoderCount)/(currentTime - _previousTime) * (60/_cpm)

  float error = _velGoal - _vel;
  float sum = 0;
  float p = _Kp * error;
  float i = _Ki * sum;
  float d = 0;
  drive(constrain(p+i+d, -255, 255));

}

void ClosedLoopMotor::drive(float speed)//-255 to 255
{
  if (speed > 0){
    digitalWrite(_motorA, HIGH);
    digitalWrite(_motorB, LOW);
    analogWrite(_motorEn, abs(speed))
  } else {
    digitalWrite(_motorA, LOW);
    digitalWrite(_motorB, HIGH);
    analogWrite(_motorEn, abs(speed))
  }
}
void ClosedLoopMotor::setVel(float vel)
{
  _velGoal = vel;

}

int ClosedLoopMotor::getCounts()
{
  return _encoderCount;
}

float ClosedLoopMotor::getVel()
{
  return _vel;
}
