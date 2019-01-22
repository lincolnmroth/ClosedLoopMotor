#include "WProgram.h"
#include "ClosedLoopMotor.h"


ClosedLoopMotor:ClosedLoopMotor(int encA, int encB, int motorA, int motorB, int motorEn, float Kp, float Ki, float Kd)
{
  _encA = encA;
  _encB = encB;
  _motorA = motorA;
  _motorB = motorB;
  _motorEn = motorEn;
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;

  _encoderCounts = 0;
  _vel = 0;

  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  pinMode(motorEn, OUTPUT);


}

int ClosedLoopMotor::getCounts()
{
  return _encoderCounts;
}

float ClosedLoopMotor::getVel()
{
  return _vel;
}
