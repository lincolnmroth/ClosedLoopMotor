#include "WProgram.h"
#include "ClosedLoopMotor.h"


ClosedLoopMotor::ClosedLoopMotor(int encA, int encB, int motorA, int motorB, int motorEn, float Kp, float Ki, float Kd, float bias, int cpm, int motorMin, int motorMax)
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
  _motorMin = motorMin;
  _motorMax = motorMax;

  encoderCount = 0;
  vel = 0;
  velGoal = 0;
  sum = 0;
  previousError = 0;
  bias = bias;

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
  vel = (_encoderCount - previousEncoderCount)/(currentTime - previousTime) * (60/_cpm)

  float error = _velGoal - _vel;
  sum = sum + (error*(currentTime-previousTime))
  float derivative = (previousError/(currentTime-previousTime));
  float p = _Kp * error;
  float i = _Ki * sum;
  float d = _Kd * derivative;
  drive(constrain(p+i+d+_bias, _motorMin, _motorMax));


  previousEncoderCount = _encoderCount;
  previousTime = currentTime;
  previousError = error;
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
void ClosedLoopMotor::setVel(float rpm)
{
  velGoal = rpm;

}

int ClosedLoopMotor::getCounts()
{
  return _encoderCount;
}

float ClosedLoopMotor::getVel()
{
  return vel;
}
