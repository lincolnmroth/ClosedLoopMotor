#ifndef ClosedLoopMotor_h
#define ClosedLoopMotor_h

#include "WProgram.h"

class ClosedLoopMotor
{
  public:
    ClosedLoopMotor(int encA, int encB, int motorA, int motorB, int motorEn, float Kp, float Ki, float Kd, float bias, int cpm, int motorMin, int motorMax);
    void setVel(float vel);
    float getVel();
    int getCounts();
    void compute();
  private:
    int _encA;
    int _encB;
    int _motorA;
    int _motorB;
    int _motorEn;
    float _Kp;
    float _Ki;
    float _Kd;
    long _encoderCount;
    long previousTime;
    long previousEncoderCount;
    float _vel;
    float velGoal;
    void encoderChange();
    int _cpm;
    void drive(float speed);
    float previousError;
    float _sum;
    float _bias;
    float _motorMin;
    float _motorMax;
};

#endif
