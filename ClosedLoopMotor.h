#ifndef ClosedLoopMotor_h
#define ClosedLoopMotor_h

#include "WProgram.h"

class ClosedLoopMotor
{
  public:
    ClosedLoopMotor(int encA, int encB, int motorA, int motorB, int motorEn, float Kp, float Ki, float Kd, int cpm);
    void setVel(float vel);
    float getVel();
    int getCounts();
    void compute();
  private:
    int _encA;
    int _encB;
    int _motorA;
    int _motorB;
    int _motorC;
    float _Kp;
    float _Ki;
    float _Kd;
    long _encoderCount;
    long _previousTime;
    long _previousEncoderCount;
    float _vel;
    float _velGoal;
    void encoderChange();
    int _cpm;

};

#endif
