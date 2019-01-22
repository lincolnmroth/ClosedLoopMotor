#ifndef ClosedLoopMotor_h
#define ClosedLoopMotor_h

#include "WProgram.h"

class ClosedLoopMotor
{
  public:
    ClosedLoopMotor(int encA, int encB, int motorA, int motorB, int motorEn, float Kp, float Ki, float Kd);
    void setVel(int vel);
    float getVel();
    int getCounts();
  private:
    int _encA;
    int _encB;
    int _motorA;
    int _motorB;
    int _motorC;
    float _Kp;
    float _Ki;
    float _Kd;
    int _encoderCounts;
    float _vel;

};

#endif
