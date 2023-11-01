#ifndef _GENEARLPIDH_
#define _GENEARLPIDH_

#include "MathFuncs.hpp"

class GeneralPID {
  public:
    unsigned long lastMS;
    float input, output, setPoint;
    float totalError, prevError;
    float kP, kI, kD;

    void init(float Kp, float Ki, float Kd, float setpoint) {
      kP = Kp;
      kI = Ki;
      kD = Kd;
      setPoint = setpoint;
    }

    float calculate(float input) {
      unsigned long now = millis();
      float period = (float)(now - lastMS);
        
      /*Compute all the working error variables*/
      float error = setPoint - input;
      totalError += (error * period);
      float dErr = (error - prevError) / period;
        
      /*Compute PID output*/
      output = kP * error + kI * totalError + kD * dErr;
        
      /*Remember some variables for next time*/
      prevError = error;
      lastMS = now;

      return output;
    }
        
};

#endif