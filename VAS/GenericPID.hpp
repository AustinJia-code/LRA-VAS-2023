#ifndef _GENEARLPIDH_
#define _GENEARLPIDH_

#include "MathFuncs.hpp"

class GenericPID {
  public:
    unsigned long lastMS;
    float input, output, setPoint;
    float totalError, prevError;
    float kP, kI, kD;
    char *tag;
    float maxTotalError;
    float errorTolerance;

    void init(char *tag, float kP, float kI, float kD, float setPoint, float maxTotalError, float errorTolerance) {
      this->tag = tag;
      this->kP = kP;
      this->kI = kI;
      this->kD = kD;
      this->setPoint = setPoint;
      this->maxTotalError = maxTotalError;
      this->errorTolerance = errorTolerance;
    }

    float calculate(float input, unsigned long runtime) {
      unsigned long now = runtime;
      float period = (float)(now - lastMS);
        
      float error = setPoint - input;
        
      /*Compute PID output*/
      if (MathFuncs::floatAbs(error) < errorTolerance) {
        totalError = 0;
        output = 0;
      } else {
        totalError = MathFuncs::clip(-MAX_INTEGRAL_TICKS, totalError - (error * period), MAX_INTEGRAL_TICKS);
        float dErr = (error - prevError) / period;
        output = kP * error + kI * totalError + kD * dErr;
      }
        
      /*Remember some variables for next time*/
      prevError = error;
      lastMS = now;

      return output;
    }
};

#endif