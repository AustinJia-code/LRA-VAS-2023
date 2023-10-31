#ifndef _PIDH_
#define _PIDH_

#include "MathFuncs.hpp"

class PID {
  public:
    const char *tag; // what the PID is controlling
    float maxDeg; // degree (max error on the axis)
    float kP, kI, kD; // unitless, [0, inf]
    float setPoint; // degree

    float errorVal_p; // [-1, 1]
    float errorVal_v; // [-inf, inf]

    float totalError; // degrees [-MAX_INTEGRAL, MAX_INTEGRAL]
    float prevErrorVal; // degrees [-maxDeg, maxDeg]

    float lastTimeStamp; // ms
    float period; // ms

    // Initialize variables
    void init(char* tag, float maxDeg, float kp, float kI, float kD, float sp, float measuredDeg) {
      this->tag = tag;
      this->maxDeg = maxDeg;
      this->kP = kP;
      this->kI = kI;
      this->kD = kD;

      setPoint = sp;

      lastTimeStamp = 0;
      period = 0;

      // convert degree error to [-1, 1] error
      prevErrorVal = MathFuncs::degToError(-maxDeg, measuredDeg - setPoint, maxDeg);
      reset();
    }

    // Set error values to 0
    void reset() {
      totalError = 0;
      prevErrorVal = 0;
      lastTimeStamp = 0;
    }

    bool atSetPoint() {
      return MathFuncs::floatAbs(errorVal_p) < ERROR_TOLERANCE;
    }

    // outputs error response [-1, 1]. 
    // Ideally, if error is 1, a pure p controller will return -1
    float calculate(float measuredDeg, float millis) {
      period = millis - lastTimeStamp;

      errorVal_p = MathFuncs::degToError(-maxDeg, setPoint - measuredDeg, maxDeg);
      errorVal_v = (errorVal_p - prevErrorVal) / period;
      totalError = MathFuncs::clip(-MAX_INTEGRAL, totalError + (errorVal_p * period), MAX_INTEGRAL);

      float output = MathFuncs::clip(-1, errorVal_p * kP + errorVal_v * kD + totalError * kI, 1);
      lastTimeStamp = millis;

      return output;
    }

    float calculateDegree(float measuredDeg, float millis) {
      // output is from [-1, 1]
      float output = calculate(measuredDeg, millis);

      // convert [-1, 1] to servo degrees, 0 being neutral, -1 being the min, 1 being the max
      return MathFuncs::clip(MIN_SERVO_DEG, (output * NEUTRAL_SERVO_DEG) + NEUTRAL_SERVO_DEG, MAX_SERVO_DEG);
    }

    float calculatePos(float measuredDeg, float millis) {
      // output is from [MIN_SERVO_DEG, MAX_SERVO_DEG]
      float output = calculateDegree(measuredDeg, millis);
      
      // convert servo degree to position
      return MathFuncs::degToServoPos(output);
    }

    char getTag() {
      return *tag;
    }
};

#endif