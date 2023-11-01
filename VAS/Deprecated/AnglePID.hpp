#ifndef _ANGLEPIDH_
#define _ANGLEPIDH_

#include "MathFuncs.hpp"

class AnglePID {
  public:
    const char *tag; // what the PID is controlling
    float maxErrorDeg; // degree (max error on the axis)
    float kP, kI, kD; // unitless, [0, inf]
    float setPointDeg; // degree

    float errorDeg; // [-1, 1]
    float errorVelo; // [-inf, inf]

    float totalErrorDeg; // degrees [-MAX_INTEGRAL, MAX_INTEGRAL]
    float prevErrorDeg; // degrees [-maxErrorDeg, maxErrorDeg]

    unsigned long lastMS; // ms
    float periodMS; // ms

    // Initialize variables
    void init(char* tag, float maxErrorDeg, float kp, float kI, float kD, float setPointDeg, float measuredDeg) {
      this->tag = tag;
      this->maxErrorDeg = maxErrorDeg;
      this->kP = kP;
      this->kI = kI;
      this->kD = kD;

      this->setPointDeg = setPointDeg;

      lastMS = 0;
      periodMS = 0;

      // convert degree error to [-1, 1] error
      prevErrorDeg = MathFuncs::degToError(-maxErrorDeg, measuredDeg - setPointDeg, maxErrorDeg);
      reset();
    }

    // Set error values to 0
    void reset() {
      totalErrorDeg = 0;
      prevErrorDeg = 0;
      lastMS = 0;
    }

    bool atSetPoint() {
      return MathFuncs::floatAbs(errorDeg) < ERROR_TOLERANCE_DEG;
    }

    // outputs error response [-1, 1]. 
    // Ideally, if error is 1, a pure p controller will return -1
    float calculate(float measuredDeg) {
      float output;
      periodMS = (float)(millis() - lastMS);
      // negative error clockwise
      float errorDeg = setPointDeg - measuredDeg;

      errorDeg = MathFuncs::degToError(-maxErrorDeg, errorDeg, maxErrorDeg);

      if (atSetPoint()) {
        output = 0;
      } else {
        // velocity is change in deg / change in ms
        errorVelo = (errorDeg - prevErrorDeg) / periodMS;
        // add the average velo for this period * the period length to the total error
        totalErrorDeg = MathFuncs::clip(-MAX_INTEGRAL_DEG, totalErrorDeg + (errorDeg * periodMS), MAX_INTEGRAL_DEG);

        output = MathFuncs::clip(-1, (errorDeg * kP + errorVelo * kD + totalErrorDeg * kI) / MAX_ERROR_RESPONSE, 1);
      }
    
      lastMS = millis;

      return output;
    }

    // Converts error from calculate method to degree
    // [-1, 1] -> [MIN_SERVO_DEG, MAX_SERVO_DEG]
    float calculateDegree(float measuredDeg) {
      // output is from [-1, 1]
      float output = calculate(measuredDeg);

      // convert [-1, 1] to servo degrees, 0 being neutral, -1 being the min, 1 being the max
      return MathFuncs::clip(MIN_SERVO_DEG, (output * NEUTRAL_SERVO_DEG) + NEUTRAL_SERVO_DEG, MAX_SERVO_DEG);
    }

    // convert degree from calculateDegree to servo ticks
    // [MIN_SERVO_DEG, MAX_SERVO_DEG] -> [0, SERVO_TICKS]
    float calculatePos(float measuredDeg) {
      // output is from [MIN_SERVO_DEG, MAX_SERVO_DEG]
      float output = calculateDegree(measuredDeg);
      
      // convert servo degree to position
      return MathFuncs::degToServoPos(output);
    }

    // returns the name of this AnglePID
    char getTag() {
      return *tag;
    }
};

#endif