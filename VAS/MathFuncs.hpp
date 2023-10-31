#ifndef _MATHFUNCSH_
#define _MATHFUNCSH_

#include "RocketConstants.hpp"

class MathFuncs {
  public:
    static int degToServoPos(float degree) {
      if (degree > MAX_SERVO_DEG) {
        degree = MAX_SERVO_DEG;
      }

      if (degree < MIN_SERVO_DEG) {
        degree = MIN_SERVO_DEG;
      }

      // degree to MIN_SERVO_DEG - MAX_SERVO_DEG to 0 - max ticks
      return (degree - MIN_SERVO_DEG) / (MAX_SERVO_DEG - MIN_SERVO_DEG) * SERVO_TICKS;
    }

    static int servoPosToRelAng(int pos) {
      // pos to 0 - 1 to degree to -neutral - neutral
      return ((float) pos / SERVO_TICKS * MAX_SERVO_DEG) - NEUTRAL_SERVO_DEG;
    }

    static float degToError(float min, float val, float max) {
      val = clip(min, val, max);

      // MIN ----- VAL ----- MAX
      // -1  -----  0  -----  1

      float range = max - min;
      float error = ((val + min) / range * 2.0) - 1;

      return error;
    }

    static float clip(float min, float val, float max) {
      if (val < min) {
        val = min;
      } else if (val > max) {
        val = max;
      }

      return val;
    }

    static float floatAbs(float val) {
      if (val < 0) {
        return -val;
      }
      return val;
    }
};

#endif