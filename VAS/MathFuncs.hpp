#ifndef _MATHFUNCSH_
#define _MATHFUNCSH_

#include "RocketConstants.hpp"

class MathFuncs
{
  public:
   /**
    * HerculeX Servo
    * Converts a degree input into servo range
    * Clips inputs outside of servo range
    * in: [0, MAX_SERVO_DEG], degrees
    * out: [0, SERVO_TICKS], ticks
    */
    static int degToServoPos (float degree)
    {
      degree = clip (MIN_SERVO_DEG, degree, MAX_SERVO_DEG);

      // degree to MIN_SERVO_DEG - MAX_SERVO_DEG to 0 - max ticks
      float percentOfRange = (degree - MIN_SERVO_DEG) / (MAX_SERVO_DEG - MIN_SERVO_DEG);
      return percentOfRange * SERVO_TICKS;
    }

   /**
    * HerculeX Servo
    * Converts a raw position to easily readable human angle
    * Clips inputs outside of servo range
    * in: [0, SERVO_TICKS], ticks
    * out: [MIN_SERVO_DEG - NEUTRAL_SERVO_DEG, MAX_SERVO_DEG - NEUTRAL_SERVO_DEG], degree
    *      ([-SERVO_RANGE_DEG / 2, SERVO_RANGE_DEG / 2])
    */
    static int servoPosToRelAng (int pos)
    {
      // pos to 0 - 1 to degree to -neutral - neutral
      float percentOfRange = (float) pos / SERVO_TICKS;
      return (percentOfRange * SERVO_RANGE_DEG) - NEUTRAL_SERVO_DEG;
    }

   /**
    * HerculeX Servo
    * Converts a degree into an error value
    * Clips inputs outside of servo range
    * in: min error, to be scaled to -1
          val, degrees
          max error, to be scaled to 1
    * out: [-1, 1], the position of val along [min, max], which is scaled to [-1, 1]
    */
    static float degToError (float min, float val, float max)
    {
      val = clip (min, val, max);

      // MIN ----- VAL ----- MAX
      // -1  -----  0  -----  1

      float range = max - min;
      float error = ((val + min) / range * 2.0) - 1;

      return error;
    }

  /**
    * Clips a value to a range
    * in: min val
          val to clip
          max val
    * out: [min, max], val
    */
    static float clip(float min, float val, float max)
    {
      if (val < min)
        val = min;
      else if (val > max)
        val = max;

      return val;
    }

    static float clipTicks(float val)
    {
      if (val < ACTIVE_MIN_TICK)
        val = 0;
      else if (val > ACTIVE_MAX_TICK)
        val = ACTIVE_MAX_TICK;

      return val;
    }

  /**
    * Returns the absolute value of a float
    * in: val, float
    * out: |val|
    */
    static float floatAbs(float val)
    {
      if (val < 0)
        return -val;

      return val;
    }
};

#endif