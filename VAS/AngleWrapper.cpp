#include "RocketConstants.hpp"
#include "AngleWrapper.hpp"

int AngleWrapper::degToServoPos(float degree) {
  if (degree > MAX_SERVO_ANGLE) {
    degree = MAX_SERVO_ANGLE;
  }

  if (degree < MIN_SERVO_ANGLE) {
    degree = MIN_SERVO_ANGLE;
  }

  // degree to 0 - 1 to 0 - max ticks
  return degree / MAX_SERVO_ANGLE * SERVO_TICKS;
}

int AngleWrapper::servoPosToRelAng(int pos) {
  // pos to 0 - 1 to degree to -neutral - neutral
  return ((float) pos / SERVO_TICKS * MAX_SERVO_ANGLE) - NEUTRAL_SERVO_ANGLE;
}