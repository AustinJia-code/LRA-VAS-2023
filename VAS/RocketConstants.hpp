#ifndef _ROCKETCONSTANTSH_
#define _ROCKETCONSTANTSH_

// Arduino
static const int BAUD_RATE = 115200;

// HerculeX Vars
static const int MAX_SERVO_DEG = 270;
static const int MIN_SERVO_DEG = 0;
static const int NEUTRAL_SERVO_DEG = (MAX_SERVO_DEG + MIN_SERVO_DEG) / 2;
static const int SERVO_RANGE_DEG = MAX_SERVO_DEG - MIN_SERVO_DEG;
static const int SERVO_TICKS = 1023;
static const int NEUTRAL_SERVO_TICK = SERVO_TICKS / 2;
static const int TICKS_PER_DEG = SERVO_TICKS / SERVO_RANGE_DEG;

static float degToTicks (float deg)
{
  deg /= MAX_SERVO_DEG;
  deg *= SERVO_TICKS;
}

// HerculeX chosen range
static const float ACTIVE_RANGE_DEG = 30;
static const int ACTIVE_RANGE_TICKS = degToTicks (ACTIVE_RANGE_DEG);
static const int ACTIVE_MIN_TICK = NEUTRAL_SERVO_TICK - ACTIVE_RANGE_TICKS / 2.0;
static const int ACTIVE_MAX_TICK = NEUTRAL_SERVO_TICK + ACTIVE_RANGE_TICKS / 2.0;

// HerculeX IDs
static const int NORTH_ID = 1;  // arbitrary ID
static const int EAST_ID = 2;   // arbitrary ID
static const int SOUTH_ID = 3;  // arbitrary ID
static const int WEST_ID = 4;   // arbitrary ID

// Arduino ports
static const int RECEIVE_PIN = 8;     // Receiving wire in pin 8
static const int TRANSMIT_PIN = 9 ;  // Transmitting in pin 9

// General program settings
static const int ACTION_RATE_MS = 100;
static const int TELEMETRY_RATE_MS = 100;
static const int BAUD_RATE = 115200;

// MPU settings
static const float ACCEL_COEF = 0.2;
static const float GYRO_COEF = 0.8;
static const int ACTIVATION_TIME_MS = -1;

// Error controller values
// Target pitch is 0 degrees
static const float PITCH_SETPOINT_DEG = 0;
static const float PITCH_KP = 1 * TICKS_PER_DEG;
static const float PITCH_KI = 0;
static const float PITCH_KD = 0;

// Target yaw is 0 degrees
static const float YAW_SETPOINT_DEG = 0;
static const float YAW_KP = 1 * TICKS_PER_DEG;
static const float YAW_KI = 0;
static const float YAW_KD = 0;

// Target roll velocity is 0 deg/ms
static const float ROLL_SETPOINT_DEGMS = 0;
static const float ROLL_KP = 1 * TICKS_PER_DEG;
static const float ROLL_KI = 0;
static const float ROLL_KD = 0;
static const float MAX_INTEGRAL_TICKS = 3.4028235E+38;
static const float MAX_INTEGRAL_DEGMS = 3.4028235E+38;
static const float ERROR_TOLERANCE_DEG = 1;
static const float ERROR_TOLERANCE_DEGMS = 1;
static const float ERROR_TOLERANCE_TICKS = degToTicks (ERROR_TOLERANCE_DEG);

// Max rocket errors (deprecated)
static const float MAX_PITCH_DEG = 20;
static const float MAX_YAW_DEG = 20;
static const float MAX_ROLL_DEG = 20;
static const float MAX_ERROR_RESPONSE = 1; // general error sensitivity

// Kalman
const float Q_ANGLE = 0.001;  // process noise for angle
const float Q_BIAS  = 0.003;  // process noise for gyro bias
const float R_MEASURE = 0.03; // measurement noise

#endif