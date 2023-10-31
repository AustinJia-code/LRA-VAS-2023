#ifndef _ROCKETCONSTANTSH_
#define _ROCKETCONSTANTSH_

// HerculeX Vars
static const int MAX_SERVO_DEG = 270;
static const int MIN_SERVO_DEG = 0;
static const int NEUTRAL_SERVO_DEG = (MAX_SERVO_DEG + MIN_SERVO_DEG) / 2;
static const int SERVO_TICKS = 1023;
static const int NORTH_ID = 1;  // arbitrary ID
static const int EAST_ID = 2;  // arbitrary ID
static const int SOUTH_ID = 3;  // arbitrary ID
static const int WEST_ID = 4;  // arbitrary ID

// Max rocket errors
static const float MAX_PITCH_DEG = 20;
static const float MAX_YAW_DEG = 20;
static const float MAX_ROLL_DEG = 20;

// Arduino ports
static const int RECEIVE_PIN = 8;   // Receiving wire in pin 8
static const int TRANSMIT_PIN = 9 ;  // Transmitting in pin 9

// General program settings
static const int ACTION_RATE_MS = 100;
static const int TELEMETRY_RATE_MS = 100;
static const int BAUD_RATE = 115200;

// MPU settings
static const float ACCEL_COEF = 0.2;
static const float GYRO_COEF = 0.8;
static const int ACTIVATION_TIME = -1;

// Error controller values
static const float PITCH_SETPOINT = 0;
static const float PITCH_KP = 0.5;
static const float PITCH_KI = 0.5;
static const float PITCH_KD = 0.5;
static const float ROLL_SETPOINT = 0;
static const float ROLL_KP = 0.2;
static const float ROLL_KI = 0.2;
static const float ROLL_KD = 0.2;
static const float YAW_SETPOINT = 0;
static const float YAW_KP = 0.5;
static const float YAW_KI = 0.5;
static const float YAW_KD = 0.5;
static const float MAX_INTEGRAL = 1;
static const float ERROR_TOLERANCE = 0.05;

#endif