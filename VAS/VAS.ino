#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HerkulexServo.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include "RocketConstants.hpp"
#include "MathFuncs.hpp"
#include "GenericPID.hpp"
#include "Kalman.hpp"

float roll, pitch, yaw;             // filtered angles
float rollRaw, pitchRaw, yawRaw;    // raw MPU angles

GenericPID rollPID, pitchPID, yawPID;
KalmanFilter rollKF, pitchKF, yawKF;

unsigned long runtime = 0;
unsigned long execTime = 0;

MPU6050 mpu6050 (Wire);
SoftwareSerial servo_serial (RECEIVE_PIN, TRANSMIT_PIN);
HerkulexServoBus herkulex_bus (servo_serial);
HerkulexServo NORTH (herkulex_bus, NORTH_ID);
HerkulexServo EAST (herkulex_bus, EAST_ID);
HerkulexServo SOUTH (herkulex_bus, SOUTH_ID);
HerkulexServo WEST (herkulex_bus, WEST_ID);

void setup ()
{
  Serial.begin (BAUD_RATE);
  servo_serial.begin (BAUD_RATE);

  NORTH.setTorqueOn ();
  EAST.setTorqueOn ();
  SOUTH.setTorqueOn ();
  WEST.setTorqueOn ();

  NORTH.setLedColor (HerkulexLed::Blue);
  EAST.setLedColor (HerkulexLed::Green);
  SOUTH.setLedColor (HerkulexLed::Purple);
  WEST.setLedColor (HerkulexLed::Cyan);

  Wire.begin ();
  mpu6050.begin ();
  mpu6050.calcGyroOffsets (true);

  // Initialize raw and filtered angles
  rollRaw = roll = -mpu6050.getAngleZ ();
  pitchRaw = pitch = -mpu6050.getAngleX ();
  yawRaw = yaw = -mpu6050.getAngleY ();

  rollKF.init (roll, Q_ANGLE, Q_BIAS, R_MEASURE);
  pitchKF.init (pitch, Q_ANGLE, Q_BIAS, R_MEASURE);
  yawKF.init (yaw, Q_ANGLE, Q_BIAS, R_MEASURE);

  rollPID.init ("ROLL", ROLL_KP, ROLL_KI, ROLL_KD, 
                        ROLL_SETPOINT_DEGMS, MAX_INTEGRAL_DEGMS, ERROR_TOLERANCE_DEGMS);
  pitchPID.init ("PITCH", PITCH_KP, PITCH_KI, PITCH_KD,
                          PITCH_SETPOINT_DEG, MAX_INTEGRAL_TICKS, ERROR_TOLERANCE_DEG);
  yawPID.init ("YAW", YAW_KP, YAW_KI, YAW_KD, 
                      YAW_SETPOINT_DEG, MAX_INTEGRAL_TICKS, ERROR_TOLERANCE_DEG);
}

void loop ()
{
  herkulex_bus.update ();
  unsigned long newRuntime = millis ();
  float dt_s = newRuntime - runtime;
  runtime = newRuntime;

  mpu6050.update ();

  rollRaw  = -mpu6050.getAngleZ ();
  pitchRaw = -mpu6050.getAngleX ();
  yawRaw   = -mpu6050.getAngleY ();

  roll = KalmanUpdate (rollKF,  rollRaw, mpu6050.getGyroZ (), dt_s);
  pitch = KalmanUpdate (pitchKF, pitchRaw, mpu6050.getGyroX (), dt_s);
  yaw = KalmanUpdate (yawKF, yawRaw, mpu6050.getGyroY (), dt_s);

  if (runtime > ACTIVATION_TIME_MS && runtime - execTime > ACTION_RATE_MS - 1)
  {
    execTime = runtime;
    actuate ();
  }

  if (runtime % TELEMETRY_RATE_MS == 0)
    printTelemetry();
}

// Actuate servos and update PID
void actuate ()
{
  float counterRoll  = MathFuncs::clip (-7.0, rollPID.calculate (roll, runtime), 7.0);
  float counterPitch = MathFuncs::clip (-7.0, pitchPID.calculate (pitch, runtime), 7.0);
  float counterYaw   = MathFuncs::clip (-7.0, yawPID.calculate (yaw, runtime), 7.0);

  NORTH.setPosition (counterRoll - counterYaw);
  EAST.setPosition (counterRoll + counterPitch);
  SOUTH.setPosition (counterRoll + counterYaw);
  WEST.setPosition (counterRoll - counterPitch);
}

void printTelemetry ()
{
  Serial.println ("=== Telemetry ===");
  Serial.print ("roll: "); Serial.println (roll);
  Serial.print ("pitch: "); Serial.println (pitch);
  Serial.print ("yaw: "); Serial.println (yaw);

  Serial.print ("NORTH: "); Serial.println (NORTH.getRawPosition ());
  Serial.print ("EAST: "); Serial.println (EAST.getRawPosition ());
  Serial.print ("SOUTH: "); Serial.println (SOUTH.getRawPosition ());
  Serial.print ("WEST: "); Serial.println (WEST.getRawPosition ());

  Serial.print ("PID Roll Output: "); Serial.println (rollPID.getLastOutput ());
  Serial.print ("PID Pitch Output: "); Serial.println (pitchPID.getLastOutput ());
  Serial.print ("PID Yaw Output: "); Serial.println (yawPID.getLastOutput ());
}
