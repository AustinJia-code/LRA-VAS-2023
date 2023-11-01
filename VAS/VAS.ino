// Current code for VAS Fall 2023
// Roll - Spinning
// Pitch - Leaning forward/backward (pitching forward to throw up)
// Yaw - Tilting left/right

#include <Arduino.h>
#include <SoftwareSerial.h>
//https://docs.arduino.cc/learn/built-in-libraries/software-serial
#include <HerkulexServo.h>
//https://github.com/cesarvandevelde/HerkulexServo
#include <CircularBuffer.h>
#include <MPU6050_tockn.h>
// https://github.com/electroniccats/mpu6050
#include <Wire.h>
#include "RocketConstants.hpp"
#include "MathFuncs.hpp"
#include "AnglePID.hpp"

// Attitude in degrees
float roll, pitch, yaw;
// Motor controllers
AnglePID rollPID, pitchPID, yawPID;
bool recalibrated;
// Runtims in ms
unsigned long runtime = 0;

// Intantiate gyroscope
MPU6050 mpu6050(Wire, ACCEL_COEF, GYRO_COEF);
// Instantiate a pairing between a receiving pin (RX) and transmitting pin (TX)
SoftwareSerial servo_serial(RECEIVE_PIN, TRANSMIT_PIN);
// HerculexServoBus manages communication layer and is shared between all servos
HerkulexServoBus herkulex_bus(servo_serial);
// Instantiate all servos under the servo bus with unique IDs
HerkulexServo NORTH(herkulex_bus, NORTH_ID);
HerkulexServo EAST(herkulex_bus, EAST_ID);
HerkulexServo SOUTH(herkulex_bus, SOUTH_ID);
HerkulexServo WEST(herkulex_bus, WEST_ID);

// Initialize everything
void setup() {
  Serial.begin(BAUD_RATE);
  servo_serial.begin(BAUD_RATE);

  // Turn servos on
  NORTH.setTorqueOn();
  EAST.setTorqueOn();
  SOUTH.setTorqueOn();
  WEST.setTorqueOn();

  // Set servo colors
  NORTH.setLedColor(HerkulexLed::Blue);
  EAST.setLedColor(HerkulexLed::Green);
  SOUTH.setLedColor(HerkulexLed::Purple);
  WEST.setLedColor(HerkulexLed::Cyan);

  // Calibrate gyro
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // roll + when rotating clockwise from above
  // consideration: set roll error to angular velocity? It doesn't matter what rotation we are in, as long as we aren't spinning
  roll = -mpu6050.getAngleZ();
  // pitch + when rotating clockwise from the right
  pitch = -mpu6050.getAngleX();
  // yaw + when rotating clockwise from the front
  yaw = -mpu6050.getAngleY();

  // Initialize PID controllers
  rollPID.init("ROLL", MAX_ROLL_DEG, ROLL_KP, ROLL_KI, ROLL_KD, ROLL_SETPOINT_DEG, roll);
  pitchPID.init("PITCH", MAX_PITCH_DEG, PITCH_KP, PITCH_KI, PITCH_KD, PITCH_SETPOINT_DEG, pitch);
  yawPID.init("YAW", MAX_YAW_DEG, YAW_KP, YAW_KI, YAW_KD, YAW_SETPOINT_DEG, yaw);

  recalibrated = false;
}

// Standard arduino loop
void loop() {
  herkulex_bus.update();
  runtime = millis();

  // updates angles
  roll = -mpu6050.getAngleZ();
  pitch = -mpu6050.getAngleX();
  yaw = -mpu6050.getAngleY();

  if (runtime > ACTIVATION_TIME_MS && !recalibrated) {
    
  }

  if (runtime > ACTIVATION_TIME_MS && runtime % ACTION_RATE_MS == 0) {
    // if rocket has taken off and we want to update the servos, update the MPU and actuate
    mpu6050.update();
    actuate();
  }

  if (runtime % TELEMETRY_RATE_MS == 0) {
    // if we want to print telemetry, do so
    printTelemetry();
  }
}

void actuate() {    
  // calculate the PID responses (in servo position, [0, SERVO_TICKS]) we want from the servos
  float counterRoll = rollPID.calculatePos(roll, runtime);
  float counterYaw = yawPID.calculatePos(yaw, runtime);
  float counterPitch = pitchPID.calculatePos(pitch, runtime);

  /*
  NORTH KINEMATICS:
  Rotate clockwise when roll +
  Do not rotate when pitch +
  Rotate counterclockwise when yaw +
  */
  NORTH.setPosition(counterRoll / 2 + 
                    0 + 
                   -counterYaw / 2);
  /*
  EAST KINEMATICS:
  Rotate clockwise when roll +
  Rotate clockwise when pitch +
  Do not rotate when yaw +
  */
  EAST.setPosition(counterRoll / 2 + 
                   counterPitch / 2 + 
                   0);
  /*
  SOUTH KINEMATICS:
  Rotate clockwise when roll +
  Do not rotate when pitch +
  Rotate clockwise when yaw +
  */
  SOUTH.setPosition(counterRoll / 2 + 
                    0 + 
                    counterYaw / 2);
  /*
  WEST KINEMATICS:
  Rotate clockwise when roll +
  Rotate counterclockwise when pitch +
  Do not rotate when yaw +
  */
  WEST.setPosition(counterRoll / 2 + 
                  -counterPitch / 2 + 
                   0);
}

// Telemetry controller
void printTelemetry() {
  Serial.println("MPU Updating: " + (runtime > ACTIVATION_TIME_MS && runtime % ACTION_RATE_MS == 0));
  mpuTelemetry();
  servoTelemetry();
  PIDTelemetry();
}

// MPU angles
void mpuTelemetry() { 
  Serial.println("roll: ");
  Serial.print(roll);
  Serial.println("pitch: ");
  Serial.print(pitch);
  Serial.println("yaw: ");
  Serial.print(yaw);
}

// Servo positions
void servoTelemetry() {
  Serial.println("North: ");
  Serial.print(MathFuncs::servoPosToRelAng(NORTH.getRawPosition()));
  Serial.println("East: ");
  Serial.print(MathFuncs::servoPosToRelAng(EAST.getRawPosition()));
  Serial.println("South: ");
  Serial.print(MathFuncs::servoPosToRelAng(SOUTH.getRawPosition()));
  Serial.println("West: ");
  Serial.print(MathFuncs::servoPosToRelAng(WEST.getRawPosition()));
}

// PID values
void PIDTelemetry() {

}