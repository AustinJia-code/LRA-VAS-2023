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
#include "AngleWrapper.hpp"


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
}

void loop() {
  herkulex_bus.update();
  mpu6050.update();
  runtime = millis();

  // roll + when rotating clockwise from above
  float roll = -mpu6050.getAngleZ();
  // pitch + when rotating clockwise from the right
  float pitch = -mpu6050.getAngleX();
  // yaw + when rotating clockwise from the front
  float yaw = -mpu6050.getAngleY();

  if (runtime > ACTIVATION_TIME && runtime % ACTION_RATE_MS == 0) {
    actuate(roll, pitch, yaw);
  }

  if (runtime % TELEMETRY_RATE_MS == 0) {
    Serial.println("roll : ");
    Serial.print(roll);
    Serial.println("pitch : ");
    Serial.print(pitch);
    Serial.println("yaw : ");
    Serial.print(yaw);

    Serial.println("North : ");
    Serial.print(AngleWrapper::servoPosToRelAng(NORTH.getRawPosition()));
    Serial.println("East : ");
    Serial.print(AngleWrapper::servoPosToRelAng(EAST.getRawPosition()));
    Serial.println("South : ");
    Serial.print(AngleWrapper::servoPosToRelAng(SOUTH.getRawPosition()));
    Serial.println("West : ");
    Serial.print(AngleWrapper::servoPosToRelAng(WEST.getRawPosition()));
  }
}

void actuate(float roll, float pitch, float yaw) {    
  float counterRoll = roll * ROLL_KP;
  float counterYaw = yaw * YAW_KP;
  float counterPitch = pitch * PITCH_KP;

  /*
  NORTH KINEMATICS:
  Rotate clockwise when roll +
  Do not rotate when pitch +
  Rotate counterclockwise when yaw +
  */
  NORTH.setPosition(AngleWrapper::degToServoPos(counterRoll + 0 + -counterYaw));
  /*
  EAST KINEMATICS:
  Rotate clockwise when roll +
  Rotate clockwise when pitch +
  Do not rotate when yaw +
  */
  EAST.setPosition(AngleWrapper::degToServoPos(counterRoll  + counterPitch + 0));
  /*
  SOUTH KINEMATICS:
  Rotate clockwise when roll +
  Do not rotate when pitch +
  Rotate clockwise when yaw +
  */
  SOUTH.setPosition(AngleWrapper::degToServoPos(counterRoll + 0 + counterYaw));
  /*
  WEST KINEMATICS:
  Rotate clockwise when roll +
  Rotate counterclockwise when pitch +
  Do not rotate when yaw +
  */
  WEST.setPosition(AngleWrapper::degToServoPos(counterRoll + counterPitch + 0));
}