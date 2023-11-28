// Testbed

#include <Arduino.h>
#include <SoftwareSerial.h>
//https://docs.arduino.cc/learn/built-in-libraries/software-serial
#include <HerkulexServo.h>
//https://github.com/cesarvandevelde/HerkulexServo
#include <CircularBuffer.h>
#include <MPU6050_tockn.h>
// https://github.com/electroniccats/mpu6050
#include <Wire.h>

// Create labels defining what is what
#define PIN_SW_RX 8   // in pin 8
#define PIN_SW_TX 9   // in pin 9

// Intantiate gyroscope
MPU6050 mpu6050(Wire);
// Instantiate a pairing between a receiving pin (RX) and transmitting pin (TX)
SoftwareSerial servo_serial(PIN_SW_RX, PIN_SW_TX);
// HerculexServoBus manages communication layer and is shared between all servos
HerkulexServoBus herkulex_bus(servo_serial);
// Instantiate all servos under the servo bus with unique IDs
HerkulexServo NORTH(herkulex_bus, 1);
HerkulexServo EAST(herkulex_bus, 2);
//HerkulexServo SOUTH(herkulex_bus, 3);
HerkulexServo WEST(herkulex_bus, 4);

unsigned long last = 0;
unsigned long now = 0;
bool toggle = false;

unsigned long LAUNCH_DELAY_MS = 2000;
int MIN_PERIOD_MS = 100;


void setup() {
  Serial.begin(115200);
  servo_serial.begin(115200);
  delay(500);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // Turn servos on
  NORTH.setTorqueOn();
  EAST.setTorqueOn();
  //SOUTH.setTorqueOn();
  WEST.setTorqueOn();
  // Set servo colors
  NORTH.setLedColor(HerkulexLed::Blue);
  EAST.setLedColor(HerkulexLed::Green);
  //SOUTH.setLedColor(HerkulexLed::Purple);
  WEST.setLedColor(HerkulexLed::Cyan);

  NORTH.setPosition(512);
  EAST.setPosition(512);
  //SOUTH.setPosition(512);
  WEST.setPosition(512);
}

int z_last = 0;

void loop() {
  herkulex_bus.update();
  mpu6050.update();

  now = millis();
  unsigned long period_ms = now - last;

  int x = mpu6050.getAngleX();
  int y = mpu6050.getAngleY();
  int z = mpu6050.getAngleZ();

  if (now > LAUNCH_DELAY_MS && period_ms > MIN_PERIOD_MS) {
    // get roll velocity in degrees / sec
    uint16_t z_vel = z / period_ms * 1000;
  
    uint16_t x_correction = uint16_t(x / 0.325);
    uint16_t y_correction = uint16_t(y / 0.325);
    uint16_t z_correction = uint16_t(z_vel / 0.325);

    NORTH.setPosition(512 + x_correction + z_correction);
    //SOUTH.setPosition(512 - x_correction + z_correction);
    EAST.setPosition(512 + y_correction + z_correction);
    WEST.setPosition(512 - y_correction + z_correction);
  }

  // telemetry
  if (period_ms > MIN_PERIOD_MS) {
    Serial.print("angleX : ");
    Serial.print(mpu6050.getAngleX());
    Serial.print("\tangleY : ");
    Serial.print(mpu6050.getAngleY());
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
  }

  last = now;
}