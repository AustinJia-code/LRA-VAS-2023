// Code from old VAS OneNote

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
#define SERVO_ID_1 1  // arbitrary ID
#define SERVO_ID_2 2  // arbitrary ID
//#define SERVO_ID_3 3  // arbitrary ID
#define SERVO_ID_4 4  // arbitrary ID

// Intantiate gyroscope
MPU6050 mpu6050(Wire);
// Instantiate a pairing between a receiving pin (RX) and transmitting pin (TX)
SoftwareSerial servo_serial(PIN_SW_RX, PIN_SW_TX);
// HerculexServoBus manages communication layer and is shared between all servos
HerkulexServoBus herkulex_bus(servo_serial);
// Instantiate all servos under the servo bus with unique IDs
HerkulexServo servo_ID_1(herkulex_bus, SERVO_ID_1);
HerkulexServo servo_ID_2(herkulex_bus, SERVO_ID_2);
//HerkulexServo servo_ID_3(herkulex_bus, SERVO_ID_3);
HerkulexServo servo_ID_4(herkulex_bus, SERVO_ID_4);

unsigned long last_update = 0;
unsigned long last_update2 = 0;
unsigned long now = 0;
unsigned long LAUNCH_DELAY_MS = 2000;
bool toggle = false;


void setup() {
  Serial.begin(115200);
  servo_serial.begin(115200);
  delay(500);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // Turn servos on
  servo_ID_1.setTorqueOn();
  servo_ID_2.setTorqueOn();
  //servo_ID_3.setTorqueOn();
  servo_ID_4.setTorqueOn();
  // Set servo colors
  servo_ID_1.setLedColor(HerkulexLed::Blue);
  servo_ID_2.setLedColor(HerkulexLed::Green);
  //servo_ID_3.setLedColor(HerkulexLed::Purple);
  servo_ID_4.setLedColor(HerkulexLed::Cyan);

  servo_ID_1.setPosition(512);
  servo_ID_2.setPosition(512);
  servo_ID_4.setPosition(512);
}

void loop() {
  herkulex_bus.update();
  mpu6050.update();
  now = millis();
  int x = mpu6050.getAngleX();
  int y = mpu6050.getAngleY();
  int z = mpu6050.getAngleZ();

  unsigned long period_ms = now - last_update;
  if (now > LAUNCH_DELAY_MS && period_ms > 300) {
    uint16_t pos1 = 512 + uint16_t(x / 0.325) + uint16_t(z / 0.325);
    uint16_t pos3 = 512 - uint16_t(x / 0.325) + uint16_t(z / 0.325);

    // setPosition(pos, playtime);
    // pos can be 0 - 1023, 512 is neutral
    // each tick is 0.325 degrees, 333 degrees total
    // playtime can be [0,255], each tick represents
    // 11.2 milliseconds, if unspecified servo will
    // move as fast as possible
    servo_ID_1.setPosition(pos1);
    //servo_ID_3.setPosition(pos2, 50);
    uint16_t pos2 = 512 + uint16_t(y / 0.325) + uint16_t(z / 0.325);
    uint16_t pos4 = 512 - uint16_t(y / 0.325) + uint16_t(z / 0.325);
    servo_ID_2.setPosition(pos2);
    servo_ID_4.setPosition(pos4);  
  
    last_update = now;
  }

  if ((now - last_update2) > 1000) {
    Serial.print("angleX : ");
    Serial.print(mpu6050.getAngleX());
    Serial.print("\tangleY : ");
    Serial.print(mpu6050.getAngleY());
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    last_update2 = now;
  }
}