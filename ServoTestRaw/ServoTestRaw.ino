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
#define SERVO_ID_3 3  // arbitrary ID
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
HerkulexServo servo_ID_3(herkulex_bus, SERVO_ID_3);
HerkulexServo servo_ID_4(herkulex_bus, SERVO_ID_4);

unsigned long count = 0;

void setup() {
  Serial.begin(9660);
  servo_serial.begin(9660);
  delay(500);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // Turn servos on
  servo_ID_1.setTorqueOn();
  servo_ID_2.setTorqueOn();
  servo_ID_3.setTorqueOn();
  servo_ID_4.setTorqueOn();

  // Set servo colors
  servo_ID_1.setLedColor(HerkulexLed::Blue);
  servo_ID_2.setLedColor(HerkulexLed::Green);
  servo_ID_3.setLedColor(HerkulexLed::Purple);
  servo_ID_4.setLedColor(HerkulexLed::Cyan);

  Serial.println();
}

void loop() {
  //while (count < 10) {
    count = count + 1;
    herkulex_bus.update();
    mpu6050.update();
    servo_ID_1.setPosition(512, 50);
    //delay(1000);
    //servo_ID_2.setPosition(1000, 50);
    //delay(1000);
    herkulex_bus.executeMove();
    Serial.print("Loop: ");
    Serial.println(count);
  //}
}