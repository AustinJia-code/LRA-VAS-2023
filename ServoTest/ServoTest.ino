#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HerkulexServo.h>
#include <CircularBuffer.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#define PIN_SW_RX 8
#define PIN_SW_TX 9
#define SERVO_ID_1 1
#define SERVO_ID_2 2
#define SERVO_ID_3 3
#define SERVO_ID_4 4

MPU6050 mpu6050(Wire);
SoftwareSerial   servo_serial(PIN_SW_RX, PIN_SW_TX);
HerkulexServoBus herkulex_bus(servo_serial);
HerkulexServo    servo_ID_1(herkulex_bus, SERVO_ID_1);
HerkulexServo    servo_ID_2(herkulex_bus, SERVO_ID_2);
HerkulexServo    servo_ID_3(herkulex_bus, SERVO_ID_3);
HerkulexServo    servo_ID_4(herkulex_bus, SERVO_ID_4);

unsigned long last_update = 0;
unsigned long last_update2 = 0;
unsigned long now = 0;
bool toggle = false;


void setup() {
  Serial.begin(115200);
  servo_serial.begin(115200);
  delay(500);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  servo_ID_1.setTorqueOn();
  servo_ID_2.setTorqueOn();
  servo_ID_3.setTorqueOn();
  servo_ID_4.setTorqueOn();
  servo_ID_1.setLedColor(HerkulexLed::Blue);
 servo_ID_2.setLedColor(HerkulexLed::Green);
 servo_ID_3.setLedColor(HerkulexLed::Purple);
  servo_ID_4.setLedColor(HerkulexLed::Cyan);
}

void loop() {
   herkulex_bus.update();
mpu6050.update();
  now = millis();
 int x=mpu6050.getAngleX();
 int y=mpu6050.getAngleY() ;  
 if ( (now - last_update) > 100) {
      if (x<90 and x>-90){
        uint16_t pos = 512 + uint16_t(x / 0.325);
        uint16_t pos2 = 512 - uint16_t(x / 0.325);
      servo_ID_1.setPosition(pos, 50);
      servo_ID_3.setPosition(pos2, 50);
      }
      if (y<90 and y>-90)
      {uint16_t pos = 512 + uint16_t(y / 0.325);
      uint16_t pos2 = 512 - uint16_t(y / 0.325);
         servo_ID_2.setPosition(pos2, 50);
      servo_ID_4.setPosition(pos, 50);
        
      }
        last_update = now;
 }
  
    if ( (now - last_update2) > 1000) {
   
  Serial.print("angleX : ");
  Serial.print(mpu6050.getAngleX());
  Serial.print("\tangleY : ");
  Serial.print(mpu6050.getAngleY());
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
    last_update2 = now;
    }
  }
