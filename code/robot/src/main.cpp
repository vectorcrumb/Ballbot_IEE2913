#include "Arduino.h"

#define LED 13

#define ENC1A 39
#define ENC1B 38
#define PWM1 23
#define INB1 22
#define INA1 21

#define ENC2A 17
#define ENC2B 16
#define PWM2 20
#define INB2 19
#define INA2 18

#define ENC3A 37
#define ENC3B 36
#define PWM3 30
#define INB3 34
#define INA3 33

#define XACC_OFFSET -524
#define YACC_OFFSET 677
#define ZACC_OFFSET 1731
#define XGYR_OFFSET 26
#define YGYR_OFFSET 2
#define ZGYR_OFFSET 0

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>
#include <VNHDriver.h>
#include <Encoder.h>

MPU6050 imu;

VNHDriver motor1, motor2, motor3;
Encoder enc1(ENC1A, ENC1B);
Encoder enc2(ENC2A, ENC2B);
Encoder enc3(ENC3A, ENC3B);

int16_t ax, ay, az;
int16_t gx, gy, gz;
int drive_speed = 50;

void blink_led(int period, int repeat) {
  for (int i = 0; i < repeat; i++) {
    digitalWrite(13, HIGH);
    delay(period);
    digitalWrite(13, LOW);
    delay(period);
  }
}

void setup() {
  pinMode(LED, OUTPUT);
  blink_led(25, 10);

  Serial.begin(115200);
  Serial.println("Init drivers ...");

  motor1.begin(PWM1, INA1, INB1);
  motor2.begin(PWM2, INA2, INB2);
  motor3.begin(PWM3, INA3, INB3);

  Serial.println("Motors ready.");
  Serial.println("Init IMU ...");
  Wire2.begin();
  imu.initialize();
  Serial.println(imu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  imu.setXGyroOffset(XGYR_OFFSET);
  imu.setYGyroOffset(YGYR_OFFSET);
  imu.setZGyroOffset(ZGYR_OFFSET);
  imu.setXAccelOffset(XACC_OFFSET);
  imu.setYAccelOffset(YACC_OFFSET);
  imu.setZAccelOffset(ZACC_OFFSET);
  Serial.println("Set IMU offsets.");
  Serial.println("IMU ready.");
  blink_led(50, 4);
}


void loop() {
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);

  motor1.setSpeed(drive_speed);
  motor2.setSpeed(drive_speed);
  motor3.setSpeed(drive_speed);
  delay(1000);
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  delay(100);

  motor1.setSpeed(-1*drive_speed);
  motor2.setSpeed(-1*drive_speed);
  motor3.setSpeed(-1*drive_speed);
  delay(1000);
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  delay(100);

  Serial.print("Enc1: "); Serial.print(enc1.read()); Serial.print("\t");
  Serial.print("Enc2: "); Serial.print(enc2.read()); Serial.print("\t");
  Serial.print("Enc3: "); Serial.println(enc3.read());
}

void loop()
{
  digitalWriteFast(LED, !digitalReadFast(LED));
  delay(500);
}
