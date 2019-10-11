#include "Arduino.h"

// #define IMU_SERIAL

#define LED_BOARD 13
#define LEDR1 2
#define LEDG1 5
#define LEDR2 6

#define ENC1A 17
#define ENC1B 16
#define PWM1 23
#define INB1 22
#define INA1 21

#define ENC2A 39
#define ENC2B 38
#define PWM2 20
#define INB2 19
#define INA2 18

#define ENC3A 37
#define ENC3B 36
#define PWM3 30
#define INB3 34
#define INA3 33

#define IMU_INT 24
#define SDA_ALT 17
#define SCL_ALT 16

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Wire.h>
#include <VNHDriver.h>
#include <Encoder.h>
#include <quaternionFilters.h>
#include <MPU9250.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "logo.h"

#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0
#define MAGNETIC_DECLINATION 1.33
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

MPU9250 imu(MPU9250_ADDRESS, Wire2, 400000);
VNHDriver motor1, motor2, motor3;
Encoder enc1(ENC1A, ENC1B);
Encoder enc2(ENC2A, ENC2B);
Encoder enc3(ENC3A, ENC3B);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

int drive_speed = 50;

void blink_led(int period, int repeat) {
  for (int i = 0; i < repeat; i++) {
    digitalWrite(13, HIGH);
    delay(period);
    digitalWrite(13, LOW);
    delay(period);
  }
}

void raise_error(const char* error_message) {
  Serial.println(F("Error during execution."));
  Serial.println(error_message);
  Serial.println(F("Freezing robot ..."));
  while(true) {
    digitalWriteFast(LEDG1, HIGH);
    digitalWriteFast(LEDR2, !digitalReadFast(LEDR2));
    digitalWriteFast(LEDR1, !digitalReadFast(LEDR1));
    delay(500);
  }
}

void setup() {
  // Open serial channel at 115.2 kbps
  Serial.begin(115200);
  // Pausing to wait for a Serial channel forces the robot to standby
  // when disconnected from a computer.
  // while(!Serial) {};
  Serial.println(F("Brought up serial interface. Initializing..."));
  // Initialize I2C_2 for IMU
  Wire2.begin();
  // Initialize I2C_0 for OLED
  Wire.setSDA(SDA_ALT);
  Wire.setSCL(SCL_ALT);
  Wire.begin();
  // Configure LEDS
  pinMode(LED_BOARD, OUTPUT);
  pinMode(LEDG1, OUTPUT);
  pinMode(LEDR1, OUTPUT);
  pinMode(LEDR2, OUTPUT); 
  digitalWriteFast(LED_BOARD, LOW);
  digitalWriteFast(LEDR1, LOW);
  digitalWriteFast(LEDR2, LOW);
  digitalWriteFast(LEDG1, LOW);
  // Configure other digital pins
  pinMode(IMU_INT, INPUT);
  digitalWrite(IMU_INT, LOW);
  // Flash onboard led
  delay(1);
  blink_led(50, 2);

  /**
   * OLED Configuration
   */
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    raise_error("SSD1306 allocation failed");
  } else {
    display.clearDisplay();
    display.drawBitmap(0, 0, logo, 128, 32, 1);
    display.setCursor(24, 24);
    display.setTextColor(BLACK);
    display.println("Booting...");
    display.display();
  }
  
  /**
   * IMU Configuration
   */
  int8_t imu_whoami = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 reported WHO_AM_I 0x")); Serial.println(imu_whoami, HEX);
  if (imu_whoami == 0x71) {
    Serial.println(F("MPU9250 is online..."));
    // Start by performing self test and reporting values
    imu.MPU9250SelfTest(imu.selfTest);
#ifdef IMU_SERIAL
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(imu.selfTest[0],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(imu.selfTest[1],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(imu.selfTest[2],1); Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(imu.selfTest[3],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(imu.selfTest[4],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(imu.selfTest[5],1); Serial.println("% of factory value");
#endif
    // Calibrate gyro and accelerometers, load biases in bias registers
    imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);
  } else {
    raise_error("MPU9250 error. Expected WHO_AM_I 0x71.");
  }
  // Initialize MPU
  imu.initMPU9250();
  Serial.println(F("MPU9250 initialized..."));
  // Verify MPU9250 embedded magnetometer for sanity.
  int8_t imu_mag_whoami = imu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  Serial.print(F("AK8963 reported WHO_AM_I 0x")); Serial.print(imu_mag_whoami, HEX);
  if (imu_mag_whoami == 0x48) {
    Serial.println(F("AK8963 is online..."));
  } else {
    raise_error("AK8963 error. Expected WHO_AM_I 0x48.");
  }  
  imu.initAK8963(imu.factoryMagCalibration);
  Serial.println(F("AK8963 initialized..."));
  // Display calibration values
#ifdef IMU_SERIAL
  Serial.print(F("X-Axis factory sensitivity adjustment value "));
  Serial.println(imu.factoryMagCalibration[0], 2);
  Serial.print(F("Y-Axis factory sensitivity adjustment value "));
  Serial.println(imu.factoryMagCalibration[1], 2);
  Serial.print(F("Z-Axis factory sensitivity adjustment value "));
  Serial.println(imu.factoryMagCalibration[2], 2);
#endif
  // Obtain sensor resolutions
  imu.getAres();
  imu.getGres();
  imu.getMres();
  // More values
#ifdef IMU_SERIAL
  Serial.println(F("AK8963 mag biases (mG)"));
  Serial.println(imu.magBias[0]);
  Serial.println(imu.magBias[1]);
  Serial.println(imu.magBias[2]);
  Serial.println(F("AK8963 mag scale (mG)"));
  Serial.println(imu.magScale[0]);
  Serial.println(imu.magScale[1]);
  Serial.println(imu.magScale[2]);
#endif
  // Finished IMU setup. Signal with LEDs.
  Serial.println(F("Finished configuring IMU."));
  blink_led(50, 5);

  /**
   * Motors (Drivers + Encoders)
   */
  motor1.begin(PWM1, INA1, INB1);
  motor2.begin(PWM2, INA2, INB2);
  motor3.begin(PWM3, INA3, INB3);
  Serial.println(F("Motor drivers declared."));
  enc1.write(0);
  enc2.write(0);
  enc3.write(0);
  Serial.println(F("Encoders homed."));
  Serial.println(F("Finished configuring motors."));
  blink_led(50, 1);

  display.fillRect(24, 24, 104, 8, 1);
  display.setTextColor(BLACK);
  display.setCursor(28, 24);
  display.println("Finished config!");
  display.display();
  delay(500);
}


void loop() {

  if(imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    imu.readAccelData(imu.accelCount);
    imu.ax = (float) imu.accelCount[0] * imu.aRes;
    imu.ay = (float) imu.accelCount[1] * imu.aRes;
    imu.az = (float) imu.accelCount[2] * imu.aRes;
    imu.readGyroData(imu.gyroCount);
    imu.gx = (float) imu.gyroCount[0] * imu.gRes;
    imu.gy = (float) imu.gyroCount[1] * imu.gRes;
    imu.gz = (float) imu.gyroCount[2] * imu.gRes;
    imu.readMagData(imu.magCount);
    imu.mx = (float) imu.magCount[0] * imu.mRes * imu.factoryMagCalibration[0] - imu.magBias[0];
    imu.my = (float) imu.magCount[1] * imu.mRes * imu.factoryMagCalibration[1] - imu.magBias[1];
    imu.mz = (float) imu.magCount[2] * imu.mRes * imu.factoryMagCalibration[2] - imu.magBias[2];
  }

  imu.updateTime();

  MadgwickQuaternionUpdate(imu.ax, imu.ay, imu.az, imu.gx * DEG_TO_RAD, imu.gy * DEG_TO_RAD, imu.gz * DEG_TO_RAD, imu.my, imu.mx, imu.mz, imu.deltat);

  imu.delt_t = millis() - imu.count;
  if (imu.delt_t > 500) {
    imu.yaw = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
    imu.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                      * *(getQ()+2)));
    imu.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                      * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                      * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                      * *(getQ()+3));
    
    imu.pitch *= RAD_TO_DEG;
    imu.yaw *= RAD_TO_DEG;
    imu.yaw -= MAGNETIC_DECLINATION;
    imu.roll *= RAD_TO_DEG;
#ifdef IMU_SERIAL
    Serial.print("q0 = ");  Serial.print(*getQ());
    Serial.print(" qx = "); Serial.print(*(getQ() + 1));
    Serial.print(" qy = "); Serial.print(*(getQ() + 2));
    Serial.print(" qz = "); Serial.println(*(getQ() + 3));
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(imu.yaw, 2);
    Serial.print(", ");
    Serial.print(imu.pitch, 2);
    Serial.print(", ");
    Serial.println(imu.roll, 2);
    Serial.print("rate = ");
    Serial.print((float)imu.sumCount / imu.sum, 2);
    Serial.println(" Hz");
#endif

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextColor(WHITE);
    display.println("IMU values:");
    display.print("Roll: "); display.println(imu.roll, 2);
    display.print("Pitch: "); display.println(imu.pitch, 2);
    display.print("Yaw: "); display.println(imu.yaw, 2);
    display.display();

    digitalWriteFast(LEDG1, !digitalReadFast(LEDG1));

    imu.count = millis();
    imu.sumCount = 0;
    imu.sum = 0;
  }
}


// motor1.setSpeed(drive_speed);
// motor2.setSpeed(drive_speed);
// motor3.setSpeed(drive_speed);
// delay(1000);
// motor1.setSpeed(0);
// motor2.setSpeed(0);
// motor3.setSpeed(0);
// delay(100);

// motor1.setSpeed(-1*drive_speed);
// motor2.setSpeed(-1*drive_speed);
// motor3.setSpeed(-1*drive_speed);
// delay(1000);
// motor1.setSpeed(0);
// motor2.setSpeed(0);
// motor3.setSpeed(0);
// delay(100);

// Serial.print("Enc1: "); Serial.print(enc1.read()); Serial.print("\t");
// Serial.print("Enc2: "); Serial.print(enc2.read()); Serial.print("\t");
// Serial.print("Enc3: "); Serial.println(enc3.read());