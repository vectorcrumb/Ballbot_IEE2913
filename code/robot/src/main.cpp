#include "Arduino.h"

// #define IMU_SERIAL
// #define MAG_CAL

// IMU definitions
#define MAG_BIAS1 257.82
#define MAG_BIAS2 274.11
#define MAG_BIAS3 634.31
#define MAG_SCALE1 0.79
#define MAG_SCALE2 1.17
#define MAG_SCALE3 1.13
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0
#define MAGNETIC_DECLINATION 1.33
#define IMU_INT 24
// I/O Pins
#define LED_BOARD 13
#define LEDR1 2
#define LEDG1 5
#define LEDR2 6
// Motor 1 Encoder and H-Bridge pins
#define ENC1A 15
#define ENC1B 14
#define PWM1 23
#define INB1 22
#define INA1 21
#define CS1 33
// Motor 3 Encoder and H-Bridge pins
#define ENC2A 39
#define ENC2B 38
#define PWM2 20
#define INB2 19
#define INA2 18
#define CS2 34
// Motor 3 Encoder and H-Bridge pins
#define ENC3A 37
#define ENC3B 36
#define PWM3 29
#define INB3 28
#define INA3 27
#define CS3 35
// I2C Definitions
#define SDA_ALT 17
#define SCL_ALT 16
// Other program definitions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define IMU_CALIBRATIONS 100
#define MOTOR_CONTROLLER_REFRESH_RATE 0.020
#define MOTOR_PID_TIMER_CHANNEL 0
#define MOTOR_KT 0.7273
#define SERIAL_SPEED 115200
#define ENCODER_OPTIMIZE_INTERRUPTS
// Library imports
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TorqueMotor.h>
#include <Encoder.h>
#include <quaternionFilters.h>
#include <MPU9250.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TeensyDelay.h>
// User libraries
#include "logo.h"
#include "control.h"
#include "state.h"
#include "transforms.h"
#include "dataStructs.h"

// Function prototypes
void motorPIDCallback();
void ready_display();
void blink_led(uint16_t period, uint8_t repeat);
void raise_error(const char* error_message);
void updateInterruptIMU();
void quaternionToDegrees();

// Object definitions
MPU9250 imu(MPU9250_ADDRESS, Wire2, 400000);
TorqueMotor motor1(PWM1, INA1, INB1, CS1, MOTOR_KT);
TorqueMotor motor2(PWM2, INA2, INB2, CS2, MOTOR_KT);
TorqueMotor motor3(PWM3, INA3, INB3, CS3, MOTOR_KT);
Encoder enc1(ENC1A, ENC1B);
Encoder enc2(ENC2A, ENC2B);
Encoder enc3(ENC3A, ENC3B);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

// Structs for control
State deltax;
State x0;
State K;
Torque T_virtual;
Torque T_real;
Torque u0;
MotorSignal V_torque;
MotorSignal V_PWM;
AngleState omniangles;
AngleState IMUangles;
Mat33 M_torques;
Mat33 M_od_IMUangles;
Mat33 M_od_omniangles;


// State variables
uint64_t loop_time_marker = millis();
uint32_t dt = millis();
unsigned long update_web = millis();
bool imu_calibrating = true;
uint16_t imu_calibration_count = 0;


void setup() {
  // Open serial channels at 115.2 kbps
  Serial.begin(SERIAL_SPEED);
  Serial1.begin(SERIAL_SPEED);
  Serial.println(F("Brought up serial interface. Initializing robot..."));
  // Initialize I2C_0 for OLED and I2C_2 for IMU
  Wire.setSDA(SDA_ALT);
  Wire.setSCL(SCL_ALT);
  Wire.begin();
  Wire2.begin();
  // Configure LEDS
  pinMode(LED_BOARD, OUTPUT);
  pinMode(LEDG1, OUTPUT);
  pinMode(LEDR1, OUTPUT);
  pinMode(LEDR2, OUTPUT); 
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
  Serial.print(F("AK8963 reported WHO_AM_I 0x")); Serial.println(imu_mag_whoami, HEX);
  if (imu_mag_whoami == 0x48) {
    Serial.println(F("AK8963 is online..."));
  } else {
    raise_error("AK8963 error. Expected WHO_AM_I 0x48.");
  }  
  imu.initAK8963(imu.factoryMagCalibration);
  Serial.println(F("AK8963 initialized..."));
  // Obtain sensor resolutions
  imu.getAres();
  imu.getGres();
  imu.getMres();
  // Activating the MAG_CAL flag will issue a calibration of the magnetometer
#ifdef MAG_CAL
  imu.magCalMPU9250(imu.magBias,imu.magScale);
  Serial.println(F("AK8963 mag biases (mG)"));
  Serial.println(imu.magBias[0]);
  Serial.println(imu.magBias[1]);
  Serial.println(imu.magBias[2]);
  Serial.println(F("AK8963 mag scale (mG)"));
  Serial.println(imu.magScale[0]);
  Serial.println(imu.magScale[1]);
  Serial.println(imu.magScale[2]);
#else
  imu.magBias[0] = MAG_BIAS1;
  imu.magBias[1] = MAG_BIAS2;
  imu.magBias[2] = MAG_BIAS3;
  imu.magScale[0] = MAG_SCALE1;
  imu.magScale[1] = MAG_SCALE2;
  imu.magScale[2] = MAG_SCALE3;
#endif
  Serial.println(F("Configuring IMU interrupt"));
  // Configure IMU INT pin and setup interrupt function
  pinMode(IMU_INT, INPUT);
  attachInterrupt(IMU_INT, updateInterruptIMU, RISING);
  // Wait for IMU to finish calibrating and convert quaternion
  Serial.println(F("Calibrating IMU!"));
  while (imu_calibrating) {
    delay(2);
  }
  quaternionToDegrees();
  // Finished IMU setup. Signal with LEDs.
  Serial.println(F("Finished configuring IMU."));
  blink_led(50, 1);

  /**
   * Motors (Drivers + Encoders)
   */
  motor1.begin();
  motor2.begin();
  motor3.begin();
  Serial.println(F("Motor drivers configured."));
  enc1.write(0);
  enc2.write(0);
  enc3.write(0);
  Serial.println(F("Encoders homed."));
  // Enable interrupts for updating motor PID controllers
  // TeensyDelay::begin();
  // TeensyDelay::addDelayChannel(motorPIDCallback, MOTOR_PID_TIMER_CHANNEL);
  // TeensyDelay::trigger(MOTOR_CONTROLLER_REFRESH_RATE, MOTOR_PID_TIMER_CHANNEL);
  Serial.println(F("Finished configuring motors."));
  blink_led(50, 1);

  display.fillRect(24, 24, 104, 8, 1);
  display.setTextColor(BLACK);
  display.setCursor(28, 24);
  display.println("Finished config!");
  display.display();
  delay(500);

  set_theta_offset(&deltax, imu.pitch, imu.roll, imu.yaw);
  ready_display();
  display.print(IMU_CALIBRATIONS); display.println(" IMU readings.");
  display.print("YAW: "); display.println(imu.yaw * RAD_TO_DEG);
  display.print("PITCH: "); display.println(imu.pitch * RAD_TO_DEG);
  display.print("ROLL: "); display.println(imu.roll * RAD_TO_DEG);
  display.display();
  delay(2000);

}


void loop() {
  // Begin by calculating time delta for derivatives
  loop_time_marker = millis();
  // Convert the IMU quaternion state to radians to prepare for use
  quaternionToDegrees();  
  // Get operation point parameters for current operation point
  get_opPoint(&M_od_omniangles, &M_od_IMUangles, &M_torques, &K, &x0, &u0, &IMUangles, &omniangles, 1);
  // Read IMU and encoders and differentiate them. The dt parameter is used for numerical differentiation
  read_IMU(&IMUangles, imu.pitch, imu.roll, imu.yaw, dt);
  read_enc(&omniangles, enc1.read(), enc2.read(), enc3.read(), dt);
  // Convert measurements into state variables  
  get_phi(&deltax, &x0, &M_od_omniangles, &M_od_IMUangles, &omniangles, &IMUangles, dt);
  get_theta(&deltax, &IMUangles, &x0);
  // Create control signal (virtual torques) with LQR controller, state and operating point.
  // T_virtual = u0 + delta_u = u0 + (-K * delta_x)
  // Then, convert virtual to real torques
  control_signal(&T_virtual, &K, &u0, &deltax);
  torque_conversion(&M_torques, &T_real, &T_virtual);
  // Feed real torques to PID controllers for motors. A timer updates the controller, this only sets 
  // the torque setpoint. TO-DO Implement a mutex when updating/reading reference to avoid race conditions
  motor1.setTorque(T_real.Tx1);
  motor2.setTorque(T_real.Ty2);
  motor3.setTorque(T_real.Tz3);
  // Update PID controllers
  motor1.updateMotor(dt);
  motor2.updateMotor(dt);
  motor3.updateMotor(dt);
  // Display time delta on screen
  ready_display();
  display.print("M1:"); display.print(motor1.output);
  display.print(";M2:"); display.print(motor2.output);
  display.print(";M3:"); display.println(motor3.output);

  display.print("E1:"); display.print(enc1.read());
  display.print(";E2:"); display.print(enc2.read());
  display.print(";E3:"); display.println(enc3.read());

  display.print("dt:"); display.println(dt); 
  display.display();
  // Update user interface. TO-DO convert to PJON protocol
  if (millis() - update_web > 1000) {
    Serial1.print("U");
    Serial1.print(imu.pitch,2);
    Serial1.print(",");
    Serial1.print(imu.roll,2);
    Serial1.print(",");
    Serial1.print(imu.yaw,2);
    Serial1.print(",");
    Serial1.print(T_real.Tx1,2);
    Serial1.print(",");
    Serial1.print(T_real.Ty2,2);
    Serial1.print(",");
    Serial1.print(T_real.Tz3,2);
    Serial1.print(",");
    Serial1.print(T_virtual.Tx1,2);
    Serial1.print(",");
    Serial1.print(T_virtual.Ty2,2);
    Serial1.print(",");
    Serial1.print(T_virtual.Tz3,2);
    Serial1.println("");
    update_web = millis();
  }

  dt = millis() - loop_time_marker;
}



/**
 * Callback for updating motor PID controllers
 */
void motorPIDCallback() {
  // TeensyDelay::trigger(MOTOR_CONTROLLER_REFRESH_RATE, MOTOR_PID_TIMER_CHANNEL);
  // Naively set PID delta_t to the timer callback rate
  motor1.updateMotor(MOTOR_CONTROLLER_REFRESH_RATE);
  motor2.updateMotor(MOTOR_CONTROLLER_REFRESH_RATE);
  motor3.updateMotor(MOTOR_CONTROLLER_REFRESH_RATE);
}

/**
 * Clears display, homes cursor and set text to white
 */
void ready_display() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(WHITE);
}

/**
 * Blinks onboard LED repeat times for period ms each time
 * An iteration takes 2*period ms in total. Total time is 2*period*repeat ms
 */
void blink_led(uint16_t period, uint8_t repeat) {
  for (uint8_t i = 0; i < repeat; i++) {
    digitalWrite(LED_BOARD, HIGH);
    delay(period);
    digitalWrite(LED_BOARD, LOW);
    delay(period);
  }
}

/**
 * Prints an error message to Serial and catches code in 
 * infinite loop while blinking red LEDS every 500 ms
 */
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


void updateInterruptIMU() {
  if (imu_calibrating) {
    imu_calibrating = ++imu_calibration_count >= IMU_CALIBRATIONS ? false : true;
  }
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
  
  imu.updateTime();
  MahonyQuaternionUpdate(-1*imu.ax, imu.ay, imu.az, imu.gx * DEG_TO_RAD, -1*imu.gy * DEG_TO_RAD, -1*imu.gz * DEG_TO_RAD, imu.my, -1*imu.mx, imu.mz, imu.deltat);
}


void quaternionToDegrees() {
  // Extracted from https://github.com/kriswiner/MPU9250/blob/master/MPU9250_MS5637_AHRS_t3.ino#L607
  float a12 = 2.0f * (q2() * q3() + q1() * q4());
  float a22 = q1() * q1() + q2() * q2() - q3() * q3() - q4() * q4();
  float a31 = 2.0f * (q1() * q2() + q3() * q4());
  float a32 = 2.0f * (q2() * q4() - q1() * q3());
  float a33 = q1() * q1() - q2() * q2() - q3() * q3() + q4() * q4();
  imu.yaw = atan2f(a12, a22);
  imu.pitch = -asinf(a32);
  imu.roll = atan2f(a31, a33);
  imu.yaw += MAGNETIC_DECLINATION * DEG_TO_RAD;
}

