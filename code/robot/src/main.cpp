/**
 * Escrito por Lucas Alvarez y el Pelma Garrido
 */

#include "Arduino.h"

// #define IMU_SERIAL
// #define MAG_CAL

// LED pins
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
// Pins for IMU and OLED
#define IMU_INT 24
#define SDA_ALT 17
#define SCL_ALT 16

#define ENCODER_OPTIMIZE_INTERRUPTS

#include <Wire.h>
#include <VNHDriver.h>
#include <Encoder.h>
#include <quaternionFilters.h>
#include <TorqueMotor.h>
#include <MPU9250.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TeensyDelay.h>
#include <ADC.h>
#include <Eigen.h>
#include <Kalman.hpp>

#include "logo.h"
#include "SPI.h"
#include "SD.h"
#include "control.h"
#include "state.h"
#include "transforms.h"
#include "dataStructs.h"
#include "movavg.h"

// Magnetometer bias values
#define MAG_BIAS1 257.82
#define MAG_BIAS2 274.11
#define MAG_BIAS3 634.31
#define MAG_SCALE1 0.79
#define MAG_SCALE2 1.17
#define MAG_SCALE3 1.13
// IMU Constants
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0
#define MAGNETIC_DECLINATION 1.1
// OLED Constants
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
// Timer Constants
#define TIMER_CHANNEL_ENC 0
#define TIMER_CHANNEL_MOTORS 1
#define TIMER_DELAY_MOTOR MOTOR_PID_DT_US
#define TIMER_DELAY_ENC 10000
#define TIMER_DELAY_ENC_HZ 100
#define ADC_CALIBRATION_SAMPLES 100
#define TORQUE_SATURATION_LIMIT 2.8


// IMU, motors, encoders and OLED objects
MPU9250 imu_mpu(MPU9250_ADDRESS, Wire2, 400000);
TorqueMotor motor1(PWM1, INA1, INB1, CS1, true);
TorqueMotor motor2(PWM2, INA2, INB2, CS2, true);
TorqueMotor motor3(PWM3, INA3, INB3, CS3, true);
Encoder enc1(ENC1A, ENC1B);
Encoder enc2(ENC2A, ENC2B);
Encoder enc3(ENC3A, ENC3B);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
ADC * adc = new ADC();
MovingAverageFilter omegaFilter1, omegaFilter2, omegaFilter3;
MovingAverageFilter imuFilter1, imuFilter2, imuFilter3;
KalmanFilter * KFx, * KFy, * KFz;

volatile ANGLES omniangles;
volatile ANGLES IMUangles;

int drive_speed = 50;
uint32_t start_time = 0;
uint32_t update_web = 0;
bool Thetaz_firstrun = 1;
uint32_t deltat = 0;
float imu_pitch_offset = 0, imu_roll_offset = 0, imu_yaw_offset = 0;


void blink_led(int period, int repeat);
void raise_error(const char* error_message);
void quaternionToDegrees();
bool updateInterruptIMU();
void enc_update();
void motor_update();
void saturate_torques(Eigen::Ref<Eigen::Vector3f> t_mat);
void print_mat(const Eigen::MatrixXf& X);

Eigen::Matrix3f MAT_torqueTransform;
Eigen::Matrix3f MAT_odometryEncTransform, MAT_odometryIMUTransform; 
Eigen::Vector4f VEC_stateX, VEC_stateY, VEC_stateZ, VEC_stateX0, VEC_stateY0, VEC_stateZ0;
Eigen::Vector3f VEC_torqueVirtual, VEC_torqueReal, VEC_torque0;
Eigen::Vector4f GAIN_lqrControl, GAIN_lqrYawControl;
Eigen::Vector3f TEMPVEC_phi, TEMPVEC_omega_wheels, TEMPVEC_omega_body;
Eigen::Matrix4f Axy, Cxy, Az, Cz;
Eigen::Matrix<float, 4, 1> Bxy, Bz;
Eigen::DiagonalMatrix<float, 4> Qx, Rx, Px, Qy, Ry, Py, Qz, Rz, Pz;


void setup() {

  MAT_torqueTransform << 0.942809,  0,        -0.3333, 
                        -0.471405, -0.816497, -0.3333, 
                        -0.471405,  0.816497, -0.3333;
  MAT_odometryEncTransform << -0.942809,  0.471405,   0.471405,
                              0,          -0.816497,  0.816497,
                              0.471405,   0.471405,   0.471405;
  MAT_odometryIMUTransform << 1, 0, 0, 
                              0, 1, 0, 
                              0, 0, 1;
  
  TEMPVEC_phi << 0.0, 0.0, 0.0;
  TEMPVEC_omega_wheels << 0.0, 0.0, 0.0;
  TEMPVEC_omega_body << 0.0, 0.0, 0.0;

  VEC_stateX << 0.0, 0.0, 0.0, 0.0;
  VEC_stateY << 0.0, 0.0, 0.0, 0.0;
  VEC_stateZ << 0.0, 0.0, 0.0, 0.0;
  VEC_stateX0 << 0.0, 0.0, 0.0, 0.0;
  VEC_stateY0 << 0.0, 0.0, 0.0, 0.0;
  VEC_stateZ0 << 0.0, 0.0, 0.0, 0.0;

  VEC_torqueVirtual << 0.0, 0.0, 0.0;
  VEC_torqueReal << 0.0, 0.0, 0.0;
  VEC_torque0 << 0.0, 0.0, 0.0;

  // GAIN_lqrControl << 0.0039, 9.8109, 0.1275, 3.4596;
  // GAIN_lqrControl << 0.0052, 61.0197, 0.1217, 5.5012;

  // GAIN_lqrControl << 0.00070257015141, 9.547963918279745, 0.016563046688473, 1.771180854469351;
  // GAIN_lqrYawControl << 0.0, 0.001814119036152, 0.0, 0.605896706265266;

  // GAIN_lqrControl << 0.0039, 4.8109, 0.0575, 2.2596;
  // GAIN_lqrControl << 0.0039, 5.8109, 0.0575, 2.2596;

  // Law from pole placement
  GAIN_lqrControl << 0.030106, 9.972895, 0.370508, 4.359583;

  GAIN_lqrYawControl << 0.5, 0.1, 0.8, 2.6;

  Rx.diagonal() << 1.0077, 1, 70.7356, 140.7356;
  Ry.diagonal() << 1.0, 1.0, 60.0, 180.0;
  Rz.diagonal() << 1.0, 1.0062, 48.6637, 48.6637;

  Qx.diagonal() << 10, 0.01, 0.1, 1;
  Qy.diagonal() << 10, 0.01, 0.1, 1;
  Qz.diagonal() << 10, 0.01, 0.1, 1;

  Px.diagonal() << 0.01, 0.1, 1, 100;
  Py.diagonal() << 0.01, 0.1, 1, 100;
  Pz.diagonal() << 0.01, 0.1, 1, 100;

  Axy << 1.0, -0.00293460478, 0.02, -0.0000195616125,
         0.0, 1.00185581, 0.0, 0.0200123705,
         0.0, -0.293551212, 1.0, -0.00293460478,
         0.0, 0.18563838, 0.0, 1.00185581;
  Bxy <<  -0.00990146,
          0.00141342,
          -0.99021501,
          0.14138581;
  Cxy.setIdentity();
  Az << 1.0, 0.02, 0.0, 1.0;
  Bz << 0.01647834, 1.64783416;
  Cz.setIdentity();

  KFx = new KalmanFilter(0.010, Axy, Bxy, Cxy, Qx, Rx, Px);
  KFy = new KalmanFilter(0.010, Axy, Bxy, Cxy, Qy, Ry, Py);
  KFz = new KalmanFilter(0.010, Az, Bz, Cz, Qz, Rz, Pz);


  /**
   * General Configuration
   */
  // Open serial channel at 115.2 kbps
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println(F("Brought up serial interfaces. Initializing..."));
  // Initialize I2C_2 for IMU
  Wire2.begin();
  pinMode(IMU_INT, LOW);
  digitalWrite(IMU_INT, LOW);
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
  // Configure ADC object 
  adc->setReference(ADC_REFERENCE::REF_3V3, ADC_0);
  adc->setAveraging(32, ADC_0);
  adc->setResolution(12, ADC_0);
  adc->setReference(ADC_REFERENCE::REF_3V3, ADC_1);
  adc->setAveraging(32, ADC_1);
  adc->setResolution(12, ADC_1);

  

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
  int8_t imu_whoami = imu_mpu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 reported WHO_AM_I 0x")); Serial.println(imu_whoami, HEX);
  if (imu_whoami == 0x71) {
    Serial.println(F("MPU9250 is online..."));
    // Start by performing self test and reporting values
    imu_mpu.MPU9250SelfTest(imu_mpu.selfTest);
    // Calibrate gyro and accelerometers, load biases in bias registers
    imu_mpu.calibrateMPU9250(imu_mpu.gyroBias, imu_mpu.accelBias);
  } else {
    raise_error("MPU9250 error. Expected WHO_AM_I 0x71.");
  }
  // Initialize MPU
  imu_mpu.initMPU9250();
  Serial.println(F("MPU9250 initialized..."));
  // Verify MPU9250 embedded magnetometer for sanity.
  int8_t imu_mag_whoami = imu_mpu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  Serial.print(F("AK8963 reported WHO_AM_I 0x")); Serial.print(imu_mag_whoami, HEX);
  if (imu_mag_whoami == 0x48) {
    Serial.println(F("AK8963 is online..."));
  } else {
    raise_error("AK8963 error. Expected WHO_AM_I 0x48.");
  }  
  imu_mpu.initAK8963(imu_mpu.factoryMagCalibration);
  Serial.println(F("AK8963 initialized..."));
  // Obtain sensor resolutions
  imu_mpu.getAres();
  imu_mpu.getGres();
  imu_mpu.getMres();
#ifdef MAG_CAL
  imu_mpu.magCalMPU9250(imu_mpu.magBias,imu_mpu.magScale);
#else
  imu_mpu.magBias[0] = MAG_BIAS1;
  imu_mpu.magBias[1] = MAG_BIAS2;
  imu_mpu.magBias[2] = MAG_BIAS3;
  imu_mpu.magScale[0] = MAG_SCALE1;
  imu_mpu.magScale[1] = MAG_SCALE2;
  imu_mpu.magScale[2] = MAG_SCALE3;
#endif
  // Finished IMU setup. Signal with LEDs.
  Serial.println(F("Finished configuring IMU."));

  /**
   * Encoders configuration
   */
  enc1.write(0);
  enc2.write(0);
  enc3.write(0);
  Serial.println(F("Encoders homed."));

  // Alert end of startup
  display.fillRect(24, 24, 104, 8, 1);
  display.setTextColor(BLACK);
  display.setCursor(28, 24);
  display.println("Finished config!");
  display.display();
  delay(100);

  uint8_t convergence_iterations = 0;

  while (convergence_iterations < 200){
    if(updateInterruptIMU()) convergence_iterations++;
    delay(5);
  }
  quaternionToDegrees();

  imu_pitch_offset = imu_mpu.pitch;
  imu_roll_offset = imu_mpu.roll;
  imu_yaw_offset = imu_mpu.yaw;

  // motor1.zeroPointCurrent = 0.05;
  // motor2.zeroPointCurrent = 0.11;
  // motor3.zeroPointCurrent = 0.18;

  TeensyDelay::begin();
  TeensyDelay::addDelayChannel(enc_update, TIMER_CHANNEL_ENC);
  TeensyDelay::trigger(TIMER_DELAY_ENC, TIMER_CHANNEL_ENC);
  TeensyDelay::addDelayChannel(motor_update, TIMER_CHANNEL_MOTORS);
  TeensyDelay::trigger(TIMER_DELAY_MOTOR, TIMER_CHANNEL_MOTORS);

  start_time = micros();
  update_web = millis();

  KFx->init(millis() / 1000.0, VEC_stateX0);
  KFy->init(millis() / 1000.0, VEC_stateY0);
  KFz->init(millis() / 1000.0, VEC_stateZ0);
}

void loop() {

  start_time = micros();
  
  // Proximamente: State estimation
  // Torque reference system transform
  VEC_torqueReal = MAT_torqueTransform * VEC_torqueVirtual;
  // Saturate outputs
  saturate_torques(VEC_torqueReal);
  // Update torque setpoints. Motors 2 and 3 are flipped due to CW order in model and CCW orden in robot
  motor1.setTorque(VEC_torqueReal(0));
  motor3.setTorque(VEC_torqueReal(1));
  motor2.setTorque(VEC_torqueReal(2));
  
  

  if (millis() - update_web > 100) {
    update_web = millis();

    Serial.println("Virtual torques vector:");
    print_mat(VEC_torqueVirtual);

    // Timestamp
    // Serial.print(millis()); Serial.print(",");

    // Debug virtual torques
    // Serial.print(VEC_torqueVirtual(0)); Serial.print(",");
    // Serial.print(VEC_torqueVirtual(1)); Serial.print(",");
    // Serial.print(VEC_torqueVirtual(2)); Serial.println("");

    // Debug real torques
    // Serial.print(VEC_torqueVirtual(0)); Serial.print(",");
    // Serial.print(VEC_torqueVirtual(1)); Serial.print(",");
    // Serial.print(VEC_torqueVirtual(2)); Serial.println("");

    // Debug measured torques
    // Serial.print(motor1.torque_measured); Serial.print(",");
    // Serial.print(motor2.torque_measured); Serial.print(",");
    // Serial.print(motor3.torque_measured); Serial.println("");

    // Debug torque setpoints
    // Serial.print(motor1.torque_setpoint); Serial.print(",");
    // Serial.print(motor2.torque_setpoint); Serial.print(",");
    // Serial.print(motor3.torque_setpoint); Serial.println("");

    // Debug voltages
    // Serial.print(motor1.output); Serial.print(",");
    // Serial.print(motor2.output); Serial.print(",");
    // Serial.print(motor3.output); Serial.println("");

    // Debug state X
    // Serial.print(VEC_stateX(0) * RAD_TO_DEG); Serial.print(",");
    // Serial.print(VEC_stateX(1) * RAD_TO_DEG); Serial.print(",");
    // Serial.print(VEC_stateX(2) * RAD_TO_DEG); Serial.print(",");
    // Serial.print(VEC_stateX(3) * RAD_TO_DEG); Serial.println("");

    // Debug state Y
    // Serial.print(VEC_stateY(0) * RAD_TO_DEG); Serial.print(",");
    // Serial.print(VEC_stateY(1) * RAD_TO_DEG); Serial.print(",");
    // Serial.print(VEC_stateY(2) * RAD_TO_DEG); Serial.print(",");
    // Serial.print(VEC_stateY(3) * RAD_TO_DEG); Serial.println("");

    // Debug state Z
    // Serial.print(VEC_stateZ(0) * RAD_TO_DEG); Serial.print(",");
    // Serial.print(VEC_stateZ(1) * RAD_TO_DEG); Serial.print(",");
    // Serial.print(VEC_stateZ(2) * RAD_TO_DEG); Serial.print(",");
    // Serial.print(VEC_stateZ(3) * RAD_TO_DEG); Serial.println("");
  }

  deltat = micros() - start_time;
}

void enc_update() {
  // Restart time trigger
  TeensyDelay::trigger(TIMER_DELAY_ENC, TIMER_CHANNEL_ENC);
  // Read encoders and filter
  read_enc(&omniangles, enc1.read(), enc2.read(), enc3.read());
  omniangles.dw1 = omegaFilter1.updateFilter(omniangles.dw1);
  omniangles.dw2 = omegaFilter2.updateFilter(omniangles.dw2);
  omniangles.dw3 = omegaFilter3.updateFilter(omniangles.dw3);
  // Read IMU and filter
  updateInterruptIMU();
  quaternionToDegrees();
  read_IMU(&IMUangles, -1*(imu_mpu.pitch - imu_pitch_offset), -1*(imu_mpu.roll - imu_roll_offset), -1*(imu_mpu.yaw - imu_yaw_offset));
  IMUangles.dw1 = imuFilter1.updateFilter(IMUangles.dw1);
  IMUangles.dw2 = imuFilter2.updateFilter(IMUangles.dw2);
  IMUangles.dw3 = imuFilter3.updateFilter(IMUangles.dw3);
  // State reconstruction. 
  TEMPVEC_omega_wheels << (float) omniangles.dw1, (float) omniangles.dw2, (float) omniangles.dw3;
  TEMPVEC_omega_body << (float) IMUangles.dw1, (float) IMUangles.dw2, (float) IMUangles.dw3;
  TEMPVEC_phi = MAT_odometryEncTransform * TEMPVEC_omega_wheels + MAT_odometryIMUTransform * TEMPVEC_omega_body;
  // Phi reconstruction for X and Y
  VEC_stateX(0) += 0.000001 * deltat * TEMPVEC_phi(0);
  VEC_stateY(0) += 0.000001 * deltat * TEMPVEC_phi(1); 
  VEC_stateZ(0) += 0.000001 * deltat * TEMPVEC_phi(2);
  // dPhi reconstruction
  VEC_stateX(2) = TEMPVEC_phi(0);
  VEC_stateY(2) = TEMPVEC_phi(1);
  VEC_stateZ(2) = TEMPVEC_phi(2);
  // Theta and dTheta reconstruction
  VEC_stateX(1) = IMUangles.w1;
  VEC_stateY(1) = IMUangles.w2;
  VEC_stateZ(1) = IMUangles.w3;
  VEC_stateX(3) = IMUangles.dw1;
  VEC_stateY(3) = IMUangles.dw2;
  VEC_stateZ(3) = IMUangles.dw3;
  // Control Law: LQR
  VEC_torqueVirtual(0) = GAIN_lqrControl.dot((Eigen::Vector4f) KFx->state());
  VEC_torqueVirtual(1) = GAIN_lqrControl.dot((Eigen::Vector4f) KFy->state());
  VEC_torqueVirtual(2) = GAIN_lqrYawControl.dot((Eigen::Vector4f) KFz->state());
  VEC_torqueVirtual(0) *= -1;
  VEC_torqueVirtual(1) *= -1;
  VEC_torqueVirtual(2) *= -1;
  // State estimation
  KFx->update(VEC_stateX, VEC_torqueVirtual(0));
  KFy->update(VEC_stateY, VEC_torqueVirtual(1));
  KFz->update(VEC_stateZ, VEC_torqueVirtual(2));
}

void motor_update() {
  TeensyDelay::trigger(TIMER_DELAY_MOTOR, TIMER_CHANNEL_MOTORS);
  motor1.updateMotor(adc->analogRead(motor1.csPin));
  motor2.updateMotor(adc->analogRead(motor2.csPin));
  motor3.updateMotor(adc->analogRead(motor3.csPin));
}

void saturate_torques(Eigen::Ref<Eigen::Vector3f> t_mat) {
  for (int i = 0; i < 3; i++) {
    if (t_mat(i) > TORQUE_SATURATION_LIMIT) t_mat(i) = TORQUE_SATURATION_LIMIT;
    if (t_mat(i) < -TORQUE_SATURATION_LIMIT) t_mat(i) = -TORQUE_SATURATION_LIMIT;
  }
}

bool updateInterruptIMU() {
  bool success = false;
  if(imu_mpu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    imu_mpu.readAccelData(imu_mpu.accelCount);
    imu_mpu.ax = (float) imu_mpu.accelCount[0] * imu_mpu.aRes;
    imu_mpu.ay = (float) imu_mpu.accelCount[1] * imu_mpu.aRes;
    imu_mpu.az = (float) imu_mpu.accelCount[2] * imu_mpu.aRes;
    imu_mpu.readGyroData(imu_mpu.gyroCount);
    imu_mpu.gx = (float) imu_mpu.gyroCount[0] * imu_mpu.gRes;
    imu_mpu.gy = (float) imu_mpu.gyroCount[1] * imu_mpu.gRes;
    imu_mpu.gz = (float) imu_mpu.gyroCount[2] * imu_mpu.gRes;
    imu_mpu.readMagData(imu_mpu.magCount);
    imu_mpu.mx = (float) imu_mpu.magCount[0] * imu_mpu.mRes * imu_mpu.factoryMagCalibration[0] - imu_mpu.magBias[0];
    imu_mpu.my = (float) imu_mpu.magCount[1] * imu_mpu.mRes * imu_mpu.factoryMagCalibration[1] - imu_mpu.magBias[1];
    imu_mpu.mz = (float) imu_mpu.magCount[2] * imu_mpu.mRes * imu_mpu.factoryMagCalibration[2] - imu_mpu.magBias[2];
    success = true;
  }
  imu_mpu.updateTime();
  MahonyQuaternionUpdate(imu_mpu.ax, imu_mpu.ay, imu_mpu.az, imu_mpu.gx * DEG_TO_RAD, imu_mpu.gy * DEG_TO_RAD, imu_mpu.gz * DEG_TO_RAD, imu_mpu.my, imu_mpu.mx, -1*imu_mpu.mz, imu_mpu.deltat);
  return success;
}

void quaternionToDegrees() {
  imu_mpu.yaw = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3  ));
  imu_mpu.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                      * *(getQ()+2)));
  imu_mpu.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                      * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                      * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                      * *(getQ()+3));
  imu_mpu.yaw -= MAGNETIC_DECLINATION * DEG_TO_RAD;
}

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

void print_mat(const Eigen::MatrixXf& X) {
  int i, j, nrow, ncol;
  nrow = X.rows();
  ncol = X.cols();
  for (i = 0; i < nrow; i++) {
    for (j = 0; j < ncol; j++) {
      Serial.print(X(i, j), 6);
      Serial.print(", ");
    }
    Serial.println();
  }
  Serial.println();
}