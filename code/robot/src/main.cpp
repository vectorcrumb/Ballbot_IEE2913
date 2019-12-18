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
#define MAGNETIC_DECLINATION 1.33
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

// IMU, motors, encoders and OLED objects
MPU9250 imu(MPU9250_ADDRESS, Wire2, 400000);
TorqueMotor motor1(PWM1, INA1, INB1, CS1);
TorqueMotor motor2(PWM2, INA2, INB2, CS2);
TorqueMotor motor3(PWM3, INA3, INB3, CS3);
Encoder enc1(ENC1A, ENC1B);
Encoder enc2(ENC2A, ENC2B);
Encoder enc3(ENC3A, ENC3B);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
ADC * adc = new ADC();
MovingAverageFilter omegaFilter1, omegaFilter2, omegaFilter3;


STATE_DATA deltax;
STATE_DATA x0;
STATE_DATA K;
TORQUES T_virtual;
TORQUES T_real;
TORQUES u0;
VOLTAGES V_torque;
VOLTAGES V_PWM;
volatile ANGLES omniangles;
ANGLES IMUangles;
MATRIX M_torques;
MATRIX M_od_IMUangles;
MATRIX M_od_omniangles;

int drive_speed = 50;
uint32_t start_time = 0;
uint32_t update_web = 0;
bool Thetaz_firstrun = 1;

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


void quaternionToDegrees();
void updateInterruptIMU();
void enc_update();
void motor_update();

void setup() {

  Eigen::MatrixXf Pp(2,2);

  Pp << 1, 2,
        3, 4;

  /**
   * General Configuration
   */
  // Open serial channel at 115.2 kbps
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println(F("Brought up serial interfaces. Initializing..."));
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
  int8_t imu_whoami = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 reported WHO_AM_I 0x")); Serial.println(imu_whoami, HEX);
  if (imu_whoami == 0x71) {
    Serial.println(F("MPU9250 is online..."));
    // Start by performing self test and reporting values
    imu.MPU9250SelfTest(imu.selfTest);
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
  // Obtain sensor resolutions
  imu.getAres();
  imu.getGres();
  imu.getMres();
#ifdef MAG_CAL
  imu.magCalMPU9250(imu.magBias,imu.magScale);
#else
  imu.magBias[0] = MAG_BIAS1;
  imu.magBias[1] = MAG_BIAS2;
  imu.magBias[2] = MAG_BIAS3;
  imu.magScale[0] = MAG_SCALE1;
  imu.magScale[1] = MAG_SCALE2;
  imu.magScale[2] = MAG_SCALE3;
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

  while (convergence_iterations < 150){
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
      imu.updateTime();
      MahonyQuaternionUpdate(imu.ax, imu.ay, imu.az, imu.gx * DEG_TO_RAD, imu.gy * DEG_TO_RAD, imu.gz * DEG_TO_RAD, imu.my, imu.mx, -1*imu.mz, imu.deltat);
      convergence_iterations++;
    }
    delay(5);
  }
  quaternionToDegrees();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(WHITE);
  display.println("Iterated 50 readings!");
  display.print("YAW: "); display.println(imu.yaw);
  display.print("PITCH: "); display.println(imu.pitch);
  display.print("ROLL: "); display.println(imu.roll);
  display.display();
  delay(100);

  deltax.Thetax_offset=imu.pitch;
  deltax.Thetay_offset=imu.roll;
  deltax.Thetaz_offset=imu.yaw;


  // motor1._setMotorSpeed(1);
  // delay(500);
  // motor1._setMotorSpeed(0);
  // float m1_torque_offset = 0;
  // for (int i = 0; i <= 50; i++) {
  //   m1_torque_offset += motor1.getTorque() / 50;
  //   Serial.print("ZPC1: "); Serial.println(m1_torque_offset);
  // }
  // motor1.zeroPointCurrent = m1_torque_offset;

  // motor2._setMotorSpeed(0.5);
  // delay(500);
  // motor2._setMotorSpeed(0);
  // float m2_torque_offset = 0;
  // for (int i = 0; i <= 50; i++) {
  //   m2_torque_offset += motor2.getTorque() / 100;
  // }
  // motor2.zeroPointCurrent = m2_torque_offset;

  // motor3._setMotorSpeed(0.5);
  // delay(500);
  // float m3_torque_offset = 0;
  // motor3._setMotorSpeed(0);
  // for (int i = 0; i <= 50; i++) {
  //   m3_torque_offset += motor3.getTorque() / 100;
  // }
  // motor3.zeroPointCurrent = m3_torque_offset;
  




  // He aqui un delay.
  // delay(1000);
  // Calibrate sensor offset
  // float m1_adc_offset = 0;
  // float m2_adc_offset = 0;
  // float m3_adc_offset = 0;
  // for (int i = 0; i <= 100; i++) {
  //   m1_adc_offset = motor1.getTorque();
  //   Serial.print("ZPC1: "); Serial.println(m1_adc_offset);
  //   m2_adc_offset += motor2.getTorque() / 100.0;
  //   m3_adc_offset += motor3.getTorque() / 100.0;
  //   delay(2);
  // }
  // motor1.zeroPointCurrent = m1_adc_offset;
  // motor2.zeroPointCurrent = m2_adc_offset;
  // motor3.zeroPointCurrent = m3_adc_offset;

  // motor2.zeroPointCurrent = 0.11;
  // motor3.zeroPointCurrent = 0.18;

  TeensyDelay::begin();
  TeensyDelay::addDelayChannel(enc_update, TIMER_CHANNEL_ENC);
  TeensyDelay::trigger(TIMER_DELAY_ENC, TIMER_CHANNEL_ENC);
  TeensyDelay::addDelayChannel(motor_update, TIMER_CHANNEL_MOTORS);
  TeensyDelay::trigger(TIMER_DELAY_MOTOR, TIMER_CHANNEL_MOTORS);

  start_time = micros();
  update_web = millis();

}

uint32_t deltat = 0;

bool mot3_sign = false;

void loop() {

  updateInterruptIMU();
  quaternionToDegrees();

  start_time = micros();

  //Código principal
  
  //Esto probablemente deba ir en una interrupción cuando la RasPi manda que se actualice el estado
  //get_opPoint(&M_od_omniangles, &M_od_IMUangles, &M_torques, &K, &x0, &u0, 1);
  
  //-------->Funciones para leer angulos IMU, encoders, que entregan structs para omniangulos y angulos encoder
  //read_IMU(&IMUangles, imu, deltat);
  
  //Arma estado deltax
  //get_phi(&deltax, &x0, &M_od_omniangles, &M_od_IMUangles, &omniangles, &IMUangles, deltat);
  //get_theta(&deltax, &IMUangles, &x0);

  //Se obtiene T_virtual=u0+deltau=-kdeltax+u0
  //control_signal(&T_virtual, &K, &u0, &deltax);
  //torque_conversion(&M_torques, &T_real, &T_virtual);

  motor1.setTorque(1.5);
  motor2.setTorque(1.0);
  motor3.setTorque(0.5);

  Serial.print("$"); Serial.print(motor1.getTorque());
  Serial.print(" "); Serial.print(motor2.getTorque());
  Serial.print(" "); Serial.print(motor3.getTorque());
  Serial.println(";");


  if (millis() - update_web > 2000) {
    update_web = millis();

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextColor(WHITE);

    display.print("t1: "); display.print(motor1.getTorque(), 2); display.print("|e1: "); display.println(motor1.getError(), 2);
    display.print("t2: "); display.print(motor2.getTorque(), 2); display.print("|e2: "); display.println(motor2.getError(), 2);
    display.print("t3: "); display.print(motor3.getTorque(), 2); display.print("|e3: "); display.println(motor3.getError(), 2);
    display.setCursor(0, 24);
    display.print("dt:"); display.println(deltat);
    display.display();
  }

  deltat = micros() - start_time;
}


void enc_update() {
  TeensyDelay::trigger(TIMER_DELAY_ENC, TIMER_CHANNEL_ENC);
  read_enc(&omniangles, enc1.read(), enc2.read(), enc3.read());
  omniangles.dw1 = omegaFilter1.updateFilter(omniangles.dw1);
  omniangles.dw1 = omegaFilter2.updateFilter(omniangles.dw2);
  omniangles.dw3 = omegaFilter3.updateFilter(omniangles.dw3);
  
}

void motor_update() {
  TeensyDelay::trigger(TIMER_DELAY_MOTOR, TIMER_CHANNEL_MOTORS);
  motor1.updateMotor(adc->analogRead(motor1.csPin));
  motor2.updateMotor(adc->analogRead(motor2.csPin));
  motor3.updateMotor(adc->analogRead(motor3.csPin));
}


void updateInterruptIMU() {
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

  //MadgwickQuaternionUpdate(imu.ax, imu.ay, imu.az, imu.gx * DEG_TO_RAD, imu.gy * DEG_TO_RAD, imu.gz * DEG_TO_RAD, imu.my, imu.mx, -1*imu.mz, imu.deltat);
  // MahonyQuaternionUpdate(-1*imu.ax, imu.ay, imu.az, imu.gx * DEG_TO_RAD, -1*imu.gy * DEG_TO_RAD, -1*imu.gz * DEG_TO_RAD, imu.my, -1*imu.mx, imu.mz, imu.deltat);
  MahonyQuaternionUpdate(-1*imu.ax, imu.ay, imu.az, imu.gx * DEG_TO_RAD, -1*imu.gy * DEG_TO_RAD, -1*imu.gz * DEG_TO_RAD, imu.my, -1*imu.mx, imu.mz, imu.deltat);
}


void quaternionToDegrees() {
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
  imu.yaw -= MAGNETIC_DECLINATION;
}