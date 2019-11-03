#include "Arduino.h"


// #define IMU_SERIAL
// #define MAG_CAL

#define MAG_BIAS1 257.82
#define MAG_BIAS2 274.11
#define MAG_BIAS3 634.31
#define MAG_SCALE1 0.79
#define MAG_SCALE2 1.17
#define MAG_SCALE3 1.13

#define LED_BOARD 13
#define LEDR1 2
#define LEDG1 5
#define LEDR2 6

#define ENC1A 15
#define ENC1B 14
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
#define PWM3 35
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
#include "SPI.h"
#include "SD.h"
#include "control.h"
#include "state.h"
#include "transforms.h"

#include "dataStructs.h"

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

STATE_DATA deltax;
STATE_DATA x0;
STATE_DATA K;
TORQUES T_virtual;
TORQUES T_real;
TORQUES u0;
VOLTAGES V_torque;
VOLTAGES V_PWM;
ANGLES omniangles;
ANGLES IMUangles;
MATRIX M_torques;
MATRIX M_od_IMUangles;
MATRIX M_od_omniangles;

int drive_speed = 50;
unsigned long long start_time = micros();
unsigned long update_web = millis();
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

void setup() {
  digitalWrite(PWM1, LOW);
  digitalWrite(INB1, LOW);
  digitalWrite(INA1, LOW);
  digitalWrite(PWM2, LOW);
  digitalWrite(INB2, LOW);
  digitalWrite(INA2, LOW);
  digitalWrite(PWM3, LOW);
  digitalWrite(INB3, LOW);
  digitalWrite(INA3, LOW);
  // Open serial channel at 115.2 kbps
  Serial.begin(115200);
  Serial1.begin(115200);
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
  blink_led(50, 1);

  /**
   * Motors (Drivers + Encoders)
   */
  motor1.begin(PWM1, INA1, INB1, 1);
  motor2.begin(PWM2, INA2, INB2, 1);
  motor3.begin(PWM3, INA3, INB3, 1);
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
  delay(1000);

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

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(WHITE);
  display.println("Iterated 50 readings!");
  display.print("YAW: "); display.println(imu.yaw);
  display.print("PITCH: "); display.println(imu.pitch);
  display.print("ROLL: "); display.println(imu.roll);
  display.display();
  delay(2000);

  deltax.Thetax_offset=imu.pitch;
  deltax.Thetay_offset=imu.roll;
  deltax.Thetaz_offset=imu.yaw;

}


void loop() {

  updateIMU();
  quaternionToDegrees();

  //Código principal
  float deltat = (micros() - start_time);
  
  //Esto probablemente deba ir en una interrupción cuando la RasPi manda que se actualice el estado
  get_opPoint(&M_od_omniangles, &M_od_IMUangles, &M_torques, &K, &x0, &u0, 1);
  
  //-------->Funciones para leer angulos IMU, encoders, que entregan structs para omniangulos y angulos encoder
  read_IMU(&IMUangles, imu, deltat);
  read_enc(&omniangles, enc1, enc2, enc3, deltat);
  
  //Arma estado deltax
  get_phi(&deltax, &x0, &M_od_omniangles, &M_od_IMUangles, &omniangles, &IMUangles, deltat);
  get_theta(&deltax, &IMUangles, &x0, Thetaz_firstrun);

  if (Thetaz_firstrun){
    Thetaz_firstrun =0;
  }

  //Se obtiene T_virtual=u0+deltau=-kdeltax+u0
  control_signal(&T_virtual, &K, &u0, &deltax);
  torque_conversion(&M_torques, &T_real, &T_virtual);

  //Voltaje de motores, conversión a PWM
  //--------->Falta función para leer V_battery. Reemplazar 12 por V_battery cuando se lea
  voltage_motors(&V_torque, &T_real, &omniangles, deltat);
  voltage_pwm(&V_torque,  &V_PWM, 12.4);

  start_time = micros();

  float PWM_1 = 255*(V_PWM.V1);
  float PWM_2 = 255*(V_PWM.V2);
  float PWM_3 = 255*(V_PWM.V3);

  motor1.setSpeed(PWM_1, 1);
  motor2.setSpeed(PWM_2, 1);
  motor3.setSpeed(PWM_3, 1);

  // Serial.println(imu.roll);
  // Serial.println("  ");

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(WHITE);
  // display.print("Thetax: "); display.println(deltax.thetax); 
  // display.print("Thetay: "); display.println(deltax.thetay); 
  // display.print("Thetaz: "); display.println(deltax.thetaz); 
  display.print("TV:"); display.print(T_virtual.Tx1, 2);
  display.print("|"); display.print(T_virtual.Ty2, 2);
  display.print("|"); display.println(T_virtual.Tz3, 2);

  display.print("V:"); display.print(V_torque.V1, 2);
  display.print("|"); display.print(V_torque.V2, 2);
  display.print("|"); display.print(V_torque.V3, 2);

  display.setCursor(70, 24);
  display.print("dt:"); display.println(deltat*0.001,2); 
  display.display();
  
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
}


void updateIMU() {
  // TO-DO: Convert this check to a INT pin check
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
  // MadgwickQuaternionUpdate(imu.ax, imu.ay, imu.az, imu.gx * DEG_TO_RAD, imu.gy * DEG_TO_RAD, imu.gz * DEG_TO_RAD, imu.my, imu.mx, -1*imu.mz, imu.deltat);
  // MahonyQuaternionUpdate(-1*imu.ax, imu.ay, imu.az, imu.gx * DEG_TO_RAD, -1*imu.gy * DEG_TO_RAD, -1*imu.gz * DEG_TO_RAD, imu.my, -1*imu.mx, imu.mz, imu.deltat);
  MahonyQuaternionUpdate(-1*imu.ax, imu.ay, imu.az, imu.gx * DEG_TO_RAD, -1*imu.gy * DEG_TO_RAD, -1*imu.gz * DEG_TO_RAD, imu.my, -1*imu.mx, imu.mz, imu.deltat);
}

void quaternionToDegrees() {
  // Extracted from https://github.com/kriswiner/MPU9250/blob/master/MPU9250_MS5637_AHRS_t3.ino#L607
  float a12 = 2.0f * (q1() * q2() + q0() * q3());
  float a22 = q1() * q1() + q2() * q2() - q3() * q3() - q4() * q4();
  float a31 = 2.0f * (q1() * q2() + q3() * q4());
  float a32 = 2.0f * (q2() * q4() - q1() * q3());
  float a33 = q1() * q1() - q2() * q2() - q3() * q3() + q4() * q4();
  imu.yaw = atan2f(a12, a22);
  imu.pitch = -asinf(a32);
  imu.roll = atan2f(a31, a33);

  imu.yaw += MAGNETIC_DECLINATION * DEG_TO_RAD;
}