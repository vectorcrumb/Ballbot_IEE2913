#include "state.h"


void get_phi(State * deltax, State * x0, 
            Mat33 * M_od_omniangles, Mat33 * M_od_IMUangles, 
            AngleState * omniangles, AngleState * IMUangles, 
            uint32_t deltat) {
    float dPhix = M_od_omniangles->m11 * omniangles->dw1 + M_od_omniangles->m12 * omniangles->dw2 + M_od_omniangles->m13 * omniangles->dw3 
                + M_od_IMUangles->m11 * IMUangles->dw1 + M_od_IMUangles->m12 * IMUangles->dw2 + M_od_IMUangles->m13 * IMUangles->dw3;

    float dPhiy = M_od_omniangles->m21 * omniangles->dw1 + M_od_omniangles->m22 * omniangles->dw2 + M_od_omniangles->m23 * omniangles->dw3 
                + M_od_IMUangles->m21 * IMUangles->dw1 + M_od_IMUangles->m22 * IMUangles->dw2 + M_od_IMUangles->m23 * IMUangles->dw3;

    float Phix = (x0->phix + deltax->phix) + dPhix * (deltat / 1000);
    float Phiy = (x0->phiy + deltax->phiy) + dPhiy * (deltat / 1000);

    deltax->dphix = dPhix - x0->dphix;
    deltax->dphiy = dPhiy - x0->dphiy;
    deltax->phix  = Phix  - x0->phix;
    deltax->phiy  = Phiy  - x0->phiy;
    
    Serial.print(F("AAAAAAAHHHHHHHHH Omni DW1: ")); Serial.println(omniangles->dw1);
    Serial.print(F("AAAAAAAHHHHHHHHH IMU DW1: ")); Serial.println(IMUangles->dw1);
}


void set_theta_offset(State * deltax, float x_offset, float y_offset, float z_offset) {
    deltax->Thetax_offset = z_offset;
    deltax->Thetay_offset = y_offset;
    deltax->Thetaz_offset = z_offset;
}

void get_theta(State * deltax, AngleState * IMUangles, State * x0) {

    deltax->thetax = IMUangles->w1 - x0->thetax - deltax->Thetax_offset;
    deltax->thetay = IMUangles->w2 - x0->thetay - deltax->Thetay_offset;
    deltax->thetaz = IMUangles->w3 - x0->thetaz - deltax->Thetaz_offset;

    deltax->dthetax = IMUangles->dw1 - x0->dthetax;
    deltax->dthetay = IMUangles->dw2 - x0->dthetay;
    deltax->dthetaz = IMUangles->dw3 - x0->dthetaz;
}

/**
 * Obtains a given operation point determined by op_idx. The op-point
 * is written to the provided structs, which determine the operation 
 * point conditions as well as odometry and torque transformations.
 * TO-DO Implement op-point generation by reading from SD card
 * TO-DO Implement early exit if op_idx is unchanged. Maybe convert this to a class ?
 */
void get_opPoint(Mat33 * M_odometry_omniangles, 
                Mat33 * M_odometry_IMUangles, 
                Mat33 * M_torques, 
                State * K, 
                State * x0, 
                Torque * u0,
                AngleState * imu,
                AngleState * omni,
                uint16_t op_idx) {
    // LQR Controller vectors
    K->phix=K1;
    K->thetax=K2;
    K->dphix=K3;
    K->dthetax=K4; // X controller
    K->phiy=K1;
    K->thetay=K2;
    K->dphiy=K3;
    K->dthetay=K4; // Y controller
    K->thetaz=4.5175;
    K->dthetaz=0.9804;  // Z controller
    // 
    M_torques->m11=0.9428;
    M_torques->m12=0;
    M_torques->m13=-0.3333;
    M_torques->m21=-0.4714;
    M_torques->m22=-0.8165;
    M_torques->m23=-0.3333;
    M_torques->m31=-0.4714;
    M_torques->m32=0.8165;
    M_torques->m33=-0.3333;
    // 
    M_odometry_omniangles->m11=-0.9428;
    M_odometry_omniangles->m12=0.4714;
    M_odometry_omniangles->m13=0.4714;
    M_odometry_omniangles->m21=0;
    M_odometry_omniangles->m22=-0.8165;
    M_odometry_omniangles->m23=0.8165;
    M_odometry_omniangles->m31=0.4714;
    M_odometry_omniangles->m32=0.4714;
    M_odometry_omniangles->m33=0.4714;
    // 
    M_odometry_IMUangles->m11=1;
    M_odometry_IMUangles->m12=0;
    M_odometry_IMUangles->m13=0;
    M_odometry_IMUangles->m21=0;
    M_odometry_IMUangles->m22=1;
    M_odometry_IMUangles->m23=0;
    M_odometry_IMUangles->m31=0;
    M_odometry_IMUangles->m32=0;
    M_odometry_IMUangles->m33=1;
    // Initial conditions: states
    x0->phix=0;
    x0->thetax=0;
    x0->dphix=0;
    x0->dthetax=0;
    x0->phiy=0;
    x0->thetay=0;
    x0->dphiy=0;
    x0->dthetay=0;
    x0->thetaz=0;
    x0->dthetaz=0;
    // Initial conditions: inputs
    u0->Tx1=0;
    u0->Ty2=0;
    u0->Tz3=0;

    imu->w1 = 0;
    imu->w2 = 0;
    imu->w3 = 0;
    imu->dw1 = 0;
    imu->dw2 = 0;
    imu->dw3 = 0;

    omni->w1 = 0;
    omni->w2 = 0;
    omni->w3 = 0;
    omni->dw1 = 0;
    omni->dw2 = 0;
    omni->dw3 = 0;
}