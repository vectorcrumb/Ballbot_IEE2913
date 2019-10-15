#include "state.h"

void get_phi(STATE_DATA* deltax, STATE_DATA x0, MATRIX M_od_omniangles, MATRIX M_od_IMUangles, ANGLES omniangles, ANGLES IMUangles, float deltat){
    float dPhix = M_od_omniangles.m11*omniangles.dw1+M_od_omniangles.m12*omniangles.dw2+M_od_omniangles.m13*omniangles.dw3+M_od_IMUangles.m11*IMUangles.dw1+M_od_IMUangles.m12*IMUangles.dw2+M_od_IMUangles.m13*IMUangles.dw3;
    float dPhiy = M_od_omniangles.m21*omniangles.dw1+M_od_omniangles.m22*omniangles.dw2+M_od_omniangles.m23*omniangles.dw3+M_od_IMUangles.m21*IMUangles.dw1+M_od_IMUangles.m22*IMUangles.dw2+M_od_IMUangles.m23*IMUangles.dw3;
    float Phix = (x0.phix+deltax->phix)+deltat*dPhix;
    float Phiy = (x0.phiy+deltax->phiy)+deltat*dPhiy;
    deltax->dphix = dPhix-x0.dphix;
    deltax->dphiy = dPhiy-x0.dphiy;
    deltax->phix = Phix-x0.phix;
    deltax->phiy = Phiy-x0.phiy;
}

void get_theta(STATE_DATA* deltax, ANGLES IMUangles, STATE_DATA x0){
    deltax->thetax = IMUangles.w1-x0.thetax;
    deltax->thetay = IMUangles.w2-x0.thetay;
    deltax->thetaz = IMUangles.w3-x0.thetaz;
    deltax->dthetax = IMUangles.dw1-x0.dthetax;
    deltax->dthetay = IMUangles.dw2-x0.dthetay;
    deltax->dthetaz = IMUangles.dw3-x0.dthetaz;
}