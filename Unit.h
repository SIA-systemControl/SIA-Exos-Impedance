//
// Created by yc on 2021/6/25.
//

#ifndef EXOS_IMPEDANCECONTROL_UNIT_H
#define EXOS_IMPEDANCECONTROL_UNIT_H


/**
 * Ankle link encoder is 13 bits!
 */
#define encoder_17bits 131072.0
#define encoder_13bits   8192.0

#define transmission_hip 120.0
#define transmission_knee 80.0
#define transmission_ankle 80.0
#define speeder_ankle 40.0/140.0

#define Pi 3.1415926

inline int relative_encoder_cnt(int current_value, int init_value){
    return (current_value-init_value);
}

inline double pos_rad2deg(double rad){
    return (180.0/Pi*rad);
}

inline double pos_deg2rad(double deg){
    return (Pi/180.0*deg);
}

inline double pos_inc2deg_17bits(int inc){
    return (inc*360.0/encoder_17bits);
}

inline double pos_inc2rad_17bits(int inc){
    return (inc*2.0*Pi/encoder_17bits);
}

inline int pos_deg2inc_17bits(double deg){
    return (int)(deg*encoder_17bits/360.0);
}

inline int pos_rad2inc_17bits(double rad){
    return (int)(rad*encoder_17bits/(2*Pi));
}

inline double pos_inc2deg_13bits(int inc){
    return (inc*360.0/encoder_13bits);
}

inline double pos_inc2rad_13bits(int inc){
    return (inc*2.0*Pi/encoder_13bits);
}

inline int pos_deg2inc_13bits(double deg){
    return (int)(deg*encoder_13bits/360.0);
}

inline int pos_rad2inc_13bits(double rad){
    return (int)(rad*encoder_13bits/(2*Pi));
}

inline double vel_RPM2deg(double RPM){
    return (RPM*6);
}

inline double vel_RPM2rad(double RPM){
    return (RPM*Pi/30.0);
}

inline double vel_rad2RPM(double rad){
    return (rad*30.0/Pi);
}

inline double tau_thousand2Nm(double thousand, double Rated_torque){
    return (thousand/1000.0)*Rated_torque;
}

inline int tau_Nm2thousand(double torque, double Rated_torque){
    return (torque/Rated_torque)*1000;
}


#endif //EXOS_IMPEDANCECONTROL_UNIT_H
