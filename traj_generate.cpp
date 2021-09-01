//
// Created by yc on 2021/6/24.
//
#include <cmath>
#include "traj_generate.h"

#define Ts 0.001

double base_Fourier_8th(double GaitCycle, const double *a, const double *b, double w, double q0) {
    double sum = 0;
    for (int i = 0; i < 5; i++)
        sum += a[i] * cos((i + 1) * GaitCycle * w) + b[i] * sin((i + 1) * GaitCycle * w);
    sum += q0;
    return sum;
}

double differentia_1st_Fourier_5th(double GaitCycle, const double *a, const double *b, double w, double q0, double P) {
    double sum = 0;
    for (int i = 0; i < 5; i++)
        sum += -((i + 1) * w / (P / 1000)) * a[i] * sin((i + 1) * GaitCycle * w) +
               ((i + 1) * w / (P / 1000)) * b[i] * cos((i + 1) * GaitCycle * w);
    return sum;
}

double differentia_2ed_Fourier_5th(double GaitCycle, const double *a, const double *b, double w, double q0, double P) {
    double sum = 0;
    for (int i = 0; i < 5; i++)
        sum += -((i + 1) * w/ (P / 1000)) * ((i + 1) * w/ (P / 1000)) * a[i] * cos((i + 1) * GaitCycle * w) +
               -((i + 1) * w/ (P / 1000)) * ((i + 1) * w/ (P / 1000)) * b[i] * sin((i + 1) * GaitCycle * w);
    return sum;
}

double differentia_3rd_Fourier_5th(double GaitCycle, const double *a, const double *b, double w, double q0, double P){
    double sum = 0;
    for (int i = 0; i < 5; i++)
        sum += pow((i + 1) * w/ (P / 1000),3) * a[i] * sin((i + 1) * GaitCycle * w) +
               -pow((i + 1) * w/ (P / 1000),3) * b[i] * cos((i + 1) * GaitCycle * w);
    return sum;
}

double differentia_4th_Fourier_5th(double GaitCycle, const double *a, const double *b, double w, double q0, double P){
    double sum = 0;
    for (int i = 0; i < 5; i++)
        sum += pow((i + 1) * w/ (P / 1000),4) * a[i] * cos((i + 1) * GaitCycle * w) +
               +pow((i + 1) * w/ (P / 1000),4) * b[i] * sin((i + 1) * GaitCycle * w);
    return sum;
}

double sineWave(int curve_count, double freq, double amp) {
    return amp * (sin(2 * 3.1415926 * freq * curve_count / 1000.0));
}

double cosineWave(int curve_count, double freq, double amp) {
    return amp * (cos(2 * 3.1415926 * freq * curve_count / 1000.0));
}

double Fourier_series_velocity_8th(int curve_count, double a0, const double *a, const double *b, double w) {
    double sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += a[i] * cos((i + 1) * curve_count * Ts * w) + b[i] * sin((i + 1) * curve_count * Ts * w);
    }
    return sum;
}

double Fourier_series_position_8th(int curve_count, double a0, const double *a, const double *b, double w) {
    double sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += a[i] / (w * (i + 1)) * sin((i + 1) * curve_count * Ts * w) -
               b[i] / (w * (i + 1)) * cos((i + 1) * curve_count * Ts * w);
    }
    return sum + a0;
}

double Fourier_series_acceleration_8th(int curve_count, double a0, const double *a, const double *b, double w) {
    double sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += -a[i] * (w * (i + 1)) * sin((i + 1) * curve_count * Ts * w) +
               b[i] * (w * (i + 1)) * cos((i + 1) * curve_count * Ts * w);
    }
    return sum;
}

double Fourier_series_velocity_10th(int curve_count, double a0, const double *a, const double *b, double w) {
    double sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += a[i] * cos((i + 1) * curve_count * Ts * w) + b[i] * sin((i + 1) * curve_count * Ts * w);
    }
    return sum;
}

double Fourier_series_position_10th(int curve_count, double a0, const double *a, const double *b, double w) {
    double sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += a[i] / (w * (i + 1)) * sin((i + 1) * curve_count * Ts * w) -
               b[i] / (w * (i + 1)) * cos((i + 1) * curve_count * Ts * w);
    }
    return sum + a0;
}

double Fourier_series_acceleration_10th(int curve_count, double a0, const double *a, const double *b, double w) {
    double sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += -a[i] * (w * (i + 1)) * sin((i + 1) * curve_count * Ts * w) +
               b[i] * (w * (i + 1)) * cos((i + 1) * curve_count * Ts * w);
    }
    return sum;
}

