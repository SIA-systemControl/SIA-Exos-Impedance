//
// Created by yc on 2021/6/25.
//

#ifndef EXOS_IMPEDANCECONTROL_PID_H
#define EXOS_IMPEDANCECONTROL_PID_H

#include <cmath>

double sign(double num) {
    double ret;
    if (num > 0)
        ret = 1;
    else
        ret = -1;
    return ret;
}

class PID_position {
private:
    double Kp;
    double Kd;
    double Ki;
    double target;
    double actual;
    double e;
    double e_pre;
    double integral;

public:
    // construct method
    PID_position();

//    ~PID_position();

    PID_position(double p, double i, double d);

    double pid_control(double target, double actual, double u_limit);

    double pid_control_ff(double target, double actual, double u_limit, double ff);

    void pid_set_params(double p,double i,double d);
};

PID_position::PID_position() : Kp(0), Ki(0), Kd(0), target(0), actual(0), integral(0) {
    e = target - actual;
    e_pre = e;
}

PID_position::PID_position(double p, double i, double d) : Kp(p), Ki(i), Kd(d), target(0), actual(0), integral(0) {
    e = target - actual;
    e_pre = e;
}

double PID_position::pid_control(double tar, double act, double u_limit) {
    double u;
    this->target = tar;
    this->actual = act;
    e = target - actual;
    integral += e;
    u = Kp * e + Ki * integral + Kd * (e - e_pre);
    e_pre = e;

    if (abs(u) > u_limit)
        u = sign(u) * u_limit;

    return u;
}

double PID_position::pid_control_ff(double tar, double act, double u_limit, double ff){
    double u;
    this->target = tar;
    this->actual = act;
    e = target - actual;
    integral += e;
    u = Kp * e + Ki * integral + Kd * (e - e_pre);
    e_pre = e;

    u += sign(u)*ff;

    if (abs(u) > u_limit)
        u = sign(u) * u_limit;

    return u;
}

void PID_position::pid_set_params(double p, double i, double d) {
    this->Kp = p;
    this->Ki = i;
    this->Kd = d;
}

#endif //EXOS_IMPEDANCECONTROL_PID_H
