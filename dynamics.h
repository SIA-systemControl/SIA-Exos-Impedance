//
// Created by yc on 2021/6/24.
//

#ifndef MATRIX_DYNAMICS_DYNAMICS_H
#define MATRIX_DYNAMICS_DYNAMICS_H

#include <cmath>
#include <Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

struct DiscretedMatrix{
    Eigen::Matrix3d Ad;
    Eigen::Vector3d Bd;
};

class dynamics {
private:
    double L1zz, L2zz, L3zz;
    double lx1, lx2, lx3;
    double Im1, Im2, Im3;
    double m1, m2, m3;
    double L1, L2;
    double m1_r, m2_r, m3_r;
    double N1, N2, N3;
    double grav_acc;
    double fv1, fc1, fv2, fc2, fv3, fc3;
    double fv1_m, fc1_m, fv2_m, fc2_m, fv3_m, fc3_m;
    Eigen::Matrix<double, 3, 3> Inertia;
    Eigen::Vector3d Coriolis;
    Eigen::Vector3d Gravity;
    Eigen::Vector3d friction_l;
    Eigen::Vector3d friction_m;
    Eigen::Vector3d theta_d;
    Eigen::Vector3d dtheta_d;
    Eigen::Vector3d ddtheta_d;
public:
    dynamics();

    dynamics(bool part);

    Eigen::Matrix<double, 3, 3> Inertia_term(Eigen::Vector3d q);

    Eigen::Vector3d Coriolis_term(Eigen::Vector3d dq, Eigen::Vector3d q);

    Eigen::Vector3d Gravity_term(Eigen::Vector3d q);

    Eigen::Vector3d Friction_link(Eigen::Vector3d dq);

    Eigen::Vector3d Friction_motor(Eigen::Vector3d dtheta);

    Eigen::Vector3d
    coupling_dynamics(Eigen::Vector3d ddq, Eigen::Vector3d dq, Eigen::Vector3d q, Eigen::Vector3d ddtheta,
                      Eigen::Vector3d dtheta);

    Eigen::Vector3d link_dynamics(Eigen::Vector3d ddq, Eigen::Vector3d dq, Eigen::Vector3d q, Eigen::Vector3d ddtheta);

    Eigen::Vector3d motor_dynamics(Eigen::Vector3d ddq, Eigen::Vector3d ddtheta, Eigen::Vector3d dtheta);

    Eigen::Vector3d
    tau_spring(Eigen::Vector3d tau_m, Eigen::Vector3d ddq, Eigen::Vector3d ddtheta, Eigen::Vector3d dtheta);

    Eigen::Vector3d
    feedforward_dynamics(Eigen::Vector3d d4q, Eigen::Vector3d d3q, Eigen::Vector3d ddq, Eigen::Vector3d dq,
                         Eigen::Vector3d q, Eigen::Matrix3d K);

    DiscretedMatrix Discretization(Eigen::Matrix3d A, Eigen::Vector3d B, double Ts);

    double get_para();
};

/**
 * identification para : 21/07/06
 */
dynamics::dynamics() {
//    L1zz = 0.782;lx1 = 2.0985;
//    L2zz = 0.2870;lx2 = 0.8246;
//    L3zz = 0.0033;lx3 = 0.0041;
//    m1 = 4.52;m2 = 1.95;m3 = 0.95 + 4.09;
//    m1_r = 0.37;m2_r = 0.37;m3_r = 0.23;
//    N1 = 120;N2 = 80;N3 = 80;
//    grav_acc = 9.81;
//    fv1 = 0.2753;fc1 = 0.4823;
//    fv2 = -0.5029;fc2 = 0.3106;
//    fv3 = -2.3965;fc3 = 0.6854;
//    fv1_m = 21.5333;fc1_m = 9.9352;
//    fv2_m = 7.3022;fc2_m = 2.8866;
//    fv3_m = 5.4917;fc3_m = 3.6898;
//    L1 = 0.4;L2 = 0.395;
//    Im1 = 0.8786;Im2 = 0.4693;Im3 = 0.0150;

    L1zz = 0.8862;
    lx1 = 2.8132;
    L2zz = 0.5833;
    lx2 = 1.9342;
    L3zz = -0.0387;
    lx3 = 0.1282;
    m1 = 4.52;
    m2 = 1.95;
    m3 = 0.95 + 4.09;
    m1_r = 0.37;
    m2_r = 0.37;
    m3_r = 0.23;
    N1 = 120;
    N2 = 80;
    N3 = 80;
    grav_acc = 9.81;
    fv1 = 1.3929;
    fc1 = 0.1743;
    fv2 = -1.4827;
    fc2 = 1.5793;
    fv3 = -0.0114;
    fc3 = 0.0365;
    fv1_m = 3.7436;
    fc1_m = 9.9992;
    fv2_m = 2.7897;
    fc2_m = 2.1693;
    fv3_m = -3.4303;
    fc3_m = 2.5243;
    L1 = 0.4;
    L2 = 0.395;
    Im1 = 1.1717;
    Im2 = 0.9954;
    Im3 = 0.1865;
}

Eigen::Matrix<double, 3, 3> dynamics::Inertia_term(Eigen::Vector3d q) {
    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);

    double M11 = L1zz + L2zz + L3zz + lx3 * (2 * (L1 * cos(q2 + q3) + L2 * cos(q3))) + lx2 * (2 * L1 * cos(q2));
    double M12 = L2zz + L3zz + lx3 * (2 * L2 * cos(q3) + L1 * cos(q2 + q3)) + lx2 * L1 * cos(q2);
    double M13 = L3zz + lx3 * (L1 * cos(q2 + q3) + L2 * cos(q3));
    double M22 = L2zz + L3zz + lx3 * 2 * L2 * cos(q3);
    double M23 = L3zz + lx3 * L2 * cos(q3);
    double M33 = L3zz;

    Eigen::Matrix<double, 3, 3> Inertia;
    Inertia << M11, M12, M13,
            M12, M22, M23,
            M13, M23, M33;

    return Inertia;

}

Eigen::Vector3d dynamics::Coriolis_term(Eigen::Vector3d dq, Eigen::Vector3d q) {
    double dq1 = dq(0);
    double dq2 = dq(1);
    double dq3 = dq(2);
    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);

    double C1, C2, C3;
    C1 = -lx3 *
         (L1 * sin(q2 + q3) * (pow(dq2, 2) + pow(dq3, 2)) + L2 * sin(q3) * pow(dq3, 2) + 2 * L2 * sin(q3) * dq1 * dq3 +
          2 * L2 * sin(q3) * dq2 * dq3 + 2 * L1 * sin(q2 + q3) * dq1 * dq2 +
          2 * L1 * sin(q2 + q3) * dq1 * dq3 + 2 * L1 * sin(q2 + q3) * dq2 * dq3) -
         lx2 * (L1 * sin(q1) * pow(dq2, 2) + 2 * L1 * sin(q2) * dq1 * dq2);

    C2 = lx3 * (L1 * sin(q1 + q2 + q3) * pow(dq1, 2) - L2 * sin(q3) * pow(dq3, 2) - 2 * L2 * sin(q3) * dq1 * dq3 -
                2 * L2 * sin(q3) * dq2 * dq3) + lx2 * L1 * sin(q2) * pow(dq1, 2);

    C3 = lx3 * (L1 * sin(q2 + q3) * pow(dq1, 2) + L2 * sin(q3) * pow(dq1, 2) + L2 * sin(q3) * pow(dq2, 2) +
                2 * L2 * sin(q3) * dq1 * dq2);

    Eigen::Vector3d Coriolis;
    Coriolis << C1, C2, C3;

    return Coriolis;
}

Eigen::Vector3d dynamics::Gravity_term(Eigen::Vector3d q) {

    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);

    double G1, G2, G3;

    G1 = lx3 * sin(q1 + q2 + q3) * grav_acc + lx2 * sin(q1 + q2) * grav_acc + lx1 * sin(q1) * grav_acc;
    G2 = lx3 * sin(q1 + q2 + q3) * grav_acc + lx2 * sin(q1 + q2) * grav_acc;
    G3 = lx3 * sin(q1 + q2 + q3) * grav_acc;

    Eigen::Vector3d Gravity_term;
    Gravity_term << G1, G2, G3;
    return Gravity_term;
}


Eigen::Vector3d dynamics::Friction_link(Eigen::Vector3d dq) {
    double dq1 = dq(0);
    double dq2 = dq(1);
    double dq3 = dq(2);

    double friction_l_1, friction_l_2, friction_l_3;

    friction_l_1 = fv1 * dq1 + fc1 * tanh(2 * dq1 / 0.01);
    friction_l_2 = fv2 * dq2 + fc2 * tanh(2 * dq2 / 0.01);
    friction_l_3 = fv3 * dq3 + fc3 * tanh(2 * dq3 / 0.01);

    Eigen::Vector3d Friction_L;
    Friction_L << friction_l_1, friction_l_2, friction_l_3;
    return Friction_L;
}

Eigen::Vector3d dynamics::Friction_motor(Eigen::Vector3d dtheta) {
    double dtheta1 = dtheta(0);
    double dtheta2 = dtheta(1);
    double dtheta3 = dtheta(2);

    double friction_m_1, friction_m_2, friction_m_3;

    friction_m_1 = fv1_m * dtheta1 + fc1_m * tanh(2 * dtheta1 / 0.02);
    friction_m_2 = fv2_m * dtheta2 + fc2_m * tanh(2 * dtheta2 / 0.02);
    friction_m_3 = fv3_m * dtheta3 + fc3_m * tanh(2 * dtheta3 / 0.02);

    Eigen::Vector3d Friction_M;
    Friction_M << friction_m_1, friction_m_2, friction_m_3;
    return Friction_M;
}


Eigen::Vector3d
dynamics::coupling_dynamics(Eigen::Vector3d ddq, Eigen::Vector3d dq, Eigen::Vector3d q, Eigen::Vector3d ddtheta,
                            Eigen::Vector3d dtheta) {

    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);

    double dq1 = dq(0);
    double dq2 = dq(1);
    double dq3 = dq(2);

    double ddq1 = ddq(0);
    double ddq2 = ddq(1);
    double ddq3 = ddq(2);

    double dtheta1 = dtheta(0);
    double dtheta2 = dtheta(1);
    double dtheta3 = dtheta(2);

    double ddtheta1 = ddtheta(0);
    double ddtheta2 = ddtheta(1);
    double ddtheta3 = ddtheta(2);

    Inertia = dynamics::Inertia_term(q);
    Coriolis = dynamics::Coriolis_term(dq, q);
    Gravity = dynamics::Gravity_term(q);
    friction_l = dynamics::Friction_link(dq);
    friction_m = dynamics::Friction_motor(dtheta);
//    friction_m(1) = 0;

    Eigen::Matrix<double, 3, 3> Inertia_m;
    double Mr11, Mr12, Mr22;
    Mr11 = (m2_r + m3_r) * pow(L1, 2) + m3_r * pow(L2, 2) + 2 * m3_r * L1 * L2 * cos(q2);
    Mr12 = m3_r * pow(L2, 2) + m3_r * L1 * L2 * cos(q2);
    Mr22 = m3_r * pow(L2, 2);

    Inertia_m << Mr11, Mr12, 0,
            Mr12, Mr22, 0,
            0, 0, 0;

    Eigen::Matrix3d B;
    Eigen::Vector3d I_mot;
    I_mot << Im1, Im2, Im3;
    B = I_mot.asDiagonal();


    Eigen::Matrix3d S;
    S << 0, Im2 / N2, Im3 / N3,
            0, 0, Im3 / N3,
            0, 0, 0;

//    S << 0, 0, 0, 0, 0, 0, 0, 0, 0;

    Eigen::Matrix3d SBS;
    SBS << Im2 / pow(N2, 2) + Im3 / pow(N3, 2), Im3 / pow(N3, 2), 0,
            Im3 / pow(N3, 2), Im3 / pow(N3, 2), 0,
            0, 0, 0;

//    SBS << 0, 0, 0, 0, 0, 0, 0, 0, 0;

    Eigen::Matrix3d H;
    H = Inertia + SBS;

    Eigen::Vector3d tau_m;
    tau_m = (H + S.transpose()) * ddq + (S + B) * ddtheta + Coriolis + Gravity + friction_l + friction_m;
//    tau_m = (H + S.transpose()) * ddq_l + (S + B) * ddtheta + Coriolis + Gravity + friction_l;
    return tau_m;
}


Eigen::Vector3d
dynamics::link_dynamics(Eigen::Vector3d ddq, Eigen::Vector3d dq, Eigen::Vector3d q, Eigen::Vector3d ddtheta) {
    Inertia = dynamics::Inertia_term(q);
    Coriolis = dynamics::Coriolis_term(dq, q);
    Gravity = dynamics::Gravity_term(q);
    friction_l = dynamics::Friction_link(dq);

    Eigen::Matrix3d S;
    S << 0, Im2 / N2, Im3 / N3,
            0, 0, Im3 / N3,
            0, 0, 0;

    Eigen::Matrix3d SBS;
    SBS << Im2 / pow(N2, 2) + Im3 / pow(N3, 2), Im3 / pow(N3, 2), 0,
            Im3 / pow(N3, 2), Im3 / pow(N3, 2), 0,
            0, 0, 0;

    Eigen::Matrix3d H;
    H = Inertia + SBS;

    Eigen::Vector3d dyn_link;

    dyn_link = H * ddq + Coriolis + Gravity + S * ddtheta + friction_l;

    return dyn_link;
}

Eigen::Vector3d dynamics::motor_dynamics(Eigen::Vector3d ddq, Eigen::Vector3d ddtheta, Eigen::Vector3d dtheta) {
    Eigen::Vector3d I_mot;
    I_mot << Im1, Im2, Im3;
    I_mot.asDiagonal();

    /* TODO:
     *      Here need stiffness of spring,
     */
    return I_mot;
}

Eigen::Vector3d
dynamics::tau_spring(Eigen::Vector3d tau_m, Eigen::Vector3d ddq, Eigen::Vector3d ddtheta, Eigen::Vector3d dtheta) {
    friction_m = dynamics::Friction_motor(dtheta);
    Eigen::Matrix3d S;
    S << 0, Im2 / N2, Im3 / N3,
            0, 0, Im3 / N3,
            0, 0, 0;

    Eigen::Matrix3d B;
    Eigen::Vector3d I_mot;
    I_mot << Im1, Im2, Im3;
    B = I_mot.asDiagonal();

    Eigen::Vector3d tau_spr;
    tau_spr = tau_m - S.transpose() * ddq - B * ddtheta - friction_m;
    return tau_spr;
}

Eigen::Vector3d
dynamics::feedforward_dynamics(Eigen::Vector3d d4q, Eigen::Vector3d d3q, Eigen::Vector3d ddq, Eigen::Vector3d dq,
                               Eigen::Vector3d q, Eigen::Matrix3d K) {

    double q1, q2, q3;
    double dq1, dq2, dq3;
    double ddq1, ddq2, ddq3;
    double d3q1, d3q2, d3q3;
    double d4q1, d4q2, d4q3;

    q1 = q(0);
    q2 = q(1);
    q3 = q(2);
    dq1 = dq(0);
    dq2 = dq(1);
    dq3 = dq(2);
    ddq1 = ddq(0);
    ddq2 = ddq(1);
    ddq3 = ddq(2);
    d3q1 = q(0);
    d3q2 = d3q(1);
    d3q3 = d3q(2);
    d4q1 = q(0);
    d4q2 = q(1);
    d4q3 = q(2);


    Inertia = dynamics::Inertia_term(q);
    Coriolis = dynamics::Coriolis_term(dq, q);
    Gravity = dynamics::Gravity_term(q);

    Eigen::Matrix3d S;
    S << 0, Im2 / N2, Im3 / N3,
            0, 0, Im3 / N3,
            0, 0, 0;

    Eigen::Matrix3d SBS;
    SBS << Im2 / pow(N2, 2) + Im3 / pow(N3, 2), Im3 / pow(N3, 2), 0,
            Im3 / pow(N3, 2), Im3 / pow(N3, 2), 0,
            0, 0, 0;

    Eigen::Matrix3d H;
    H = Inertia + SBS;

    theta_d = K.inverse() * (H * ddq + Coriolis + Gravity + K * q);

    Eigen::Matrix3d dM, ddM;

    dM << -dq2 * (2 * L1 * lx3 * sin(q2 + q3) + 2 * L1 * lx2 * sin(q2)) -
          dq3 * lx3 * (2 * L1 * sin(q2 + q3) + 2 * L2 * sin(q3)),
            -dq2 * (L1 * lx3 * sin(q2 + q3) + L1 * lx2 * sin(q2)) - dq3 * lx3 * (L1 * sin(q2 + q3) + 2 * L2 * sin(q3)),
            -dq3 * lx3 * (L1 * sin(q2 + q3) + L2 * sin(q3)) - L1 * dq2 * lx3 * sin(q2 + q3),
            -dq2 * (L1 * lx3 * sin(q2 + q3) + L1 * lx2 * sin(q2)) - dq3 * lx3 * (L1 * sin(q2 + q3) + 2 * L2 * sin(q3)),
            -2 * L2 * dq3 * lx3 * sin(q3), -L2 * dq3 * lx3 * sin(q3),
            -dq3 * lx3 * (L1 * sin(q2 + q3) + L2 * sin(q3)) - L1 * dq2 * lx3 * sin(q2 + q3), -L2 * dq3 * lx3 *
                                                                                             sin(q3), 0;

    ddM << -dq2 * (dq2 * (2 * L1 * lx3 * cos(q2 + q3) + 2 * L1 * lx2 * cos(q2)) + 2 * L1 * dq3 * lx3 * cos(q2 + q3)) -
           dq3 * (dq3 * lx3 * (2 * L1 * cos(q2 + q3) + 2 * L2 * cos(q3)) + 2 * L1 * dq2 * lx3 * cos(q2 + q3)) -
           ddq2 * (2 * L1 * lx3 * sin(q2 + q3) + 2 * L1 * lx2 * sin(q2)) -
           ddq3 * lx3 * (2 * L1 * sin(q2 + q3) + 2 * L2 * sin(q3)),
            -dq2 * (dq2 * (L1 * lx3 * cos(q2 + q3) + L1 * lx2 * cos(q2)) + L1 * dq3 * lx3 * cos(q2 + q3)) -
            dq3 * (dq3 * lx3 * (L1 * cos(q2 + q3) + 2 * L2 * cos(q3)) + L1 * dq2 * lx3 * cos(q2 + q3)) -
            ddq2 * (L1 * lx3 * sin(q2 + q3) + L1 * lx2 * sin(q2)) - ddq3 * lx3 * (L1 * sin(q2 + q3) + 2 * L2 * sin(q3)),
            -lx3 * (L1 * dq2 * dq2 * cos(q2 + q3) + L1 * dq3 * dq3 * cos(q2 + q3) + L2 * dq3 * dq3 * cos(q3) +
                    L1 * ddq2 * sin(q2 + q3) + L1 * ddq3 * sin(q2 + q3) + L2 * ddq3 * sin(q3) +
                    2 * L1 * dq2 * dq3 * cos(q2 + q3)),
            -dq2 * (dq2 * (L1 * lx3 * cos(q2 + q3) + L1 * lx2 * cos(q2)) + L1 * dq3 * lx3 * cos(q2 + q3)) -
            dq3 * (dq3 * lx3 * (L1 * cos(q2 + q3) + 2 * L2 * cos(q3)) + L1 * dq2 * lx3 * cos(q2 + q3)) -
            ddq2 * (L1 * lx3 * sin(q2 + q3) + L1 * lx2 * sin(q2)) - ddq3 * lx3 * (L1 * sin(q2 + q3) + 2 * L2 * sin(q3)),
            -2 * L2 * lx3 * (cos(q3) * dq3 * dq3 + ddq3 * sin(q3)), -L2 * lx3 * (cos(q3) * dq3 * dq3 + ddq3 * sin(q3)),
            -lx3 * (L1 * dq2 * dq2 * cos(q2 + q3) + L1 * dq3 * dq3 * cos(q2 + q3) + L2 * dq3 * dq3 * cos(q3) +
                    L1 * ddq2 * sin(q2 + q3) + L1 * ddq3 * sin(q2 + q3) + L2 * ddq3 * sin(q3) +
                    2 * L1 * dq2 * dq3 * cos(q2 + q3)), -L2 * lx3 * (cos(q3) * dq3 * dq3 + ddq3 * sin(q3)), 0;


    Eigen::Vector3d dC, ddC;

    dC << -lx2 * (2 * L1 * sin(q1) * dq2 * ddq2 + 2 * L1 * sin(q2) * dq1 * ddq2 + 2 * L1 * sin(q2) * dq2 * ddq1 +
                  L1 * cos(q1) * dq2 * dq2 * dq1 + 2 * L1 * cos(q2) * dq1 * dq2 * dq2 - lx3 * (L1 * sin(q2 + q3) *
                                                                                               (2 * dq2 * ddq2 +
                                                                                                2 * dq3 * ddq3 +
                                                                                                2 * L2 * sin(q3) * dq1 *
                                                                                                ddq3 +
                                                                                                2 * L2 * sin(q3) * dq3 *
                                                                                                ddq1 +
                                                                                                2 * L2 * sin(q3) * dq2 *
                                                                                                ddq3 +
                                                                                                2 * L2 * sin(q3) * dq3 *
                                                                                                ddq2 +
                                                                                                2 * L2 * sin(q3) * dq3 *
                                                                                                ddq3 +
                                                                                                2 * L1 * sin(q2 + q3) *
                                                                                                dq1 * ddq2 +
                                                                                                2 * L1 * sin(q2 + q3) *
                                                                                                dq2 * ddq1 +
                                                                                                2 * L1 * sin(q2 + q3) *
                                                                                                dq1 * ddq3 +
                                                                                                2 * L1 * sin(q2 + q3) *
                                                                                                dq3 * ddq1 +
                                                                                                2 * L1 * sin(q2 + q3) *
                                                                                                dq2 * ddq3 +
                                                                                                2 * L1 * sin(q2 + q3) *
                                                                                                dq3 * ddq2 +
                                                                                                L1 * cos(q2 + q3) *
                                                                                                (dq2 * dq2 +
                                                                                                 dq3 * dq3) *
                                                                                                (dq2 + dq3) +
                                                                                                L2 * cos(q3) * dq3 *
                                                                                                dq3 * dq3 +
                                                                                                2 * L2 * cos(q3) * dq1 *
                                                                                                dq3 * dq3 +
                                                                                                2 * L2 * cos(q3) * dq2 *
                                                                                                dq3 * dq3 +
                                                                                                2 * L1 * cos(q2 + q3) *
                                                                                                dq1 * dq2 * dq2 + dq3) +
                                                                                               2 * L1 * cos(q2 + q3) *
                                                                                               dq1 * dq3 * (dq2 + dq3) +
                                                                                               2 * L1 * cos(q2 + q3) *
                                                                                               dq2 * dq3 *
                                                                                               (dq2 + dq3))),
            2 * L1 * lx3 * sin(q1 + q2 + q3) * dq1 * ddq1 + L1 * lx2 * cos(q2) * dq1 * dq1 * dq2 -
            L2 * lx3 * cos(q3) * dq3 * dq3 * dq3 + L1 * lx3 * cos(q1 + q2 + q3) * dq1 * dq1 * dq1 +
            L1 * lx3 * cos(q1 + q2 + q3) * dq1 * dq1 * dq2 + L1 * lx3 * cos(q1 + q2 + q3) * dq1 * dq1 * dq3 +
            2 * L1 * lx2 * sin(q2) * dq1 * ddq1 - 2 * L2 * lx3 * sin(q3) * dq1 * ddq3 -
            2 * L2 * lx3 * sin(q3) * dq3 * ddq1 - 2 * L2 * lx3 * sin(q3) * dq2 * ddq3 -
            2 * L2 * lx3 * sin(q3) * dq3 * ddq2 - 2 * L2 * lx3 * sin(q3) * dq3 * ddq3 -
            2 * L2 * lx3 * cos(q3) * dq1 * dq3 * dq3 - 2 * L2 * lx3 * cos(q3) * dq2 * dq3 * dq3,
            lx3 * (L1 * cos(q2 + q3) * dq1 * dq1 * (dq2 + dq3) + 2 * L2 * sin(q3) * dq1 * ddq1 +
                   2 * L2 * sin(q3) * dq1 * ddq2 + 2 * L2 * sin(q3) * dq2 * ddq1 + 2 * L2 * sin(q3) * dq2 * ddq2 +
                   2 * L1 * sin(q2 + q3) * dq1 * ddq1 + L2 * cos(q3) * dq1 * dq1 * dq3 +
                   L2 * cos(q3) * dq2 * dq2 * dq3 + 2 * L2 * cos(q3) * dq1 * dq2 * dq3);

    ddC << (L1 * lx2 * (2 * dq3 * dq3 * lx3 * cos(q2 + q3) - 8 * ddq1 * ddq2 * sin(q2) - 4 * d3q1 * dq2 * sin(q2) -
                        4 * d3q2 * dq1 * sin(q2) - 4 * d3q2 * dq2 * sin(q1) - 4 * ddq2 * ddq2 * sin(q1) +
                        4 * ddq2 * ddq2 * lx3 * sin(q2 + q3) + 4 * ddq3 * ddq3 * lx3 * sin(q2 + q3) -
                        2 * ddq1 * dq2 * dq2 * cos(q1) - 8 * ddq1 * dq2 * dq2 * cos(q2) +
                        4 * dq1 * dq2 * dq2 * dq2 * sin(q2) + 2 * dq1 * dq1 * dq2 * dq2 * sin(q1) +
                        2 * ddq3 * lx3 * sin(q2 + q3) - 8 * dq2 * dq2 * dq3 * dq3 * lx3 * sin(q2 + q3) +
                        2 * dq2 * dq3 * lx3 * cos(q2 + q3) +
                        2 * L1 * dq2 * dq2 * dq2 * dq2 * lx3 * cos(2 * q2 + 2 * q3) +
                        2 * L1 * dq3 * dq3 * dq3 * dq3 * lx3 * cos(2 * q2 + 2 * q3) +
                        4 * d3q2 * dq2 * lx3 * sin(q2 + q3) + 4 * d3q3 * dq3 * lx3 * sin(q2 + q3) -
                        8 * ddq2 * dq1 * dq2 * cos(q1) - 12 * ddq2 * dq1 * dq2 * cos(q2) +
                        4 * ddq1 * dq3 * dq3 * lx3 * cos(q2 + q3) + 4 * ddq2 * dq2 * dq2 * lx3 * cos(q2 + q3) +
                        4 * ddq2 * dq3 * dq3 * lx3 * cos(q2 + q3) + 4 * ddq3 * dq2 * dq2 * lx3 * cos(q2 + q3) +
                        4 * ddq3 * dq3 * dq3 * lx3 * cos(q2 + q3) - 4 * dq1 * dq3 * dq3 * dq3 * lx3 * sin(q2 + q3) -
                        4 * dq2 * dq3 * dq3 * dq3 * lx3 * sin(q2 + q3) -
                        4 * dq2 * dq2 * dq2 * dq3 * lx3 * sin(q2 + q3) + 2 * L2 * ddq3 * ddq3 * lx3 * cos(q2) -
                        2 * L2 * ddq3 * ddq3 * lx3 * cos(q2 + 2 * q3) +
                        2 * L2 * dq3 * dq3 * dq3 * dq3 * lx3 * cos(q2 + 2 * q3) + 4 * L1 * ddq1 * ddq2 * lx3 +
                        4 * L1 * ddq1 * ddq3 * lx3 + 4 * L1 * ddq2 * ddq3 * lx3 + 2 * L1 * d3q1 * dq2 * lx3 +
                        2 * L1 * d3q2 * dq1 * lx3 + 2 * L1 * d3q1 * dq3 * lx3 + 2 * L1 * d3q3 * dq1 * lx3 +
                        2 * L1 * d3q2 * dq3 * lx3 + 2 * L1 * d3q3 * dq2 * lx3 +
                        4 * L1 * dq2 * dq2 * dq3 * dq3 * lx3 * cos(2 * q2 + 2 * q3) -
                        8 * dq1 * dq2 * dq3 * dq3 * lx3 * sin(q2 + q3) -
                        4 * dq1 * dq2 * dq2 * dq3 * lx3 * sin(q2 + q3) + L2 * dq2 * dq3 * dq3 * lx3 * cos(q2) +
                        2 * L2 * ddq1 * dq3 * dq3 * lx3 * sin(q2) + 2 * L2 * ddq2 * dq3 * dq3 * lx3 * sin(q2) -
                        2 * L2 * ddq3 * dq2 * dq2 * lx3 * sin(q2) + 3 * L2 * ddq3 * dq3 * dq3 * lx3 * sin(q2) -
                        4 * L1 * ddq1 * ddq2 * lx3 * cos(2 * q2 + 2 * q3) -
                        4 * L1 * ddq1 * ddq3 * lx3 * cos(2 * q2 + 2 * q3) -
                        4 * L1 * ddq2 * ddq3 * lx3 * cos(2 * q2 + 2 * q3) -
                        2 * L1 * d3q1 * dq2 * lx3 * cos(2 * q2 + 2 * q3) -
                        2 * L1 * d3q2 * dq1 * lx3 * cos(2 * q2 + 2 * q3) -
                        2 * L1 * d3q1 * dq3 * lx3 * cos(2 * q2 + 2 * q3) -
                        2 * L1 * d3q3 * dq1 * lx3 * cos(2 * q2 + 2 * q3) -
                        2 * L1 * d3q2 * dq3 * lx3 * cos(2 * q2 + 2 * q3) -
                        2 * L1 * d3q3 * dq2 * lx3 * cos(2 * q2 + 2 * q3) +
                        4 * L2 * dq1 * dq3 * dq3 * dq3 * lx3 * cos(q2 + 2 * q3) +
                        5 * L2 * dq2 * dq3 * dq3 * dq3 * lx3 * cos(q2 + 2 * q3) +
                        6 * L2 * ddq1 * dq3 * dq3 * lx3 * sin(q2 + 2 * q3) +
                        6 * L2 * ddq2 * dq3 * dq3 * lx3 * sin(q2 + 2 * q3) +
                        2 * L2 * ddq3 * dq2 * dq2 * lx3 * sin(q2 + 2 * q3) +
                        7 * L2 * ddq3 * dq3 * dq3 * lx3 * sin(q2 + 2 * q3) +
                        2 * L2 * dq2 * dq2 * dq3 * dq3 * lx3 * cos(q2) + 4 * ddq1 * dq2 * dq3 * lx3 * cos(q2 + q3) +
                        4 * ddq2 * dq1 * dq3 * lx3 * cos(q2 + q3) + 4 * ddq3 * dq1 * dq2 * lx3 * cos(q2 + q3) +
                        12 * ddq2 * dq2 * dq3 * lx3 * cos(q2 + q3) + 8 * ddq3 * dq1 * dq3 * lx3 * cos(q2 + q3) +
                        12 * ddq3 * dq2 * dq3 * lx3 * cos(q2 + q3) +
                        4 * L1 * dq1 * dq2 * dq2 * dq2 * lx3 * cos(2 * q2 + 2 * q3) +
                        4 * L1 * dq2 * dq3 * dq3 * dq3 * lx3 * cos(2 * q2 + 2 * q3) +
                        4 * L1 * dq2 * dq2 * dq2 * dq3 * lx3 * cos(2 * q2 + 2 * q3) +
                        2 * L2 * dq2 * dq2 * dq3 * dq3 * lx3 * cos(q2 + 2 * q3) +
                        6 * L1 * ddq1 * dq2 * dq2 * lx3 * sin(2 * q2 + 2 * q3) +
                        4 * L1 * ddq1 * dq3 * dq3 * lx3 * sin(2 * q2 + 2 * q3) +
                        3 * L1 * ddq2 * dq2 * dq2 * lx3 * sin(2 * q2 + 2 * q3) +
                        5 * L1 * ddq2 * dq3 * dq3 * lx3 * sin(2 * q2 + 2 * q3) +
                        5 * L1 * ddq3 * dq2 * dq2 * lx3 * sin(2 * q2 + 2 * q3) +
                        3 * L1 * ddq3 * dq3 * dq3 * lx3 * sin(2 * q2 + 2 * q3) + 4 * L2 * ddq1 * ddq3 * lx3 * cos(q2) +
                        4 * L2 * ddq2 * ddq3 * lx3 * cos(q2) + 2 * L2 * d3q1 * dq3 * lx3 * cos(q2) +
                        2 * L2 * d3q3 * dq1 * lx3 * cos(q2) + 2 * L2 * d3q2 * dq3 * lx3 * cos(q2) +
                        2 * L2 * d3q3 * dq2 * lx3 * cos(q2) + 2 * L2 * d3q3 * dq3 * lx3 * cos(q2) -
                        4 * L2 * ddq1 * ddq3 * lx3 * cos(q2 + 2 * q3) - 4 * L2 * ddq2 * ddq3 * lx3 * cos(q2 + 2 * q3) -
                        2 * L2 * d3q1 * dq3 * lx3 * cos(q2 + 2 * q3) - 2 * L2 * d3q3 * dq1 * lx3 * cos(q2 + 2 * q3) -
                        2 * L2 * d3q2 * dq3 * lx3 * cos(q2 + 2 * q3) - 2 * L2 * d3q3 * dq2 * lx3 * cos(q2 + 2 * q3) -
                        2 * L2 * d3q3 * dq3 * lx3 * cos(q2 + 2 * q3) +
                        4 * L1 * dq1 * dq2 * dq2 * dq3 * lx3 * cos(2 * q2 + 2 * q3) -
                        2 * L2 * ddq1 * dq2 * dq3 * lx3 * sin(q2) - 2 * L2 * ddq3 * dq1 * dq2 * lx3 * sin(q2) -
                        2 * L2 * ddq2 * dq2 * dq3 * lx3 * sin(q2) + 4 * L2 * ddq3 * dq1 * dq3 * lx3 * sin(q2) +
                        2 * L2 * ddq3 * dq2 * dq3 * lx3 * sin(q2) + 2 * L2 * ddq1 * dq2 * dq3 * lx3 * sin(q2 + 2 * q3) +
                        2 * L2 * ddq3 * dq1 * dq2 * lx3 * sin(q2 + 2 * q3) +
                        2 * L2 * ddq2 * dq2 * dq3 * lx3 * sin(q2 + 2 * q3) +
                        8 * L2 * ddq3 * dq1 * dq3 * lx3 * sin(q2 + 2 * q3) +
                        10 * L2 * ddq3 * dq2 * dq3 * lx3 * sin(q2 + 2 * q3) +
                        2 * L2 * dq1 * dq2 * dq3 * dq3 * lx3 * cos(q2) +
                        2 * L2 * dq1 * dq2 * dq3 * dq3 * lx3 * cos(q2 + 2 * q3) +
                        8 * L1 * ddq2 * dq1 * dq2 * lx3 * sin(2 * q2 + 2 * q3) +
                        8 * L1 * ddq1 * dq2 * dq3 * lx3 * sin(2 * q2 + 2 * q3) +
                        4 * L1 * ddq2 * dq1 * dq3 * lx3 * sin(2 * q2 + 2 * q3) +
                        4 * L1 * ddq3 * dq1 * dq2 * lx3 * sin(2 * q2 + 2 * q3) +
                        6 * L1 * ddq2 * dq2 * dq3 * lx3 * sin(2 * q2 + 2 * q3) +
                        4 * L1 * ddq3 * dq1 * dq3 * lx3 * sin(2 * q2 + 2 * q3) +
                        6 * L1 * ddq3 * dq2 * dq3 * lx3 * sin(2 * q2 + 2 * q3))) / 2,
            2 * L1 * ddq1 * ddq1 * lx3 * sin(q1 + q2 + q3) - L1 * dq1 * dq1 * dq1 * dq1 * lx3 * sin(q1 + q2 + q3) +
            2 * L1 * ddq1 * ddq1 * lx2 * sin(q2) - 2 * L2 * ddq3 * ddq3 * lx3 * sin(q3) +
            L2 * dq3 * dq3 * dq3 * dq3 * lx3 * sin(q3) + 2 * L1 * d3q1 * dq1 * lx3 * sin(q1 + q2 + q3) +
            L1 * ddq2 * dq1 * dq1 * lx2 * cos(q2) - 4 * L2 * ddq1 * dq3 * dq3 * lx3 * cos(q3) -
            4 * L2 * ddq2 * dq3 * dq3 * lx3 * cos(q3) - 5 * L2 * ddq3 * dq3 * dq3 * lx3 * cos(q3) +
            2 * L2 * dq1 * dq3 * dq3 * dq3 * lx3 * sin(q3) + 2 * L2 * dq2 * dq3 * dq3 * dq3 * lx3 * sin(q3) +
            5 * L1 * ddq1 * dq1 * dq1 * lx3 * cos(q1 + q2 + q3) + L1 * ddq2 * dq1 * dq1 * lx3 * cos(q1 + q2 + q3) +
            L1 * ddq3 * dq1 * dq1 * lx3 * cos(q1 + q2 + q3) - 2 * L1 * dq1 * dq1 * dq1 * dq2 * lx3 * sin(q1 + q2 + q3) -
            2 * L1 * dq1 * dq1 * dq1 * dq3 * lx3 * sin(q1 + q2 + q3) - L1 * dq1 * dq1 * dq2 * dq2 * lx2 * sin(q2) -
            4 * L2 * ddq1 * ddq3 * lx3 * sin(q3) - 4 * L2 * ddq2 * ddq3 * lx3 * sin(q3) +
            2 * L1 * d3q1 * dq1 * lx2 * sin(q2) - 2 * L2 * d3q1 * dq3 * lx3 * sin(q3) -
            2 * L2 * d3q3 * dq1 * lx3 * sin(q3) - 2 * L2 * d3q2 * dq3 * lx3 * sin(q3) -
            2 * L2 * d3q3 * dq2 * lx3 * sin(q3) - 2 * L2 * d3q3 * dq3 * lx3 * sin(q3) -
            L1 * dq1 * dq1 * dq2 * dq2 * lx3 * sin(q1 + q2 + q3) -
            L1 * dq1 * dq1 * dq3 * dq3 * lx3 * sin(q1 + q2 + q3) + 4 * L1 * ddq1 * dq1 * dq2 * lx2 * cos(q2) -
            6 * L2 * ddq3 * dq1 * dq3 * lx3 * cos(q3) - 6 * L2 * ddq3 * dq2 * dq3 * lx3 * cos(q3) +
            4 * L1 * ddq1 * dq1 * dq2 * lx3 * cos(q1 + q2 + q3) + 4 * L1 * ddq1 * dq1 * dq3 * lx3 * cos(q1 + q2 + q3) -
            2 * L1 * dq1 * dq1 * dq2 * dq3 * lx3 * sin(q1 + q2 + q3),
            lx3 *
            (2 * L1 * ddq1 * ddq1 * sin(q2 + q3) + 2 * L2 * ddq1 * ddq1 * sin(q3) + 2 * L2 * ddq2 * ddq2 * sin(q3) -
             L2 * dq1 * dq1 * dq3 * dq3 * sin(q3) - L2 * dq2 * dq2 * dq3 * dq3 * sin(q3) +
             2 * L1 * d3q1 * dq1 * sin(q2 + q3) + 4 * L2 * ddq1 * ddq2 * sin(q3) + 2 * L2 * d3q1 * dq1 * sin(q3) +
             2 * L2 * d3q1 * dq2 * sin(q3) + 2 * L2 * d3q2 * dq1 * sin(q3) + 2 * L2 * d3q2 * dq2 * sin(q3) +
             L1 * ddq2 * dq1 * dq1 * cos(q2 + q3) + L1 * ddq3 * dq1 * dq1 * cos(q2 + q3) +
             L2 * ddq3 * dq1 * dq1 * cos(q3) + L2 * ddq3 * dq2 * dq2 * cos(q3) -
             L1 * dq1 * dq1 * dq2 * dq2 * sin(q2 + q3) - L1 * dq1 * dq1 * dq3 * dq3 * sin(q2 + q3) -
             2 * L2 * dq1 * dq2 * dq3 * dq3 * sin(q3) + 4 * L1 * ddq1 * dq1 * dq2 * cos(q2 + q3) +
             4 * L1 * ddq1 * dq1 * dq3 * cos(q2 + q3) + 4 * L2 * ddq1 * dq1 * dq3 * cos(q3) +
             4 * L2 * ddq1 * dq2 * dq3 * cos(q3) + 4 * L2 * ddq2 * dq1 * dq3 * cos(q3) +
             2 * L2 * ddq3 * dq1 * dq2 * cos(q3) + 4 * L2 * ddq2 * dq2 * dq3 * cos(q3) -
             2 * L1 * dq1 * dq1 * dq2 * dq3 * sin(q2 + q3));

    Eigen::Vector3d dG, ddG;

    dG << dq2 * (grav_acc * lx3 * cos(q1 + q2 + q3) + grav_acc * lx2 * cos(q1 + q2)) +
          dq1 * (grav_acc * lx1 * cos(q1) + grav_acc * lx3 * cos(q1 + q2 + q3) + grav_acc * lx2 * cos(q1 + q2)) +
          dq3 * grav_acc * lx3 * cos(q1 + q2 + q3),
            dq1 * (grav_acc * lx3 * cos(q1 + q2 + q3) + grav_acc * lx2 * cos(q1 + q2)) +
            dq2 * (grav_acc * lx3 * cos(q1 + q2 + q3) + grav_acc * lx2 * cos(q1 + q2)) +
            dq3 * grav_acc * lx3 * cos(q1 + q2 + q3),
            grav_acc * lx3 * cos(q1 + q2 + q3) * (dq1 + dq2 + dq3);

    ddG << ddq1 * grav_acc * lx2 * cos(q1 + q2) - dq2 * dq2 * grav_acc * lx3 * sin(q1 + q2 + q3) -
           dq3 * dq3 * grav_acc * lx3 * sin(q1 + q2 + q3) - dq1 * dq1 * grav_acc * lx3 * sin(q1 + q2 + q3) +
           ddq2 * grav_acc * lx2 * cos(q1 + q2) + ddq1 * grav_acc * lx1 * cos(q1) -
           dq1 * dq1 * grav_acc * lx2 * sin(q1 + q2) - dq2 * dq2 * grav_acc * lx2 * sin(q1 + q2) +
           ddq1 * grav_acc * lx3 * cos(q1 + q2 + q3) + ddq2 * grav_acc * lx3 * cos(q1 + q2 + q3) +
           ddq3 * grav_acc * lx3 * cos(q1 + q2 + q3) - dq1 * dq1 * grav_acc * lx1 * sin(q1) -
           2 * dq1 * dq2 * grav_acc * lx3 * sin(q1 + q2 + q3) - 2 * dq1 * dq3 * grav_acc * lx3 * sin(q1 + q2 + q3) -
           2 * dq2 * dq3 * grav_acc * lx3 * sin(q1 + q2 + q3) - 2 * dq1 * dq2 * grav_acc * lx2 * sin(q1 + q2),
            ddq1 * grav_acc * lx2 * cos(q1 + q2) - dq2 * dq2 * grav_acc * lx3 * sin(q1 + q2 + q3) -
            dq3 * dq3 * grav_acc * lx3 * sin(q1 + q2 + q3) - dq1 * dq1 * grav_acc * lx3 * sin(q1 + q2 + q3) +
            ddq2 * grav_acc * lx2 * cos(q1 + q2) - dq1 * dq1 * grav_acc * lx2 * sin(q1 + q2) -
            dq2 * dq2 * grav_acc * lx2 * sin(q1 + q2) + ddq1 * grav_acc * lx3 * cos(q1 + q2 + q3) +
            ddq2 * grav_acc * lx3 * cos(q1 + q2 + q3) + ddq3 * grav_acc * lx3 * cos(q1 + q2 + q3) -
            2 * dq1 * dq2 * grav_acc * lx3 * sin(q1 + q2 + q3) - 2 * dq1 * dq3 * grav_acc * lx3 * sin(q1 + q2 + q3) -
            2 * dq2 * dq3 * grav_acc * lx3 * sin(q1 + q2 + q3) - 2 * dq1 * dq2 * grav_acc * lx2 * sin(q1 + q2),
            ddq1 * grav_acc * lx3 * cos(q1 + q2 + q3) + ddq2 * grav_acc * lx3 * cos(q1 + q2 + q3) +
            ddq3 * grav_acc * lx3 * cos(q1 + q2 + q3) - dq1 * grav_acc * lx3 * sin(q1 + q2 + q3) * (dq1 + dq2 + dq3) -
            dq2 * grav_acc * lx3 * sin(q1 + q2 + q3) * (dq1 + dq2 + dq3) -
            dq3 * grav_acc * lx3 * sin(q1 + q2 + q3) * (dq1 + dq2 + dq3);


    Eigen::Matrix3d dMR, ddMR;

    dMR << -2 * L1 * L2 * m3_r * sin(q2) * dq2, -L1 * L2 * m3_r * sin(q2) * dq2, 0,
            -L1 * L2 * m3_r * sin(q2) * dq2, 0, 0,
            0, 0, 0;

    ddMR << -2 * L1 * L2 * m3_r * (cos(q2) * dq2 * dq2 + sin(q2) * ddq2), -L1 * L2 * m3_r *
                                                                          (cos(q2) * dq2 * dq2 + sin(q2) * ddq2), 0,
            -L1 * L2 * m3_r * (cos(q2) * dq2 * dq2 + sin(q2) * ddq2), 0, 0,
            0, 0, 0;


    Eigen::Matrix3d dH, ddH;

    dH = dM + dMR;
    ddH = ddM + ddMR;

    dtheta_d = K.inverse() * (H * d3q + dH * ddq + dC + dG + K * dq);

    ddtheta_d = K.inverse() * (H * d4q + 2 * dH * d3q + ddC + ddG + (ddH + K) * ddq);

    Eigen::Vector3d tau_ff;
//    tau_ff = coupling_dynamics(ddq, dq, q, ddtheta_d, dtheta_d);

    tau_ff = Gravity + Friction_link(dq);

    return tau_ff;
}

dynamics::dynamics(bool part) {
    // left = 1;
    // right = 0;
    if (part) {
        L1zz = 0.8862;
        lx1 = 2.8132;
        L2zz = 0.5833;
        lx2 = 1.9342;
        L3zz = -0.0387;
        lx3 = 0.1282;
        m1 = 4.52;
        m2 = 1.95;
        m3 = 0.95 + 4.09;
        m1_r = 0.37;
        m2_r = 0.37;
        m3_r = 0.23;
        N1 = 120;
        N2 = 80;
        N3 = 80;
        grav_acc = 9.81;
        fv1 = 1.3929;
        fc1 = 0.1743;
        fv2 = -1.4827;
        fc2 = 1.5793;
        fv3 = -0.0114;
        fc3 = 0.0365;
        fv1_m = 3.7436;
        fc1_m = 9.9992;
        fv2_m = 2.7897;
        fc2_m = 2.1693;
        fv3_m = -3.4303;
        fc3_m = 2.5243;
        L1 = 0.4;
        L2 = 0.395;
        Im1 = 1.1717;
        Im2 = 0.9954;
        Im3 = 0.1865;
    } else {
        L1zz = 0.6183;
        lx1 = 2.0460;
        L2zz = 0.1867;
        lx2 = 1.0420;
        L3zz = 0.0002;
        lx3 = 0.1872;
        m1 = 4.52;
        m2 = 1.95;
        m3 = 0.95 + 3.23;
        m1_r = 0.37;
        m2_r = 0.37;
        m3_r = 0.23;
        N1 = 120;
        N2 = 80;
        N3 = 80;
        grav_acc = 9.81;
        fv1 = 0.5281;
        fc1 = 0.2853;
        fv2 = -1.5621;
        fc2 = 0.5547;
        fv3 = -0.4610;
        fc3 = 0.1616;
        fv1_m = -8.9757;
        fc1_m = 8.5407;
        fv2_m = -0.0162;
        fc2_m = 4.1907;
        fv3_m = 4.3321;
        fc3_m = 3.8353;
        L1 = 0.4;
        L2 = 0.395;
        Im1 = 1.3238;
        Im2 = 0.8411;
        Im3 = 0.2316;
    }

}

double dynamics::get_para() {
    return this->Im1;
}

DiscretedMatrix dynamics::Discretization(Eigen::Matrix3d A, Eigen::Vector3d B, double Ts) {
    DiscretedMatrix discretedMatrix;
    Eigen::MatrixXd A_exp;
    A_exp = (A*Ts).exp();
    int nx = (B).rows();
    int n =4;
    double h = Ts/n;
    int Coef = 2;
    Eigen::Matrix3d Ai = Eigen::Matrix3d::Identity(nx,nx) + A_exp;
    Eigen::Vector3d Bd;

    for (int i = 1; i < n; i++) {
        if(Coef == 2)
            Coef = 4;
        else
            Coef = 2;

        Ai += Coef * (A*i*h).exp();
    }

    Bd = (h/3) * Ai * B;

    discretedMatrix.Ad = A_exp;
    discretedMatrix.Bd = Bd;

    return discretedMatrix;
}

#endif //MATRIX_DYNAMICS_DYNAMICS_H
