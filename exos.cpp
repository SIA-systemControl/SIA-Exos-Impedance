//
// Created by yc on 2021/6/25.
// Last modified on 2021/8/26
// TODO:
//  (1) summary the polarity of each encoder ( for next-gen Exos )
//  (2) generate right leg gait (done)
//  (3) check right Velocity Filter.
//

#include "exos.h"
#include "dataDecode.h"

//#define SineWave
#define Right_Part
#define asynchronous_enable

bool EtherCAT_ONLINE = true;
bool POST_RESET = false;
bool RIGHT_GC_START = false;
bool FLG_LEFT_GC_1st = false;
bool FLG_RIGHT_GC_1st = false;
bool FLG_LEFT_GC_end = false;

std::ofstream left_outFile;
std::ofstream left_velFile;
std::ofstream left_save_file;

std::ofstream left_filter_file;
std::ofstream right_filter_file;

std::vector<unsigned int> error_code_sequence;

#ifdef Right_Part
std::ofstream right_outFile;
std::ofstream right_velFile;
std::ofstream right_save_file;
#endif

std::ofstream gaitData;

/**
 * vel_des = Kp * (q_d - q_c)
 */
PID_position l_Hip_reset_pid(1400, 0, 300);
PID_position l_Knee_reset_pid(1200, 0, 250);
PID_position l_Ankle_reset_pid(850, 0, 400);

PID_position r_Hip_reset_pid(1400, 0, 350);
PID_position r_Knee_reset_pid(1500, 0, 200);
PID_position r_Ankle_reset_pid(1500, 0, 200);

PID_position l_Hip_pd(0, 0, 0);
PID_position l_Knee_pd(0, 0, 0);
PID_position l_Ankle_pd(0, 0, 0);

dynamics left_SEA_dynamics(left);

double *input_1 = new double[100];
double *output_1 = new double[100];
double *input_2 = new double[100];
double *output_2 = new double[100];
double *input_3 = new double[100];
double *output_3 = new double[100];
Eigen::Vector3d d4q_l, d3q_l, ddq_l, dq_l, q_l;
Eigen::Vector3d dq_e_l, q_e_l;

#ifdef Right_Part
PID_position r_Hip_pd(0, 0, 0);
PID_position r_Knee_pd(0, 0, 0);
PID_position r_Ankle_pd(0, 0, 0);
double *input_4 = new double[100];
double *output_4 = new double[100];
double *input_5 = new double[100];
double *output_5 = new double[100];
double *input_6 = new double[100];
double *output_6 = new double[100];
Eigen::Vector3d d4q_r, d3q_r, ddq_r, dq_r, q_r;
Eigen::Vector3d dq_e_r, q_e_r;
dynamics right_SEA_dynamics(right);
#endif

Eigen::Matrix3d Select_Matrix_d;

Eigen::Vector3d tau;

Eigen::IOFormat PrettyPrint(4, 0, ",", "\n", "[", "]", "[", "]");

double freq = 1;

ButterworthLP bw_link = ButterworthLP(1000, 5, 2);

static int SIG_cnt = 0;

double Gc_main = 0;
double Gc_sub = 0;
static int Gc_cnt = 0;

Eigen::Matrix2d GaitMatrix;

static void SIG_handle(int sig) {
    if (sig == SIGINT) {
        SIG_cnt++;
        if (gTaskFsm.m_gtaskFSM != task_working_Control)
            EtherCAT_ONLINE = false;
        POST_RESET = true;
        if (SIG_cnt == 1)
            std::cout << "[post-reset working...]" << std::endl;
    }
}


void *robotcontrol(void *arg) {

    gSysRunning.m_gWorkStatus = sys_working_POWER_ON;
    gTaskFsm.m_gtaskFSM = task_working_RESET;

    double Kp = 0;
    double Kd = 0;

    struct PD_para *recv_Para;
    recv_Para = (struct PD_para *) arg;
    Kp = (*recv_Para).Kp;
    Kd = (*recv_Para).Kd;

    if (gSysRunning.m_gWorkStatus == sys_working_POWER_ON) {
        ActivateMaster();
        ecstate = 0;
        gSysRunning.m_gWorkStatus = sys_working_SAFE_MODE;
        std::cout << "sys_working_SAFE_MODE." << std::endl;
    }

    ecrt_master_receive(master);

    left_outFile.open("left_FourierTest.csv", std::ios::out);
    left_outFile << "ref-hip" << ',' << "ref-knee" << ',' << "ref-ankle" << ','
                 << "link-hip" << ',' << "link-knee" << ',' << "link-ankle" << ','
                 << "motor-hip" << ',' << "motor-knee" << ',' << "motor-ankle" << ','
                 << "tau-m1" << ',' << "tau-m2" << ',' << "tau-m3" << ','
                 << "tau-dyn-1" << ',' << "tau-dyn-2" << ',' << "tau-dyn-3" << std::endl;

    left_velFile.open("left_FourierVel.csv", std::ios::out);
    left_velFile << "ref-hip" << ',' << "ref-knee" << ',' << "ref-ankle" << ','
                 << "link-hip" << ',' << "link-knee" << ',' << "link-ankle" << ','
                 << "motor-hip" << ',' << "motor-knee" << ',' << "motor-ankle" << ','
                 << "tau-m1" << ',' << "tau-m2" << ',' << "tau-m3" << std::endl;

    left_save_file.open("left_save.csv", std::ios::out);
    left_save_file << "Hip desired" << ',' << "Hip actual" << ',' << "Knee desired" << ',' << "Knee actual" << ','
                   << "Ankle desired" << ',' << "Ankle actual" << ',' << "deformation of ankle" << std::endl;

    left_filter_file.open("left_filter.csv", std::ios::out);
    left_filter_file << "original" << ',' << "cpp filter" << std::endl;

#ifdef Right_Part
    right_outFile.open("right_FourierTest.csv", std::ios::out);
    right_outFile << "ref-hip" << ',' << "ref-knee" << ',' << "ref-ankle" << ','
                  << "link-hip" << ',' << "link-knee" << ',' << "link-ankle" << ','
                  << "motor-hip" << ',' << "motor-knee" << ',' << "motor-ankle" << ','
                  << "tau-m1" << ',' << "tau-m2" << ',' << "tau-m3" << ','
                  << "tau-dyn-1" << ',' << "tau-dyn-2" << ',' << "tau-dyn-3" << std::endl;

    right_velFile.open("right_FourierVel.csv", std::ios::out);
    right_velFile << "ref-hip" << ',' << "ref-knee" << ',' << "ref-ankle" << ','
                  << "link-hip" << ',' << "link-knee" << ',' << "link-ankle" << ','
                  << "motor-hip" << ',' << "motor-knee" << ',' << "motor-ankle" << ','
                  << "tau-m1" << ',' << "tau-m2" << ',' << "tau-m3" << std::endl;

    right_save_file.open("right_save.csv", std::ios::out);
    right_save_file << "Hip desired" << ',' << "Hip actual" << ',' << "Knee desired" << ',' << "Knee actual" << ','
                    << "Ankle desired" << ',' << "Ankle actual" << ',' << "deformation of ankle" << std::endl;

    right_filter_file.open("right_filter.csv", std::ios::out);
    right_filter_file << "original" << ',' << "cpp filter" << std::endl;
#endif

    gaitData.open("gaitData.csv", std::ios::out);

    GaitMatrix << 1, 1, 1, 1;
    gMFSM.m_gaitMatrixFsm = Stance;

    while (EtherCAT_ONLINE) {
        if (gSysRunning.m_gWorkStatus == sys_woring_INIT_Failed) {
            EtherCAT_ONLINE = false;
            break;
            std::cout << "Slave(s) initialized failed." << std::endl;
        }
        signal(SIGINT, SIG_handle);
        usleep(1000000 / TASK_FREQUENCY);
        cyclic_task(Kp, Kd);
    }
    releaseMaster();

    left_outFile.close();
    left_velFile.close();
    left_save_file.close();
    left_filter_file.close();

#ifdef Right_Part
    right_outFile.close();
    right_velFile.close();
    right_save_file.close();
#endif

    gaitData.close();

    pthread_exit(nullptr);
}

void cyclic_task(double Kp, double Kd) {
    static int current_pos = 0;
    int raw_init_motor_pos[joint_num] = {0, 0, 0, 0, 0, 0};
    int raw_init_spring_pos[joint_num] = {0, 0, 0, 0, 0, 0};

    if (gSysRunning.m_gWorkStatus == sys_working_POWER_ON)
        return;

    static int cycle_count = 0;
    cycle_count++;

    ecrt_master_receive(master);
    ecrt_domain_process(domainRx);
    ecrt_domain_process(domainTx);

    check_domain_state();

    if (!(cycle_count % 500)) {
        check_master_state();
        check_slave_config_states();
    }

    switch (gSysRunning.m_gWorkStatus) {
        case sys_working_SAFE_MODE: {
            check_master_state();
            check_slave_config_states();

            if ((master_state.al_states & ETHERCAT_STATUS_OP)) {
                bool tmp = true;

                for (int i = 0; i < active_num; i++) {
                    if (!(sc_state[i].al_state & ETHERCAT_STATUS_OP)) {
                        std::cout << "slave " << i << " al_state: " << sc_state[i].al_state << std::endl;
                        tmp = false;
                        gSysRunning.m_gWorkStatus = sys_woring_INIT_Failed;
                        break;
                    }
                }
                if (tmp) {
                    ecstate = 0;
                    gSysRunning.m_gWorkStatus = sys_working_OP_MODE;
                    std::cout << "sys_working_OP_MODE" << std::endl;
                }
            }
        }
            break;

        case sys_working_OP_MODE: {
            ecstate++;
            if (SERVE_OP < active_num) { // Traverse all slave(s)
                if (ecstate <= 10) {
                    switch (ecstate) {
                        case 1:
                            for (int i = 0; i < active_num; i++) {
                                int E_code = EC_READ_U16(domainTx_pd + offset.Error_code[i]);

                                if (E_code != 0 || (EC_READ_U16(domainTx_pd + offset.status_word[i]) & 0x0008)) {
                                    std::cout << "[Error] occured at slave: " << i << std::endl;
                                    EC_WRITE_U16(domainRx_pd + offset.ctrl_word[i],
                                                 (EC_READ_U16(domainTx_pd + offset.status_word[i]) |
                                                  0x0080));
                                } // bit7 set to 1 to reset fault
                                EC_WRITE_U8(domainRx_pd + offset.operation_mode[i], CSP);
                            }
                            break;
                        case 7:
                            for (int i = 0; i < active_num; i++) {
                                int cur_pos = EC_READ_S32(domainTx_pd + offset.actual_position[i]);
                                EC_WRITE_S32(domainRx_pd + offset.target_position[i], cur_pos);
                                std::cout << "Axis-" << i << " current position[cnt]: " << cur_pos << std::endl;
                            }
                            break;
                        default:
                            break;
                    }
                } else {
                    for (int i = 0; i < active_num; i++) {
                        unsigned int cur_status = EC_READ_U16(domainTx_pd + offset.status_word[i]);
                        if ((cur_status & 0x4f) == 0x40)
                            EC_WRITE_U16(domainRx_pd + offset.ctrl_word[i], 0x06);
                        else if ((cur_status & 0x6f) == 0x21)
                            EC_WRITE_U16(domainRx_pd + offset.ctrl_word[i], 0x07);
                        else if ((cur_status & 0x6f) == 0x23) {
                            EC_WRITE_U16(domainRx_pd + offset.ctrl_word[i], 0x0F);
                            SERVE_OP++;
                        }
                    }
                }
            } else { // all slave(s) change ethercat FSM to OP-enable, then check slave(s) status
                int tmp = true;
                for (int i = 0; i < active_num; i++) {
                    unsigned int cur_status = EC_READ_U16(domainTx_pd + offset.status_word[i]);
                    unsigned int E_code = EC_READ_U16(domainTx_pd + offset.Error_code[i]);

                    if ((EC_READ_U16(domainTx_pd + offset.status_word[i]) & 0x6f) == 0x27)
                        std::cout << "Slave [" << i << "] Enable operation" << std::endl;
                    else {
                        std::cout << "Slave [" << i << "] not in <Enable Operation>" << std::endl;
                        std::cout << "Slave [" << i << "] State: " << cur_status << std::endl;
                        std::cout << "Slave [" << i << "] E-code: " << E_code << std::endl;
                        ErrorParse(E_code);
                    }

                    if ((EC_READ_U16(domainTx_pd + offset.status_word[i]) & (STATUS_SERVO_ENABLE_BIT)) == 0) {
                        tmp = false;
                        ecstate = 0;
                        break;
                    }
                }
                if (tmp) {
                    ecstate = 0;
                    gSysRunning.m_gWorkStatus = sys_working_WORK_MODE;
                    std::cout << "sys_working_WORK_MODE" << std::endl;
                }
            }
        }
            break;

        case sys_working_WORK_MODE: {
            static int reset_ready_cnt = 0;
            static int id_ready_cnt = 0;

            static int PD_cnt = 0;
            static int reset_timeout = 3000;

            /** Preprocess of data */
            /**
             * Encoder cnt (Raw)
             */

            int raw_pos_motor_1 = EC_READ_S32(domainTx_pd + offset.actual_position[l_hip]);
            int raw_vel_motor_1 = EC_READ_S32(domainTx_pd + offset.actual_velocity[l_hip]);

            int raw_pos_spring_1 = EC_READ_S32(domainTx_pd + offset.second_position[l_hip]);
            int raw_vel_spring_1 = EC_READ_S32(domainTx_pd + offset.second_velocity[l_hip]);

            int raw_pos_motor_2 = EC_READ_S32(domainTx_pd + offset.actual_position[l_knee]);
            int raw_vel_motor_2 = EC_READ_S32(domainTx_pd + offset.actual_velocity[l_knee]);

            int raw_pos_spring_2 = EC_READ_S32(domainTx_pd + offset.second_position[l_knee]);
            int raw_vel_spring_2 = EC_READ_S32(domainTx_pd + offset.second_velocity[l_knee]);

            int raw_pos_motor_3 = EC_READ_S32(domainTx_pd + offset.actual_position[l_ankle]);
            int raw_vel_motor_3 = EC_READ_S32(domainTx_pd + offset.actual_velocity[l_ankle]);

            int raw_pos_encoder_3 = EC_READ_S32(domainTx_pd + offset.second_position[l_ankle]);
            int raw_vel_encoder_3 = EC_READ_S32(domainTx_pd + offset.second_velocity[l_ankle]);

            int absolute_pos_motor_1 = relative_encoder_cnt(raw_pos_motor_1, left_hip_init_motor_pos);
            int absolute_pos_spring_1 = relative_encoder_cnt(raw_pos_spring_1, left_hip_init_spring_pos);

            int absolute_pos_motor_2 = relative_encoder_cnt(raw_pos_motor_2, left_knee_init_motor_pos);
            int absolute_pos_spring_2 = relative_encoder_cnt(raw_pos_spring_2, left_knee_init_spring_pos);

            int absolute_pos_motor_3 = relative_encoder_cnt(raw_pos_motor_3, left_ankle_init_motor_pos);
            int absolute_pos_encoder_3 = relative_encoder_cnt(raw_pos_encoder_3, left_ankle_init_link_pos);

#ifdef Right_Part
            int raw_pos_motor_4 = EC_READ_S32(domainTx_pd + offset.actual_position[r_hip]);
            int raw_vel_motor_4 = EC_READ_S32(domainTx_pd + offset.actual_velocity[r_hip]);

            int raw_pos_spring_4 = EC_READ_S32(domainTx_pd + offset.second_position[r_hip]);
            int raw_vel_spring_4 = EC_READ_S32(domainTx_pd + offset.second_velocity[r_hip]);

            int raw_pos_motor_5 = EC_READ_S32(domainTx_pd + offset.actual_position[r_knee]);
            int raw_vel_motor_5 = EC_READ_S32(domainTx_pd + offset.actual_velocity[r_knee]);

            int raw_pos_spring_5 = EC_READ_S32(domainTx_pd + offset.second_position[r_knee]);
            int raw_vel_spring_5 = EC_READ_S32(domainTx_pd + offset.second_velocity[r_knee]);

            int raw_pos_motor_6 = EC_READ_S32(domainTx_pd + offset.actual_position[r_ankle]);
            int raw_vel_motor_6 = EC_READ_S32(domainTx_pd + offset.actual_velocity[r_ankle]);

            int raw_pos_encoder_6 = EC_READ_S32(domainTx_pd + offset.second_position[r_ankle]);
            int raw_vel_encoder_6 = EC_READ_S32(domainTx_pd + offset.second_velocity[r_ankle]);

            int absolute_pos_motor_4 = relative_encoder_cnt(raw_pos_motor_4, right_hip_init_motor_pos);
            int absolute_pos_spring_4 = relative_encoder_cnt(raw_pos_spring_4, right_hip_init_spring_pos);

            int absolute_pos_motor_5 = relative_encoder_cnt(raw_pos_motor_5, right_knee_init_motor_pos);
            int absolute_pos_spring_5 = relative_encoder_cnt(raw_pos_spring_5, right_knee_init_spring_pos);

            int absolute_pos_motor_6 = relative_encoder_cnt(raw_pos_motor_6, right_ankle_init_motor_pos);
            int absolute_pos_encoder_6 = relative_encoder_cnt(raw_pos_encoder_6, right_ankle_init_link_pos);
#endif

            /**
             * Data Processing
             */

            /**
             * Motor radians after reducer transmission
             */
            double lever_arm_1_rad = pos_inc2rad_17bits(absolute_pos_motor_1 / transmission_hip);
            double lever_arm_2_rad = pos_inc2rad_17bits(absolute_pos_motor_2 / transmission_knee);
            double lever_arm_3_rad = pos_inc2rad_17bits(absolute_pos_motor_3 / transmission_ankle);
#ifdef Right_Part
            double lever_arm_4_rad = pos_inc2rad_17bits(absolute_pos_motor_4 / transmission_hip);
            double lever_arm_5_rad = pos_inc2rad_17bits(absolute_pos_motor_5 / transmission_knee);
            double lever_arm_6_rad = pos_inc2rad_17bits(absolute_pos_motor_6 / transmission_ankle);
#endif

            /**
             * Link radians ( alpha = theta - q_l ---> q_l = theta - alpha)
             * Here speeder_ankle is 140:40 caused by encoder Pulley
             */

            double link_1_rad = lever_arm_1_rad - pos_inc2rad_17bits(absolute_pos_spring_1);
            double link_2_rad = lever_arm_2_rad + pos_inc2rad_17bits(absolute_pos_spring_2);
            double link_3_rad = speeder_ankle * pos_inc2rad_13bits(absolute_pos_encoder_3);
#ifdef Right_Part
            double link_4_rad = lever_arm_4_rad - pos_inc2rad_17bits(absolute_pos_spring_4);
            double link_5_rad = lever_arm_5_rad - pos_inc2rad_17bits(absolute_pos_spring_5);
            double link_6_rad = speeder_ankle * pos_inc2rad_13bits(absolute_pos_encoder_6);
#endif

            /**
             * Velocity [RPM]
             */
            double lever_arm_1_vel = raw_vel_motor_1 / transmission_hip;
            double lever_arm_2_vel = raw_vel_motor_2 / transmission_knee;
            double lever_arm_3_vel = raw_vel_motor_3 / transmission_ankle;

            double link_1_vel = lever_arm_1_vel - raw_vel_spring_1;
            double link_2_vel = lever_arm_2_vel +
                                raw_vel_spring_2; // TODO: minus/plus is decided by the configuration of encoder polarity
            double link_3_vel = speeder_ankle * raw_vel_encoder_3;
#ifdef Right_Part
            double lever_arm_4_vel = raw_vel_motor_4 / transmission_hip;
            double lever_arm_5_vel = raw_vel_motor_5 / transmission_knee;
            double lever_arm_6_vel = raw_vel_motor_6 / transmission_ankle;

            double link_4_vel = lever_arm_4_vel - raw_vel_spring_4;
            double link_5_vel = lever_arm_5_vel - raw_vel_spring_5;
            double link_6_vel = speeder_ankle * raw_vel_encoder_6;
#endif

            /**
             * Per thousand of Rated Torque
             */
            double tau_mot_1 = EC_READ_S16(domainTx_pd + offset.actual_torque[l_hip]);
            double tau_mot_2 = EC_READ_S16(domainTx_pd + offset.actual_torque[l_knee]);
            double tau_mot_3 = EC_READ_S16(domainTx_pd + offset.actual_torque[l_ankle]);
            double tau_mot_4 = EC_READ_S16(domainTx_pd + offset.actual_torque[r_hip]);
            double tau_mot_5 = EC_READ_S16(domainTx_pd + offset.actual_torque[r_knee]);
            double tau_mot_6 = EC_READ_S16(domainTx_pd + offset.actual_torque[r_ankle]);

            /** Task FSM = (RESET = 0, CONTROL = 1)
             *   =================
             *   | ---- [Loop 2] ---- |
             *   |                    v
             *   0 --> 1 -[Loop 1]-> EOP
             *   ^     |
             *   | <-- |
             *   =================
             *   Loop 1 := Normal processing still running to prescribed time-stamp
             *   Loop 2 := Interrupt by Ctrl-C and then process post-reset
             */

            switch (gTaskFsm.m_gtaskFSM) {
                case task_working_RESET: {

                    /** A. Reset to perscribed motor position (vertical direction as reference zero [rad] ) */
                    if (!(cycle_count % 10)) { // show message per 1 ms.
                        std::cout << "=====================" << std::endl;
                        std::cout << "task_working_RESET" << std::endl;
                        std::cout << "Time : " << PD_cnt / TASK_FREQUENCY << "[s]" << std::endl;
                        std::cout << "=====================" << std::endl;
//                  Following output can be used for [Calibrating]
//                        std::cout << "spr cnt: " << EC_READ_S32(domainTx_pd + offset.second_position[r_knee])
//                                  << std::endl;
//                        std::cout << "motor cnt:" << EC_READ_S32(domainTx_pd + offset.actual_position[r_knee])
//                                  << std::endl;

                        for (int i = 0; i < active_num; i++) {
                            unsigned int E_code = EC_READ_U16(domainTx_pd + offset.Error_code[i]);
                            ErrorParse(E_code);
                        }
                    }

                    if (reset_step == 0) {
                        Change_one_side_OP_mode(left, CSV);
#ifdef Right_Part
                        Change_one_side_OP_mode(right, CSV);
#endif
                        reset_step = 1;
                    }
                    /**
                     *  reset and hold reset-point
                     */
                    if (reset_step == 1 &&
                        reset_ready_cnt != 0) { // reset_ready_cnt != 0 avoid assigning velocity in CSP mode.

                        double reset_l_hip_vel = l_Hip_reset_pid.pid_control(left_hip_id_init_rad,
                                                                             link_1_rad, 2200);
                        double reset_l_knee_vel = l_Knee_reset_pid.pid_control(left_knee_id_init_rad,
                                                                               link_2_rad, 2400);
                        double reset_l_ankle_vel = l_Ankle_reset_pid.pid_control(left_ankle_id_init_rad, link_3_rad,
                                                                                 2200);

//                        EC_WRITE_S32(domainRx_pd + offset.target_velocity[l_hip], reset_l_hip_vel);
//                        EC_WRITE_S32(domainRx_pd + offset.target_velocity[l_knee], reset_l_knee_vel);
                        Assign_slave_Velocity(l_hip, reset_l_hip_vel);
                        Assign_slave_Velocity(l_knee, reset_l_knee_vel);
                        Assign_slave_Velocity(l_ankle, reset_l_ankle_vel);

#ifdef Right_Part
                        double reset_r_hip_vel = r_Hip_reset_pid.pid_control(right_hip_id_init_rad,
                                                                             link_4_rad, 2200);
                        double reset_r_knee_vel = r_Knee_reset_pid.pid_control(right_knee_id_init_rad,
                                                                               link_5_rad, 2400);
                        double reset_r_ankle_vel = r_Ankle_reset_pid.pid_control(right_ankle_id_init_rad,
                                                                                 link_6_rad,
                                                                                 2200);
                        Assign_slave_Velocity(r_hip, reset_r_hip_vel);
                        Assign_slave_Velocity(r_knee, reset_r_knee_vel);
                        Assign_slave_Velocity(r_ankle, reset_r_ankle_vel);
#endif

                        if (PD_cnt++ > reset_timeout) {
                            // if timeout, force switch state machine.
                            Assign_side_slaves_zero_velocity(left);
#ifdef Right_Part
                            Assign_side_slaves_zero_velocity(right);
#endif
                            if (SIG_cnt == 0) {
                                gTaskFsm.m_gtaskFSM = task_working_Control; //  Task FSM: [0] -> [1]
                                reset_step = 0;
                                reset_ready_cnt = 0;
                                PD_cnt = 0;
                                reset_timeout = 3000;
                            } else
                                EtherCAT_ONLINE = false;
                        }
                    }
                    reset_ready_cnt++;
                }
                    break;

                case task_working_Control: {

                    /**
                     * obtain GRF information and detect gait phase
                     */
                    unsigned int An_Left_1 = EC_READ_U16(domainTx_pd + offset.analog_in1[l_hip]);
                    unsigned int An_Left_2 = EC_READ_U16(domainTx_pd + offset.analog_in2[l_hip]);
                    unsigned int An_Right_1 = EC_READ_U16(domainTx_pd + offset.analog_in1[r_hip]);
                    unsigned int An_Right_2 = EC_READ_U16(domainTx_pd + offset.analog_in2[r_hip]);

                    GaitMatrix = UpdateGaitMatrix(An_Left_1, An_Left_2, An_Right_1, An_Right_2);
                    int GaitPhase = GaitMatrixParse(GaitMatrix);

                    /** B. Assign operation to motor(s) */

                    /** count for generated curve */
                    static int curve_count_main = 0;   // [ms]
                    static int curve_count_sub = 0;   //  [ms]
#ifdef SineWave
                    //                    if(!(curve_count_main % 1000))
                    //                        freq += 0.5;

                    double hip_ref_rad = cosineWave(curve_count_main, freq, 0.1);
                    double knee_ref_rad = cosineWave(curve_count_main, freq, 0);
                    double ankle_ref_rad = cosineWave(curve_count_main, freq, 0.2);

                    double hip_ref_vel = 2 * Pi * freq * sineWave(curve_count_main, freq, -0.1);
                    double knee_ref_vel = 2 * Pi * freq * sineWave(curve_count_main, freq, 0);
                    double ankle_ref_vel = 2 * Pi * freq * sineWave(curve_count_main, freq, -0.2);

                    double hip_ref_acc = pow(2 * Pi * freq, 2) * cosineWave(curve_count_main, freq, -0.1);
                    double knee_ref_acc = pow(2 * Pi * freq, 2) * cosineWave(curve_count_main, freq, -0);
                    double ankle_ref_acc = pow(2 * Pi * freq, 2) * cosineWave(curve_count_main, freq, 0.2);
#else
                    double beta = 0.8;

                    // 1st Gc start at 40%
                    // --> a little bit early than TO
                    // --> <TO,stance> == 45%
                    // Select offset = 5%
                    // TODO: accurate offset is necessary?

                    /**
                     *  when FLG_LEFT_GC_1st == false
                     *  --> corresponding timing of std.Gc is about 40%, so the offset of curve is it.
                     *  --> when Gc is over 100%, there are 2 case:
                     *  ----> (Gait Phase == HS_HO) indicates next Gc is beginning, reset curve_count and switch gMFSM to HS_HO
                     *  ----> (Gait Phase != HS_HO) indicates the desired trajectory of Exos is ended but the motion of human limb isn't, desired trajectory must pause.
                     *  TODO: Here I used GaitPhase to detect HSHO phase first, rather than gMFSM
                     *        the difference of GaitPhase and gMFSM is that:
                     *        --> GaitPhase is real-time and is not in sequence
                     *        --> gMFSM is strictly in sequence
                     *        <- the (if-else) FLG_LEFT_GC_1st operation does need gMFSM ?? ->
                     *        The current logic is that select first HS_HO as gMFSM start-point
                     *        however the 2xstance FSM doesn't use.
                     */

                    if (!FLG_LEFT_GC_1st) { // first forward step
                        Gc_main = 1.0 * (curve_count_main % P) / P + 0.4;
                        if (Gc_main >= 1.0) {
//                            if (GaitPhase == HSHO) {
                            // means left start new GC
                            Gc_main = 0;
                            FLG_LEFT_GC_1st = true;
                            RIGHT_GC_START = true;
                            curve_count_main = 0;
//                            gMFSM.m_gaitMatrixFsm = HS_HO;
                        }
//                            else if (GaitPhase != HSHO)
//                                curve_count_main--;
//                        }
                    } else { // normal cyclic walking
                        /**
                         * when FLG_LEFT_GC_1st == true
                         * indicates that the gMFSM changed from [2xstance] to [HS_HO]
                         * the normal Gc is start. since the mod operator(%), the Gc_main will not reach 1.0 ( rather than 0.0 )
                         * Detectint event:
                         * --> when (Gc_main == 0.0), dose (gMFSM == HS_HO)?
                         * ----> if not -->  hold curve_count;
                         * ----> else   -->  enter next Gait cycle
                         * Since the gFSM is strictly in sequence. the next HS_HO indicates last GC is completed.
                         */
                        Gc_main = 1.0 * (curve_count_main % P) / P;
                        if (Gc_main == 0.0)
//                            if (gMFSM.m_gaitMatrixFsm != HS_HO)  // waiting for <left>[Heel Strike] occur.
                            curve_count_main = 0;
                    }

                    curve_count_main++;

                    /** Gait Phase FSM **/
//                    switch (GaitPhase) {
//                        case HSHO:
//                            if (gMFSM.m_gaitMatrixFsm == S_ST) {
//                                FLG_LEFT_GC_end = true;
//                                gMFSM.m_gaitMatrixFsm = HS_HO;
//                            }
//                            break;
//                        case STO: {
//                            if ((gMFSM.m_gaitMatrixFsm == HS_HO) || (gMFSM.m_gaitMatrixFsm == Stance))
//                                gMFSM.m_gaitMatrixFsm = ST_T;
//                        }
//                            break;
//                        case SSw:
//                            if (gMFSM.m_gaitMatrixFsm == ST_T)
//                                gMFSM.m_gaitMatrixFsm = ST_S;
//                            break;
//                        case HOHS:
//                            if (gMFSM.m_gaitMatrixFsm == ST_S)
//                                gMFSM.m_gaitMatrixFsm = HO_HS;
//                            break;
//                        case TOS:
//                            if ((gMFSM.m_gaitMatrixFsm == HO_HS) || (gMFSM.m_gaitMatrixFsm == Stance))
//                                gMFSM.m_gaitMatrixFsm = T_ST;
//                            break;
//                        case SwS:
//                            if (gMFSM.m_gaitMatrixFsm == T_ST)
//                                gMFSM.m_gaitMatrixFsm = S_ST;
//                            break;
//                        case doubleStance:
//                            if ((gMFSM.m_gaitMatrixFsm == ST_S) || (gMFSM.m_gaitMatrixFsm == S_ST))
//                                gMFSM.m_gaitMatrixFsm = Stance;
//                            break;
//                        default:
//                            break;
//                    }

                    if (!POST_RESET) {
                        double hip_ref_rad_l = 0.6 * base_Fourier_8th(Gc_main, a_hip, b_hip, w_hip, a0_hip);
                        double knee_ref_rad_l = 0.6 * base_Fourier_8th(Gc_main, a_knee, b_knee, w_knee, a0_knee);
                        double ankle_ref_rad_l = 1.0 * base_Fourier_8th(Gc_main, a_ankle, b_ankle, w_ankle, a0_ankle);

                        double hip_ref_vel_l =
                                0.6 * differentia_1st_Fourier_8th(Gc_main, a_hip, b_hip, w_hip, a0_knee, P);
                        double knee_ref_vel_l =
                                0.6 * differentia_1st_Fourier_8th(Gc_main, a_knee, b_knee, w_knee, a0_knee, P);
                        double ankle_ref_vel_l =
                                1.0 * differentia_1st_Fourier_8th(Gc_main, a_ankle, b_ankle, w_ankle, a0_ankle, P);

                        double hip_ref_acc_l =
                                0.6 * differentia_2ed_Fourier_8th(Gc_main, a_hip, b_hip, w_hip, a0_hip, P);
                        double knee_ref_acc_l =
                                0.6 * differentia_2ed_Fourier_8th(Gc_main, a_knee, b_knee, w_knee, a0_knee, P);
                        double ankle_ref_acc_l =
                                1.0 * differentia_2ed_Fourier_8th(Gc_main, a_ankle, b_ankle, w_ankle, a0_ankle, P);

                        double hip_ref_3rd_l =
                                0.6 * differentia_3rd_Fourier_8th(Gc_main, a_hip, b_hip, w_hip, a0_hip, P);
                        double knee_ref_3rd_l =
                                0.6 * differentia_3rd_Fourier_8th(Gc_main, a_knee, b_knee, w_knee, a0_knee, P);
                        double ankle_ref_3rd_l =
                                1.0 * differentia_3rd_Fourier_8th(Gc_main, a_ankle, b_ankle, w_ankle, a0_ankle, P);

                        double hip_ref_4th_l =
                                0.6 * differentia_4th_Fourier_8th(Gc_main, a_hip, b_hip, w_hip, a0_hip, P);
                        double knee_ref_4th_l =
                                0.6 * differentia_4th_Fourier_8th(Gc_main, a_knee, b_knee, w_knee, a0_knee, P);
                        double ankle_ref_4th_l =
                                1.0 * differentia_4th_Fourier_8th(Gc_main, a_ankle, b_ankle, w_ankle, a0_ankle, P);

                        /**
                         * the configuration of Paradigm is [left first]
                         * --> which indicates the right motion must waiting left finished first Gc.
                         * --> when (FLG_LEFT_GC_1st == False)
                         * ----> hold 2xStance percent (40%) of right limb Gc ( here right limb share same gait curve prototype with left one)
                         * --> whne (FLG_LEFT_GC_1st == True)
                         * ----> repeat operation as left one.
                         */
                        if (FLG_LEFT_GC_1st) { // left first GC is finished
                            if (!FLG_RIGHT_GC_1st) { // right first GC is not finished
                                Gc_sub = 1.0 * (curve_count_sub % P) / P + 0.4; // offset = 40% Gc
                                if (Gc_sub >= 1.0) {
//                                    if (GaitPhase == HOHS) { // right heel strike occur
                                    Gc_sub = 0.0;
                                    FLG_RIGHT_GC_1st = true; // FLAG of the end of right 1st GC.
                                    curve_count_sub = 0;
//                                    }
//                                    else if (GaitPhase != HOHS)
//                                        curve_count_sub--; // holding
                                }
                            } else // right first GC is finished, enter normal walking
                            {
                                Gc_sub = 1.0 * (curve_count_sub % P) / P;
                                if (Gc_sub == 1.0 || Gc_sub == 0.0)
//                                    if (gMFSM.m_gaitMatrixFsm != HO_HS)  // wait for Heel strike occur.
                                    curve_count_sub = 0;
                            }
                        } else { // left first GC is not finished --> hold stance initial pose
                            Gc_sub = 0.4; // correspond to offset
                        }

                        if (FLG_LEFT_GC_1st)
                            curve_count_sub++;


                        double hip_ref_rad_r = 0.6 * base_Fourier_8th(Gc_sub, a_hip, b_hip, w_hip, a0_hip);
                        double knee_ref_rad_r = 0.6 * base_Fourier_8th(Gc_sub, a_knee, b_knee, w_knee, a0_knee);
                        double Ankle_ref_rad_r = base_Fourier_8th(Gc_sub, a_ankle, b_ankle, w_ankle, a0_ankle);

                        double hip_ref_vel_r =
                                0.6 * differentia_1st_Fourier_8th(Gc_sub, a_hip, b_hip, w_hip, a0_knee, P);
                        double knee_ref_vel_r =
                                0.6 * differentia_1st_Fourier_8th(Gc_sub, a_knee, b_knee, w_knee, a0_knee, P);
                        double Ankle_ref_vel_r =
                                differentia_1st_Fourier_8th(Gc_sub, a_ankle, b_ankle, w_ankle, a0_ankle, P);

                        double hip_ref_acc_r =
                                0.6 * differentia_2ed_Fourier_8th(Gc_sub, a_hip, b_hip, w_hip, a0_hip, P);
                        double knee_ref_acc_r =
                                0.6 * differentia_2ed_Fourier_8th(Gc_sub, a_knee, b_knee, w_knee, a0_knee, P);
                        double Ankle_ref_acc_r =
                                differentia_2ed_Fourier_8th(Gc_sub, a_ankle, b_ankle, w_ankle, a0_ankle, P);

                        double hip_ref_3rd_r =
                                0.6 * differentia_3rd_Fourier_8th(Gc_sub, a_hip, b_hip, w_hip, a0_hip, P);
                        double knee_ref_3rd_r =
                                0.6 * differentia_3rd_Fourier_8th(Gc_sub, a_knee, b_knee, w_knee, a0_knee, P);
                        double Ankle_ref_3rd_r =
                                differentia_3rd_Fourier_8th(Gc_sub, a_ankle, b_ankle, w_ankle, a0_ankle, P);

                        double hip_ref_4th_r =
                                0.6 * differentia_4th_Fourier_8th(Gc_sub, a_hip, b_hip, w_hip, a0_hip, P);
                        double knee_ref_4th_r =
                                0.6 * differentia_4th_Fourier_8th(Gc_sub, a_knee, b_knee, w_knee, a0_knee, P);
                        double Ankle_ref_4th_r =
                                differentia_4th_Fourier_8th(Gc_sub, a_ankle, b_ankle, w_ankle, a0_ankle, P);


#endif
                        /**
                         ** Filter Part
                         **/

                        double hip_l_vel, knee_l_vel, ankle_l_vel;
                        double hip_r_vel, knee_r_vel, ankle_r_vel;
                        int filter_window = 30;
                        int avg_window = 10;

                        if (curve_count_main < filter_window) {
                            if (curve_count_main < avg_window) {
                                input_1[curve_count_main] = link_1_vel;
                                input_2[curve_count_main] = link_2_vel;
                                input_3[curve_count_main] = link_3_vel;
#ifdef Right_Part
                                input_4[curve_count_main] = link_4_vel;
                                input_5[curve_count_main] = link_5_vel;
                                input_6[curve_count_main] = link_6_vel;
#endif
                            } else {
                                double tmp1 =
                                        sum_of_array(input_1, curve_count_main - avg_window, curve_count_main) /
                                        avg_window;
                                double tmp2 =
                                        sum_of_array(input_2, curve_count_main - avg_window, curve_count_main) /
                                        avg_window;
                                double tmp3 =
                                        sum_of_array(input_3, curve_count_main - avg_window, curve_count_main) /
                                        avg_window;
                                input_1[curve_count_main] = tmp1;
                                input_2[curve_count_main] = tmp2;
                                input_3[curve_count_main] = tmp3;
#ifdef Right_Part
                                double tmp4 =
                                        sum_of_array(input_4, curve_count_main - avg_window, curve_count_main) /
                                        avg_window;
                                double tmp5 =
                                        sum_of_array(input_5, curve_count_main - avg_window, curve_count_main) /
                                        avg_window;
                                double tmp6 =
                                        sum_of_array(input_6, curve_count_main - avg_window, curve_count_main) /
                                        avg_window;
                                input_4[curve_count_main] = tmp4;
                                input_5[curve_count_main] = tmp5;
                                input_6[curve_count_main] = tmp6;
#endif
                            }
                            hip_l_vel = input_1[curve_count_main];
                            knee_l_vel = input_2[curve_count_main];
                            ankle_l_vel = input_3[curve_count_main];
#ifdef Right_Part
                            hip_r_vel = input_4[curve_count_main];
                            knee_r_vel = input_5[curve_count_main];
                            ankle_r_vel = input_6[curve_count_main];
#endif
                        } else {
                            left_shift_array(input_1, link_1_vel, filter_window);
                            left_shift_array(input_2, link_2_vel, filter_window);
                            left_shift_array(input_3, link_3_vel, filter_window);
                            bw_link.filter(input_1, output_1, filter_window, 0, true);
                            bw_link.filter(input_2, output_2, filter_window, 0, true);
                            bw_link.filter(input_3, output_3, filter_window, 0, true);
                            hip_l_vel = output_1[filter_window - 1];
                            knee_l_vel = output_2[filter_window - 1];
                            ankle_l_vel = output_3[filter_window - 1];
#ifdef Right_Part
                            left_shift_array(input_4, link_4_vel, filter_window);
                            left_shift_array(input_5, link_5_vel, filter_window);
                            left_shift_array(input_6, link_6_vel, filter_window);
                            bw_link.filter(input_4, output_4, filter_window, 0, true);
                            bw_link.filter(input_5, output_5, filter_window, 0, true);
                            bw_link.filter(input_6, output_6, filter_window, 0, true);
                            hip_r_vel = output_4[filter_window - 1];
                            knee_r_vel = output_5[filter_window - 1];
                            ankle_r_vel = output_6[filter_window - 1];
#endif
                        }

                        // RPM -> rad/s

                        hip_l_vel *= 2;
                        knee_l_vel *= 2;
                        ankle_l_vel *= 2;

                        hip_l_vel = vel_RPM2rad(hip_l_vel);
                        knee_l_vel = vel_RPM2rad(knee_l_vel);
                        ankle_l_vel = vel_RPM2rad(ankle_l_vel);

                        double l_hip_m_vel = vel_RPM2rad(lever_arm_1_vel);
                        double l_knee_m_vel = vel_RPM2rad(lever_arm_2_vel);
                        double l_ankle_m_vel = vel_RPM2rad(lever_arm_3_vel);

#ifdef Right_Part
                        hip_r_vel *= 2;
                        knee_r_vel *= 2;
                        ankle_r_vel *= 2;

                        hip_r_vel = vel_RPM2rad(hip_r_vel);
                        knee_r_vel = vel_RPM2rad(knee_r_vel);
                        ankle_r_vel = vel_RPM2rad(ankle_r_vel);

                        double r_hip_m_vel = vel_RPM2rad(lever_arm_4_vel);
                        double r_knee_m_vel = vel_RPM2rad(lever_arm_5_vel);
                        double r_ankle_m_vel = vel_RPM2rad(lever_arm_6_vel);
#endif

                        Eigen::Matrix3d Ks_l, Ks_r;
                        Ks_l << 43.93, 0, 0,
                                0, 90.84, 0,
                                0, 0, 33.13;

                        Ks_r << 38.97, 0, 0,
                                0, 59.32, 0,
                                0, 0, 30.23;

                        Eigen::Vector3d M, B_l, K_l, B_r, K_r;
#ifdef Tracking_Impendance
                        K_l << 40, 40, 20;
                        B_l << 0.6, 0.9, 0.5;// 1.8 1.2 0.6
                        K_r << 50, 40, 20;
                        B_r << 0.8, 0.8, 0.5; // 0.6 0.6 0.5

                        d4q_l << hip_ref_4th_l, knee_ref_4th_l, ankle_ref_4th_l;
                        d3q_l << hip_ref_3rd_l, knee_ref_3rd_l, ankle_ref_3rd_l;
                        ddq_l << hip_ref_acc_l, (knee_ref_acc_l), (ankle_ref_acc_l);
                        dq_l << hip_ref_vel_l, (knee_ref_vel_l), (ankle_ref_vel_l);
                        q_l << hip_ref_rad_l, (knee_ref_rad_l), (ankle_ref_rad_l);
                        dq_e_l << hip_ref_vel_l - hip_l_vel, (knee_ref_vel_l - knee_l_vel), (ankle_ref_vel_l -
                                                                                             ankle_l_vel);
                        q_e_l << hip_ref_rad_l - link_1_rad, (knee_ref_rad_l - link_2_rad), (ankle_ref_rad_l -
                                                                                             link_3_rad);
#ifdef Right_Part
                        d4q_r << hip_ref_4th_r, knee_ref_4th_r, Ankle_ref_4th_r;
                        d3q_r << hip_ref_3rd_r, knee_ref_3rd_r, Ankle_ref_3rd_r;
                        ddq_r << hip_ref_acc_r, (knee_ref_acc_r), (Ankle_ref_acc_r);
                        dq_r << hip_ref_vel_r, (knee_ref_vel_r), (Ankle_ref_vel_r);
                        q_r << hip_ref_rad_r, (knee_ref_rad_r), (Ankle_ref_rad_r);
                        dq_e_r << hip_ref_vel_r - hip_r_vel, (knee_ref_vel_r - knee_r_vel), (Ankle_ref_vel_r -
                                                                                             ankle_r_vel);
                        q_e_r << hip_ref_rad_r - link_4_rad, (knee_ref_rad_r - link_5_rad), (Ankle_ref_rad_r -
                                                                                             link_6_rad);
#endif

#endif

#ifdef Drag_Impendance
                        K << 0,0,0;
                        B << 0.8, 1.2, 0.4;
                        d4q_l << 0, 0, 0;
                        d3q_l << 0, 0, 0;
                        ddq_l << 0, (0), (0);
                        dq_l << 0, (0), (0);
                        q_l << link_1_rad, (link_2_rad), (link_3_rad);
                        dq_e_l << 0 - hip_l_vel, (0 - knee_l_vel), (0 - ankle_l_vel);
                        q_e_l << hip_ref_rad_l - link_1_rad, (knee_ref_rad_l - link_2_rad), (ankle_ref_rad_l - link_3_rad);
#endif

                        Select_Matrix_d << 1, 0, 0,
                                0, 1, 0,
                                0, 0, 1;

                        // all diagonal element == 1 meanings all-selected.
                        // Selecting states
                        dq_e_l = Select_Matrix_d * dq_e_l;
                        q_e_l = Select_Matrix_d * q_e_l;

                        ddq_l = Select_Matrix_d * ddq_l;
                        dq_l = Select_Matrix_d * dq_l;
                        q_l = Select_Matrix_d * q_l;

                        // calculate feedforward torque. (after transmission)
                        Eigen::Vector3d compensation_l;
//                        compensation_l = left_SEA_dynamics.feedforward_dynamics(d4q_l, d3q_l, ddq_l, dq_l, q_l,
//                                                                                Ks_l);
                        compensation_l = left_SEA_dynamics.Gravity_term(q_l);

                        compensation_l = 1.0 * compensation_l; //0.8

                        l_Hip_pd.pid_set_params(0.5, 0, 4); // 0.6,4 (0.85 boundary)
                        l_Knee_pd.pid_set_params(0.45, 0, 2.5); // 0.45,2.5
                        l_Ankle_pd.pid_set_params(0.5, 0, 3); // 0.45,3

                        double l_hip_Impedance =
                                0.8 * compensation_l(0) + (B_l(0) * dq_e_l(0) + K_l(0) * q_e_l(0));
                        double l_knee_Impedance =
                                compensation_l(1) + (B_l(1) * dq_e_l(1) + K_l(1) * q_e_l(1));
                        double l_ankle_Impedance =
                                compensation_l(2) + (B_l(2) * dq_e_l(2) + K_l(2) * q_e_l(2));

                        double l_hip_tau_spr = Ks_l(0, 0) * (lever_arm_1_rad - link_1_rad);
                        double l_knee_tau_spr = Ks_l(1, 1) * (lever_arm_2_rad - link_2_rad);
                        double l_ankle_tau_spr = Ks_l(2, 2) * (lever_arm_3_rad - link_3_rad);

                        double tau_pd_hip_l = l_Hip_pd.pid_control(
                                l_hip_Impedance, l_hip_tau_spr, 2);

                        double tau_pd_knee_l = l_Knee_pd.pid_control(
                                l_knee_Impedance, l_knee_tau_spr, 2);

                        double tau_pd_ankle_l = l_Ankle_pd.pid_control(
                                l_ankle_Impedance, l_ankle_tau_spr, 1.5);

                        int tau_dyn_thousand_1 = tau_Nm2thousand(tau_pd_hip_l, 1.43);
                        int tau_dyn_thousand_2 = tau_Nm2thousand(tau_pd_knee_l, 1.43);
                        int tau_dyn_thousand_3 = tau_Nm2thousand(tau_pd_ankle_l, 0.74);

#ifdef Right_Part
                        r_Hip_pd.pid_set_params(0.6, 0, 6);
                        r_Knee_pd.pid_set_params(0.5, 0, 4.5);
                        r_Ankle_pd.pid_set_params(0.5, 0, 3);

                        Eigen::Vector3d compensation_r;
                        compensation_r = right_SEA_dynamics.feedforward_dynamics(d4q_r, d3q_r, ddq_r, dq_r, q_r,
                                                                                 Ks_r);

                        compensation_r = 0.85 * compensation_r;

                        double r_hip_Impedance =
                                compensation_r(0) + (B_r(0) * dq_e_r(0) + K_r(0) * q_e_r(0));
                        double r_knee_Impedance =
                                compensation_r(1) + (B_r(1) * dq_e_r(1) + K_r(1) * q_e_r(1));
                        double r_ankle_Impedance =
                                compensation_r(2) + (B_r(2) * dq_e_r(2) + K_r(2) * q_e_r(2));

                        double r_hip_tau_spr = Ks_r(0, 0) * (lever_arm_4_rad - link_4_rad);
                        double r_knee_tau_spr = Ks_r(1, 1) * (lever_arm_5_rad - link_5_rad);
                        double r_ankle_tau_spr = Ks_r(2, 2) * (lever_arm_6_rad - link_6_rad);
#endif

                        double tau_pd_hip_r = r_Hip_pd.pid_control(
                                r_hip_Impedance, r_hip_tau_spr, 2);

                        double tau_pd_knee_r = r_Knee_pd.pid_control(
                                r_knee_Impedance, r_knee_tau_spr, 2);

                        double tau_pd_ankle_r = r_Ankle_pd.pid_control(
                                r_ankle_Impedance, r_ankle_tau_spr, 1.5);

                        int tau_dyn_thousand_4 = tau_Nm2thousand(tau_pd_hip_r, 1.43);
                        int tau_dyn_thousand_5 = tau_Nm2thousand(tau_pd_knee_r, 1.43);
                        int tau_dyn_thousand_6 = tau_Nm2thousand(tau_pd_ankle_r, 0.74);

                        /**
                         **    Apply torque control
                         */


#ifndef asynchronous_enable


                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_hip], CST);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_knee], CST);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_ankle], CST);

                        EC_WRITE_S16(domainRx_pd + offset.target_torque[l_hip], tau_dyn_thousand_1);
                        EC_WRITE_S16(domainRx_pd + offset.target_torque[l_knee], tau_dyn_thousand_2);
                        EC_WRITE_S16(domainRx_pd + offset.target_torque[l_ankle], tau_dyn_thousand_3);
#ifdef Right_Part
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_hip], CST);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_knee], CST);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_ankle], CST);
//
                        EC_WRITE_S16(domainRx_pd + offset.target_torque[r_hip], tau_dyn_thousand_4);
                        EC_WRITE_S16(domainRx_pd + offset.target_torque[r_knee], tau_dyn_thousand_5);
                        EC_WRITE_S16(domainRx_pd + offset.target_torque[r_ankle], tau_dyn_thousand_6);
#endif

#endif

#ifdef asynchronous_enable
                        if (!FLG_LEFT_GC_1st) {
                            EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_hip], CST);
                            EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_knee], CST);
                            EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_ankle], CST);
                        }
//
                        EC_WRITE_S16(domainRx_pd + offset.target_torque[l_hip], tau_dyn_thousand_1);
                        EC_WRITE_S16(domainRx_pd + offset.target_torque[l_knee], tau_dyn_thousand_2);
                        EC_WRITE_S16(domainRx_pd + offset.target_torque[l_ankle], tau_dyn_thousand_3);
//
                        if (RIGHT_GC_START) {
                            if (!FLG_RIGHT_GC_1st) {
                                EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_hip], CST);
                                EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_knee], CST);
                                EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_ankle], CST);
                            }
                            EC_WRITE_S16(domainRx_pd + offset.target_torque[r_hip], tau_dyn_thousand_4);
                            EC_WRITE_S16(domainRx_pd + offset.target_torque[r_knee], tau_dyn_thousand_5);
                            EC_WRITE_S16(domainRx_pd + offset.target_torque[r_ankle], tau_dyn_thousand_6);
                        }
#endif
                        /** ----------------------------------------------------------------------------------- */
                        if (!(cycle_count % 10)) { // show message per 1 ms.
                            std::cout << "===============================" << std::endl;
                            std::cout << "task_working_Control" << std::endl;
                            std::cout << "===============================" << std::endl;
                            std::cout << "time: " << 1.0 * curve_count_main / TASK_FREQUENCY << " [s] " << std::endl;
                            std::cout << "dq_e_l: " << std::endl;
                            std::cout << dq_e_l.format(PrettyPrint) << std::endl;
                            std::cout << "q_e_l: " << std::endl;
                            std::cout << q_e_l.format(PrettyPrint) << std::endl;
                            std::cout << "r_Hip motor [rad] = " << lever_arm_4_rad << std::endl;
                            std::cout << "r_Hip link [rad] = " << link_4_rad << std::endl;
                            std::cout << "r_Ankle motor [cnt] = "
                                      << EC_READ_S32(domainTx_pd + offset.actual_position[r_ankle]) << std::endl;
                            std::cout << "r_Ankle link [cnt] = "
                                      << EC_READ_S32(domainTx_pd + offset.second_position[r_ankle]) << std::endl;
                            int OP_mode = EC_READ_S8(domainTx_pd + offset.modes_of_operation_display[l_hip]);
                            switch (OP_mode) {
                                case 8:
                                    std::cout << "OP mode : CSP" << std::endl;
                                    break;
                                case 9:
                                    std::cout << "OP mode : CSV" << std::endl;
                                    break;
                                case 10:
                                    std::cout << "OP mode : CST" << std::endl;
                                    break;
                                default:
                                    break;
                            }


                            for (int i = 0; i < active_num; i++)
                                error_code_sequence.emplace_back(EC_READ_U16(domainTx_pd + offset.Error_code[i]));


                            std::cout << " ---- Servo Error Status ---- " << std::endl;

                            for (int j = 0; j < active_num; j++) {
                                switch (error_code_sequence[j]) {
                                    case 0x3331:
                                        std::cout << "Joint [" << j << "] --> Field circuit interrupted" << std::endl;
                                        break;
                                    case 0x2220:
                                        std::cout << "Joint [" << j << "] --> Continuous over current (device internal)"
                                                  << std::endl;
                                        break;
                                    case 0x0000:
                                        std::cout << "Joint [" << j << "] --> NULL" << std::endl;
                                        break;
                                    default:
                                        std::cout << "Ref to data sheet." << std::endl;
                                        break;
                                }
                            }

                            std::vector<unsigned int>().swap(error_code_sequence);

                            std::cout << " ---- Gait Cycle ----" << std::endl;
                            std::cout << Gc_main << std::endl;

                            if (FLG_LEFT_GC_end) {
                                Gc_cnt++;
                                FLG_LEFT_GC_end = false;
                            } else {
//                                std::cout << "Current Gc is not completed." << std::endl;
                                std::cout << "Current Phase: " << gMFSM.m_gaitMatrixFsm << std::endl;
                            }
                            std::cout << "Current Gc is " << Gc_cnt << std::endl;


                            switch (gMFSM.m_gaitMatrixFsm) {
                                // 0 -> [1 -> 2 -> 3 -> 4 -> 5 -> 6]
                                case Stance: {
                                    std::cout << "2x stance" << std::endl;
                                    gaitData << "2x stance" << ',' << Stance << std::endl;
                                }
                                    break;
                                case ST_T: {
                                    std::cout << "Stance-Toe_off" << std::endl;
                                    gaitData << "Stance-Toe_off" << ',' << ST_T << std::endl;
                                }
                                    break;
                                case ST_S: {
                                    std::cout << "Stance-Swing" << std::endl;
                                    gaitData << "Stance-Swing" << ',' << ST_S << std::endl;
                                }
                                    break;
                                case HO_HS: {
                                    std::cout << "Heel_off - Heel_strike" << std::endl;
                                    gaitData << "Heel_off - Heel_strike" << ',' << HO_HS << std::endl;
                                }
                                    break;
                                case HS_HO: {
                                    std::cout << "Heel_strike - Heel_off" << std::endl;
                                    gaitData << "Heel_strike - Heel_off" << ',' << HS_HO << std::endl;
                                }
                                    break;
                                case S_ST: {
                                    std::cout << "Swing-Stance" << std::endl;
                                    gaitData << "Swing-Stance" << ',' << S_ST << std::endl;
                                }
                                    break;
                                case T_ST: {
                                    std::cout << "Toe_off-stance" << std::endl;
                                    gaitData << "Toe_off-stance" << ',' << T_ST << std::endl;
                                }
                                    break;
                                default:
                                    break;
                            }
//                            std::cout << "GaitPhase : " << GaitPhase << std::endl;
                            std::cout << "Left #1 :" << An_Left_1 << std::endl;
                            std::cout << "Left #2 :" << An_Left_2 << std::endl;
                            std::cout << "Right #1 :" << An_Right_1 << std::endl;
                            std::cout << "Right #2 :" << An_Right_2 << std::endl;
                        }

                        left_outFile << hip_ref_rad_l << ',' << knee_ref_rad_l << ',' << ankle_ref_rad_l << ','
                                     << link_1_rad << ',' << link_2_rad << ',' << link_3_rad << ','
                                     << lever_arm_1_rad << ',' << lever_arm_2_rad << ',' << lever_arm_3_rad << ','
                                     << tau_mot_1 << ',' << tau_mot_2 << ',' << tau_mot_3 << ','
                                     << tau_pd_hip_l << ',' << tau_pd_knee_l << ',' << tau_pd_ankle_l
                                     << std::endl;

                        left_velFile << hip_ref_vel_l << ',' << knee_ref_vel_l << ',' << ankle_ref_vel_l << ','
                                     << hip_l_vel << ',' << knee_l_vel << ',' << ankle_l_vel << ','
                                     << l_hip_m_vel << ',' << l_knee_m_vel << ',' << l_ankle_m_vel << ','
                                     << tau_mot_1 << ',' << tau_mot_2 << ',' << tau_mot_3 << ','
                                     << tau_pd_hip_l << ',' << tau_pd_knee_l << ',' << tau_pd_ankle_l << std::endl;

                        left_save_file << l_hip_Impedance << ',' << l_hip_tau_spr << ',' << l_knee_Impedance << ','
                                       << l_knee_tau_spr
                                       << ',' <<
                                       l_ankle_Impedance << ',' << l_ankle_tau_spr << ','
                                       << lever_arm_3_rad - link_3_rad
                                       << std::endl;

                        left_filter_file << compensation_l(1) << ',' << (B_l(1) * dq_e_l(1)) << ','
                                         << (K_l(1) * q_e_l(1))
                                         << std::endl;

#ifdef  Right_Part
                        right_outFile << hip_ref_rad_r << ',' << knee_ref_rad_r << ',' << Ankle_ref_rad_r << ','
                                      << link_4_rad << ',' << link_5_rad << ',' << link_6_rad << ','
                                      << r_hip_m_vel << ',' << r_knee_m_vel << ',' << r_ankle_m_vel << ','
                                      << tau_mot_1 << ',' << tau_mot_2 << ',' << tau_mot_3 << ','
                                      << tau_pd_hip_r << ',' << tau_pd_knee_r << ',' << tau_pd_ankle_r
                                      << std::endl;

                        right_velFile << hip_ref_vel_r << ',' << knee_ref_vel_r << ',' << Ankle_ref_vel_r << ','
                                      << hip_r_vel << ',' << knee_r_vel << ',' << ankle_r_vel << ','
                                      << lever_arm_4_vel << ',' << lever_arm_5_rad << ',' << lever_arm_6_rad << ','
                                      << tau_mot_1 << ',' << tau_mot_2 << ',' << tau_mot_3 << ','
                                      << tau_pd_hip_r << ',' << tau_pd_knee_r << ',' << tau_pd_ankle_r << std::endl;

                        right_save_file << r_hip_Impedance << ',' << r_hip_tau_spr << ',' << r_knee_Impedance << ','
                                        << r_knee_tau_spr
                                        << ',' <<
                                        r_ankle_Impedance << ',' << r_ankle_tau_spr << ','
                                        << lever_arm_3_rad - link_3_rad
                                        << std::endl;

                        right_filter_file << vel_RPM2rad(link_4_vel) << ',' << hip_r_vel << std::endl;
#endif
                    }

                    if (POST_RESET)
                        gTaskFsm.m_gtaskFSM = task_working_RESET;

                    id_ready_cnt++;
                    if (curve_count_main >= P * (10 + 0.35)) {
                        EtherCAT_ONLINE = false;
                        std::cout << "[End of Program...]" << std::endl;
                        break;
                    }

                }
                    break;

                default:
                    break;
            }
        }
            break;
    }

    // send process data objects
    ecrt_domain_queue(domainRx);
    ecrt_domain_queue(domainTx);
    ecrt_master_send(master);
}

