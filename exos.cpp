//
// Created by yc on 2021/6/25.
// Last modified on 2021/8/19
//

#include "exos.h"
#include "dataDecode.h"

//#define SineWave
#define Right_Part

bool EtherCAT_ONLINE = true;
bool POST_RESET = false;

std::ofstream left_outFile;
std::ofstream left_velFile;
std::ofstream left_save_file;

#ifdef Right_Part
std::ofstream right_outFile;
std::ofstream right_velFile;
std::ofstream right_save_file;
#endif

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

ButterworthLP bw_link = ButterworthLP(1000, 10, 2);

static int SIG_cnt = 0;

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
#endif

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
    pthread_exit(nullptr);
}

void cyclic_task(double Kp, double Kd) {
    static int current_pos = 0;
    int raw_init_motor_pos[joint_num] = {0, 0, 0, 0, 0, 0};
    int raw_init_spring_pos[joint_num] = {0, 0, 0, 0, 0, 0};
//    bw_link.stepInitialization(0);

    if (gSysRunning.m_gWorkStatus == sys_working_POWER_ON)
        return;

    // TODO: when cycle_count >= 90000 reset counting ( and stop )
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
            if (SERVE_OP < active_num) {
                if (ecstate <= 10) {
                    switch (ecstate) {
                        case 1:
                            for (int i = 0; i < active_num; i++) {
                                int E_code = EC_READ_U16(domainTx_pd + offset.Error_code[i]);

//                                if (E_code != 0) {
//                                    std::cout << "[Error occured at slave: " << i << std::endl;
//                                    std::cout << E_code << std::endl;
//                                    std::cout << "===================" << std::endl;
//                                    if (E_code == 0x7500) {
//                                        std::cout << "Communication error." << std::endl;
//                                        pause_to_continue();
//                                    }
//                                }


                                if (E_code != 0 || (EC_READ_U16(domainTx_pd + offset.status_word[i]) & 0x0008)) {
                                    std::cout << "[Error occured at slave: " << i << std::endl;
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
            } else {
                int tmp = true;
                for (int i = 0; i < active_num; i++) {
                    unsigned int cur_status = EC_READ_U16(domainTx_pd + offset.status_word[i]);
                    unsigned int E_code = EC_READ_U16(domainTx_pd + offset.Error_code[i]);

                    if ((EC_READ_U16(domainTx_pd + offset.status_word[i]) & 0x6f) == 0x27)
                        std::cout << "Slave [" << i << "] Enable operation" << std::endl;
                    else{
                        std::cout << "Slave [" << i << "] not in Enable operation" << std::endl;
                        std::cout << "Slave [" << i << "] State: "<< cur_status << std::endl;
                        std::cout << "Slave [" << i << "] E-code: " << E_code << std::endl;
                        switch (E_code) {
                            case 0x7500:
                                std::cout << " Communication Error." << std::endl;
                                break;
                            case 0x7300:
                                std::cout << " Sensor Error(Maybe CRC)." << std::endl;
                                break;
                            case 0x2220:
                                std::cout << "  Continuous over current" << std::endl;
                                break;
                            default:
                                std::cout << " Other Error. (ref to datasheet)" << std::endl;
                                break;
                        }
                    }


                    if ((EC_READ_U16(domainTx_pd + offset.status_word[i]) & (STATUS_SERVO_ENABLE_BIT)) == 0) {
                        tmp = false;
                        ecstate = 0;
                        break;
                    }
                }
                    if (tmp) {
                        ecstate = 0;
                        gSysRunning.m_gWorkStatus = sys_working_WORK_STATUS;
                        std::cout << "sys_working_WORK_STATUS" << std::endl;
                    }
            }
        }
            break;

        default: {
            static int reset_ready_cnt = 0;
            static int id_ready_cnt = 0;
            static int left_hip_id_init_cnt = 0;
            static int left_knee_id_init_cnt = 0;
            static int left_ankle_id_init_cnt = 0;
#ifdef Right_Part
            static int right_hip_id_init_cnt = 0;
            static int right_knee_id_init_cnt = 0;
            static int right_ankle_id_init_cnt = 0;
#endif

            static double vel_m_hip_old = 0;
            static double vel_m_knee_old = 0;
            static double vel_m_ankle_old = 0;
            static double ankle_l_vel_old = 0;

            static int PD_cnt = 0;
            static int post_reset_cnt = 0;

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
             * Derived data
             */

            /**
             * Motor radians after transmission
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
             * Here  speeder_ankle is 140:40
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
             * Velocity(RPM integer based)
             */
            double lever_arm_1_vel = raw_vel_motor_1 / transmission_hip;
            double lever_arm_2_vel = raw_vel_motor_2 / transmission_knee;
            double lever_arm_3_vel = raw_vel_motor_3 / transmission_ankle;

            double link_1_vel = lever_arm_1_vel - raw_vel_spring_1;
            double link_2_vel = lever_arm_2_vel + raw_vel_spring_2;
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
             * thousand ratio of rated torque
             */
            double tau_mot_1 = EC_READ_S16(domainTx_pd + offset.actual_torque[l_hip]);
            double tau_mot_2 = EC_READ_S16(domainTx_pd + offset.actual_torque[l_knee]);
            double tau_mot_3 = EC_READ_S16(domainTx_pd + offset.actual_torque[l_ankle]);
            double tau_mot_4 = EC_READ_S16(domainTx_pd + offset.actual_torque[r_hip]);
            double tau_mot_5 = EC_READ_S16(domainTx_pd + offset.actual_torque[r_knee]);
            double tau_mot_6 = EC_READ_S16(domainTx_pd + offset.actual_torque[r_ankle]);

            switch (gTaskFsm.m_gtaskFSM) {
                case task_working_RESET: {

                    /** A. Reset to default motor position(vertical direction as reference link) */
                    if (!(cycle_count % 10)) { // show message per 1 ms.
                        std::cout << "=====================" << std::endl;
                        std::cout << "task_working_RESET" << std::endl;
                        std::cout << "Time : " << PD_cnt / TASK_FREQUENCY << "[s]" << std::endl;
                        std::cout << "=====================" << std::endl;
                        std::cout << "rad of link-4: " << link_4_rad << std::endl;
                        std::cout << "rad of link-5: " << link_5_rad << std::endl;
                        std::cout << "rad of link-6: " << link_6_rad << std::endl;
                        std::cout << "vel of link-4: " << link_4_vel << std::endl;
                        std::cout << "vel of link-5: " << link_5_vel << std::endl;
                        std::cout << "vel of link-6: " << link_6_vel << std::endl;


                        int OP_mode = EC_READ_S8(domainTx_pd + offset.modes_of_operation_display[r_hip]);
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

                    }

                    if (reset_step == 0) {
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_hip], CSV);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_knee], CSV);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_ankle], CSV);
#ifdef Right_Part
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_hip], CSV);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_knee], CSV);
                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_ankle], CSV);
#endif
                        reset_step = 1;
                    }
                    /**
                     * Step1 reset l_hip and hold reset-point
                     */
                    if (reset_step == 1 &&
                        reset_ready_cnt != 0) { // reset_ready_cnt != 0 avoid assigning velocity in CSP mode.

                        double reset_l_hip_vel = l_Hip_reset_pid.pid_control(left_hip_id_init_rad,
                                                                             link_1_rad, 2200);
                        double reset_l_knee_vel = l_Knee_reset_pid.pid_control(left_knee_id_init_rad,
                                                                               link_2_rad, 2400);
                        double reset_l_ankle_vel = l_Ankle_reset_pid.pid_control(left_ankle_id_init_rad, link_3_rad,
                                                                                 2200);

                        EC_WRITE_S32(domainRx_pd + offset.target_velocity[l_hip], reset_l_hip_vel);
                        EC_WRITE_S32(domainRx_pd + offset.target_velocity[l_knee], reset_l_knee_vel);
                        EC_WRITE_S32(domainRx_pd + offset.target_velocity[l_ankle], reset_l_ankle_vel);

#ifdef Right_Part
                        double reset_r_hip_vel = r_Hip_reset_pid.pid_control(right_hip_id_init_rad,
                                                                             link_4_rad, 2200);
                        double reset_r_knee_vel = r_Knee_reset_pid.pid_control(right_knee_id_init_rad,
                                                                               link_5_rad, 2400);
                        double reset_r_ankle_vel = r_Ankle_reset_pid.pid_control(right_ankle_id_init_rad,
                                                                                 link_6_rad,
                                                                                 2200);

                        EC_WRITE_S32(domainRx_pd + offset.target_velocity[r_hip], reset_r_hip_vel);
                        EC_WRITE_S32(domainRx_pd + offset.target_velocity[r_knee], reset_r_knee_vel);
                        EC_WRITE_S32(domainRx_pd + offset.target_velocity[r_ankle], reset_r_ankle_vel);
#endif

                        if (PD_cnt++ > reset_timeout) {
                            // if timeout, force switch state machine.
                            EC_WRITE_S32(domainRx_pd + offset.target_velocity[l_hip], 0);
                            EC_WRITE_S32(domainRx_pd + offset.target_velocity[l_knee], 0);
                            EC_WRITE_S32(domainRx_pd + offset.target_velocity[l_ankle], 0);

                            left_hip_id_init_cnt = EC_READ_S32(domainTx_pd + offset.actual_position[l_hip]);
                            left_knee_id_init_cnt = EC_READ_S32(domainTx_pd + offset.actual_position[l_knee]);
                            left_ankle_id_init_cnt = EC_READ_S32(domainTx_pd + offset.actual_position[l_ankle]);
#ifdef Right_Part
                            EC_WRITE_S32(domainRx_pd + offset.target_velocity[r_hip], 0);
                            EC_WRITE_S32(domainRx_pd + offset.target_velocity[r_hip], 0);
                            EC_WRITE_S32(domainRx_pd + offset.target_velocity[r_hip], 0);

                            right_hip_id_init_cnt = EC_READ_S32(domainTx_pd + offset.actual_position[r_hip]);
                            right_knee_id_init_cnt = EC_READ_S32(domainTx_pd + offset.actual_position[r_knee]);
                            right_ankle_id_init_cnt = EC_READ_S32(domainTx_pd + offset.actual_position[r_ankle]);
#endif
                            if (SIG_cnt == 0) {
                                gTaskFsm.m_gtaskFSM = task_working_Control;
                                reset_step = 0;
                                reset_ready_cnt = 0;
                                PD_cnt = 0;
                                reset_timeout = 2000;
                            } else
                                EtherCAT_ONLINE = false;

                        }
                    }
                    reset_ready_cnt++;
                }
                    break;

                case task_working_Control: {

                    /** B. Assign operation to motor(s) */

                    /** count for generated curve */
                    static int curve_count = 0;   // unit(ms)
                    static int settle_cnt = 0;
                    static float max_err = 0;
                    static double l_hip_rad_old = 0;
                    static double l_hip_vel_old = 0;
                    static double l_knee_rad_old = 0;
                    static double l_knee_vel_old = 0;
                    static double l_ankle_rad_old = 0;
                    static double l_ankle_vel_old = 0;
#ifdef Right_Part
                    static double r_hip_rad_old = 0;
                    static double r_hip_vel_old = 0;
                    static double r_knee_rad_old = 0;
                    static double r_knee_vel_old = 0;
                    static double r_ankle_rad_old = 0;
                    static double r_ankle_vel_old = 0;
#endif
                    static double tau_dyn_1_old = 0;
                    static double tau_dyn_2_old = 0;
                    curve_count++;

#ifdef SineWave
                    //                    if(!(curve_count % 1000))
                    //                        freq += 0.5;

                    double hip_ref_rad = cosineWave(curve_count, freq, 0.1);
                    double knee_ref_rad = cosineWave(curve_count, freq, 0);
                    double ankle_ref_rad = cosineWave(curve_count, freq, 0.2);

                    double hip_ref_vel = 2 * Pi * freq * sineWave(curve_count, freq, -0.1);
                    double knee_ref_vel = 2 * Pi * freq * sineWave(curve_count, freq, 0);
                    double ankle_ref_vel = 2 * Pi * freq * sineWave(curve_count, freq, -0.2);

                    double hip_ref_acc = pow(2 * Pi * freq, 2) * cosineWave(curve_count, freq, -0.1);
                    double knee_ref_acc = pow(2 * Pi * freq, 2) * cosineWave(curve_count, freq, -0);
                    double ankle_ref_acc = pow(2 * Pi * freq, 2) * cosineWave(curve_count, freq, 0.2);
#else
                    double beta = 0.8;
                    double Gc;
                    Gc = 1.0 * (curve_count % P) / P;

                    if (!POST_RESET) {
                        //TODO: right part fourier
                        double hip_ref_rad = base_Fourier_8th(Gc, a_hip, b_hip, w, a0_hip);
                        double knee_ref_rad = beta * base_Fourier_8th(Gc, a_knee, b_knee, w, a0_knee);
                        double ankle_ref_rad = beta * base_Fourier_8th(Gc, a_ankle, b_ankle, w, a0_ankle);

                        double hip_ref_vel = differentia_1st_Fourier_8th(Gc, a_knee, b_knee, w, a0_knee, P);
                        double knee_ref_vel = beta * differentia_1st_Fourier_8th(Gc, a_knee, b_knee, w, a0_knee, P);
                        double ankle_ref_vel =
                                beta * differentia_1st_Fourier_8th(Gc, a_ankle, b_ankle, w, a0_ankle, P);

                        double hip_ref_acc = differentia_2ed_Fourier_8th(Gc, a_hip, b_hip, w, a0_hip, P);
                        double knee_ref_acc = beta * differentia_2ed_Fourier_8th(Gc, a_knee, b_knee, w, a0_knee, P);
                        double ankle_ref_acc =
                                beta * differentia_2ed_Fourier_8th(Gc, a_ankle, b_ankle, w, a0_ankle, P);

                        double hip_ref_3rd = differentia_3rd_Fourier_8th(Gc, a_hip, b_hip, w, a0_hip, P);
                        double knee_ref_3rd = beta * differentia_3rd_Fourier_8th(Gc, a_knee, b_knee, w, a0_knee, P);
                        double ankle_ref_3rd =
                                beta * differentia_3rd_Fourier_8th(Gc, a_ankle, b_ankle, w, a0_ankle, P);

                        double hip_ref_4th = differentia_4th_Fourier_8th(Gc, a_hip, b_hip, w, a0_hip, P);
                        double knee_ref_4th = beta * differentia_4th_Fourier_8th(Gc, a_knee, b_knee, w, a0_knee, P);
                        double ankle_ref_4th =
                                beta * differentia_4th_Fourier_8th(Gc, a_ankle, b_ankle, w, a0_ankle, P);
#endif
                        /**
                         ** Controller Part
                         **/

                        double hip_l_vel, knee_l_vel, ankle_l_vel;
                        double hip_r_vel, knee_r_vel, ankle_r_vel;
                        int filter_window = 30;
                        int avg_window = 10;

                        if (curve_count < filter_window) {
                            if (curve_count < avg_window) {
                                input_1[curve_count] = link_1_vel;
                                input_2[curve_count] = link_2_vel;
                                input_3[curve_count] = link_3_vel;
#ifdef Right_Part
                                input_4[curve_count] = link_4_vel;
                                input_5[curve_count] = link_5_vel;
                                input_6[curve_count] = link_6_vel;
#endif
                            } else {
                                double tmp1 =
                                        sum_of_array(input_1, curve_count - avg_window, curve_count) / avg_window;
                                double tmp2 =
                                        sum_of_array(input_2, curve_count - avg_window, curve_count) / avg_window;
                                double tmp3 =
                                        sum_of_array(input_3, curve_count - avg_window, curve_count) / avg_window;
                                input_1[curve_count] = tmp1;
                                input_2[curve_count] = tmp2;
                                input_3[curve_count] = tmp3;
#ifdef Right_Part
                                double tmp4 =
                                        sum_of_array(input_4, curve_count - avg_window, curve_count) / avg_window;
                                double tmp5 =
                                        sum_of_array(input_5, curve_count - avg_window, curve_count) / avg_window;
                                double tmp6 =
                                        sum_of_array(input_6, curve_count - avg_window, curve_count) / avg_window;
                                input_4[curve_count] = tmp4;
                                input_5[curve_count] = tmp5;
                                input_6[curve_count] = tmp6;
#endif
                            }
                            hip_l_vel = input_1[curve_count];
                            knee_l_vel = input_2[curve_count];
                            ankle_l_vel = input_3[curve_count];
#ifdef Right_Part
                            hip_r_vel = input_4[curve_count];
                            knee_r_vel = input_5[curve_count];
                            ankle_r_vel = input_6[curve_count];
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
                        hip_l_vel = vel_RPM2rad(hip_l_vel);
                        knee_l_vel = vel_RPM2rad(knee_l_vel);
                        ankle_l_vel = vel_RPM2rad(ankle_l_vel);

                        double l_hip_m_vel = vel_RPM2rad(lever_arm_1_vel);
                        double l_knee_m_vel = vel_RPM2rad(lever_arm_2_vel);
                        double l_ankle_m_vel = vel_RPM2rad(lever_arm_3_vel);

#ifdef Right_Part
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

                        Eigen::Vector3d M, B, K;
#ifdef Tracking_Impendance
                        K << 30, 30, 20;
                        B << 0.4, 1.0, 0.4;
                        d4q_l << hip_ref_4th, knee_ref_4th, ankle_ref_4th;
                        d3q_l << hip_ref_3rd, knee_ref_3rd, ankle_ref_3rd;
                        ddq_l << hip_ref_acc, (knee_ref_acc), (ankle_ref_acc);
                        dq_l << hip_ref_vel, (knee_ref_vel), (ankle_ref_vel);
                        q_l << hip_ref_rad, (knee_ref_rad), (ankle_ref_rad);
                        dq_e_l << hip_ref_vel - hip_l_vel, (knee_ref_vel - knee_l_vel), (ankle_ref_vel -
                                                                                         ankle_l_vel);
                        q_e_l << hip_ref_rad - link_1_rad, (knee_ref_rad - link_2_rad), (ankle_ref_rad -
                                                                                         link_3_rad);
#ifdef Right_Part
                        d4q_r << hip_ref_4th, knee_ref_4th, ankle_ref_4th;
                        d3q_r << hip_ref_3rd, knee_ref_3rd, ankle_ref_3rd;
                        ddq_r << hip_ref_acc, (knee_ref_acc), (ankle_ref_acc);
                        dq_r << hip_ref_vel, (knee_ref_vel), (ankle_ref_vel);
                        q_r << hip_ref_rad, (knee_ref_rad), (ankle_ref_rad);
                        dq_e_r << hip_ref_vel - hip_r_vel, (knee_ref_vel - knee_r_vel), (ankle_ref_vel -
                                                                                         ankle_r_vel);
                        q_e_r << hip_ref_rad - link_4_rad, (knee_ref_rad - link_5_rad), (ankle_ref_rad -
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
                        q_e_l << hip_ref_rad - link_1_rad, (knee_ref_rad - link_2_rad), (ankle_ref_rad - link_3_rad);
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
                        compensation_l = left_SEA_dynamics.feedforward_dynamics(d4q_l, d3q_l, ddq_l, dq_l, q_l,
                                                                                Ks_l);
                        compensation_l = 0.85 * compensation_l;

                        l_Hip_pd.pid_set_params(0.6, 0, 4); // 0.6,4 (0.85 boundary)
                        l_Knee_pd.pid_set_params(0.45, 0, 2.5); // 0.45,2.5
                        l_Ankle_pd.pid_set_params(0.5, 0, 3); // 0.45,3

                        double l_hip_Impedance =
                                compensation_l(0) / transmission_hip + (B(0) * dq_e_l(0) + K(0) * q_e_l(0));
                        double l_knee_Impedance =
                                compensation_l(1) / transmission_knee + (B(1) * dq_e_l(1) + K(1) * q_e_l(1));
                        double l_ankle_Impedance =
                                compensation_l(2) / transmission_ankle + (B(2) * dq_e_l(2) + K(2) * q_e_l(2));

                        double l_hip_tau_spr = Ks_l(0, 0) * (lever_arm_1_rad - link_1_rad);
                        double l_knee_tau_spr = Ks_l(1, 1) * (lever_arm_2_rad - link_2_rad);
                        double l_ankle_tau_spr = Ks_l(2, 2) * (lever_arm_3_rad - link_3_rad);

                        double tau_pd_hip_l = l_Hip_pd.pid_control(
                                l_hip_Impedance, l_hip_tau_spr, 3);

                        double tau_pd_knee_l = l_Knee_pd.pid_control(
                                l_knee_Impedance, l_knee_tau_spr, 3);

                        double tau_pd_ankle_l = l_Ankle_pd.pid_control(
                                l_ankle_Impedance, l_ankle_tau_spr, 1.5);

                        int tau_dyn_thousand_1 = tau_Nm2thousand(tau_pd_hip_l, 1.43);
                        int tau_dyn_thousand_2 = tau_Nm2thousand(tau_pd_knee_l, 1.43);
                        int tau_dyn_thousand_3 = tau_Nm2thousand(tau_pd_ankle_l, 0.74);

#ifdef Right_Part
                        r_Hip_pd.pid_set_params(0.2, 0, 0);
                        r_Knee_pd.pid_set_params(0.2, 0, 0);
                        r_Ankle_pd.pid_set_params(0.4, 0, 0);

                        Eigen::Vector3d compensation_r;
                        compensation_r = right_SEA_dynamics.feedforward_dynamics(d4q_r, d3q_r, ddq_r, dq_r, q_r,
                                                                                 Ks_r);
                        compensation_r = 0.85 * compensation_r;

                        double r_hip_Impedance =
                                (B(0) * dq_e_r(0) + K(0) * q_e_r(0));
                        double r_knee_Impedance =
                                (B(1) * dq_e_r(1) + K(1) * q_e_r(1));
                        double r_ankle_Impedance =
                                (0 * dq_e_r(2) + K(2) * q_e_r(2));

                        double r_hip_tau_spr = Ks_r(0, 0) * (lever_arm_4_rad - link_4_rad);
                        double r_knee_tau_spr = Ks_r(1, 1) * (lever_arm_5_rad - link_5_rad);
                        double r_ankle_tau_spr = Ks_r(2, 2) * (lever_arm_6_rad - link_6_rad);
#endif

                        double tau_pd_hip_r = r_Hip_pd.pid_control(
                                r_hip_Impedance, r_hip_tau_spr, 3);

                        double tau_pd_knee_r = r_Knee_pd.pid_control(
                                r_knee_Impedance, r_knee_tau_spr, 3);

                        double tau_pd_ankle_r = r_Ankle_pd.pid_control(
                                r_ankle_Impedance, r_ankle_tau_spr, 1.5);

                        int tau_dyn_thousand_4 = tau_Nm2thousand(tau_pd_hip_r, 1.43);
                        int tau_dyn_thousand_5 = tau_Nm2thousand(tau_pd_knee_r, 1.43);
                        int tau_dyn_thousand_6 = tau_Nm2thousand(tau_pd_ankle_r, 0.74);

                        /**
                         **    Apply torque control
                         */

                        unsigned int An_in = EC_READ_U16(domainTx_pd + offset.analog_in1[l_hip]);

                        bool logic_1 = false;
                        if (An_in > 200)
                            logic_1 = true;

//                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_hip], CST);
//                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_knee], CST);
//                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[l_ankle], CST);
//
//                        EC_WRITE_S16(domainRx_pd + offset.target_torque[l_hip], tau_dyn_thousand_1);
//                        EC_WRITE_S16(domainRx_pd + offset.target_torque[l_knee], tau_dyn_thousand_2);
//                        EC_WRITE_S16(domainRx_pd + offset.target_torque[l_ankle], tau_dyn_thousand_3);
#ifdef Right_Part
//                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_hip], CST);
//                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_knee], CST);
//                        EC_WRITE_S8(domainRx_pd + offset.operation_mode[r_ankle], CST);
//
//                        EC_WRITE_S16(domainRx_pd + offset.target_torque[r_hip], tau_dyn_thousand_4);
//                        EC_WRITE_S16(domainRx_pd + offset.target_torque[r_knee], tau_dyn_thousand_5);
//                        EC_WRITE_S16(domainRx_pd + offset.target_torque[r_ankle], tau_dyn_thousand_6);
#endif
                        /** ----------------------------------------------------------------------------------- */
                        if (!(cycle_count % 10)) { // show message per 1 ms.
                            std::cout << "===============================" << std::endl;
                            std::cout << "task_working_Control" << std::endl;
                            std::cout << "===============================" << std::endl;
                            std::cout << "time: " << 1.0 * curve_count / TASK_FREQUENCY << " [s] " << std::endl;
                            std::cout << "dq_e_l: " << std::endl;
                            std::cout << dq_e_l.format(PrettyPrint) << std::endl;
                            std::cout << "q_e_l: " << std::endl;
                            std::cout << q_e_l.format(PrettyPrint) << std::endl;
                            std::cout << "r_Hip motor [rad] = " << lever_arm_4_rad << std::endl;
                            std::cout << "r_Hip link [rad] = " << link_4_rad << std::endl;
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
                            std::cout << "Analog in(0-4095): " << An_in << std::endl;
                            std::cout << "Result: " << logic_1 << std::endl;
                        }


                        left_outFile << hip_ref_rad << ',' << knee_ref_rad << ',' << ankle_ref_rad << ','
                                     << link_1_rad << ',' << link_2_rad << ',' << link_3_rad << ','
                                     << lever_arm_1_rad << ',' << lever_arm_2_rad << ',' << lever_arm_3_rad << ','
                                     << tau_mot_1 << ',' << tau_mot_2 << ',' << tau_mot_3 << ','
                                     << tau_pd_hip_l << ',' << tau_pd_knee_l << ',' << tau_pd_ankle_l << ','
                                     << q_e_r(2)
                                     << std::endl;

                        left_velFile << hip_ref_vel << ',' << knee_ref_vel << ',' << ankle_ref_vel << ','
                                     << hip_l_vel << ',' << knee_l_vel << ',' << ankle_l_vel << ','
                                     << lever_arm_1_vel << ',' << lever_arm_2_vel << ',' << lever_arm_3_vel << ','
                                     << tau_mot_1 << ',' << tau_mot_2 << ',' << tau_mot_3 << ','
                                     << tau_pd_hip_l << ',' << tau_pd_knee_l << ',' << tau_pd_ankle_l << std::endl;

                        left_save_file << l_hip_Impedance << ',' << l_hip_tau_spr << ',' << l_knee_Impedance << ','
                                       << l_knee_tau_spr
                                       << ',' <<
                                       l_ankle_Impedance << ',' << l_ankle_tau_spr << ','
                                       << lever_arm_3_rad - link_3_rad
                                       << std::endl;
#ifdef  Right_Part
                        right_outFile << hip_ref_rad << ',' << knee_ref_rad << ',' << ankle_ref_rad << ','
                                      << link_4_rad << ',' << link_5_rad << ',' << link_6_rad << ','
                                      << lever_arm_4_rad << ',' << lever_arm_5_rad << ',' << lever_arm_6_rad << ','
                                      << tau_mot_1 << ',' << tau_mot_2 << ',' << tau_mot_3 << ','
                                      << tau_pd_hip_r << ',' << tau_pd_knee_r << ',' << tau_pd_ankle_r << ','
                                      << q_e_l(2)
                                      << std::endl;

                        right_velFile << hip_ref_vel << ',' << knee_ref_vel << ',' << ankle_ref_vel << ','
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
#endif
                    }

                    if (POST_RESET)
                        gTaskFsm.m_gtaskFSM = task_working_RESET;

                    id_ready_cnt++;
                    if (curve_count >= P * 10) {
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

