//
// Created by yc on 2021/6/25.
//

#ifndef EXOS_IMPEDANCECONTROL_EXOS_H
#define EXOS_IMPEDANCECONTROL_EXOS_H

#include <stdlib.h>
#include <termio.h>
#include <iostream>
#include <errno.h>
#include "signal.h"
#include "stdio.h"
#include "sys/resource.h"
#include "sys/time.h"
#include "sys/types.h"
#include "unistd.h"
#include "sys/mman.h"
#include <pthread.h>
#include <cmath>

// FilterData
#include "Utils.h"
#include "ButterWorthLP.h"

// Unit transformation
#include "Unit.h"

// Trajectory generation
#include "traj_generate.h"

// PID controller class
#include "PID.h"

// Dynamics
#include "dynamics.h"

// EtherCAT header
#include "ecrt.h"

// CSV header
#include <cstring>
#include <vector>
#include <fstream>
#include <sstream>

#include <vector>

// Slave(s) Error Parse
#include "ErrorParse.h"

#define left  true
#define right false

//#define Drag_Impendance
#define Tracking_Impendance

#define TASK_FREQUENCY 1000.0 // Hz
#define TARGET_VELOCITY 100 // PUU (Synapticon default PUU is rpm)

/* NOTICE:
 * Change TASK_FREQUENCY to higher value (4kHz++) may cause losing heartbeats
 */

// 1 -- profile position mode
// 3 -- profile velocity mode
// 4 -- Torque profile mode
// 8 -- cyclic sync position mode
// 9 -- cyclic sync velocity mode
// 10-- cyclic sync torque mode

#define CSP 8
#define CSV 9
#define CST 10

#define ETHERCAT_STATUS_OP 0x08
#define STATUS_SERVO_ENABLE_BIT (0x04)

#define joint_num 6
#define active_num 6

// EtherCAT port Mapping
#define l_hip   2
#define l_knee  1
#define l_ankle 0
#define r_hip   3
#define r_knee  4
#define r_ankle 5

/** ------ Left Part ----- **/

#define left_hip_init_motor_pos -142703165
#define left_hip_init_spring_pos 42322

#define left_knee_init_motor_pos -657451167
#define left_knee_init_spring_pos -3206

/**  Ankle range = (35,-45)  */
#define left_ankle_init_motor_pos -154667671
#define left_ankle_init_link_pos 82443

/** ------ Right Part ----- **/

#define right_hip_init_motor_pos -804143624
#define right_hip_init_spring_pos -23821

#define right_knee_init_motor_pos 259479224
#define right_knee_init_spring_pos 49030

#define right_ankle_init_motor_pos 122718707
#define right_ankle_init_link_pos 179652

#ifdef Tracking_Impendance
#define left_hip_id_init_rad -0
#define left_knee_id_init_rad -0.15
#define left_ankle_id_init_rad 0.1
#endif

#ifdef Drag_Impendance
#define left_hip_id_init_rad -0.05
#define left_knee_id_init_rad 0
#define left_ankle_id_init_rad 0
#endif

#define right_hip_id_init_rad -0.0
#define right_knee_id_init_rad -0.15
#define right_ankle_id_init_rad 0.1

/**
 * Im3_r = 0.00006724
 * Fv3_r = 0.00261046
 *
 */

// etherCAT object
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domainTx = NULL;
static ec_domain_state_t domainTx_state = {};
static ec_domain_t *domainRx = NULL;
static ec_domain_state_t domainRx_state = {};

//==================== PDO =========================
static uint8_t *domainRx_pd = NULL;
static uint8_t *domainTx_pd = NULL;

// length change with number of slave(s)
static ec_slave_config_t *sc[joint_num];
static ec_slave_config_state_t sc_state[joint_num] = {};
static ec_sdo_request_t *sdo[joint_num];

// hardware specification
#define SynapticonSlave1 0,2 // 1st slave
#define SynapticonSlave2 0,1 // 2ed slave
#define SynapticonSlave3 0,0 // 3rd slave
#define SynapticonSlave4 0,3 // 1st slave
#define SynapticonSlave5 0,4 // 2ed slave
#define SynapticonSlave6 0,5 // 3rd slave

#define Synapticon  0x000022d2,0x00000201// vendor id + product id

// EtherCAT state machine enum
typedef enum _workingStatus {
    sys_working_POWER_ON,
    sys_working_SAFE_MODE,
    sys_working_OP_MODE,
    sys_working_LINK_DOWN,
    sys_working_WORK_MODE,
    sys_woring_INIT_Failed
} workingStatus;

typedef struct _GsysRunningParm {
    workingStatus m_gWorkStatus;
} GsysRunningParm;

// OP task FSM
typedef enum _TaskFSM {
    task_working_RESET,
    task_working_Control
} TaskFSM;

typedef struct _GTaskFSM {
    TaskFSM m_gtaskFSM;
} GTaskFSM;

GsysRunningParm gSysRunning;
GTaskFSM gTaskFsm;

int SERVE_OP = 0;
int ecstate = 0;
int reset_step = 0;

/**
 * Fourier Series of Joint angle (for trajectory)
 */
double w_hip = 6.27659139567477;  // angle freq  = 2*pi*f where f is base freq
double w_knee = w_hip;  // angle freq  = 2*pi*f where f is base freq
double w_ankle = w_hip;  // angle freq  = 2*pi*f where f is base freq

double a_hip[8] = {0.308750228166298, -0.0301760062107781, 0.00505607355750842, 0.00159201009938250,
                   -0.00281684827129251, 0.00129439626305321, -0.00504290840263287, 0.00382003697292678};
double b_hip[8] = {-0.0917919985841033, -0.0229946587230083, 0.0228749586595578, -8.60002514422098e-05,
                   0.00199416878210645, -0.00444905211999233, 0.00132143317171056, 0.000876821564775400};
double a0_hip = 0.187072145486077;

double a_knee[8] = {0.0985635337735540, 0.278740647777703, 0.0286360377526829, 0.0250057934881560, 0.0219855831069594,
                    0.000554154299564611, 0.00690268692893109, 0.000709369274812899};
double b_knee[8] = {0.408481929825363, -0.107510874156436, -0.0702907692220593, 0.0143563230603128,
                    -0.00664071675999920, 0.0125278445195671, 0.0128800581276489, 0.00223652070388307};
double a0_knee = -0.475945721635276;

double a_ankle[8] = {-0.109863815437901, -0.00833917060475227, -0.0989598858020730, 0.0353155563750233,
                     -0.00891323739557765, 0.00351452349973219, 0.00955865603010861, -0.00393433684806532};
double b_ankle[8] = {0.0549542830785040, -0.109350901714820, 0.0184124040787744, 0.0114596100947102,
                     -0.0141854746312045, 0.0107960677657798, -0.00738513480814346, -0.000616125357164900};
double a0_ankle = -0.0364079750372623;

int P = 3000; // time of Gait Cycle [ms]

// Offsets for PDO entries
static struct {
    /* RxPDOs: 0x1600 */
    unsigned int operation_mode[joint_num];
    unsigned int ctrl_word[joint_num];
    unsigned int target_velocity[joint_num];
    unsigned int target_position[joint_num];
    unsigned int target_torque[joint_num];
    /* RxPDOs: 0x1601 */
    unsigned int digital_out1[joint_num];
    unsigned int digital_out2[joint_num];
    unsigned int digital_out3[joint_num];
    unsigned int digital_out4[joint_num];

    /* TxPDOs: 0x1A00 */
    unsigned int status_word[joint_num];
    unsigned int modes_of_operation_display[joint_num];
    unsigned int actual_position[joint_num];
    unsigned int actual_velocity[joint_num];
    unsigned int actual_torque[joint_num];
    /* TxPDOs: 0x1A01 */
    unsigned int second_position[joint_num];
    unsigned int second_velocity[joint_num];
    unsigned int analog_in1[joint_num];
    unsigned int analog_in2[joint_num];
    unsigned int analog_in3[joint_num];
    unsigned int analog_in4[joint_num];
    /* TxPDOs: 0x1A02 */
    unsigned int digital_in1[joint_num];
    unsigned int digital_in2[joint_num];
    unsigned int digital_in3[joint_num];
    unsigned int digital_in4[joint_num];
    /* Error Code */
    unsigned int Error_code[joint_num];
} offset;


/* NOTICE:
 * domain_reg must aligned to pdo_entries
 * the PDOs are configured to slaves not master
 *  slaves -> master = TxPOD
 *  master -> slaves = RxPOD
 */

// output_1 domain register (RxPDO mapped objects)
ec_pdo_entry_reg_t domain_Rx_reg[] = {
        // slave - 1
        {SynapticonSlave1, Synapticon, 0x6040, 0, &offset.ctrl_word[l_hip]},
        {SynapticonSlave1, Synapticon, 0x6060, 0, &offset.operation_mode[l_hip]},
        {SynapticonSlave1, Synapticon, 0x60FF, 0, &offset.target_velocity[l_hip]},
        {SynapticonSlave1, Synapticon, 0x607A, 0, &offset.target_position[l_hip]},
        {SynapticonSlave1, Synapticon, 0x6071, 0, &offset.target_torque[l_hip]},
        // slave - 2
        {SynapticonSlave2, Synapticon, 0x6040, 0, &offset.ctrl_word[l_knee]},
        {SynapticonSlave2, Synapticon, 0x6060, 0, &offset.operation_mode[l_knee]},
        {SynapticonSlave2, Synapticon, 0x60FF, 0, &offset.target_velocity[l_knee]},
        {SynapticonSlave2, Synapticon, 0x607A, 0, &offset.target_position[l_knee]},
        {SynapticonSlave2, Synapticon, 0x6071, 0, &offset.target_torque[l_knee]},
        // slave - 3
        {SynapticonSlave3, Synapticon, 0x6040, 0, &offset.ctrl_word[l_ankle]},
        {SynapticonSlave3, Synapticon, 0x6060, 0, &offset.operation_mode[l_ankle]},
        {SynapticonSlave3, Synapticon, 0x60FF, 0, &offset.target_velocity[l_ankle]},
        {SynapticonSlave3, Synapticon, 0x607A, 0, &offset.target_position[l_ankle]},
        {SynapticonSlave3, Synapticon, 0x6071, 0, &offset.target_torque[l_ankle]},
        // slave - 4
        {SynapticonSlave4, Synapticon, 0x6040, 0, &offset.ctrl_word[r_hip]},
        {SynapticonSlave4, Synapticon, 0x6060, 0, &offset.operation_mode[r_hip]},
        {SynapticonSlave4, Synapticon, 0x60FF, 0, &offset.target_velocity[r_hip]},
        {SynapticonSlave4, Synapticon, 0x607A, 0, &offset.target_position[r_hip]},
        {SynapticonSlave4, Synapticon, 0x6071, 0, &offset.target_torque[r_hip]},
        // slave - 5
        {SynapticonSlave5, Synapticon, 0x6040, 0, &offset.ctrl_word[r_knee]},
        {SynapticonSlave5, Synapticon, 0x6060, 0, &offset.operation_mode[r_knee]},
        {SynapticonSlave5, Synapticon, 0x60FF, 0, &offset.target_velocity[r_knee]},
        {SynapticonSlave5, Synapticon, 0x607A, 0, &offset.target_position[r_knee]},
        {SynapticonSlave5, Synapticon, 0x6071, 0, &offset.target_torque[r_knee]},
        // slave - 6
        {SynapticonSlave6, Synapticon, 0x6040, 0, &offset.ctrl_word[r_ankle]},
        {SynapticonSlave6, Synapticon, 0x6060, 0, &offset.operation_mode[r_ankle]},
        {SynapticonSlave6, Synapticon, 0x60FF, 0, &offset.target_velocity[r_ankle]},
        {SynapticonSlave6, Synapticon, 0x607A, 0, &offset.target_position[r_ankle]},
        {SynapticonSlave6, Synapticon, 0x6071, 0, &offset.target_torque[r_ankle]},
        {}
};

// input_1 domain register (TxPDO mapped objects)
ec_pdo_entry_reg_t domain_Tx_reg[] = {
        // slave - 1
        {SynapticonSlave1, Synapticon, 0x6041, 0, &offset.status_word[l_hip]},
        {SynapticonSlave1, Synapticon, 0x6061, 0, &offset.modes_of_operation_display[l_hip]},
        {SynapticonSlave1, Synapticon, 0x6077, 0, &offset.actual_torque[l_hip]},
        {SynapticonSlave1, Synapticon, 0x6064, 0, &offset.actual_position[l_hip]},
        {SynapticonSlave1, Synapticon, 0x606C, 0, &offset.actual_velocity[l_hip]},

        {SynapticonSlave1, Synapticon, 0x230A, 0, &offset.second_position[l_hip]},
        {SynapticonSlave1, Synapticon, 0x230B, 0, &offset.second_velocity[l_hip]},
        {SynapticonSlave1, Synapticon, 0x2401, 0, &offset.analog_in1[l_hip]},
        {SynapticonSlave1, Synapticon, 0x2402, 0, &offset.analog_in2[l_hip]},
        {SynapticonSlave1, Synapticon, 0x603F, 0, &offset.Error_code[l_hip]},

        // slave - 2
        {SynapticonSlave2, Synapticon, 0x6041, 0, &offset.status_word[l_knee]},
        {SynapticonSlave2, Synapticon, 0x6061, 0, &offset.modes_of_operation_display[l_knee]},
        {SynapticonSlave2, Synapticon, 0x6077, 0, &offset.actual_torque[l_knee]},
        {SynapticonSlave2, Synapticon, 0x6064, 0, &offset.actual_position[l_knee]},
        {SynapticonSlave2, Synapticon, 0x606C, 0, &offset.actual_velocity[l_knee]},

        {SynapticonSlave2, Synapticon, 0x230A, 0, &offset.second_position[l_knee]},
        {SynapticonSlave2, Synapticon, 0x230B, 0, &offset.second_velocity[l_knee]},
        {SynapticonSlave2, Synapticon, 0x603F, 0, &offset.Error_code[l_knee]},

        // slave - 3
        {SynapticonSlave3, Synapticon, 0x6041, 0, &offset.status_word[l_ankle]},
        {SynapticonSlave3, Synapticon, 0x6061, 0, &offset.modes_of_operation_display[l_ankle]},
        {SynapticonSlave3, Synapticon, 0x6077, 0, &offset.actual_torque[l_ankle]},
        {SynapticonSlave3, Synapticon, 0x6064, 0, &offset.actual_position[l_ankle]},
        {SynapticonSlave3, Synapticon, 0x606C, 0, &offset.actual_velocity[l_ankle]},

        {SynapticonSlave3, Synapticon, 0x230A, 0, &offset.second_position[l_ankle]},
        {SynapticonSlave3, Synapticon, 0x230B, 0, &offset.second_velocity[l_ankle]},
        {SynapticonSlave3, Synapticon, 0x603F, 0, &offset.Error_code[l_ankle]},

        // slave - 4
        {SynapticonSlave4, Synapticon, 0x6041, 0, &offset.status_word[r_hip]},
        {SynapticonSlave4, Synapticon, 0x6061, 0, &offset.modes_of_operation_display[r_hip]},
        {SynapticonSlave4, Synapticon, 0x6077, 0, &offset.actual_torque[r_hip]},
        {SynapticonSlave4, Synapticon, 0x6064, 0, &offset.actual_position[r_hip]},
        {SynapticonSlave4, Synapticon, 0x606C, 0, &offset.actual_velocity[r_hip]},
        {SynapticonSlave4, Synapticon, 0x2401, 0, &offset.analog_in1[r_hip]},
        {SynapticonSlave4, Synapticon, 0x2402, 0, &offset.analog_in2[r_hip]},

        {SynapticonSlave4, Synapticon, 0x230A, 0, &offset.second_position[r_hip]},
        {SynapticonSlave4, Synapticon, 0x230B, 0, &offset.second_velocity[r_hip]},
        {SynapticonSlave4, Synapticon, 0x603F, 0, &offset.Error_code[r_hip]},

        // slave - 5
        {SynapticonSlave5, Synapticon, 0x6041, 0, &offset.status_word[r_knee]},
        {SynapticonSlave5, Synapticon, 0x6061, 0, &offset.modes_of_operation_display[r_knee]},
        {SynapticonSlave5, Synapticon, 0x6077, 0, &offset.actual_torque[r_knee]},
        {SynapticonSlave5, Synapticon, 0x6064, 0, &offset.actual_position[r_knee]},
        {SynapticonSlave5, Synapticon, 0x606C, 0, &offset.actual_velocity[r_knee]},

        {SynapticonSlave5, Synapticon, 0x230A, 0, &offset.second_position[r_knee]},
        {SynapticonSlave5, Synapticon, 0x230B, 0, &offset.second_velocity[r_knee]},
        {SynapticonSlave5, Synapticon, 0x603F, 0, &offset.Error_code[r_knee]},

        // slave - 6
        {SynapticonSlave6, Synapticon, 0x6041, 0, &offset.status_word[r_ankle]},
        {SynapticonSlave6, Synapticon, 0x6061, 0, &offset.modes_of_operation_display[r_ankle]},
        {SynapticonSlave6, Synapticon, 0x6077, 0, &offset.actual_torque[r_ankle]},
        {SynapticonSlave6, Synapticon, 0x6064, 0, &offset.actual_position[r_ankle]},
        {SynapticonSlave6, Synapticon, 0x606C, 0, &offset.actual_velocity[r_ankle]},

        {SynapticonSlave6, Synapticon, 0x230A, 0, &offset.second_position[r_ankle]},
        {SynapticonSlave6, Synapticon, 0x230B, 0, &offset.second_velocity[r_ankle]},
        {SynapticonSlave6, Synapticon, 0x603F, 0, &offset.Error_code[r_ankle]},
//        {SynapticonSlave1, Synapticon, 0x2401, 0, &offset.analog_in1[0]},
//        {SynapticonSlave2, Synapticon, 0x2401, 0, &offset.analog_in1[0]},
//        {SynapticonSlave1, Synapticon, 0x2402, 0, &offset.analog_in2},
//        {SynapticonSlave1, Synapticon, 0x2403, 0, &offset.analog_in3},
//        {SynapticonSlave1, Synapticon, 0x2404, 0, &offset.analog_in4},
//
//        {SynapticonSlave1, Synapticon, 0x2501, 0, &offset.digital_in1},
//        {SynapticonSlave1, Synapticon, 0x2502, 0, &offset.digital_in2},
//        {SynapticonSlave1, Synapticon, 0x2503, 0, &offset.digital_in3},
//        {SynapticonSlave1, Synapticon, 0x2504, 0, &offset.digital_in4},
        {}
};

// PDO entries
static ec_pdo_entry_info_t pdo_entries_Rx[] = {
        /* RxPdo 0x1600 */
        {0x6040, 0x00, 16}, // control word
        {0x6060, 0x00, 8}, // Modes of operation
        {0x60FF, 0x00, 32}, // Target velocity
        {0x607A, 0x00, 32}, // target position
        {0x6071, 0x00, 16}, // Target torque
};

static ec_pdo_entry_info_t pdo_entries_Tx[] = {
        /* TxPdo 0x1A00 */
        {0x6041, 0x00, 16}, // Status word
        {0x6061, 0x00, 8},  // modes of operation display
        {0x606C, 0x00, 32}, // actual velocity
        {0x6064, 0x00, 32}, // actual position
        {0x6077, 0x00, 16}, // actual torque (unit is per thousand of rated torque.)
        /* TxPdo 0x1A01 */
        {0x230A, 0x00, 32},  // Second position
        {0x230B, 0x00, 32},  // Second velocity
        {0x2401, 0x00, 16},  // Analog Input 1
        {0x2402, 0x00, 16},  // Analog Input 2
        {0x603F, 0x00, 16},  // Error code
        {}
};

// RxPDO
static ec_pdo_info_t RxPDOs[] = {
        /* RxPdo 0x1600 */
        {0x1600, 5, pdo_entries_Rx},
};

static ec_pdo_info_t TxPDOs[] = {
        /* TxPdo 0x1A00 */
        {0x1A00, 5, pdo_entries_Tx + 0},
        {0x1A01, 5, pdo_entries_Tx + 5}
};

/*
 * output_1 = values written by master
 * input_1  = values written by slaves
 */
static ec_sync_info_t device_syncs[] = {
//        {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
//        {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 1, RxPDOs, EC_WD_DISABLE},
        {3, EC_DIR_INPUT,  2, TxPDOs, EC_WD_DISABLE},
        {0xFF}
};

void releaseMaster(void) {
    if (master) {
        std::cout << std::endl;
        std::cout << "End of program, release master." << std::endl;
        ecrt_release_master(master);
        master = NULL;
    }
}

int configPDO() {
    std::cout << "Configuring PDOs ... " << std::endl;
    domainRx = ecrt_master_create_domain(master);
    if (!domainRx)
        exit(EXIT_FAILURE);
    domainTx = ecrt_master_create_domain(master);
    if (!domainTx)
        exit(EXIT_FAILURE);

    std::cout << "Creating slave configurations ... " << std::endl;

    /*
     * Obtain configuration of slaves
     */
    // slave 1
    if (!(sc[l_hip] = ecrt_master_slave_config(master, SynapticonSlave1, Synapticon))) {
        std::cout << "Failed to get slave 1 configuration. " << std::endl;
        exit(EXIT_FAILURE);
    }

    // slave 2
    if (!(sc[l_knee] = ecrt_master_slave_config(master, SynapticonSlave2, Synapticon))) {
        std::cout << "Failed to get slave 2 configuration. " << std::endl;
        exit(EXIT_FAILURE);
    }

    // slave 3
    if (!(sc[l_ankle] = ecrt_master_slave_config(master, SynapticonSlave3, Synapticon))) {
        std::cout << "Failed to get slave 3 configuration. " << std::endl;
        exit(EXIT_FAILURE);
    }
    // slave 4
    if (!(sc[r_hip] = ecrt_master_slave_config(master, SynapticonSlave4, Synapticon))) {
        std::cout << "Failed to get slave 4 configuration. " << std::endl;
        exit(EXIT_FAILURE);
    }
    // slave 5
    if (!(sc[r_knee] = ecrt_master_slave_config(master, SynapticonSlave5, Synapticon))) {
        std::cout << "Failed to get slave 5 configuration. " << std::endl;
        exit(EXIT_FAILURE);
    }
    // slave 6
    if (!(sc[r_ankle] = ecrt_master_slave_config(master, SynapticonSlave6, Synapticon))) {
        std::cout << "Failed to get slave 6 configuration. " << std::endl;
        exit(EXIT_FAILURE);
    }

    /*
     * Configuring slaves' PDOs
     */
    // slave 1
    if (ecrt_slave_config_pdos(sc[l_hip], EC_END, device_syncs)) {
        std::cout << "Failed to config slave 1 PDOs" << std::endl;
        exit(EXIT_FAILURE);
    }
    // slave 2
    if (ecrt_slave_config_pdos(sc[l_knee], EC_END, device_syncs)) {
        std::cout << "Failed to config slave 2 PDOs" << std::endl;
        exit(EXIT_FAILURE);
    }
    // slave 3
    if (ecrt_slave_config_pdos(sc[l_ankle], EC_END, device_syncs)) {
        std::cout << "Failed to config slave 3 PDOs" << std::endl;
        exit(EXIT_FAILURE);
    }

    // slave 4
    if (ecrt_slave_config_pdos(sc[r_hip], EC_END, device_syncs)) {
        std::cout << "Failed to config slave 4 PDOs" << std::endl;
        exit(EXIT_FAILURE);
    }
    // slave 5
    if (ecrt_slave_config_pdos(sc[r_knee], EC_END, device_syncs)) {
        std::cout << "Failed to config slave 5 PDOs" << std::endl;
        exit(EXIT_FAILURE);
    }
    // slave 6
    if (ecrt_slave_config_pdos(sc[r_ankle], EC_END, device_syncs)) {
        std::cout << "Failed to config slave 6 PDOs" << std::endl;
        exit(EXIT_FAILURE);
    }

//    sdo[l_ankle] = ecrt_slave_config_create_sdo_request(sc[l_ankle],0x3102,2,2);

    if (ecrt_domain_reg_pdo_entry_list(domainRx, domain_Rx_reg)) {
        std::cout << "PDO entry registration failed." << std::endl;
        exit(EXIT_FAILURE);
    }
    if (ecrt_domain_reg_pdo_entry_list(domainTx, domain_Tx_reg)) {
        std::cout << "PDO entry registration failed." << std::endl;
        exit(EXIT_FAILURE);
    }

    return 0;
}


// =================== Function =======================

void check_domain_state(void) {
    ec_domain_state_t ds = {};
    ec_domain_state_t ds1 = {};

    ecrt_domain_state(domainTx, &ds);
    if (ds.working_counter != domainTx_state.working_counter)
        std::cout << "domainTx: WC " << ds.working_counter << std::endl;
    if (ds.wc_state != domainTx_state.wc_state)
        std::cout << "domainTx: state " << ds.wc_state << std::endl;

    domainTx_state = ds;

    ecrt_domain_state(domainRx, &ds1);
    if (ds1.working_counter != domainRx_state.working_counter)
        std::cout << "domainRx: WC " << ds1.working_counter << std::endl;
    if (ds1.wc_state != domainRx_state.wc_state)
        std::cout << "domainRx: state " << ds1.wc_state << std::endl;

    domainRx_state = ds1;
}

void check_master_state(void) {
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
        std::cout << ms.slaves_responding << " slave(s)." << std::endl;
    if (ms.al_states != master_state.al_states)
        std::cout << "AL state : " << ms.al_states << std::endl;
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    master_state = ms;
}

void check_slave_config_states(void) {
    ec_slave_config_state_t s[joint_num];

    for (int j = 0; j < joint_num; j++) {
        ecrt_slave_config_state(sc[j], &s[j]);
        if (s[j].al_state != sc_state[j].al_state)
            printf("Slave %d: State 0x%02X.\n", j, s[j].al_state);
        if (s[j].online != sc_state[j].online)
            printf("Slave %d: %s.\n", j, s[j].online ? "online" : "offline");
        if (s[j].operational != sc_state[j].operational)
            printf("slave %d: %soperational.\n", j, s[j].operational ? "" : "Not ");
        sc_state[j] = s[j];
    }
}

int ActivateMaster(void) {
    int ret;
    std::cout << "Requesting master ... " << std::endl;

    if (master)
        return 0;

    master = ecrt_request_master(0);
    if (!master) {
        return -1;
    }

    configPDO();

    std::cout << "Activating master... " << std::endl;

    if (ecrt_master_activate(master)) {
        exit((EXIT_FAILURE));
        std::cout << "Activating master...failed. " << std::endl;
    }

    if (!(domainTx_pd = ecrt_domain_data(domainTx))) {
        std::cout << "Failed to get domain data pointer. " << std::endl;
    }

    if (!(domainRx_pd = ecrt_domain_data(domainRx))) {
        std::cout << "Failed to get domain data pointer. " << std::endl;
    }

    std::cout << "Activating master...success. " << std::endl;

    return 0;
}

// =================== Interface ======================
// ----------------------------------------------------
// =================== Change OP mode ==================
inline void Change_slave_OP_mode(int slave_port, int mode) {
    EC_WRITE_S8(domainRx_pd + offset.operation_mode[slave_port], mode);
}

void Change_one_side_OP_mode(bool side, int mode) {
    if (left == side) {
        Change_slave_OP_mode(l_hip, mode);
        Change_slave_OP_mode(l_knee, mode);
        Change_slave_OP_mode(l_ankle, mode);
    } else if (right == side) {
        Change_slave_OP_mode(r_hip, mode);
        Change_slave_OP_mode(r_knee, mode);
        Change_slave_OP_mode(r_ankle, mode);
    }
};

void Change_all_slaves_OP_mode(int mode) {
    Change_slave_OP_mode(l_hip, mode);
    Change_slave_OP_mode(l_knee, mode);
    Change_slave_OP_mode(l_ankle, mode);
    Change_slave_OP_mode(r_hip, mode);
    Change_slave_OP_mode(r_knee, mode);
    Change_slave_OP_mode(r_ankle, mode);
};

// ----------------------------------------------------
// =================== Assign Vel [RPM]=====================
inline void Assign_slave_Velocity(int slave_port, double target_velocity) {
    int mode = EC_READ_S8(domainTx_pd + offset.modes_of_operation_display[slave_port]);

    if (CSV == mode)
        EC_WRITE_S32(domainRx_pd + offset.target_velocity[slave_port], target_velocity);
    else {
        std::cout << "Assign action doesn't match current operation mode." << std::endl;
        EC_WRITE_S8(domainRx_pd + offset.operation_mode[slave_port], CSV);
        EC_WRITE_S32(domainRx_pd + offset.target_velocity[slave_port], target_velocity);
    }
}

void Assign_side_slaves_zero_velocity(bool side) {
    if (left == side) {
        Assign_slave_Velocity(l_hip, 0);
        Assign_slave_Velocity(l_knee,0);
        Assign_slave_Velocity(l_ankle,0);
    }else if(right == side){
        Assign_slave_Velocity(r_hip, 0);
        Assign_slave_Velocity(r_knee,0);
        Assign_slave_Velocity(r_ankle,0);
    }
}

// -------------------------------------------------------
// =================== Assign Torque [per thousand]=====================

inline void Assign_slave_Torque(int slave_port, int target_Torque) {
    int mode = EC_READ_S8(domainTx_pd + offset.modes_of_operation_display[slave_port]);

    if (CST == mode)
        EC_WRITE_S32(domainRx_pd + offset.target_velocity[slave_port], target_Torque);
    else {
        std::cout << "Assign action doesn't match current operation mode." << std::endl;
        EC_WRITE_S8(domainRx_pd + offset.operation_mode[slave_port], CST);
        EC_WRITE_S32(domainRx_pd + offset.target_velocity[slave_port], target_Torque);
    }
}

void Assign_side_slaves_zero_Torque(bool side) {
    if (left == side) {
        Assign_slave_Torque(l_hip, 0);
        Assign_slave_Torque(l_knee,0);
        Assign_slave_Torque(l_ankle,0);
    }else if(right == side){
        Assign_slave_Torque(r_hip, 0);
        Assign_slave_Torque(r_knee,0);
        Assign_slave_Torque(r_ankle,0);
    }
}

// -------------------------------------------------------
// =================== Update Pos [cnt]=====================




// =================== Thread   =======================

void cyclic_task(double Kp, double Kd);

int pause_to_continue() {
    std::cout << "Pause Now." << std::endl;
    std::cout << "Press any key to continue..." << std::endl;
    struct termios tm, tm_old;
    int fd = STDIN_FILENO, c;
    if (tcgetattr(fd, &tm) < 0)
        return -1;
    tm_old = tm;
    cfmakeraw(&tm);
    if (tcsetattr(fd, TCSANOW, &tm) < 0)
        return -1;
    c = fgetc(stdin);

    if (tcsetattr(fd, TCSANOW, &tm_old) < 0)
        return -1;
    return c;
}

#endif //EXOS_IMPEDANCECONTROL_EXOS_H
