//
// Created by yc on 2021/7/9.
//

#ifndef EXOS_IMPEDANCE_NEW_UTILS_H
#define EXOS_IMPEDANCE_NEW_UTILS_H

#define myDEBUG 0

#include <Eigen/Dense>

#define doubleStance 1
#define STO 2
#define SSw 3
#define HOHS 4
#define TOS 5
#define SwS 6
#define HSHO 7
#define Wrong 8



typedef enum _GaitFSM {
    Stance, // 0
    HS_HO,  // 1
    ST_T,   // 2
    ST_S,   // 3
    HO_HS,  // 4
    T_ST,   // 5
    S_ST,   // 6
} GaitFSM;

typedef struct _GaitMatrixFSM {
    GaitFSM m_gaitMatrixFsm;
} gaitMatrixFSM;

gaitMatrixFSM gMFSM;

void left_shift_array(double *arr, double input, int filter_window) {
    for (int i = 1; i < filter_window; i++) {
        arr[i - 1] = arr[i];
    }
    arr[filter_window - 1] = input;
}

double sum_of_array(double *arr, int begin, int end) {
    double sum = 0;
    for (int i = begin; i < end; i++) {
        sum += arr[i];
    }
    return sum;
}

void Dec2Bin(unsigned int decimal) {
    int Bin[10];
    int j = 0;
    while (decimal) {
        Bin[j] = decimal % 2;
        decimal /= 2;
        j++;
    }
    for (int k = 0; k < 8; k++)
        std::cout << Bin[7 - k];
    std::cout << std::endl;
}

void Dec2Hex(unsigned int decimal) {
    int Hex[16];
    int j = 0;
    while (decimal) {
        Hex[j] = decimal % 16;
        decimal /= 16;
        j++;
    }
    for (int k = 0; k < 4; k++)
        std::cout << Hex[3 - k];
    std::cout << std::endl;
}

inline int Pressure_Threshold(unsigned int In) {
    if (In > 120)
        return 1;
    else
        return 0;
}

Eigen::Matrix2d
UpdateGaitMatrix(unsigned int An_Left_1, unsigned int An_Left_2, unsigned int An_Right_1, unsigned int An_Right_2) {
    int left_1 = Pressure_Threshold(An_Left_1);
    int left_2 = Pressure_Threshold(An_Left_2);
    int right_1 = Pressure_Threshold(An_Right_1);
    int right_2 = Pressure_Threshold(An_Right_2);

    Eigen::Matrix2d GaitMatrix;

    GaitMatrix << left_1, right_1, left_2, right_2;

    return GaitMatrix;
}

int GaitMatrixParse(Eigen::Matrix2d GaitMatrix) {
    /**
     * [1 1; 1 1] --> 2xStance
     * ========================================
     * [1 1; 1 0] --> (Stance,Toe-off) = STO
     * [1 0; 1 0] --> (Stance,Swing) = SSw
     * [1 0; 0 1] --> (Heel-off, Heel-strike) = HOHS
     * [1 1; 0 1] --> (Toe-off, Stance) = TOS
     * [0 1; 0 1] --> (Swing, Stance) = SwS
     * [0 1; 1 0] --> (Heel-Strike, Heel-off) = HSHO
     * [0 0; 0 0] --> Wrong
     */

    Eigen::Matrix2d m2xStance, mSTO, mSSw, mHOHS, mTOS, mSws, mHSHO, mempty;
    m2xStance << 1, 1, 1, 1;
    mSTO << 1, 1, 1, 0;
    mSSw << 1, 0, 1, 0;
    mHOHS << 1, 0, 0, 1;
    mTOS << 1, 1, 0, 1;
    mSws << 0, 1, 0, 1;
    mHSHO << 0, 1, 1, 0;
    mempty << 0, 0, 0, 0;

    if (myDEBUG){
        if (GaitMatrix == m2xStance) {
            std::cout << "Gait Phase: doubleStance" << std::endl;
            return doubleStance;
        }
        else if (GaitMatrix == mSTO) {
            std::cout << "Gait Phase: (Stance,Toe-off)" << std::endl;
            return STO;
        }
        else if (GaitMatrix == mSSw) {
            std::cout << "Gait Phase: (Stance,Swing)" << std::endl;
            return SSw;
        }
        else if (GaitMatrix == mHOHS) {
            std::cout << "Gait Phase: (Heel-off, Heel-strike)" << std::endl;
            return HOHS;
        }
        else if (GaitMatrix == mTOS) {
            std::cout << "Gait Phase: (Toe-off, Stance)" << std::endl;
            return TOS;
        }
        else if (GaitMatrix == mSws) {
            std::cout << "Gait Phase: (Swing, Stance)" << std::endl;
            return SwS;
        }
        else if (GaitMatrix == mHSHO) {
            std::cout << "Gait Phase: (Heel-Strike, Heel-off)" << std::endl;
            return HSHO;
        }
        else if (GaitMatrix == mempty) {
            std::cout << "Gait Phase: Wrong" << std::endl;
            return Wrong;
        }
        else{
            return 0;
        }
    } else{
        if (GaitMatrix == m2xStance) {
            return doubleStance;
        }
        else if (GaitMatrix == mSTO) {
            return STO;
        }
        else if (GaitMatrix == mSSw) {
            return SSw;
        }
        else if (GaitMatrix == mHOHS) {
            return HOHS;
        }
        else if (GaitMatrix == mTOS) {
            return TOS;
        }
        else if (GaitMatrix == mSws) {
            return SwS;
        }
        else if (GaitMatrix == mHSHO) {
            return HSHO;
        }
        else if (GaitMatrix == mempty) {
            return Wrong;
        }
        else{
            return 0;
        }
    }


}

//int GaitPhaseParse(int MatrixRet, double GaitCycle){
//    if (MatrixRet == SwS && GaitCycle > 0.35)
//        gMFSM.m_gaitMatrixFsm = ST_S;
//    if (MatrixRet == HSHO && GaitCycle > 0.1)
//        gMFSM.m_gaitMatrixFsm = HS_HO;
//}

#endif //EXOS_IMPEDANCE_NEW_UTILS_H
