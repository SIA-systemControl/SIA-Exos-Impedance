//
// Created by yc on 2021/7/9.
//

#ifndef EXOS_IMPEDANCE_NEW_UTILS_H
#define EXOS_IMPEDANCE_NEW_UTILS_H

void left_shift_array(double *arr, double input, int filter_window) {
    for (int i = 1; i < filter_window; i++) {
        arr[i - 1] = arr[i];
    }
    arr[filter_window - 1] = input;
}

double sum_of_array(double *arr, int begin, int end){
    double sum = 0;
    for (int i = begin; i < end; i++) {
        sum+= arr[i];
    }
    return sum;
}

void Dec2Bin(unsigned int decimal){
    int Bin[16];
    int j = 0;
    while (decimal){
        Bin[j] = decimal%2;
        decimal/=2;
        j++;
    }
    for (int k = 0; k < 8; k++)
        std::cout << Bin[7-k];
    std::cout << std::endl;
}
#endif //EXOS_IMPEDANCE_NEW_UTILS_H
