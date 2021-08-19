#include <iostream>
#include <fstream>
#include <sstream>
#include "dataDecode.h"


pthread_mutex_t mutex;
pthread_cond_t cond;

void *robotcontrol(void *arg);

int main(int argc, char *argv[]) {
    /* Thread-related operation */
    pthread_mutex_init(&mutex, nullptr);

    pthread_t robot;            // new pthread object
    pthread_attr_t attr_robot;  // new pthread object attribute

    double Kd, Kp;
    Kp = 0;
    Kd = 0;
    switch (argc) {
        case 1: {
            std::cout << "default P-D params." << std::endl;
            Kp = 0;
            Kd = 0;
            break;
        }
        case 2: {
            std::cout << "custom P params." << std::endl;
            Kp = std::stod(argv[1]);
            Kd = 0;
            std::cout << Kp << std::endl;
            break;
        }
        case 3: {
            std::cout << "custom P-D params." << std::endl;
            Kp = std::stod(argv[1]);
            Kd = std::stod(argv[2]);
            std::cout << Kp << " ; " << Kd << std::endl;
            break;
        }
        default:
            break;
    }

    struct PD_para pdPara{};
    pdPara.Kp = Kp;
    pdPara.Kd = Kd;

    if (pthread_attr_init(&attr_robot) != 0)
        perror("[ROBOT-THREAD INIT FAILURE!]");

    if (pthread_create(&robot, &attr_robot, robotcontrol, &pdPara) != 0) {
        perror("[ROBOT-THREAD CREATE FAILURE!]");
        return EXIT_FAILURE;
    }

    pthread_join(robot, nullptr); // wait for releasing

    std::cout << "[ROBOT-THREAD DESTROYED!]" << std::endl;

    pthread_attr_destroy(&attr_robot);

    pthread_cond_destroy(&cond);
    pthread_mutex_destroy(&mutex);

    return EXIT_SUCCESS;
}
