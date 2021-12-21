cmake_minimum_required(VERSION 3.16)
project(SIA_Exos_Gait)

set(CMAKE_CXX_STANDARD 20)

INCLUDE_DIRECTORIES(/opt/etherlab/include)
LINK_DIRECTORIES(/opt/etherlab/lib)

add_executable(SIA_Exos_Gait main.cpp exos.cpp exos.h dynamics.h dataDecode.h ButterWorthLP.cpp ButterWorthLP.h PID.h traj_generate.cpp traj_generate.h Unit.h Utils.h ErrorParse.h)

target_link_libraries(${PROJECT_NAME} ethercat pthread)