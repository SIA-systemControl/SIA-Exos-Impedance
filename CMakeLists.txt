cmake_minimum_required(VERSION 3.16)
project(Exos_model_free)

set(CMAKE_CXX_STANDARD 20)

find_package(PythonLibs REQUIRED 2.7)

include_directories(${PYTHON_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(/opt/etherlab/include)
LINK_DIRECTORIES(/opt/etherlab/lib)

add_executable(Exos_model_free main.cpp matplotlibcpp.h exos.h exos.cpp PID.h Unit.h traj_generate.cpp traj_generate.h ButterWorthLP.h ButterWorthLP.cpp Utils.h dataDecode.h)

target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES} ethercat pthread)