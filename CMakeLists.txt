cmake_minimum_required(VERSION 3.2)
project(RobotArmLib)

set(CMAKE_CXX_STANDARD 11)
#set(CXX_FLAGS "-g -Wall -Wextra")
#set(CMAKE_CXX_FLAGS ${CXX_FLAGS})

#find extra library (Eigen3) or add diretory
find_package(Eigen3)     #REQUIRED
#add_library(Eigen3 NO_MODULE)
if(EIGEN3_FOUND)
    message("find Eigen3!\n")
else(EIGEN3_FOUND)
    message("No Eigen3. Include directory mannually.\n")
    include_directories(
        /usr/include/eigen3/
        /usr/local/include/eigen3/
        )
endif(EIGEN3_FOUND)

# build rakl library
add_library(rakl SHARED
    src/rakl.cpp
    src/rakl.h
    )
if(EIGEN3_FOUND)
    target_link_libraries(rakl Eigen3::Eigen)
else(EIGEN3_FOUND)
    target_include_directories( rakl PUBLIC "/usr/include/eigen3")
endif(EIGEN3_FOUND)

#build test run exec
add_executable(test_run test/main.cpp)
target_link_libraries(test_run rakl)

