cmake_minimum_required(VERSION 3.5)
project(RobotArmLib)
# The project version number.
set(VERSION_MAJOR   0   CACHE STRING "Project major version number.")
set(VERSION_MINOR   0   CACHE STRING "Project minor version number.")
set(VERSION_PATCH   6   CACHE STRING "Project patch version number.")

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

# build rbkin library
add_library(rbkin SHARED
    src/kin/artic.cpp
    src/kin/artic.h
    src/kin/kinematic_chain.cpp
    src/kin/kinematic_chain.h
    src/kin/link.cpp
    src/kin/link.h
    )
if(EIGEN3_FOUND)
    target_link_libraries(rbkin Eigen3::Eigen)
else(EIGEN3_FOUND)
    target_include_directories( rbkin PUBLIC "/usr/include/eigen3")
endif(EIGEN3_FOUND)

#build test run exec
add_executable(test_run
    test/test_run.cpp
    src/math/polynomial.h
    )
target_link_libraries(test_run rbkin)

# install test run in ./bin
install(TARGETS test_run DESTINATION ${CMAKE_SOURCE_DIR}/bin)
install(TARGETS rbkin DESTINATION ${CMAKE_SOURCE_DIR}/bin)


# Build Doxygen Document
find_package(Doxygen)
if(DOXYGEN_FOUND)
    message("You Have Doxygen!\n")
    set(BUILD_DOC_DIR ${CMAKE_SOURCE_DIR}/build/docs)
    if(NOT EXISTS ${BUILD_DOC_DIR})
        file(MAKE_DIRECTORY ${BUILD_DOC_DIR})
    endif()

    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    message("Doxygen build started")
    add_custom_target(Doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)

    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/docs/html
            DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/docs)
else(DOXYGEN_FOUND)
    message("No Doxygen, can't build Doc!\n")
endif(DOXYGEN_FOUND)
