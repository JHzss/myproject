cmake_minimum_required(VERSION 2.8)
project(myproject)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_models)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#######################dependencies###################
#EIGEN
include_directories("/usr/include/eigen3")
#OPENCV
set(OpenCV_DIR /home/jh/opencv-3.2.0/build)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIR})

#G2O
find_package(G2O REQUIRED)
include_directories(${G20_INCLUDE_DIR})

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        g2o_core g2o_stuff g2o_types_sba
        )
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_library(${PROJECT_NAME}
        src/Camera.cpp
        src/parameters.cpp
        src/Read_dataset.cpp
        src/Feature_tracking.cpp
        )
add_subdirectory( test )
