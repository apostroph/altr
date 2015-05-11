# Copyright: (C) 2010 RobotCub Consortium
# Author: Paul Fitzpatrick
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
# YARP needs CMake 2.8.7 or greater
cmake_minimum_required(VERSION 2.8.7)
# find YARP
find_package(YARP REQUIRED)
find_package(OpenCV REQUIRED )
find_package(ICUB REQUIRED)

add_definitions(-std=gnu++0x)

# add YARP include directories
include_directories(${YARP_INCLUDE_DIRS})

# Predictor
add_executable(Predictor Predictor/main.cpp Predictor/Predictor.cpp Predictor/Predictor.h )
# link with YARP libraries and Opencv Librariesr
target_link_libraries(Predictor ${YARP_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(Predictor icubmod)

install(TARGETS Predictor DESTINATION bin)