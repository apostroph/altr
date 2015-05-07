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

# Recognition
add_executable(Recognition Recognition/main.cpp Recognition/Recognition.cpp Recognition/Recognition.h 
Recognition/state.h Recognition/state.cpp
Recognition/simpleTrackingModule/simpleTrackingModule.h Recognition/simpleTrackingModule/simpleTrackingModule.cpp 
Recognition/simpleTrackingModule/trackingC.h Recognition/simpleTrackingModule/trackingC.cpp
Recognition/state.h Recognition/state.cpp)
# link with YARP libraries and Opencv Librariesr
target_link_libraries(Recognition ${YARP_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(Recognition icubmod)

install(TARGETS Recognition DESTINATION bin)