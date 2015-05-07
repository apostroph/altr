# Copyright: (C) 2010 RobotCub Consortium
# Author: Paul Fitzpatrick
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
# YARP needs CMake 2.8.7 or greater
cmake_minimum_required(VERSION 2.8.7)
# find YARP
find_package(YARP REQUIRED)
find_package( OpenCV REQUIRED )
find_package(ICUB)
# add YARP include directories
include_directories(${YARP_INCLUDE_DIRS})

# Action
add_executable(Action Action/main.cpp Action/Action.h Action/Action.cpp)
# link with YARP libraries
target_link_libraries(Action ${YARP_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(Action icubmod ctrlLib)


