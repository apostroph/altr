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
add_executable(Recognition RGB_tracker/src/main.cpp RGB_tracker/src/Recognition.cpp RGB_tracker/src/Recognition.h 
RGB_tracker/src/state.h RGB_tracker/src/state.cpp
RGB_tracker/src/simpleTrackingModule/simpleTrackingModule.h RGB_tracker/src/simpleTrackingModule/simpleTrackingModule.cpp 
RGB_tracker/src/simpleTrackingModule/trackingC.h RGB_tracker/src/simpleTrackingModule/trackingC.cpp
RGB_tracker/src/state.h RGB_tracker/src/state.cpp)
# link with YARP libraries and Opencv Librariesr
target_link_libraries(Recognition ${YARP_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(Recognition icubmod)

# Predictor
add_executable(Predictor Predictor/src/main.cpp Predictor/src/Predictor.cpp Predictor/src/Predictor.h )
# link with YARP libraries and Opencv Librariesr
target_link_libraries(Predictor ${YARP_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(Predictor icubmod)

install(TARGETS Predictor DESTINATION bin)

# Action
add_executable(Action Action/src/main.cpp Action/src/Action.h Action/src/Action.cpp)
# link with YARP libraries
target_link_libraries(Action ${YARP_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(Action icubmod ctrlLib)

install(TARGETS Action DESTINATION bin)

# Gaze
add_executable(Gaze Gaze/src/main.cpp Gaze/src/Gaze.h Gaze/src/Gaze.cpp)
# link with YARP libraries
target_link_libraries(Gaze ${YARP_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(Gaze icubmod ctrlLib)

install(TARGETS Gaze DESTINATION bin)

# Gaze
add_executable(Test Test/main.cpp Test/test.cpp Test/test.h)
# link with YARP libraries
target_link_libraries(Test ${YARP_LIBRARIES})

install(TARGETS Test DESTINATION bin)