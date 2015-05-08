/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#ifndef _Recognition_H_
#define _Recognition_H_

#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <time.h>

#include <iostream>
#include <iomanip>

#include "simpleTrackingModule/simpleTrackingModule.h"
#include "simpleTrackingModule/trackingC.h"
#include "state.h"

#include <unordered_map> //Only with C++ 11 or above

using namespace std;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

YARP_DECLARE_DEVICES(icubmod);


class Recognition : public RFModule {

public:
    /** 
     * document your methods too.
     */
    Recognition(ResourceFinder &rf);
    ~Recognition();

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();
    
    bool initPorts(yarp::os::ResourceFinder &rf);
    
private:
  //variables
  simpleTrackingModule tracker;
  
  //Robot head motion method
  string moduleName;
  unordered_map<int, string> relationToString;
  
  bool ICUB;
  bool ROBOT;
  bool BALL;
  
  State *bestState;
    
  string camLeftName;
  BufferedPort<ImageOf<PixelRgb> > camLeft;
    
  string portOutName;
  yarp::os::Port portOut; // a port to receive information
    
  string portWorldName;
  BufferedPort<Bottle> portWorld; // a port to receive information
  double xB, zB;
  double angleB;
  
  //Video capture class
  VideoCapture capture;
  Mat newImg;
  ImageOf<PixelRgb> *imgLeft;
  
  //FPS
  double begin, end;
  
  //States
  std::vector<State> listOfState;
  
  //methods
  void getStateList();
  void cleanStateList();
  void drawStates();
  void drawObjects();
  void addFPS();
  void addObjectAndStateInfo();
  
  void findBestState();
  
  void sendInformation();
  void moveObject();
  
};


#endif // __Recognition_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

