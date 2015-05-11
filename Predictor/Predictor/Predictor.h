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

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

YARP_DECLARE_DEVICES(icubmod);


class Predictor : public RFModule {

public:
    /** 
     * document your methods too.
     */
    Predictor(ResourceFinder &rf);
    ~Predictor();

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();
    
    bool initPorts(yarp::os::ResourceFinder &rf);
    
private:
  string moduleName;
  
  string portOutName;
  yarp::os::Port portOut; // a port to receive information
 
  
  void findBestState();
  
  void sendInformation();
  void moveObject();
  
};


#endif // __Recognition_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

