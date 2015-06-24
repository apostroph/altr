/*
* Copyright: none
* Author: Jimmy Baraglia
* CopyPolicy: All copy allowed
*/

#ifndef _ACTION_H_
#define _ACTION_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>

#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>

#include <iCub/ctrl/math.h>
#include <cmath>

//typedef cv::Point3_<double> Point3d;

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace iCub::ctrl;

class Action : public RFModule {

public:
	Action(ResourceFinder &rf);
    ~Action();
    
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod();
    bool updateModule();

private:
    string moduleName;
    Network *yarp;
    
    string remoteNameLeft;
    string remoteNameRight;
    
    string localNameLeft;
    string localNameRight;
    
    string remoteNameLeftHand;
    string remoteNameRightHand;
    
    string localNameLeftHand;
    string localNameRightHand;
    
    enum {right = 0, left = 1};
    
    bool targetInRange(int arm, double x, double y, double z);
    
    void moves(ICartesianControl *icart, double x, double y, double z, double orientation, double push);
    void sendActionNode(string name);
    void sendObsNode(string name);
    
    
    string portOutName;
    yarp::os::Port portOut;
    
    string portInName;
    yarp::os::Port portIn;
    
    string portGazeName;
    yarp::os::Port portGaze;
    
    string portPreName;
    yarp::os::Port portPre;
    
    PolyDriver *clientCartCtrlLeft;
    ICartesianControl *cartesianCtrlLeft;
    
    PolyDriver *clientCartCtrlRight;
    ICartesianControl *cartesianCtrlRight;
    
    
    PolyDriver *clientCartCtrlRightHand;
    IPositionControl *posRightHand;
    IVelocityControl *velRightHand;
    IEncoders *encRightHand;
    double angleRight;

    PolyDriver *clientCartCtrlLeftHand;
    IPositionControl *posLeftHand;
    IVelocityControl *velLeftHand;
    IEncoders *encLeftHand;
    double angleLeft;
    
    bool gazeON;
    bool gazeTC;
    void sendHandPos();
        
    //Initialisation methods
    bool openCartCon(yarp::os::ResourceFinder &rf, string localName, string remoteName, PolyDriver **clientCartCtrl, ICartesianControl **cartesianCtrl);
    bool openHandCtrl(string localName, string remoteName, PolyDriver **clientCartCtrl, IPositionControl **pos, IVelocityControl ** vel, IEncoders **enc);
    bool openPorts(yarp::os::ResourceFinder &rf);
    bool openControllers(yarp::os::ResourceFinder &rf);
    
    //Environment meta-data
    double tableHeight;
    bool torsoEnabled;
    
    void a_move(int arm, double x, double y, double z);
    void a_grasp(int arm);
    void a_release(int arm);
    void a_push(int arm,  double x, double y, double z);
    
    void goHome();
    
    void selectAction(int ID, int arm, double x, double y, double z);
    
    bool waitUntilTimeOut(double milliSec, ICartesianControl* cartesianCtrl);
    
    bool enableBody(ICartesianControl* cartesianCtrl, yarp::sig::Vector* dof);
    bool desableBody(ICartesianControl* cartesianCtrl, yarp::sig::Vector* dof);
    
    void start(int arm);
    void stop();
    void gaze(double x, double y, double z);
    
    void doAction(int actionID);
    
    double string_to_double(const std::string& s);
};

#endif

