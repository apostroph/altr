/*
* Copyright: none
* Author: Jimmy Baraglia
* CopyPolicy: All copy allowed
*/

#include "Action.h"

#define TABLE_HEIGHT 0.05

#define XA 0.0009375
#define XB -0.3

#define YA 0.0004167
#define YB -0.4

double leftHandOpen[9]  = {30., 16., 0., 10., 0., 20., 0., 0., 20.};
double leftHandClose[9]  = {30., 16., 0., 10., 60., 140., 60., 90., 180.};

double rightHandOpen[9] = {30., 20., 13., 10., 20., 20., 10., 20., 20.};
double rightHandClose[9] = {30., 20., 13., 10., 60., 140., 50., 145., 180.};

YARP_DECLARE_DEVICES(icubmod)

Action::Action(ResourceFinder &rf){
}

Action::~Action(){
	
}

bool Action::openPorts(yarp::os::ResourceFinder &rf){
	YARP_REGISTER_DEVICES(icubmod);

	//init the network
	yarp = new Network();
	Network::init();
	
	moduleName            = rf.check("name", 
			      Value("Action"), 
			      "module name (string)").asString();
	    /*
	* before continuing, set the module name before getting any other parameters, 
	* specifically the port names which are dependent on the module name
	*/
	setName(moduleName.c_str());
	
	// Open OUT
	    portOutName = "/";
	portOutName += getName() + "/Out";

	if (!portOut.open(portOutName.c_str())) {           
	    cout << getName() << ": Unable to open port " << portOutName << endl;  
	    return false;
	}
	
	// Open RPC
	portInName = "/";
	portInName += getName() + "/rpc";

	if (!portIn.open(portInName.c_str())) {           
	    cout << getName() << ": Unable to open port " << portInName << endl;  
	    return false;
	}
	attach(portIn);
    
	return true;
}

bool Action::openControllers(yarp::os::ResourceFinder &rf){
	bool b_robot = rf.find("realRobot").asInt();
	cout<<"Using real robot: "<<b_robot<<endl;
	
	if(b_robot){
		remoteNameLeft="/icub/cartesianController/left_arm";
		remoteNameRight="/icub/cartesianController/right_arm";
		
		remoteNameLeftHand="/icub/left_arm";
		remoteNameRightHand="/icub/right_arm";
	}
	else{
		remoteNameLeft="/icubSim/cartesianController/left_arm";
		remoteNameRight="/icubSim/cartesianController/right_arm";
		
		remoteNameLeftHand="/icubSim/left_arm";
		remoteNameRightHand="/icubSim/right_arm";
	}
		
	localNameLeft="/client/left_arm";
	localNameRight="/client/right_arm";
	
	localNameLeftHand="/client/left_hand";
	localNameRightHand="/client/right_hand";
	
	openCartCon(rf, localNameRight, remoteNameRight, &clientCartCtrlRight, &cartesianCtrlRight);
	    
	openCartCon(rf, localNameLeft, remoteNameLeft, &clientCartCtrlLeft, &cartesianCtrlLeft);
	
	
	if(cartesianCtrlLeft != NULL && cartesianCtrlRight != NULL){
		cout<<"Cartesian ctrl not null"<<endl;
		cartesianCtrlLeft->setPosePriority("position");
		cartesianCtrlRight->setPosePriority("position");
	}else{
		cerr<<"Cartesian ctrl null"<<endl;
	}
	
	openHandCtrl(localNameRightHand, remoteNameRightHand, &clientCartCtrlRightHand, &posRightHand, &velRightHand, &encRightHand);
	openHandCtrl(localNameLeftHand, remoteNameLeftHand, &clientCartCtrlLeftHand, &posLeftHand, &velLeftHand, &encLeftHand);
	
	return true;
}


bool Action::openCartCon(yarp::os::ResourceFinder &rf, string localName, string remoteName, PolyDriver **clientCartCtrl, ICartesianControl **cartesianCtrl){
	Vector dof;
	
	Property option("(device cartesiancontrollerclient)");
	option.put("remote",remoteName.c_str());
	option.put("local",localName.c_str());	

	*clientCartCtrl = new PolyDriver(option);

	*cartesianCtrl = NULL;

	if ((*clientCartCtrl)->isValid()) {
		(*clientCartCtrl)->view(*cartesianCtrl);
		cout<<"CartesianControl Valid"<<endl;
	}else{
		cout<<"CartesianControl non Valid"<<endl;
		return false;
	}
	   
	(*cartesianCtrl)->setPosePriority("position");

	(*cartesianCtrl)->getDOF(dof);
	
	if(rf.find("torso").asInt()){
	      cout<<"The torso is in used"<<endl;
	      enableBody(*cartesianCtrl, &dof);
	}else{
	      cout<<"The torso is not in used"<<endl;
	      desableBody(*cartesianCtrl, &dof);
	}
	
	(*cartesianCtrl)->setTrajTime(2);

	return true;
	
}


bool Action::openHandCtrl(string localName, string remoteName, PolyDriver **clientCartCtrl, IPositionControl **pos, IVelocityControl ** vel, IEncoders **enc){
	
	Property option("(device remote_controlboard)");
	option.put("remote",remoteName.c_str());
	option.put("local",localName.c_str());	

	*clientCartCtrl = new PolyDriver(option);


	if ((*clientCartCtrl)->isValid()) {
		cout<<"HandControl Valid"<<endl;
	}else{
		cout<<"HandControl non Valid"<<endl;
		return false;
	}
	   
	(*clientCartCtrl)->view(*pos);
	(*clientCartCtrl)->view(*vel);
	(*clientCartCtrl)->view(*enc);
	
	if (*pos==0) {
	    cerr<<"Error getting IPositionControl interface."<<endl;
	    return 1;
	}

	int jnts = 0;
	(*pos)->getAxes(&jnts);

	Vector tmp;
	Vector encoders;
	tmp.resize(jnts);
	encoders.resize(jnts);
	
	/* we need to set reference accelerations used to generate the velocity */
	/* profile, here 50 degrees/sec^2 */
	int i;
	for (i = 7; i < jnts; i++) {
		tmp[i] = 50.0;
	}
	(*pos)->setRefAccelerations(tmp.data());
}

bool Action::configure(yarp::os::ResourceFinder &rf){
	
	bool bEveryThingisGood = true;
	
	
    //ICartesianControllers
	bEveryThingisGood = openPorts(rf);
	bEveryThingisGood = openControllers(rf);
	
	//Environment meta-data
	tableHeight = 0.5;
	
	cout<<"Init Action over"<<endl;
    
    return bEveryThingisGood;
}

bool Action::interruptModule(){
	return true;
}   

bool Action::close(){
	cout<<"Closing"<<endl;
	clientCartCtrlLeft->close();
	clientCartCtrlRight->close();
	
	clientCartCtrlRightHand->close();
	clientCartCtrlLeftHand->close();
	return true;
}

bool Action::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply){
	
	if(command.get(0).asString().find("Act1") != -1){
	  
		selectAction(command.get(1).asInt(), command.get(2).asInt(), command.get(3).asDouble(), 
			     command.get(4).asDouble(), command.get(5).asDouble());
	}
	else if(command.get(0).asString().find("Act2") != -1){
	      
		selectAction(command.get(1).asInt(), 0, command.get(2).asDouble(), 
			     command.get(3).asDouble(), command.get(4).asDouble());
		selectAction(command.get(5).asInt(), 1, command.get(6).asDouble(), 
			     command.get(7).asDouble(), command.get(8).asDouble());
	}
	
	if(command.get(0).asString().find("TO") != -1){
		cartesianCtrlRight->waitMotionDone(0.04);
		cartesianCtrlLeft->waitMotionDone(0.04);
	  
	}
	reply.clear();
	reply.addString("ACK");
	return true;
}

void Action::goHome(){
	bool done = false;
	
	moves(cartesianCtrlRight, -0.25, 0.2, 0, 0, -1); //Home position	
	moves(cartesianCtrlLeft, -0.25, -0.2, 0, 0, -1); //Home position
}

void Action::sendActionNode(string name){
	Bottle ACKNode2;
	ACKNode2.addString("Act");
	ACKNode2.addString(name);
	portOut.write(ACKNode2);
}

void Action::sendObsNode(string name){
	Bottle ACKNode2;
	ACKNode2.addString("ObsA");
	ACKNode2.addString(name);
	portOut.write(ACKNode2);
}



    
void Action::a_move(int arm, double x, double y, double z){
	if(arm == 0){
	      moves(cartesianCtrlRight, x, y, z, 1, 0); //Move Right
	}else if(arm == 1){
	      moves(cartesianCtrlLeft, x, y, z, -1, 0); //Move left
	}
}

void Action::a_push(int arm, double x, double y, double z){
	if(arm == 0){
	      moves(cartesianCtrlRight, x, y, z, 0, -1); //Push Right
	}else if(arm == 1){
	      moves(cartesianCtrlLeft, x, y, z, 0, -1); //Push left
	}
}

void Action::a_grasp(int arm){
	Vector encoders;
	int jnts = 0;
	
	posRightHand->getAxes(&jnts);	
	encoders.resize(jnts);
	
	if(arm == 0){
	      encRightHand->getEncoders(encoders.data());
	      for(int i = 7; i < jnts; i++){
		    encoders[i] = rightHandClose[i-7];
	      }
	      posRightHand->positionMove(encoders.data());
	      angleRight = 0;
	}else if(arm == 1){
	      encLeftHand->getEncoders(encoders.data());
	      for(int i = 7; i < jnts; i++){
		    encoders[i] = leftHandClose[i-7];
	      }
	      posLeftHand->positionMove(encoders.data());
	      angleLeft = 0;
	}
}

void Action::a_release(int arm){
	Vector encoders;
	int jnts = 0;
	
	posRightHand->getAxes(&jnts);	
	encoders.resize(jnts);
	
	if(arm == 0){
	      encRightHand->getEncoders(encoders.data());
	      for(int i = 7; i < jnts; i++){
		    encoders[i] = rightHandOpen[i-7];
	      }
	      posRightHand->positionMove(encoders.data());
	      angleRight = 90;
	}else if(arm == 1){
	      encLeftHand->getEncoders(encoders.data());
	      for(int i = 7; i < jnts; i++){
		    encoders[i] = leftHandOpen[i-7];
	      }
	      posLeftHand->positionMove(encoders.data());
	      angleLeft = 90;
	}
}

void Action::start(){
	Bottle ACKNode2;
	ACKNode2.addString("actionStarted");
	portOut.write(ACKNode2);
}

void Action::stop(){
	Bottle ACKNode2;
	ACKNode2.addString("actionStoped");
	portOut.write(ACKNode2);
}

bool Action::enableBody(ICartesianControl* cartesianCtrl, yarp::sig::Vector* dof){
	Vector newDof(3);
	
	newDof[0]=1; // torso pitch: 1 => enable
	newDof[1]=0; // torso roll: 2 => skip
	newDof[2]=1; // torso yaw: 1 => enable
	
	torsoEnabled = true;
	
	return cartesianCtrl->setDOF(newDof,*dof);
}

bool Action::desableBody(ICartesianControl* cartesianCtrl, yarp::sig::Vector* dof){
	Vector newDof(3);
	
	newDof[0]=0; // torso pitch: 1 => skip
	newDof[1]=0; // torso roll: 2 => skip
	newDof[2]=0; // torso yaw: 1 => skip
	
	torsoEnabled = false;
	
	return cartesianCtrl->setDOF(newDof,*dof);
}

bool Action::targetInRange(int arm, double x, double y, double z){
	double xMax = 0.25;
	double yMax = 0.25;
	double zMax = 0.2;
	
	if(torsoEnabled)
	    xMax += 0.20;
	
	if(x > -0.2 || x < -0.2-xMax){
	      return false;
	}else if((arm == 0 && (y > yMax || y < -0.05)) || (arm == 1 && (y > yMax || y > 0.05))){
	      return false;
	}else if(z < 0 || z > zMax){
	      return false;
	}
	
	return true;
}

void Action::moves(ICartesianControl *icart, double x, double y, double z, double orientation, double push)
{
	Vector xd, od;
	icart->getPose(xd,od);
	
	double distance = sqrt(pow(xd[0]-x,2)+pow(xd[1]-y,2)+pow(xd[2]-z,2)); //3D euclidean distance from current position to target
	
	xd[0] = x;  xd[1] = y;  xd[2] = z; 
	
	double trajectoryTime = (1.5/0.2)*distance;
	if(trajectoryTime < 0.5)
	    trajectoryTime = 0.5;
	
	cout<<trajectoryTime<<endl;
	icart->setTrajTime(trajectoryTime); //Trajectory time in seconds. The arm moves 20cm/seconds
	
	yarp::sig::Matrix r(3,3);
	r.zero();
	
	r(0,0) = -1; r(0,1) = 0.0; r(0,2) = 0.0;
	r(1,0) = 0.0; r(1,1) = orientation; r(1,2) = push;
	r(2,0) = 0.0; r(2,1) = -1+(std::abs(orientation)); r(2,2) = -1.0+std::abs(push);

	od = yarp::math::dcm2axis(r);
	
	cout<<"Move to :"<<xd.toString()<<endl;
	
	start();
	icart->goToPose(xd, od);   // send request and forget
	stop();
	
}

bool Action::waitUntilTimeOut(double milliSec, ICartesianControl* cartesianCtrl){
	double timer = 0;
	bool motionSuccess = true;
	
	while(!cartesianCtrl->waitMotionDone(0.01, 0.1)){
		if(timer >= 5){
			motionSuccess = false;
			break;
		}
		timer += 0.1;
	}
	
	double timerStop = yarp::os::Time::now();
	double timerStart = timerStop;
	
	while(1000.*(timerStop-timerStart) < milliSec){
		timerStop = yarp::os::Time::now();
	}
	//usleep(milliSec);
	
	return motionSuccess;
}

void doAction(int actionID){
	
}

bool Action::updateModule(){
	Bottle handsPos;
	Vector xdR, odR;
	Vector xdL, odL;
	
	handsPos.addString("HandsPos");
	
	cartesianCtrlRight->getPose(xdR,odR);
	handsPos.addDouble(xdR[0]);
	handsPos.addDouble(xdR[1]);
	handsPos.addDouble(xdR[2]);
	
	handsPos.addDouble(odR[0]);
	handsPos.addDouble(odR[1]);
	handsPos.addDouble(odR[2]);
	
	cartesianCtrlLeft->getPose(xdL,odL);
	handsPos.addDouble(xdL[0]);
	handsPos.addDouble(xdL[1]);
	handsPos.addDouble(xdL[2]);
	
	handsPos.addDouble(odR[0]);
	handsPos.addDouble(odR[1]);
	handsPos.addDouble(odR[2]);
	
	portOut.write(handsPos);
	
	return true;
}

void Action::selectAction(int ID, int arm, double x, double y, double z){	
	if(targetInRange(arm, x, y, z)){
		switch(ID){
			case 1: sendActionNode("ReachFor"); a_move(arm, x, y, z); break;
			case 2: sendActionNode("Push"); a_push(arm, x, y, z); break;
			case 3: sendActionNode("Grasp"); a_grasp(arm); break;
			case 4: sendActionNode("OpenHand"); a_release(arm); break;
			case 5: sendActionNode("ROOT"); //Send Node
				goHome(); break;
		}
	}else{
		cerr<<"Target not in range"<<endl;
	}
  
}


/**
 * Convert string to double
 * In: 
 * const std::string s: string to convert to double
 * Out: double contained the argument, "nan" if string not a number
 * */
double Action::string_to_double(const std::string& s){
   std::istringstream i(s);
   double x;
   if (!(i >> x)){
	return std::numeric_limits<double>::quiet_NaN();
   }
   return x;
} 

double Action::getPeriod(){
	return 0.5;
}

