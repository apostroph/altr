/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#define QUEUE_SIZE 5

#include "Predictor.h"


Predictor::Predictor(ResourceFinder &rf)
{    
    
}

bool Predictor::configure(yarp::os::ResourceFinder &rf) {    
    
	//init the network
	Network::init();
	
	YARP_REGISTER_DEVICES(icubmod);
	
	initPorts(rf);
	
	return true;
}

bool Predictor::initPorts(yarp::os::ResourceFinder &rf){
	moduleName            = rf.check("name", 
			      Value("Predictor"), 
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

	return true;
}

/* Called periodically every getPeriod() seconds */
bool Predictor::updateModule() {
    
	
    return true;
}

double Predictor::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */    
    return 0.05;
}

bool Predictor::interruptModule() {
    return true;
}

bool Predictor::close() {
	
    return true;
}

bool Predictor::respond(const Bottle& command, Bottle& reply) {
	
   return true;
}

Predictor::~Predictor(){
  
}

