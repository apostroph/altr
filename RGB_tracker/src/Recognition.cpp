/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#define QUEUE_SIZE 5

#include "Recognition.h"


Recognition::Recognition(ResourceFinder &rf):
angleB(0), heightBall(0), upDown(false)
{    
    namedWindow("Vision and states", CV_WINDOW_AUTOSIZE+WINDOW_OPENGL);
	
    //Init the blob tracker
    tracker = simpleTrackingModule();
    
    //Init the video capture class
    capture = VideoCapture(1); //Webcam 1
    
    Mat frame;
    if (!capture.read(frame)) {
        std::cout << "Unable to retrieve frame from video stream." << std::endl;
	capture = VideoCapture(0); //Webcam 0
	
    }    
    
    relationToString[0] = "going toward";
    relationToString[1] = "going away of";
    relationToString[2] = "moving together with";
    relationToString[3] = "touching";
    
    cout<<"Ready start the Recognition"<<endl;;

    
}

bool Recognition::configure(yarp::os::ResourceFinder &rf) {    
    
	//init the network
	Network::init();
	
	YARP_REGISTER_DEVICES(icubmod);
	
	ICUB = rf.find("ICUB").asInt();
	ROBOT = rf.find("ROBOT").asInt();
	BALL = rf.find("BALL").asInt();
	POSITION = rf.find("POS").asInt();
	MOTOR = rf.find("MOTOR").asInt();
	COLOR = rf.find("COLOR").asInt();
	learning = rf.find("LEARNING").asInt();
	
	cout<<ICUB<<endl;
	
	initPorts(rf);
	
	//Init the blob tracker
	tracker = simpleTrackingModule();
	
	return true;
}

bool Recognition::initPorts(yarp::os::ResourceFinder &rf){
	moduleName            = rf.check("name", 
			      Value("Recognition"), 
			      "module name (string)").asString();
	    /*
	* before continuing, set the module name before getting any other parameters, 
	* specifically the port names which are dependent on the module name
	*/
	setName(moduleName.c_str());

	// Open cam left
	camLeftName = "/";
	camLeftName += getName() + "/in/left";

	if (!camLeft.open(camLeftName.c_str())) {           
	    cout << getName() << ": Unable to open port " << camLeftName << endl;  
	    return false;
	}

	// Open OUT
	portOutName = "/";
	portOutName += getName() + "/out";

	if (!portOut.open(portOutName.c_str())) {           
		cout << getName() << ": Unable to open port " << portOutName << endl;  
		return false;
	}

	if(BALL){
		// Open World
		portWorldName = "/";
		portWorldName += getName() + "/world/out";

		if (!portWorld.open(portWorldName.c_str())) {           
			cout << getName() << ": Unable to open port " << portWorldName << endl;  
			return false;
		}
	}
	
	return true;
}

void Recognition::sendInformation(){
	Bottle visionOutBottle;	
	
	if(learning){
		visionOutBottle.addString("learning");
	}else{
		visionOutBottle.addString("testing");
	}
	
	if(bestState != NULL && bestState->getStrength() > 0.1){
	  
		for(int i = 0; i < 3; i++){
			if(i == bestState->getRelationValue())
				visionOutBottle.addDouble((int)(0.5+bestState->getRValue(i)));
			else
				visionOutBottle.addDouble(0);
		}
		
		if(MOTOR){
			visionOutBottle.addInt(1);
			visionOutBottle.addInt(0);
		}else{
			visionOutBottle.addInt(0);
			visionOutBottle.addInt(0);
		}
		if(POSITION){
		      visionOutBottle.addInt(tracker.getPositionOfBlob(bestState->getID1()).x);
		      visionOutBottle.addInt(tracker.getPositionOfBlob(bestState->getID1()).y);
		      visionOutBottle.addInt(tracker.getTrajectory(bestState->getID1())[tracker.getTrajectory(bestState->getID1()).size()-2].x);
		      visionOutBottle.addInt(tracker.getTrajectory(bestState->getID1())[tracker.getTrajectory(bestState->getID1()).size()-2].y);
		      
		      visionOutBottle.addInt(tracker.getPositionOfBlob(bestState->getID2()).x);
		      visionOutBottle.addInt(tracker.getPositionOfBlob(bestState->getID2()).y);
		      visionOutBottle.addInt(tracker.getTrajectory(bestState->getID2())[tracker.getTrajectory(bestState->getID2()).size()-2].x);
		      visionOutBottle.addInt(tracker.getTrajectory(bestState->getID2())[tracker.getTrajectory(bestState->getID2()).size()-2].y);
		}
		if(COLOR){
		      visionOutBottle.addInt((int)(0.5+(tracker.getRGBColorOfBlob(bestState->getID1())[0]/255.)));
		      visionOutBottle.addInt((int)(0.5+(tracker.getRGBColorOfBlob(bestState->getID1())[1]/255.)));
		      visionOutBottle.addInt((int)(0.5+(tracker.getRGBColorOfBlob(bestState->getID1())[2]/255.)));
		      
		      visionOutBottle.addInt((int)(0.5+(tracker.getRGBColorOfBlob(bestState->getID2())[0]/255.)));
		      visionOutBottle.addInt((int)(0.5+(tracker.getRGBColorOfBlob(bestState->getID2())[1]/255.)));
		      visionOutBottle.addInt((int)(0.5+(tracker.getRGBColorOfBlob(bestState->getID2())[2]/255.)));
		}
		
		uint32_t distance = bestState->getDistance(); 
		distance = distance/43; //640 to 15 (4 bits)
		visionOutBottle.addInt(distance & (1 << 0) ? 1 : 0);
		visionOutBottle.addInt(distance & (1 << 1) ? 1 : 0);
		visionOutBottle.addInt(distance & (1 << 2) ? 1 : 0);
		visionOutBottle.addInt(distance & (1 << 3) ? 1 : 0);
		      
		if(bestState->getRValue(0) != 0 || bestState->getRValue(1) != 0 || bestState->getRValue(2) != 0){
			cout<<visionOutBottle.toString()<<endl;
			portOut.write(visionOutBottle);
		}
	}
	
}

void Recognition::moveObject(){
	Bottle& toWorld = portWorld.prepare();
	
	angleB += (double)(rand()%150)/1000.;
	if(angleB > 2*M_PI)
		angleB = 0;
	
	heightBall += 0.005-(double)(rand()%100)/10000;
	if(heightBall > 0.5)
		heightBall = 0.5;
	else if(heightBall < -0.5)
		heightBall = -0.5;
	
	xB = -0.5+0.5*cos(angleB);
	if(upDown)
	      zB = 1+heightBall+0.1*abs(sin(angleB));
	else
	      zB = 1+heightBall-0.1*abs(sin(angleB));
	
	toWorld.clear();
	
	toWorld.addString("world");
		  
	toWorld.addString("set");
	toWorld.addString("ssph");
	
	toWorld.addInt(1);
	
	toWorld.addDouble(xB); //x
	toWorld.addDouble(zB); //y
	toWorld.addDouble(2); //z
	
	portWorld.write();
}

/* Called periodically every getPeriod() seconds */
bool Recognition::updateModule() {
    
	bestState = NULL;
	
	if(!ICUB){
		capture>>newImg;
		output_cap.write(newImg);
	}else{	
		imgLeft = camLeft.read();
		newImg = Mat((IplImage*)imgLeft->getIplImage(), true);
		
		cvtColor(newImg, newImg, CV_BGR2RGB);
	}
		
	tracker.newImage(newImg);
	  
	cleanStateList();
	getStateList();
	
	drawObjects();
	drawStates();
	
	addObjectAndStateInfo();
	addFPS();
	
	findBestState();
	
	imshow("Vision and states", newImg);
	
	sendInformation();
	if(BALL){
		moveObject();
	}
	
	waitKey(1);
	
    return true;
}

double Recognition::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */    
    return 0.05;
}

bool Recognition::interruptModule() {
    return true;
}

bool Recognition::close() {
	
    output_cap.release();
    return true;
}

bool Recognition::respond(const Bottle& command, Bottle& reply) {
	
   return true;
}

void Recognition::drawObjects(){
	for(int count1 = 0; count1 < tracker.getNumberOfTrackedObject(); count1++){	
		if(tracker.getBlobState(count1) >= 1){
		      circle(newImg, tracker.getPositionOfBlob(count1), 10, Scalar(0, 255, 0, 127), 3);
		      tracker.getTrackedObject(count1)->drawTrajectory(newImg);
		}
	}
}

void Recognition::drawStates(){
	for(unsigned int count = 0; count < listOfState.size(); count++)
	{
		if(listOfState[count].isMarked()){
			Scalar color;
			switch(listOfState[count].getRelationValue()){
			  case 0: color = {0, 255, 0}; break;
			  case 1: color = {0, 0, 255}; break;
			  case 2: color = {255, 0, 0}; break;
			  case 3: color = {255, 255, 0}; break;
			}
			
			if(listOfState[count].getStrength() > 0)
			{
				line(newImg, tracker.getPositionOfBlob(listOfState[count].getID1()), tracker.getPositionOfBlob(listOfState[count].getID2()), color, 2.*listOfState[count].getStrength());
			}
		}
	}
}

void Recognition::addFPS(){
	end = yarp::os::Time::now();
	std::stringstream ss;
	ss << "FPS: "<<(int)(1/(end-begin));
	string s_FPS = ss.str();
	putText(newImg, s_FPS, Point(10, 10), CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(0,0,0));
		
	
	begin = yarp::os::Time::now();
	end = begin;
}

void Recognition::addObjectAndStateInfo(){
	std::stringstream ss;
	ss << "NB_State: "<<(int)listOfState.size();
	string s_state = ss.str();
	putText(newImg, s_state, Point(100, 10), CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(0,0,0));
	
	std::stringstream ss2;
	ss2 << "NB_Obj: "<<(int)tracker.getNumberOfActiveTrackedObject();
	s_state = ss2.str();
	putText(newImg, s_state, Point(200, 10), CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(0,0,0));
}

void Recognition::getStateList(){
	for(int count1 = 0; count1 < tracker.getNumberOfTrackedObject(); count1++){	
		for(int count2 = 0; count2 < tracker.getNumberOfTrackedObject(); count2++){
			if(count1 != count2 && tracker.getBlobState(count1) >= 1 && tracker.getBlobState(count2) >= 1){
				int recognizedIndex = -1;
				
				State newState(tracker.getTrackedObject(count1), count1, tracker.getTrackedObject(count2), count2);
				
				for(unsigned int count3 = 0; count3 < listOfState.size(); count3++)
				{
					if(listOfState[count3].isRecognized(count1, count2)){
						recognizedIndex = count3;
						listOfState[count3].mark();
						break;
					}
				}
				
				if(recognizedIndex != -1)
				{
					listOfState[recognizedIndex].update(tracker.getTrackedObject(count1), tracker.getTrackedObject(count2));
				}
				else
				{
					newState.mark();
					newState.update(tracker.getTrackedObject(count1), tracker.getTrackedObject(count2));
					listOfState.push_back(newState);
				}
			}
		}
	}  
	
}

void Recognition::findBestState(){
	if(listOfState.size() != 0)
	{
		double maxStrength = 0;
		for(unsigned int c = 0; c < listOfState.size(); c++)
		{
			if(listOfState[c].getStrength() > maxStrength)
			{
				bestState = &listOfState[c];
			}
		}
	}
	
	if(bestState != NULL)
	{
		std::stringstream ss;
		ss << "Obj nb"<<bestState->getID1()<<" is "<<relationToString[bestState->getRelationValue()]<<" Obj nb"<<bestState->getID2()<<endl;
		string s_state = ss.str();
		putText(newImg, s_state, Point(20, 20), CV_FONT_HERSHEY_COMPLEX, 0.7, Scalar(0,0,0));
	}
}

void Recognition::cleanStateList(){
	for(unsigned int count = 0; count < listOfState.size(); count++)
	{
		if(!listOfState[count].isMarked()){
			listOfState[count].update();
		}
		
		if(!listOfState[count].isMarked() && listOfState[count].getSizeData() == 0){
			listOfState.erase(listOfState.begin()+count);
			count--;
		}
		else
		{
			listOfState[count].unmark();
		}
	}
}

Recognition::~Recognition(){
  
}

