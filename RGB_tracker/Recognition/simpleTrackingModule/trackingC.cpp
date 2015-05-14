#include "trackingC.h"

#define T_POSITION 1
#define T_COLOR 1

#define CHOSE_COLOR 0

#define MAX_QUEUE 10


trackingC::trackingC(Point in_position, double color, double size):
position(in_position), approxSize(size), heightVelocity(0), meanColor(color), isVisible(false), isOcluded(true),
isGone(false), isMoved(false), acquired(false), pTraveled(0), recognized(false)
{
	dTraveled.x = 0;
	dTraveled.y = 0;
	velocity.x = 0;
	velocity.y = 0;
	
	time(&timerStart);
	timerStop = timerStart;
}

bool trackingC::isRecognized(Point position, double color, double size, Mat dst){
	recognized = false;
	
	searchRectangle.width = 3*approxSize + abs(3*velocity.x);
	searchRectangle.height = 3*approxSize + abs(3*velocity.y);
	
	if((!isGone && !tracked && (isOcluded || (position.x > (this->position.x - searchRectangle.width)) && 
	  (position.x < (this->position.x + searchRectangle.width)) && 
	  (position.y > (this->position.y - searchRectangle.height)) && 
	  (position.y < (this->position.y + searchRectangle.height))))){	  
	  
		if(((color > (meanColor-5)) && (color < (meanColor+5)))){
		    tracked = true;
		    if(!acquired){
			    recognitionThreshold ++;
		    }
		    if(recognitionThreshold > 20){
			    acquired = true;
			    recognitionThreshold = 0;
		    }
		    recognized = true;
		}
	}
	
	time(&timerStop);
	if((double)(timerStop-timerStart) > 3 && !acquired){
		isGone = true;
	}
	
	return recognized;
}


bool trackingC::isContained(Point position, double color, double size, Mat dst){	
	bool contained = false;
	
	searchRectangle.width = 3*approxSize + abs(3*velocity.x);
	searchRectangle.height = 3*approxSize + abs(3*velocity.y);
	
	if(tracked && (position.x > (this->position.x - searchRectangle.width)) && 
	  (position.x < (this->position.x + searchRectangle.width)) && 
	  (position.y > (this->position.y - searchRectangle.height)) && 
	  (position.y < (this->position.y + searchRectangle.height))){	  
	  
		if(((color > (meanColor-5)) && (color < (meanColor+5)))){
		    contained = true;
		}
	}
	
	return contained;
}

bool trackingC::update(Point position, double color, double size, double height){
	if(!isGone){ //If the object is not lost
		if(!acquired){
		    //If the object has not been detected
		    isVisible = false;
		    isOcluded = true;
		    isGone = false;
		    isMoved = false; 
		}else{
		    //If the object has been detected
		    isVisible = true;
		    isOcluded = isGone = false;
		    
		    //Variable are updated
		    
		    //The color of the object is updated
		    this->meanColor = color;
		    
		    //The varaible for the motion detection is updated
		    this->dTraveled.x = 0.99*this->dTraveled.x + this->position.x-position.x;
		    this->dTraveled.y = 0.99*this->dTraveled.y + this->position.y-position.y;

		    pTraveled = sqrt((double) ((dTraveled.x*dTraveled.x) + (dTraveled.y*dTraveled.y)));
		    
		    //The velocity is updated
		    this->velocity.x = position.x-this->position.x;
		    this->velocity.y = position.y-this->position.y;
		    heightVelocity = height-this->height;
		    
		    //The position is updated
		    this->position.x = position.x;
		    this->position.y = position.y;
		    
		    approxSize = size;
		    
		    //3D values are updated
		    this->height = height;	
		    
		    //The trajectory points if the object position changed
		    trajectory.push_back(this->position);
		    if(trajectory.size() > MAX_QUEUE){
			  trajectory.pop_front();
		    }
		    
		    //The object is detected in motion if the distance traveled is above 20
		    if( pTraveled > 20){
			    isMoved = true;
			    inMotion = true;
			    
			    dTraveled.x = dTraveled.y = 0;
		    }
		    else{
			    inMotion = isMoved = false;
		    }
		}
		return true;
	}
	return true;
}

int trackingC::step(Mat img){
	if(tracked){
		//If the object was recogized, the timer is initialized
		time(&timerStart);
		timerStop = timerStart;
	}else{
		//If the object was not recogized, the timer is updated
		time(&timerStop);
		if((double)(timerStop-timerStart) > 1 && !isOcluded){
			//If the object is no recognized for 1s, it becomes ocluded
			isOcluded = true;
			isVisible = false;
			trajectory.clear();
		}else if((double)(timerStop-timerStart) > 1.5 && !isGone){
			//If the object is no recognized for 1s, it is lost
			if(CHOSE_COLOR == 0)
			    isGone = true;
		}
	}
	
	//Anotating the objects' position and state on the image
	/*if(!isGone && isVisible && !isMoved){
		circle(img, this->position, 10, Scalar(0, 200, 0), -1);
		line(img, position, Point(position.x+velocity.x, position.y+velocity.y), Scalar(255,255,255), 3);
		putText(img,  "L", Point(position.x, position.y-10), CV_FONT_HERSHEY_COMPLEX, 1.2, Scalar(0,255,0));
	}else if (!isGone && isVisible && isMoved){
		putText(img,  "M", Point(position.x-10, position.y-10), CV_FONT_HERSHEY_COMPLEX, 1.2, Scalar(0,255,0));
		line(img, position, Point(position.x+velocity.x, position.y+velocity.y), Scalar(255,255,255), 3);
	}else if(!isGone && isOcluded){
		putText(img,  "X", Point(position.x-10, position.y-10), CV_FONT_HERSHEY_COMPLEX, 1.2, Scalar(0,255,0));
	}*/
	
	//re-initializing the tracked value for the next detection step
	tracked = false;

	return 0;
}

void trackingC::drawTrajectory(Mat img){
	for(int count = 1; count < trajectory.size(); count++){
	    int thickness = 3;
	    line(img, trajectory[count-1], trajectory[count], Scalar(180, 0, 0), thickness);
	}
}

int trackingC::getState(){
	if(isVisible && !isMoved && !isGone && !isOcluded){
		return 1;
	}else if(isVisible && isMoved && !isGone && !isOcluded){
		return 2;
	}else if(isOcluded && !isGone){
		return 0;
	}else if(isGone){
		return -1;
	}else if(recognitionThreshold < 0){
		return -1;
	}else{
		return -2;
	}
}

bool trackingC::isAlive(){
     return !isGone;
}


bool trackingC::isFound(){
     return recognized;
}
 
 
double trackingC::getHue(){
      return meanColor;
  
}

Scalar trackingC::getRGB(){
      double R, G, B;
      if(meanColor <= 60){
	      R = 255;
	      G = 4.25*meanColor;
	      B = 0;
      }else if(meanColor <= 120){
	      R = 255-(4.25*(meanColor-60));
	      G = 255;
	      B = 0;
      }else if(meanColor <= 180){
	      R = 0;
	      G = 255;
	      B = (4.25*(meanColor-120));
      }
      Scalar RGB(R,G,B);
      return RGB;
}

std::deque<Point> trackingC::getTrajectory(){
      return trajectory;
}


trackingC::~trackingC(void)
{
}
