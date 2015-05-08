#include "state.h"

#define TIME_WINDOW_SIZE 4
#define MIN_R_VALUE 0.15


State::State(trackingC *obj1, int id1, trackingC *obj2, int id2):
t_object1(obj1), s_object1(id1), t_object2(obj2), s_object2(id2)
{      
      marked = false;
      strength = 0;
      relation = -1;
      
      relationsVector = VectorXd(4);
    
}

bool State::isRecognized(int id1,  int id2) {
	bool b_isRecognized = true;
	
	if (id1 !=  s_object1 ||  id2 !=  s_object2) {
		 b_isRecognized = false; 
	}
	return b_isRecognized;
}

int State::update(trackingC *obj1, trackingC *obj2)
{
	if (marked && obj1 !=  NULL && obj2 !=  NULL) 
	{
		getRelation();
	}
	else if(!marked && timeWindow.size() > 0)
	{
		timeWindow.pop_front();
	}
	
	strength = 0;
	for (unsigned int count = 0; count < timeWindow.size(); count++) {
		strength += timeWindow[count];
	}
	
	return relation;
}

void State::getRelation()
{
	double max = 0;
	int maxR = -1;
	relationsVector(0) = gettingCloser();
	relationsVector(1) = movingAway();
	relationsVector(2) = movingWith();	
	relationsVector(3) = closeTo();
	
	
	for(int count = 0; count < 4; count++)
	{
		  if(relationsVector(count) > max && relationsVector(count) < 2 && relationsVector(count) >= MIN_R_VALUE)
		  {
			  max = relationsVector(count);
			  maxR = count;
		  }
	}
	
	if(maxR != -1)
	{
		//cout << relationsVector << endl;
		if(maxR != relation){
			if(timeWindow.size() > 0)
			{
				timeWindow.pop_front();
			}
			else
			{
				relation = maxR;
				timeWindow.push_back(max);
			}
			strength = 0;
		}
		else
		{
			timeWindow.push_back(max);
			while(timeWindow.size() > TIME_WINDOW_SIZE)
			{
				timeWindow.pop_front();
			}
		}
	      
	}else{
		if(timeWindow.size() > 0)
		{
			timeWindow.pop_front();
		}
	}
}

double State::gettingCloser()
{
      cv::Point p1 = t_object1->getPosition();
      cv::Point p2 = t_object2->getPosition();
      double d1 = sqrt(pow(p1.x-p2.x, 2)+pow(p1.y-p2.y, 2));
      
      double angle1 = atan2(p1.y - p2.y, p1.x - p2.x);
      
      cv::Point vp1 = t_object1->getPosition()+t_object1->getVelocity();
      cv::Point vp2 = t_object2->getPosition()+t_object2->getVelocity();
      double d2 = sqrt(pow(vp1.x-vp2.x, 2)+pow(vp1.y-vp2.y, 2));
      
      double v1 = sqrt(pow(vp1.x-p1.x,2)+pow(vp1.y-p1.y,2));
      double v2 = sqrt(pow(vp2.x-p2.x,2)+pow(vp2.y-p2.y,2));
      
      if(d2 < d1 && ((d1-d2) > 5) && v1 >= v2){
	
	    double distG = gaussian(d2, 0., 20.)/gaussian(0., 0., 20.);
	    double angle2 = atan2(p1.y - vp1.y, p1.x - vp1.x);
	    return distG*gaussian(abs(angle1-angle2), 0., M_PI/4.)/gaussian(0, 0., M_PI/4.);
      }else{
	    return 0;
      }
}

double State::movingAway()
{
      cv::Point p1 = t_object1->getPosition();
      cv::Point p2 = t_object2->getPosition();
      double d1 = sqrt(pow(p1.x-p2.x, 2)+pow(p1.y-p2.y, 2));
      
      double angle1 = atan2(p1.y - p2.y, p1.x - p2.x);
      
      cv::Point vp1 = t_object1->getPosition()+t_object1->getVelocity();
      cv::Point vp2 = t_object2->getPosition()+t_object2->getVelocity();
      double d2 = sqrt(pow(vp1.x-vp2.x, 2)+pow(vp1.y-vp2.y, 2));
      
      double v1 = sqrt(pow(vp1.x-p1.x,2)+pow(vp1.y-p1.y,2));
      double v2 = sqrt(pow(vp2.x-p2.x,2)+pow(vp2.y-p2.y,2));
                 
      if(d2 > d1  && ((d2-d1) > 2) && v1 >= v2){
	    double distG = gaussian(d2, 0., 20.)/gaussian(0., 0., 20.);
	    double angle2 = atan2(p1.y - vp1.y, p1.x - vp1.x);
	    return distG*gaussian(M_PI-abs(angle1-angle2), 0., M_PI/4.)/gaussian(0, 0., M_PI/4.);//*(1./gaussian(0., 0., 3.))*(gaussian(0., 0., 3.)-gaussian((d2-d1), 0., 3.));
      }else{
	    return 0;
      }
}

double State::closeTo()
{
      cv::Point p1 = t_object1->getPosition();
      cv::Point p2 = t_object2->getPosition();
      double d1 = sqrt(pow(p1.x-p2.x, 2)+pow(p1.y-p2.y, 2));
      
      if(d1 < 100){
	    return (1./gaussian(0., 0., 100.))*gaussian(d1, 0., 100.);
      }else{
	    return 0;
      }
}

double State::movingWith()
{
      cv::Point p1 = t_object1->getPosition();
      cv::Point p2 = t_object2->getPosition();
      
      cv::Point vp1 = t_object1->getPosition()+t_object1->getVelocity();
      cv::Point vp2 = t_object2->getPosition()+t_object2->getVelocity();
            
      double angle1 = atan2(p1.y - vp1.y, p1.x - vp1.x);
      double angle2 = atan2(p2.y - vp2.y, p2.x - vp2.x);
      
      double v1 = sqrt(pow(vp1.x-p1.x,2)+pow(vp1.y-p1.y,2));
      double v2 = sqrt(pow(vp2.x-p2.x,2)+pow(vp2.y-p2.y,2));
      
//       cout << abs(angle1-angle2) << endl;
      
      if(abs(angle1-angle2) < M_PI/8 && v1 > 5 && v2 > 5){
//  	cout <<"Velocity difference: " <<  abs(v1-v2) << endl;
	    return gaussian(abs(v1-v2), 0., M_PI)/gaussian(0, 0., M_PI);//*(1./gaussian(0., 0., 3.))*(gaussian(0., 0., 3.)-gaussian((d1-d2), 0., 3.));
      }else{
	    return 0;
      }
}

double State::gaussian(double in, double mean, double std)
{
      return (1./(sqrt(pow(std, 2)*2*M_PI))*exp(-0.5*pow((in-mean)/pow(std, 2), 2)));
}

State::~State(){
    
}