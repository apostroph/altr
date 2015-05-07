/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#ifndef _STATE_H_
#define _STATE_H_

#include <string>

#include <iostream>
#include <iomanip>

#include "simpleTrackingModule/trackingC.h"
#include <eigen3/Eigen/Dense>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

//#include <unordered_map> //Only with C++ 11 or above

using namespace std;
using namespace cv;
using namespace Eigen;


class State{

public:
    /** 
     * document your methods too.
     */
    State(trackingC *obj1, int id1, trackingC *obj2, int id2);
    
    int update(trackingC *obj1 = NULL, trackingC *obj2 = NULL);
    
    int getID1(){return s_object1;}
    int getID2(){return s_object2;}
    double getStrength(){ return strength;}
    int getRelationValue(){return relation;}
    
    int getSizeData(){return timeWindow.size();}
    
    bool isRecognized(int id1,  int id2);
    
    void mark(){marked = true;}
    void unmark(){marked = false;}
    
    bool isMarked(){return marked;}
    
    ~State();


private:
  
  //state identification variables
  bool marked;
  trackingC *t_object1;
  int s_object1;
  
  std::string s_relation;
  
  trackingC *t_object2;
  int s_object2;
  
  VectorXd relationsVector;
  
  int relation;
  std::deque<double> timeWindow;
  double strength;
  
  bool changed;
  
  //methods
  void getRelation();
  double gettingCloser();
  double movingAway();
  double closeTo();
  double movingWith();
  
  double gaussian(double in, double mean, double variance);
  
 
};


#endif // _STATE_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

