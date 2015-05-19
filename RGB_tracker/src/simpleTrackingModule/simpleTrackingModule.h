/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#ifndef __SIMPLETRACKINGMODULE_H__
#define __SIMPLETRACKINGMODULE_H__

#include <iostream>
#include <fstream>

#include <cv.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/features2d/features2d.hpp"

#include <deque>

#include "trackingC.h"

using namespace std;
using namespace cv;


typedef cv::Point3_<double> Point3d;
typedef Vec<uchar, 4> Vec4b;

class simpleTrackingModule{

public:

//Running methods
    simpleTrackingModule();
    ~simpleTrackingModule();
    
    /** newImage
	 * Call this method to update the tracking
	 * In:
	 * cv::Mat RGB: RGB image 
	 * cv::Mat depth: gray scale image containing the depth information
	 * 
	 * Out: NONE		
	 * 
	 * */
    void newImage(cv::Mat RGB, cv::Mat depth = cv::Mat());
    
//Get methods
     
     /**
      * Return the number of tracked objects
      * The objects no longer observervable are also counted
      * */
     int getNumberOfTrackedObject();
     
     /**
      * Return the number of active tracked objects
      * The objects no longer observervable are not counted
      * */
     int getNumberOfActiveTrackedObject();
     
     /**
      * Return the state of the tracked object at indexB
      * 0 => Gone
      * 1 => Occluded
      * 2 => Moving
      * 3 => stationary
      * 
      * in: indexB in the list of tracked object
      * */
     int getBlobState(int indexB);
     
     /**
      * Return the state of the tracked object at indexB
      * 
      * in: indexB in the list of tracked object
      * */
     int isBlobFound(int indexB){return objectTrajectories[indexB][0].isFound();}
     
     /**
      * Return the 3D position of the tracked object at indexB
      * in: indexB in the list of tracked object
      * */
     Point3d get3DPositionOfBlob(int indexB); 
     
     /**
      * Return the 2D position of the tracked object at indexB
      * in: indexB in the list of tracked object
      * */
     Point getPositionOfBlob(int indexB);
     
     /**
      * Return the RGB color of the tracked object at indexB
      * in: indexB in the list of tracked object
      * */
     Scalar getRGBColorOfBlob(int indexB);
     
     /**
      * Return the Hue of the tracked object at indexB
      * in: indexB in the list of tracked object
      * */
     double getHueColorOfBlob(int indexB);
     
     /**
      * Return the tracked object of the tracked object at indexB
      * in: indexB in the list of tracked object
      * */
     trackingC *getTrackedObject(int indexB){ return &(objectTrajectories[indexB][0]);}
     
     /**
      * Return the trajectory of the tracked object at indexB
      * in: indexB in the list of tracked object
      * */
     std::deque<Point> getTrajectory(int indexB);
     
// Display methods

private:
// Blob detector methods and variables

	bool initBlobDetector(const char* src);//inits the blob detector
	double getBlobColor(Mat imgBlob, Mat imgRGB, double size, Point position);//gets the color of a blob
	double getMeanColor(Mat ROI);
	
	SimpleBlobDetector::Params params; //parameters for the blob detector
	SimpleBlobDetector blob_detector; //blob detector
	vector<cv::KeyPoint> keypoints;
	
	std::vector< std::deque<trackingC> > objectTrajectories; //list of all tracked object
	
// Img processing methods ad variables
	void thresholdF(Mat src, Mat &dst); //Remove light saturation
	void skinRemvoal(Mat src, Mat &dst);
	void motionFilter(Mat src1, Mat src2);
	
	void blobDetector(Mat image); // Extract all the blobs
     
	/**
	  * Return the average height of a blob in the image objects
	  * */
	double getHeightOfBlob(Mat ROI, double size);
	
	Mat inputRGB;
	Mat pastImage;
	long long timeStep;
	
// Other methods

	double string_to_double(const std::string& s); 
	
//boolean for image process control
	int sH, sL;
	int vH, vL;
	int hueL, hueH;
	
	int hue;
	
	int medianBlurV;
	
	int open, close;
};


#endif // __SIMPLETRACKINGMODULE_H__
