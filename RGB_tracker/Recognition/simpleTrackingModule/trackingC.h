#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <deque>

#pragma once

using namespace std;
using namespace cv;

typedef cv::Point3_<double> Point3d;

class trackingC
{
public:
	trackingC(Point position, double color, double size);
		
	bool isRecognized(Point position, double color, double size, Mat dst);
	bool isContained(Point position, double color, double size, Mat dst);
	
	bool update(Point position, double color, double size, double height = 0);
	
	int step(Mat img);
	
	bool isAlive();
	bool isFound();
	
	void drawTrajectory(Mat img);
	
	int getState();
	
	double getHue();
	
	Scalar getRGB();
	
	std::deque<Point> getTrajectory();
	
	Point getPosition(){return position;}
	
	Point getVelocity(){return velocity;}
	
	void setShape(string in){shape = in;}
	string getShape(){return shape;}

	~trackingC(void);

private:	
	std::deque<Point> trajectory;
	
	string shape;
	
	Point position;

	
	double height;
	
	Point velocity;
	double heightVelocity;
	
	Point dTraveled;
	
	Rect searchRectangle;

	double pTraveled;

	double approxSize;
	double meanColor;

	bool tracked;
	bool recognized;
	
	bool inMotion;
	bool isVisible;
	bool isOcluded;
	bool isGone;
	bool isMoved;
	
	double recognitionThreshold;
	bool acquired;
	
	time_t timerStart, timerStop;
};

