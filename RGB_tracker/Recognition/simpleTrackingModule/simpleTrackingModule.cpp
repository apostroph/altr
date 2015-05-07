/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#include "simpleTrackingModule.h"

//the size fo the images is divided by this value
#define SUB 1

//Size of object tracking trajectory window 
#define sizeTrajectory 10

//Variables for the skin removale
#define RED_MIN 95
#define GREEN_MIN 40
#define BLUE_MIN 20
#define MAX_MIN_SPAN 15
#define MIN_VALUE 15

#define MAX_TRACK_NB 50


simpleTrackingModule::simpleTrackingModule():
sL(100), sH(255), vL(20), vH(255), hueL(0), hueH(180), medianBlurV(6), open(1), close(4)
{
	
	initBlobDetector("../Recognition/initialization_files/blobDetector.ini");
		
	namedWindow("RGB2", CV_WINDOW_AUTOSIZE+WINDOW_OPENGL);
	
	timeStep = 0;
}

/** newImage
 * Call this method to update the tracking
 * In:
 * cv::Mat RGB: RGB image 
 * cv::Mat depth: gray scale image containing the depth information
 * 
 * Out: NONE
 * 
 * */
void simpleTrackingModule::newImage(cv::Mat RGB, cv::Mat depth){
	Mat smallImage;
	Mat RGBsmall;
	Mat smallImageGray;
	
	//The image matrices are initialized
	smallImageGray = Scalar(0, 0, 0);
	smallImage = Scalar(0, 0, 0);
	
	//If the image isn't empty
	if(!RGB.empty()){
		RGB.copyTo(inputRGB);
		
		//The image matrice is resize to improve the computational time
		resize(inputRGB, smallImage, Size((inputRGB.cols)/SUB, (inputRGB.rows)/SUB));		
		smallImage.copyTo(RGBsmall);
		
		//The image goes through a complex thresholding 
		thresholdF(smallImage, smallImageGray);	
		
// 		if(timeStep > 0)
// 			motionFilter(pastImage, RGBsmall);
		
		//The blobs are detected
		blobDetector(smallImageGray);
		
		for(int count = 0; count < keypoints.size(); count ++){
			bool assigned = false;
			
			double color = getBlobColor(smallImageGray, smallImage, keypoints[count].size, keypoints[count].pt);
			if(color != -1){
				for(int cT = 0; cT < objectTrajectories.size(); cT++){
					if(objectTrajectories[cT][0].isAlive() && objectTrajectories[cT][0].isRecognized(SUB*keypoints[count].pt, color, SUB*keypoints[count].size, RGBsmall)){
						assigned = true;
						objectTrajectories[cT][0].update(SUB*keypoints[count].pt, color, SUB*keypoints[count].size, 0);
						
						break;
					}
				}
				
				if(!assigned){
					bool containedBlob = false;
					for(int cT = 0; cT < objectTrajectories.size(); cT++){
						if(objectTrajectories[cT][0].isContained(SUB*keypoints[count].pt, color, SUB*keypoints[count].size, RGBsmall)){
							containedBlob = true;
							break;
						}
					}
					if(!containedBlob){
						trackingC newTracker = trackingC(SUB*keypoints[count].pt, color, SUB*keypoints[count].size);
						
						std::deque<trackingC> newTrajectory;
						newTrajectory.push_front(newTracker);
						objectTrajectories.push_back(newTrajectory);
					}
				}
			}
		}
		
		for(int cT = 0; cT < objectTrajectories.size(); cT++){
			objectTrajectories[cT][0].step(RGBsmall);
			
			//Delete the tracker if inactive
			if(objectTrajectories[cT][0].getState() == -1){
			    objectTrajectories.erase(objectTrajectories.begin()+cT);
			    cT--;
			}
		}
		
		RGBsmall.copyTo(pastImage);
		timeStep ++;
		waitKey(1);
		
	}else{
		cerr<<"Error RGB image not found"<<endl;
	}
}


/** saturationThreshold
 * Remove high saturation values
 * in:
 * Mat src: source image
 * int min: HSV min saturation
 * int max: HSV max saturation
 * 
 * out:
 * Mar dst: destination image
 * */
void simpleTrackingModule::thresholdF(Mat src, Mat &dst){
	
	Mat noSkin = cv::Mat(Size((src.cols), (src.rows)), CV_8UC3);  
	Mat colorBlobs = cv::Mat(Size((src.cols), (src.rows)), CV_8UC3);  
	skinRemvoal(src, noSkin);//We remove the skin color
	
	//The matrice pixels are convert from BGR to HSV
	cvtColor(noSkin, noSkin, CV_RGB2HSV);
	
	inRange(noSkin, Scalar(hueL,sL,vL), Scalar(hueH,sH,vH), dst); 
		
	medianBlur ( dst, dst, medianBlurV*3+1 );
	
	erode(dst, dst, cv :: Mat (), cv :: Point ( - 1 , - 1 ), 3 );
	dilate(dst, dst, cv :: Mat (), cv :: Point ( - 1 , - 1 ), 3 );
	
	
	//Add a black border to the image
	rectangle(dst, Point(0, 0), Point(dst.cols, dst.rows), Scalar(0, 0, 0), 10);
	
	Mat cannyOut;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	
	//Find the contour of all the shapes
	findContours(dst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	
	dst = Scalar(0);
	
	//If the object size is too small, the contour is erased
	for(vector<vector<Point> >::iterator it = contours.begin(); it != contours.end();){
		if(it->size() < (30/pow(SUB,2))){
		      it = contours.erase(it);
		}else{
		      ++it;
		      
		}
	}
	
	//All the shapes are filled in white
	for(int i = 0; i < contours.size(); i++){		
		drawContours(dst, contours, i, Scalar(255), 3);
	}
	
	dst = Scalar(0);
	
	//All the shapes are filled in white
	for(int i = 0; i < contours.size(); i++){		
		drawContours(dst, contours, i, Scalar(255), -1);
	}
	
	//motionFilter(dst, dst);
	
	imshow("Gray image", dst);
	
}

/** Skin color threshold
 * Remove skin color
 
 * */
void simpleTrackingModule::skinRemvoal(Mat src, Mat &dst){//const unsigned char red, const unsigned char green, const unsigned char blue, const unsigned char max, const unsigned char min)
	dst = Scalar(0, 0, 0);
	for(int c = 0; c < src.cols; c++){
		for(int r = 0; r < src.rows; r++){
			Vec3b pixelColor = src.at<Vec3b>(Point(c,r));
			int maxV = std::max(pixelColor[2], std::max(pixelColor[1], pixelColor[0]));
			int minV = std::min(pixelColor[2], std::min(pixelColor[1], pixelColor[0]));
			
			if(!(pixelColor[2] > RED_MIN && pixelColor[1] > GREEN_MIN && pixelColor[0] > BLUE_MIN && (unsigned int)(maxV - minV) > MAX_MIN_SPAN &&
				(unsigned int)std::abs(pixelColor[2] - pixelColor[1]) > MIN_VALUE && pixelColor[2] > pixelColor[1] && pixelColor[2] > pixelColor[0]))
			{		
			      dst.at<Vec3b>(Point(c,r)) = pixelColor;
			}
		}
	}
}

/** Displays only the motion region of the image
 
 * */
void simpleTrackingModule::motionFilter(Mat src1, Mat src2){
	
	vector<Point2f> track_point1;
	vector<Point2f> track_point2;
	Mat _src1, _src2;
	Mat display = cv::Mat(Size(src1.cols, src1.rows), CV_8UC3);
	display = Scalar(0,0,0);
	
	
// 	cvtColor(src1, _src1, CV_BGR2GRAY);
// 	cvtColor(src2, _src2, CV_BGR2GRAY);
	
	src2.copyTo(_src2);
	src1.copyTo(_src1);
	
	vector<uchar> status;
	vector<float> err;
	
	if(!_src1.empty() && _src1.rows != 0){
	
		goodFeaturesToTrack(_src1, track_point1, MAX_TRACK_NB, 0.005, 5, Mat(), 3, 0, 0.04);
		goodFeaturesToTrack(_src2, track_point2, MAX_TRACK_NB, 0.005, 5, Mat(), 3, 0, 0.04);
	
		if(track_point1.size() > 0){
			calcOpticalFlowPyrLK(_src1, _src2, track_point1, track_point2, status, err);
			
			
			for (auto c : status){
				if(!status[c]);
					continue;
				
				circle( display, track_point1[c], 5, Scalar(0,255,0), -1);
				
			}
		}
	}
	
	imshow("Flow", display);
	
}

/** getBlobColor
 * extract the mean Hue color of an area
 * In:
 * Mat image: input color
 * */
double simpleTrackingModule::getBlobColor(Mat imgBlob, Mat imgRGB, double size, Point position){
	Mat newImg = cv::Mat(Size(imgBlob.cols, imgBlob.rows), CV_8UC3);
	resize(imgRGB, newImg, newImg.size());
	cvtColor(newImg, newImg, CV_RGB2HSV);
	
	double meanColor = -1;	
	if((position.x - size) > 0 && (position.y - size) > 0 && (position.x+size) < imgRGB.cols && (position.y+size) < imgRGB.rows){
	      Mat roi(imgRGB, Rect(position.x - size, position.y - size, 2*size, 2*size));
	      meanColor = getMeanColor(roi);
	}
	
	return meanColor;
}

/** getMeanColor
 * extract the mean color of an area
 * In:
 * Mat image: input image
 * */
double simpleTrackingModule::getMeanColor(Mat ROI){
      double meanColor = 0;
      double nbValue = 0;
      for(int c = 0; c < ROI.cols; c++){
		for(int r = 0; r < ROI.rows; r++){
			Vec3b pixelColor = ROI.at<Vec3b>(Point(c,r));
			meanColor+= (double)pixelColor[0];
			nbValue ++;
		}
	}
	meanColor /= nbValue;
	return meanColor;
}


/**
* Return the average height of a blob in the image objects
* */
double simpleTrackingModule::getHeightOfBlob(Mat ROI, double size){
	double height = 0;
	double nbPoints = 0;
	
	for(int c = 0; c < ROI.cols; c ++){
		for(int r = 0; r < ROI.rows; r++){
		      if((((c-size/2.)*(c-size/2.)) + ((r-size/2.)*(r-size/2.))) < size/2.){
			      nbPoints ++;
			      double pixelHeight = ROI.at<double>(c,r);
			      height += pixelHeight;
		      }
		}
	}
}
     

/** blobDetector
 * Extract all the blob contained in the argument image
 * In:
 * Mat image: input color
 * */
void simpleTrackingModule::blobDetector(Mat image){
	// ... any other params you don't want default value
	Point point;
	point.x = 0;
	point.y = 0;

	// detect!
	blob_detector.detect(image, keypoints);
}

/**
 * Method to initialize the blob detector
 * For more information see simpleBlobDector datasheet
 * 
 * The parameters of the blob detector 
 * are editable from ./initialization_files/blobDetector.ini
 * */
bool simpleTrackingModule::initBlobDetector(const char* src){
	bool everyThingIsOk = true;
	string line;
	double parameters[8];
	int index = 0;
	
	cout<<"Blob detector initialization starting"<<endl;
	
	ifstream myfile(src);
	
	if (myfile.is_open())
	{
		while ( getline (myfile,line) )
		{
			if(line.find("#") == std::string::npos && line.find("=") != std::string::npos){
				parameters[index] = string_to_double(line.substr(line.find("=")+1));
				index++;	
			}
			
		}
		myfile.close();
	}else
		cerr<<"Initialization file not found at "<<src<<endl;
		
	if(index == 8){
		params.minDistBetweenBlobs = (double)parameters[0];
		params.filterByInertia = (bool)parameters[1];
		params.filterByConvexity = (bool)parameters[2];
		params.filterByColor = (bool)parameters[3];
		params.filterByCircularity = (bool)parameters[4];
		params.filterByArea = (bool)parameters[5];
		params.minArea = (double)parameters[6]/(pow(SUB,2));
		params.maxArea = (double)parameters[7]/(pow(SUB,2));
		
		blob_detector = SimpleBlobDetector( params );
		cout<<"Blob detector parmaters correctly set"<<endl;
	}else{
		cerr<<"Blob detector initilization: number of argument invalid"<<endl;
	}
	
	if(!everyThingIsOk){
		cerr<<"Error during the blob detector initialization"<<endl;
	}
	cout<<"Blob detector initialization finished"<<endl;
	return everyThingIsOk;
}

//Get methods
     
/**
* Return the number of tracked objects
* The objects no longer observervable are also counted
* */
int simpleTrackingModule::getNumberOfTrackedObject(){
	return objectTrajectories.size();  
}

/**
* Return the number of active tracked objects
* The objects no longer observervable are not counted
* */
int simpleTrackingModule::getNumberOfActiveTrackedObject(){
	int value = 0;
	for(int c = 0; c < objectTrajectories.size(); c++){
		if(objectTrajectories[c][0].getState() >= 1){
			value ++;
		}
	}  
	return value;
}

/**
* Return the state of the tracked object at indexB
* -1 => Gone
* 0 => Occluded
* 1 => stationary
* 2 => Moving
* 
* in: indexB in the list of tracked object
* */
int simpleTrackingModule::getBlobState(int indexB){
	 if(objectTrajectories.size() > indexB && indexB >= 0){
		return objectTrajectories[indexB][0].getState();
	 }
	 return -1;
}

/**
* Return the 3D position of the tracked object at indexB
* in: indexB in the list of tracked object
* */
Point3d simpleTrackingModule::get3DPositionOfBlob(int indexB){
	 if(objectTrajectories.size() > indexB && indexB >= 0){
		//return listOfObjects[indexB].get3DPosition();
	 }
	 return Point3d(0,0,0);
}

/**
* Return the 2D position of the tracked object at indexB
* in: indexB in the list of tracked object
* */
Point simpleTrackingModule::getPositionOfBlob(int indexB){
	 if(objectTrajectories.size() > indexB && indexB >= 0){
		return objectTrajectories[indexB][0].getPosition();
	 }
	 return Point(0,0);
}

/**
* Return the RGB color of the tracked object at indexB
* in: indexB in the list of tracked object
* */
Scalar simpleTrackingModule::getRGBColorOfBlob(int indexB){
	 if(objectTrajectories.size() > indexB && indexB >= 0){
		return objectTrajectories[indexB][0].getRGB();
	 }
	 return Scalar(0, 0, 0);
}

/**
* Return the Hue of the tracked object at indexB
* in: indexB in the list of tracked object
* */
double simpleTrackingModule::getHueColorOfBlob(int indexB){
	 if(objectTrajectories.size() > indexB && indexB >= 0){
		return objectTrajectories[indexB][0].getHue();
	 }
	 return -1;
}

/**
* Return the trajectory of the tracked object at indexB
* in: indexB in the list of tracked object
* */
deque<Point> simpleTrackingModule::getTrajectory(int indexB){
	 if(objectTrajectories.size() > indexB && indexB >= 0){
		return objectTrajectories[indexB][0].getTrajectory();
	 }
	 deque<Point> temp;
	 return temp;
}


simpleTrackingModule::~simpleTrackingModule()
{
}

//Other methods

/**
 * Convert string to double
 * In: 
 * const std::string s: string to convert to double
 * Out: double contained the argument, "nan" if string not a number
 * */
double simpleTrackingModule::string_to_double(const std::string& s){
   std::istringstream i(s);
   double x;
   if (!(i >> x))
     return std::numeric_limits<double>::quiet_NaN();
   return x;
 } 
