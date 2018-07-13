#include "BehaviourArbitration.h"

//The old Kinect has a depth image resolution of 320 x 240 pixels with a fov of 58.5 x 46.6
// degrees resulting in an average of about 5 x 5 pixels per degree. (see source 1)
//source: http://smeenk.com/kinect-field-of-view-comparison/

using namespace cv;
using namespace std;

cv::RNG rng( 0xFFFFFFFF );

template <typename T>
std::string to_string(T value)
{
  std::ostringstream os ;
  os << value ;
  return os.str() ;
}

BehaviourArbitration::BehaviourArbitration() {
	lambdaGoalHorz = 0.5; // Linear multiplier for the angular velocity
	lambdaObstacleHorzNormal = 5;
	lambdaObstacleHorzAggressive = 50;
	lambdaObstacleHorz = lambdaObstacleHorzNormal; // Default value
	weightGoalHorz = 0.3;
	weightObstacleHorz = 0.7;
	obstacleDistanceGainHorzNormal = 0.1; // Smaller = more sensitive 
	obstacleDistanceGainHorzAggressive = 0.005; // Smaller = more sensitive
	obstacleDistanceGainHorz = obstacleDistanceGainHorzNormal; // Default value
	angularRangeHorz = 2*29*CV_PI/180; // Range is +-29° 

	lambdaObstacleVert = 5;
	angularRangeVert = 27.5*CV_PI/180;
	obstacleDistanceGainVert = 0.15;//0.2;
	noiseVariance = 0.2;

	matchedFilterKernel = (Mat_<float>(1,7) << 17,20,25,26,25,20,17);
	obstacleArrayHorz = Mat::zeros(1,7, CV_32F);

	matchedFilterExpectedResult = 3304;
	matchedFilterMargin = 800;

	depthImageScaling = 1;
}

BehaviourArbitration::BehaviourArbitration(std::string xmlPath) {
	cout << "[BA] 	Reading behaviour arbitration parameters from " << xmlPath << endl;
	FileStorage fs;//(filename, FileStorage::READ);
	fs.open(xmlPath, FileStorage::READ);

	lambdaGoalHorz = (float) fs["lambdaGoalHorz"]; // Linear multiplier for the angular velocity
	cout << "lambdaGoalHorz " << lambdaGoalHorz << endl;
	lambdaObstacleHorzNormal = (float) fs["lambdaObstacleHorzNormal"];
	lambdaObstacleHorzAggressive = (float) fs["lambdaObstacleHorzAggressive"];
	lambdaObstacleHorz = lambdaObstacleHorzNormal; // Default value
	cout << "lambdaObstacleHorz " << lambdaObstacleHorz << endl;

	weightGoalHorz = (float) fs["weightGoalHorz"];
	weightObstacleHorz = (float) fs["weightObstacleHorz"];
	obstacleDistanceGainHorzNormal = (float) fs["obstacleDistanceGainHorzNormal"]; // Smaller = more sensitive 
	obstacleDistanceGainHorzAggressive = (float) fs["obstacleDistanceGainHorzNormal"]; // Smaller = more sensitive
	obstacleDistanceGainHorz = obstacleDistanceGainHorzNormal; // Default value
	angularRangeHorz = ((float) fs["angularRangeHorz"]) * CV_PI/180; // Range is +-29° 
	cout << "angularRangeHorz " << angularRangeHorz << endl;

	lambdaObstacleVert = (float) fs["lambdaObstacleVert"];
	angularRangeVert = ((float) fs["angularRangeVert"]) * CV_PI/180;
	obstacleDistanceGainVert = (float) fs["obstacleDistanceGainVert"];//0.2;
	noiseVariance = (float) fs["noiseVariance"];

	matchedFilterKernel = (Mat_<float>(1,7) << 17,20,25,26,25,20,17);
	obstacleArrayHorz = Mat::zeros(1,7, CV_32F);

	matchedFilterExpectedResult = (float) fs["matchedFilterExpectedResult"];
	matchedFilterMargin = (float) fs["matchedFilterMargin"];

	depthImageScaling = (float) fs["depthImageScaling"];
	cout << "depthImageScaling " << depthImageScaling << endl;

}


void BehaviourArbitration::displayObstacleArrayHorz(cv::Mat obstacleMap) {
	cv::Mat displayImage = Mat::zeros(200, obstacleMap.cols*12, CV_8UC3);
	int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
  	double fontScale = 0.5;
	int thickness = 2;

	Point testPos(10, 70);
	int indexObstacle = 0;
	for (int i = 0; i< obstacleMap.cols; i += 10) {
		float depthThis = floor(obstacleMap.at<float>(0,i));
		obstacleArrayHorz.at<float>(0,indexObstacle) = depthThis;
		string depthString = to_string(depthThis);
		// cout <<"depth"<< depthString << endl;
		putText(displayImage, depthString, testPos, fontFace, fontScale, cv::Scalar(255,0,0), thickness,8);
		testPos += Point(100,0);
		indexObstacle++;
	}

	// float matchedFilterResult = matchedFilterKernel.dot(obstacleArrayHorz);
	// float matchedFilterResult = 0;
	// for (int i = 0; i < 7; i++) {
	// 	matchedFilterResult += obstacleArrayHorz.at<float>(0,i) * matchedFilterKernel.at<float>(0,i);
	// }
	// cout << "Kernel: " << matchedFilterKernel << endl;
	// 	cout << "Filter result: " << matchedFilterResult << endl;
	imshow("Depth info horizontal", displayImage);
	waitKey(1);

}

void BehaviourArbitration::displayObstacleArrayVert(cv::Mat obstacleMap) {
	cv::Mat displayImage = Mat::zeros(200, obstacleMap.cols*4, CV_8UC3);
	int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
  	double fontScale = 0.5;
	int thickness = 1;

	Point testPos(obstacleMap.cols*2, 30);
	for (int i = 0; i< obstacleMap.cols; i += 8) {
		string depthString = to_string(floor(obstacleMap.at<float>(0,i)));
		// cout <<"depth"<< depthString << endl;
		putText(displayImage, depthString, testPos, fontFace, fontScale, cv::Scalar(255,0,0), thickness,8);
		testPos += Point(0,30);
	}
	//imshow("Depth info vertical", displayImage);
	//waitKey(1);

}

void BehaviourArbitration::displayCollision(cv::Mat kinectImage) {
	double minVal;
	minMaxLoc(kinectImage, &minVal, NULL, NULL, NULL);

	cv::Mat displayImage = Mat::zeros(200, 200, CV_8UC3);
	if (minVal <= 0.5) {
		rectangle(displayImage, Rect(Point(0,0), Point(200,200)),Scalar(0,0,255), CV_FILLED);
	}
	//imshow("Collision image", displayImage);
	//waitKey(1);
}

void BehaviourArbitration::detectCorner(cv::Mat kinectImage) {
	float matchedFilterResult = matchedFilterKernel.dot(obstacleArrayHorz);
	if (abs(matchedFilterResult - matchedFilterExpectedResult) < matchedFilterMargin ) {
		lambdaObstacleHorz = lambdaObstacleHorzAggressive;
		obstacleDistanceGainHorz = obstacleDistanceGainHorzAggressive;
		cout << "Corner detected" << endl;
	}
	else {
		lambdaObstacleHorz = lambdaObstacleHorzNormal;
		obstacleDistanceGainHorz = obstacleDistanceGainHorzNormal;	
	}
}

cv::Mat BehaviourArbitration::scaleDepthImage(cv::Mat kinectImage) {
	return kinectImage * depthImageScaling;
}

/*
 * Returns the angular velocity outputted by the behaviour arbitration scheme
 * This is the obstacle avoid behaviour, should be summed with heading goal
 */
float BehaviourArbitration::avoidObstacleHorizontal(cv::Mat kinectImage, float currentBearing) {
	int image_height = kinectImage.rows;
	int image_width  = kinectImage.cols;
	int centre_row = floor(image_height/2);
	int obstacle_bins = 64;


	detectCorner(kinectImage);
	// displayCollision(kinectImage);

	// Create the obstacle array
	cv::Mat obstacleMap = Mat::zeros(1, obstacle_bins, CV_32F);
	// 64 bins ~~> sum of every 5 depth bins
	// 1 bin = 0.90625°
	// cout << "Width: " << image_width << endl;
	// imshow("Depth map", kinectImage);
	// waitKey(1);
	for (int i = 0; i < image_width; i++) {
		// Sum 5 depth pixels for each obstacle bin
		// cout << "i: " << floor(i/5) << endl;
		// cout << "Depth: " << kinectImage.at<float>(centre_row, i) << endl;
		obstacleMap.at<float>(0, floor(i*obstacle_bins/image_width)) += kinectImage.at<float>(centre_row, i);
	}
	// displayObstacleArrayHorz(obstacleMap);
	float fObstacleTotal = 0;

	for (int i = 0; i < obstacle_bins; i++) {
		// obstacleBearing between -29° and 29°
		float obstacleBearing = i*58/64-29;
		// Convert to radians
		obstacleBearing = obstacleBearing * CV_PI / 180;
		// float bearingDifference = currentBearing - obstacleBearing;
		float bearingDifference = obstacleBearing;
		// cout << "bearingDifference: " << bearingDifference << endl;
		// cout << "Obstacle distance: " << obstacleMap.at<float>(0,i) << endl;

		// Calculate the exponents
		double bearingExponent = exp(-pow(bearingDifference,2)/(2*pow(angularRangeHorz,2)));
		double distanceExponent = exp(-obstacleDistanceGainHorz * obstacleMap.at<float>(0,i));
		// cout << "Bearing exponent " << bearingExponent << endl;
		// Add product to the total sum
		fObstacleTotal += bearingDifference * bearingExponent * distanceExponent;
	}
	// Multiply the total sum by the appropriate lambda
	// cout << "fObstacleTotal: " << fObstacleTotal << endl;
	fObstacleTotal *= lambdaObstacleHorz;
	// return 0;
	return fObstacleTotal;

}
float BehaviourArbitration::avoidObstacleVertical(cv::Mat kinectImage, float currentBearing) {
	int image_height = kinectImage.rows;
	int image_width  = kinectImage.cols;
	int centre_col = floor(image_width/2);
	int obstacle_bins = 36;

	// cout << "Image height: " << image_height << endl;

	cv::Mat obstacleMap = Mat::zeros(1, obstacle_bins, CV_32F);

	for (int i = 0; i < image_height; i++) {
		obstacleMap.at<float>(0, floor(i/10)) += kinectImage.at<float>(i,centre_col);
	}
	displayObstacleArrayVert(obstacleMap);
	float fObstacleTotal = 0;

	for (int i = 0; i < obstacle_bins; i++) {
		// obstacleBearing between -23.5° and 23.5° [Vertical FOV]
		float obstacleBearing = i*47/36-23.5;
		// Convert to radians
		obstacleBearing = obstacleBearing * CV_PI / 180;
		// float bearingDifference = currentBearing - obstacleBearing;
		float bearingDifference = obstacleBearing;
		// cout << "Obstacle distance: " << obstacleMap.at<float>(0,i) << endl;

		// Calculate the exponents
		double bearingExponent = exp(-pow(bearingDifference,2)/(2*pow(angularRangeVert,2)));
		double distanceExponent = exp(-obstacleDistanceGainVert * obstacleMap.at<float>(0,i));
		// Add product to the total sum
		fObstacleTotal += bearingDifference * bearingExponent * distanceExponent;
	}

	fObstacleTotal *= lambdaObstacleVert;
	return fObstacleTotal;
}

float BehaviourArbitration::followGoal(float goalAngle, float currentBearing) {
	// return 0;
	return -lambdaGoalHorz * sin(currentBearing - goalAngle);
}

float BehaviourArbitration::sumBehavioursHorz(float angVelAvoidHorz, float angVelFollowGoal) {
	// return 0.5;
	float behaviourSum = weightObstacleHorz * angVelAvoidHorz + weightGoalHorz*angVelFollowGoal;
	//behaviourSum += rng.gaussian(noiseVariance);
	return behaviourSum;
}

float sumBehavioursVert(float angVelAvoidVert, float angVelFollowGoal) {
	return 0;
}


BehaviourArbitration::~BehaviourArbitration() {

}
