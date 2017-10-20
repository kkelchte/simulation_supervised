#include "OUNoise.h"

using namespace cv;
using namespace std;

// ROS includes
#include <ros/ros.h>

OUNoise * ounoise = 0;
cv::Mat displayImage = Mat::zeros(200, 800, CV_8UC3);
int x = 1;
double preval = 0;

void display(double val){ 
	line(displayImage, Point(x,10*preval+100), Point(x+5, 10*val+100), Scalar( 255,255,255), 5,8,0);
	preval =val;
	x = x+5;
	if( x > 800 ){
		x = 0;
		preval = 0;
	}
	imshow("OUNoise cpp", displayImage);
	waitKey(1);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "viz_ounoise", ros::init_options::AnonymousName);
	ros::NodeHandle nh;
	// ounoise = new OUNoise((double) 0.15, (double) 0, (double) 1, (int) 1, (int) 100);
	ounoise = new OUNoise(0.15);
	ros::Rate loop_rate(5);
	
	while(ros::ok()){
		double val = ounoise->noise();
		cout << val << endl;
		display(val);
		loop_rate.sleep();
		ros::spinOnce();
	}
	
}