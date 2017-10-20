#include <cmath>
#include <random>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <time.h>       /* time used for seeding*/

class OUNoise
{
public:
	OUNoise();
	OUNoise(double theta);
	// OUNoise(double theta, double mu);
	// OUNoise(double theta, double mu, double sigma);
	// OUNoise(double theta, double mu, double sigma, int n);
	// OUNoise(double t, double m, double s, int n, int sd);
	~OUNoise();
	void reset();
	// cv::Mat noise();
	double noise();

private:
	// void init(double theta);
	double state; // the current value of the noise
	double theta; // rate at which the noise reverts to the mean
	double mu; // mean 
	double sigma; // standard deviation
	// int seed; //seed random number generator
	// int dim; //action dimension
	std::default_random_engine generator;
  	std::normal_distribution<double> distribution;
};