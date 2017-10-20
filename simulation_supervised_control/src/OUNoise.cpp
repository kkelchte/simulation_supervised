#include "OUNoise.h"

using namespace cv;
using namespace std;

OUNoise::OUNoise(double theta_arg){
	// init(theta_arg);
	theta = theta_arg;
	mu = 0;
	sigma = 1;
	std::default_random_engine generator(time(NULL)); 
	std::normal_distribution<double> distribution(); 
	reset();
}

OUNoise::OUNoise(){
	// init(theta_arg);
	theta = 0.15;
	mu = 0;
	sigma = 1;
	std::default_random_engine generator(time(NULL)); 
	std::normal_distribution<double> distribution(); 
	reset();
	
}// OUNoise::OUNoise(){
// 	init(0.015);
// }
OUNoise::~OUNoise(){
}
// OUNoise::init(double theta_arg){
// 	theta = theta_arg;
// 	mu = 0;
// 	sigma = 1;
// 	std::default_random_engine generator; 
// 	std::normal_distribution<double> distribution(); 
// 	reset();
// }

void OUNoise::reset(){
	state = 0.;
	// cout << state << endl; 
}

double OUNoise::noise(){
	double sample = distribution(generator);
	double x(state);
	double dx = theta * ( mu - x ) + sigma * sample;
	state = x + dx;
	return state;
}