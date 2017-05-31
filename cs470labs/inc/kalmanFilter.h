#include <iostream>
#include <fstream>
#include <ostream>
#include <ctime>
#include "armadillo"

using namespace std;
using namespace arma;

const double INIT_POS_NOISE = 200.0;
const double INIT_VEL_NOISE = 0.1;
const double INIT_ACC_NOISE = 0.1;

const double FRICTION = 0.0;
const double DELTA_T = 0.140;

const double POS_NOISE = 4.0; // [3.0, 5.0]
const double VEL_NOISE = 25.0; // [10.0, 40.0]
const double ACC_NOISE = 50.0;

const double MEASUREMENT_NOISE = 5.0;

class KalmanFilter
{
private:
	World* world;
	int prevTime;
	int updateIter;

	filebuf* matlab_fb;
	ostream* matlab_os;

	mat mean;
	mat covariance;

	mat F;
	mat H;
	mat Z;
	mat noiseX;
	mat noiseZ;

	mat F_T;
	mat H_T;
	mat F_SQUARED;
	mat K;

	void initMean()
	{
		// Mean is 6x1 full of zeros
		mean = mat(6, 1, fill::zeros);
	}

	void initCovariance()
	{
		// Covariance is 6x6, all zeros but diagonal
		covariance = mat(6, 6, fill::zeros);
		covariance(0,0) = INIT_POS_NOISE;
		covariance(1,1) = INIT_VEL_NOISE;
		covariance(2,2) = INIT_ACC_NOISE;
		covariance(3,3) = INIT_POS_NOISE;
		covariance(4,4) = INIT_VEL_NOISE;
		covariance(5,5) = INIT_ACC_NOISE;
	}

	void initMotionModel()
	{
		F = mat(6, 6, fill::zeros);

		F(0,0) = 1.0;
		F(0,1) = DELTA_T;
		F(0,2) = (DELTA_T * DELTA_T) / 2.0;

		F(1,1) = 1.0;
		F(1,2) = DELTA_T;

		F(2,1) = (-1 * FRICTION);
		F(2,2) = 1.0;

		F(3,3) = F(0,0);
		F(3,4) = F(0,1);
		F(3,5) = F(0,2);

		F(4,4) = F(1,1);
		F(4,5) = F(1,2);

		F(5,4) = F(2,1);
		F(5,5) = F(2,2);

		F_T = F.t();
	}

	void initSensorModel()
	{
		// Only sense position
		H = mat(2, 6, fill::zeros);
		H(0,0) = 1.0;
		H(1,3) = 1.0;

		H_T = H.t();
	}

	void initNoiseX()
	{
		// Noise X is 6x6, all zeros but diagonal
		noiseX = mat(6, 6, fill::zeros);
		noiseX(0,0) = POS_NOISE;
		noiseX(1,1) = VEL_NOISE;
		noiseX(2,2) = ACC_NOISE;
		noiseX(3,3) = POS_NOISE;
		noiseX(4,4) = VEL_NOISE;
		noiseX(5,5) = ACC_NOISE;
	}

	void initNoiseZ()
	{
		// Noise Z is 2x2, all zeros but diagonal
		noiseZ = mat(2, 2, fill::zeros);
		noiseZ(0,0) = (MEASUREMENT_NOISE * MEASUREMENT_NOISE);
		noiseZ(1,1) = (MEASUREMENT_NOISE * MEASUREMENT_NOISE);
	}

	void initObservation()
	{
		Z = mat(2, 1);
	}

	void initMatlab()
	{
		matlab_fb = new filebuf();
		matlab_fb->open("./plot/testMatlab.m", std::ios::out);
		matlab_os = new ostream(matlab_fb);

		*matlab_os << "axis ([-300, 300, -300, 300]);" << endl;
		*matlab_os << "hold on;" << endl;
		*matlab_os << "x = zeros(2, 100);" << endl;
	}

	void updateObservation(int x, int y)
	{
		Z(0,0) = x;
		Z(1,0) = y;
	}

	void updateForceSquared()
	{
		F_SQUARED = (F * covariance * F_T) + noiseX;
	}

	void updateKalmanGain()
	{
		K = F_SQUARED * H_T * inv((H * F_SQUARED * H_T) + noiseZ);
	}

	void updateMean()
	{
		mean = (F * mean) + (K * (Z - (H * F * mean)));
	}

	void updateCovariance()
	{
		covariance = F_SQUARED - (K * H * F_SQUARED);
	}

	void updateMatlab()
	{
		int startIter = 50;
		if((updateIter-startIter) > 100 || (updateIter-startIter) < 1)
			return;

		*matlab_os << "x(1, " << (updateIter-startIter) << ") = " << (int) floor(mean(1,0)) << ";" << endl;
		*matlab_os << "x(2, " << (updateIter-startIter) << ") = " << (int) floor(mean(4,0)) << ";" << endl;
		*matlab_os << "plot(x(1, " << (updateIter-startIter) << "), ";
		*matlab_os << "x(2, " << (updateIter-startIter) << "), 'k.');" << endl;

		if((updateIter-startIter) == 100)
		{
			*matlab_os << "hold off;" << endl;
			*matlab_os << endl;

			*matlab_os << "sest = mean(x')';" << endl;
			*matlab_os << "hold on;" << endl;
			*matlab_os << "plot(sest(1), sest(2), 'bs');" << endl;
			*matlab_os << "hold off;" << endl;
			*matlab_os << "K = cov(x');" << endl;
			
			matlab_fb->close();
		}
	}

	int getMeanX()
	{
		return (int) floor(mean(0,0));
	}

	int getMeanY()
	{
		return (int) floor(mean(3,0));
	}

	void drawKalman()
	{
		if(updateIter % 50 != 0)
			return;

		char filename[80];
		bzero(filename, 80);
		sprintf(filename, "./plot/kalmanFilter_%i.gpi", updateIter);
		filebuf fb;
		fb.open(filename, std::ios::out);
		ostream os(&fb);

		int border = (int) floor(world->worldSize / 2.0);

		os << "set xrange[-" << border << ":" << border << "]" << endl;
		os << "set yrange[-" << border << ":" << border << "]" << endl;
		os << "set pm3d" << endl;
		os << "set view map" << endl;
		os << "unset key" << endl;
		os << "set size square" << endl;
		os << endl;

		os << "set palette model RGB functions 1-gray, 1-gray, 1-gray" << endl;
		os << "set isosamples 50" << endl;
		os << endl;

		os << "sigma_x = " << sqrt(covariance(0,0)) << endl;
		os << "sigma_y = " << sqrt(covariance(3,3)) << endl;
		os << "rho = 0" << endl;
		os << "mean_x = " << (int) floor(mean(0,0)) << endl;
		os << "mean_y = " << (int) floor(mean(3,0)) << endl;
		os << "#iter = " << updateIter << endl;
		os << endl;

		os << "splot 1.0/(2.0 * pi * sigma_x * sigma_y * sqrt(1 - rho**2)) ";
		os << "* exp(-1.0/2.0 * ((x-mean_x)**2 / sigma_x**2 + (y-mean_y)**2 / ";
		os << "sigma_y**2 - 2.0*rho*(x-mean_x)*(y-mean_y)";
		os << "/(sigma_x*sigma_y))) with pm3d" << endl;

		fb.close();
	}

	void printMean(mat* m)
	{
		cout << endl;
		cout << "-------- MEAN" << endl;
		cout << "MEAN pos_x: " << (*m)(0,0) << endl;
		cout << "MEAN vel_x: " << (*m)(1,0) << endl;
		cout << "MEAN acc_x: " << (*m)(2,0) << endl;
		cout << "MEAN pos_y: " << (*m)(3,0) << endl;
		cout << "MEAN vel_y: " << (*m)(4,0) << endl;
		cout << "MEAN acc_y: " << (*m)(5,0) << endl;
		cout << endl;
	}

public:
	KalmanFilter(World* w)
	{
		world = w;
		prevTime = MyClock::getMilliCount();
		updateIter = 0;

		initMean();
		initCovariance();
		initMotionModel();
		initSensorModel();
		initNoiseX();
		initNoiseZ();
		initObservation();

		initMatlab();
	}

	/*
	 * Updates the mean, covariance, and kalman gain
	 *
	 * @param x,y -> noisy position of enemy tank (see MEASUREMENT_NOISE)
	 */
	void updateFilter(int x, int y)
	{	
		// Draw new representation of mean/covariance
		printMean(&mean);
		drawKalman();

		// Make updates
		updateObservation(x, y);
		updateForceSquared();
		updateKalmanGain();
		updateMean();
		updateCovariance();
		updateIter++; // Increment counter

		// Get time in msec
		int currentTime = MyClock::getMilliCount();

		// Update MATLAB file
		updateMatlab();	

		// Print true DELTA_T
		cout << "Kalman Filter Time Diff: " << MyClock::getMilliSpan(prevTime, currentTime) << " milliseconds" << endl;
		prevTime = currentTime;
	} 
	/*
	 * Predicts where enemy tank will be based on enemy's estimated distance from my position
	 * Uses repeated F * mean to make future prediction
	 *
	 * @param (myX, myY) - my position
	 * @param (predictX, predictY) - where I think my enemy will be after the time it takes
	 *		for the bullet to travel to where I think my enemy is now
	 */
	void predict(int myX, int myY, int& predictX, int& predictY)
	{
		// Calculate time based on bullet speed and distance between (myX, myY) and (meanX, meanY)
		double distance = sqrt(pow((myX - getMeanX()), 2) + pow((myY - getMeanY()), 2));
		double predictedTime = distance / world->shotSpeed;

		cout << "ESTIMATED distance to target: " << distance << endl;
		cout << "ESTIMATED time for bullet to reach target: " << predictedTime << endl;

		// Multiply mean by F (predictedTime / DELTA_T) times to get prediction
		mat prediction = mat(mean);
		int i = 0;
		for(i = 0; i < predictedTime / DELTA_T; i++)
		{
			prediction = F * prediction;
			prediction(2,0) = 0.0; // Assume x-accel = 0
			prediction(5,0) = 0.0; // Assume y-accel = 0
			printMean(&prediction); // Display prediction
		}

		// Set predictX and predictY to appropriate values of future predicted mean
		predictX = floor(prediction(0,0));
		predictY = floor(prediction(3,0));
	}
};
