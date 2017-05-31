#include <sys/time.h>

#ifndef PI
#define PI 3.14159
#endif

const double Kp = 0.5;
const double Kd = 0.5;
const double margin = 0.01;

class PdController
{
private:

	BZRC* myTeam;

	int prevAngleError;
	suseconds_t prevAngleTime;

	/*
	 * Resets prev values
	 */
	void Reset()
	{
		prevAngleError = 0;
		prevAngleTime = 0;
	}

	/*
	 * Returns current angle of tank defined by index
	 * Range [0, 2*PI)
	 */
	double GetTankAngle(int index)
	{
		vector<tank_t> myTanks;
		myTeam->get_mytanks(&myTanks);
		tank_t tank = myTanks[index];
		double currentAngle = tank.angle;

		if(currentAngle > 0)
		{
			while(currentAngle >= (2 * PI))
				currentAngle -= (2 * PI);
		}
		while(currentAngle < 0)
			currentAngle += (2 * PI);

		return currentAngle;
	}

public:

	/**
	 * Constructor
	 * @param team - BZRC that will control tanks
	 */
	PdController(BZRC* team)
	{
		myTeam = team;
		Reset();
	}

	/**
	 * Turns a tank using pd controller
	 * @param index - index of tank
	 * @param targetAngle - angle tank should eventually be facing
	 */
	double TurnTank(int index, double targetAngle)
	{
		double angvel = 0;

		// Get time in usec
		struct timeval now;
		gettimeofday(&now, NULL);
		suseconds_t currentTime = now.tv_usec;

		// Get tank's current angle
		double currentAngle = GetTankAngle(index);

		// Calculate error
		double currentError = (targetAngle - currentAngle);
		cout << "Current angle: " << currentAngle << endl;
		cout << "Current error: " << currentError << endl;

		// If error within margin, reset values and return true
		if(currentError <= margin && currentError >= (margin * -1))
		{
			Reset();
			myTeam->angvel(index, 0);
			cout << "Angvel: 0" << endl;
			cout << "Speed: 1" << endl;
			return 1.0;
		}

		// Calculate derivative of error
		double derivative = 0;
		if(prevAngleTime != 0)
			derivative = (currentError - prevAngleError) / (currentTime - prevAngleTime);

		// Put angvel between -1.0 and 1.0
		angvel = (Kp * currentError) + (Kd * derivative);
		if(angvel > 0)
			angvel = min(angvel, 1.0);
		if(angvel < 0)
			angvel = max(angvel, -1.0);

		// Update tank and prev values
		myTeam->angvel(index, angvel);
		prevAngleError = currentError;
		prevAngleTime = currentTime;


		if(angvel < 0)
			angvel *= -1;
		cout << "Angvel: " << angvel << endl;
		cout << "Speed: " << 1-angvel << endl;
		return 1-angvel;
	}
};
