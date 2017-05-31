#include <sys/time.h>

#ifndef PI
#define PI 3.14159
#endif

const double Kp = 1.5;
const double Kd = 0.5;
const double margin = 0.01;

class PdController
{
private:

	BZRC* myTeam;

	int prevAngleError;
	int prevAngleTime;

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
		while(!myTeam->get_mytanks(&myTanks));
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
	 *
	 * @param team - BZRC that will control tanks
	 */
	PdController(BZRC* team)
	{
		myTeam = team;
		Reset();
	}

	/**
	 * Turns a tank using pd controller
	 *
	 * @param index - index of tank
	 * @param targetAngle - angle tank should eventually be facing
	 *
	 * @returns - current error
	 */
	double TurnTank(int index, double targetAngle)
	{
		double angvel = 0;

		// Get time in usec
		int currentTime = MyClock::getMilliCount();

		// Get tank's current angle
		double currentAngle = GetTankAngle(index);

		// Calculate error
		double currentError = (targetAngle - currentAngle);
		if(currentError > PI)
			currentError -= (2 * PI);

		// If error within margin, reset values and return true
		if(currentError <= margin && currentError >= (margin * -1))
		{
			Reset();
			while(!myTeam->angvel(index, 0));
			return 1.0;
		}

		// Calculate derivative of error
		double derivative = 0;
		if(prevAngleTime != 0)
			derivative = (currentError - prevAngleError) / MyClock::getMilliSpan(prevAngleTime, currentTime);

		// Put angvel between -1.0 and 1.0
		angvel = (Kp * currentError) + (Kd * derivative);
		if(angvel > 0)
			angvel = min(angvel, 1.0);
		if(angvel < 0)
			angvel = max(angvel, -1.0);

		// Debug Output
		cout << "PD_CTRL Current angle: " << currentAngle << endl;
		cout << "PD_CTRL Target angle:  " << targetAngle << endl;
		cout << "PD_CTRL Current error: " << currentError << endl;
		cout << "PD_CTRL Error derivative: " << derivative << endl;
		cout << "PD_CTRL Angvel: " << angvel << endl;

		// Update tank and prev values
		while(!myTeam->angvel(index, angvel));
		prevAngleError = currentError;
		prevAngleTime = currentTime;

		return currentError;
	}
};
