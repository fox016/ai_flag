#include "kalmanFilter.h"

const int HUNTER = 0;
const int SITTING_DUCK = 1;
const int CONST_VEL = 2;
const int WILD = 3;

class Tank
{
private:
	int index;
	int type;

	vector<int> xHistory;
	vector<int> yHistory;

	World* world;
	BZRC* myTeam;
	PdController* controller;

	int xTarget;
	int yTarget;

	/*
	 * @return true iff tank's last position is near target coordinates
	 */
	bool nearTarget()
	{
		int margin = 20;
		return (abs(xHistory.back() - xTarget) < margin) &&
			(abs(yHistory.back() - yTarget) < margin);
	}
	
	/*
	 * Sets the tank's target
	 */ 	
	void setTarget(int x, int y)
	{
		xTarget = x;
		yTarget = y;
	}

	/*
	 * Rotate to point towards position (x,y)
	 *
	 * @return tank error
	 */
	double rotateToPoint(int x, int y)
	{
		double angle = atan2(y - yHistory.back(), x - xHistory.back());
		if(angle < 0) angle += 2*PI;

		cout << "Rotate currentPos: (" << xHistory.back() << ", " << yHistory.back() << ")" << endl;
		cout << "Rotate targetPos: (" << x << ", " << y << ")" << endl;
		cout << "Rotate targetAngle: " << angle << endl;

		return controller->TurnTank(index, angle);
	}

	/*
	 * Shoot
	 */
	void shoot()
	{
		while(!myTeam->shoot(index));
	}

	/*
	 * Set tank speed
	 */
	void setSpeed(double vel)
	{
		while(!myTeam->speed(index, vel));
	}
	
	/*
	 * Add position history
	 */
	void addHistory(int x, int y)
	{
		xHistory.push_back(x);
		yHistory.push_back(y);
	}

	/*
	 * Defines behavior of HUNTER
	 */
	void beHunter()
	{
		// Get tank position only once, tank doesn't move
		vector <tank_t> myTanks;
		while(!myTeam->get_mytanks(&myTanks));
		addHistory(myTanks[0].pos[0], myTanks[0].pos[1]);

		// Initialize filter
		KalmanFilter filter = KalmanFilter(world);

		while(true)
		{
			// Get reading
			vector <otank_t> otherTanks;
			while(!myTeam->get_othertanks(&otherTanks));

			// Upgrade filter
			filter.updateFilter(otherTanks[0].pos[0], otherTanks[0].pos[1]);

			// Rotate
			int predictX = 0, predictY = 0;
			filter.predict(xHistory.back(), yHistory.back(), predictX, predictY);
			double error = rotateToPoint(predictX, predictY);

			// Print debug
			cout << "PREDICT future location: (" << predictX << ", " << predictY << ")" << endl;
			cout << "PREDICT angle error: " << error << endl;

			// Shoot when angular error is low enough
			if(abs(error) < 0.05)
			{
				myTeam->shoot(index);
				cout << "SHOOT PREDICTED future location" << endl;
			}
		}
	}

	/*
	 * Defines behavior of SITTING_DUCK
	 */
	void beSittingDuck()
	{
		// Go to center of playing field
		setTarget(0, 0);
		setSpeed(0.5);
		while(true)
		{
			vector<tank_t> myTanks;
			while(!myTeam->get_mytanks(&myTanks));
			tank_t tank = myTanks[index];

			addHistory(tank.pos[0], tank.pos[1]); // Needed for nearTarget()
			rotateToPoint(xTarget, yTarget);

			// If near target, stop. Otherwise, keep going
			if(nearTarget())
				setSpeed(0);
			else
				setSpeed(0.5);
		}
	}

	/*
	 * Defines what a CONST_VEL tank does
	 */
	void beConstVelTank()
	{
		int border = (int) floor(world->worldSize / 2.0);
		setTarget(0, border - 50);
		setSpeed(1.0);
		while(true)
		{
			vector<tank_t> myTanks;
			while(!myTeam->get_mytanks(&myTanks));
			tank_t tank = myTanks[index];

			addHistory(tank.pos[0], tank.pos[1]); // Needed for nearTarget()
			rotateToPoint(xTarget, yTarget);

			if(nearTarget())
			{
				setTarget(xTarget, yTarget * -1);
			}
		}
	}

	/*
	 * Defines behavior for WILD tank
	 */
	void beWild()
	{
		int i = 0;
		int iterations = 0;
		double speed, angle;
		while(true)
		{
			cout << "Iteration #" << i << " out of " << iterations << endl;
			i++;
			if(i > iterations)
			{
				i = 0;
				iterations = (rand() % 100000) + 100000;
				if(rand() % 2 == 0)
					speed = 1.0;
				else
					speed = -1.0;
				angle = fRand(0, 2*PI);

				cout << "Wild set speed: " << speed << endl;
				cout << "Wild set angle: " << angle << endl;

				setSpeed(speed);
				controller->TurnTank(index, fRand(0, 2*PI));
			}
		}
	}

	/*
	 * Returns random double between fMin and fMax
	 */
	double fRand(double fMin, double fMax)
	{
	    double f = (double)rand() / RAND_MAX;
	    return fMin + f * (fMax - fMin);
	}
public:
	Tank(int i, BZRC* team, World* w, int tank_type)
	{
		index = i;
		world = w;
		myTeam = team;
		type = tank_type;

		controller = new PdController(myTeam);

		switch(tank_type)
		{
			case HUNTER:
				beHunter();
				break;
			case SITTING_DUCK:
				beSittingDuck();
				break;
			case CONST_VEL:
				beConstVelTank();
				break;
			case WILD:
				beWild();
				break;
			default:
				cout << "Illegal tank type: " << tank_type << endl;
				exit(1);
		}
	}

	~Tank()
	{
		delete controller;
	}
};
