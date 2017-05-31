#include <math.h>

#ifndef ATTRACT
#define ATTRACT 0
#define REPEL 1
#define TANGENT_RIGHT 2
#define TANGENT_LEFT 3
#endif

#ifndef PI
#define PI 3.14159
#endif

class PotentialField
{
private:

	double pos[2];
	double minRadius;
	double maxRadius;
	double threshold;
	int type;

	/*
	 * Calculate the speed that an agent at (x, y) should
	 * go according to this potential field
	 */
	double CalculateSpeed(double x, double y)
	{
		double speed = 0;
		double distance = sqrt(pow((pos[0] - x), 2.0) + pow((pos[1] - y), 2.0));

		if(distance > maxRadius)
			speed = 0;
		else if(distance < minRadius)
			speed = 0;
		else
		{
			if(type == ATTRACT)
				speed = min(distance/100, threshold);
			else
				speed = min((maxRadius-distance), threshold);
		}

		return speed;
	}

	/*
	 * Calculate the angle that an agent at (x, y) should
	 * go according to this potential field
	 */
	double CalculateAngle(double x, double y)
	{
		double angle = atan2(pos[1] - y, pos[0] - x);
		if(angle < 0) angle += 2*PI;

		if(type == REPEL)
		{
			if((angle + PI) < (2*PI))
				angle += PI;
			else
				angle -= PI;
		}
		else if(type == TANGENT_RIGHT)
		{
			if((angle - (PI/2)) > 0)
				angle -= (PI/2);
			else
				angle += (3*PI/2);
		}
		else if(type == TANGENT_LEFT)
		{
			if((angle + (PI/2)) < (2*PI))
				angle += (PI/2);
			else
				angle -= (3*PI/2);
		}

		if(angle < 0 || angle > (2*PI))
		{
			cout << "Invalid angle: " << angle << endl;
			exit(1);
		}

		return angle;
	}

public:

	/*
	 * @param x - center x-pos
	 * @param y - center y-pos
	 * @param maxRad - maximum radius of influence
	 * @param minRad - minimum radius of influence
	 * @param strength - threshold, max speed that can be generated
	 * @param t - type (see #define)
	 */
	PotentialField(double x, double y, double maxRad, double minRad, double strength, int t)
	{
		pos[0] = x;
		pos[1] = y;
		maxRadius = maxRad;
		minRadius = minRad;
		threshold = strength;
		type = t;
	}

	/*
	 * Calculate the x- and y-component (xComp and yComp) of the vector
	 * generated by this potential field for an agent at postion (x, y)
	 */
	void CalculateVectorComponents(double x, double y, double& xComp, double& yComp)
	{
		double h = CalculateSpeed(x, y);
		double angle = CalculateAngle(x, y);

		xComp = h * cos(angle);
		yComp = h * sin(angle);
	}

	void PrintField()
	{
		cout << "---Field---" << endl;
		cout << "Pos: " << pos[0] << ", " << pos[1] << endl;
		cout << "Max radius: " << maxRadius << endl;
		cout << "Min radius: " << minRadius << endl;
		cout << "Threshold: " << threshold << endl;
		cout << "Type: " << type << endl;
		cout << "--- End Field---" << endl;
	}
};