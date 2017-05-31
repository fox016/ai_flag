#include <iostream>
#include <fstream>
#include <ostream>
#include "470bot.h"
#include "world.h"
#include "gridFilter.h"
#include "pdController.h"
#include "potentialField.h"
#include "fieldGenerator.h"
#include "tank.h"

using namespace std;

void initWorld(BZRC* myTeam);
void writeGnuFile(ostream& os, BZRC* myTeam, vector<PotentialField*>* fields);
void getVectorComponents(vector<PotentialField*>* fields, double x, double y, double& speed,
			double& angle, double& xPart, double& yPart);

const char *kDefaultServerName = "localhost";
const int kDefaultServerPort = 4000;
const int defaultIsDumb = 0;

int main(int argc, char *argv[])
{
	const char *pcHost;
	int nPort;
	int isDumb;

	// Get server name and port from arguments
	if(argc < 2)
		pcHost = kDefaultServerName;
	else
		pcHost = argv[1];
	if(argc < 3)
		nPort = kDefaultServerPort;
	else
		nPort = atoi(argv[2]);
	if(argc < 4)
		isDumb = defaultIsDumb;
	else
		isDumb = atoi(argv[3]);

	// Connect to BZRC server
	BZRC MyTeam = BZRC(pcHost, nPort, false);
	if(!MyTeam.GetStatus())
	{
		cout << "Can't connect to BZRC server." << endl;
		exit(1);
	}

	// Initialize PD controller
	PdController controller = PdController(&MyTeam);

	// Initialize world
	initWorld(&MyTeam);
	World world = World(&MyTeam);

	// Initialize field generator
	vector<PotentialField*> fields;
	FieldGenerator fieldGen = FieldGenerator(&MyTeam, &world);
	fieldGen.reloadFields();

	// Initialize stream to write gnuplot file
	filebuf fb;
	fb.open("./plot/test.gpi", std::ios::out);
	ostream os(&fb);
	writeGnuFile(os, &MyTeam, &fields);
	fb.close();

	// Really dumb agents
	if(isDumb == 1)
	{
		while(true)
		{
			moveTankTime(&MyTeam, 0, 0.5, 3000000);
			MyTeam.shoot(0);
			turnTankTime(&MyTeam, 0, 0.5, 2000000);
			MyTeam.shoot(0);
			moveTankTime(&MyTeam, 1, 0.5, 3000000);
			MyTeam.shoot(1);
			turnTankTime(&MyTeam, 1, 0.5, 2000000);
			MyTeam.shoot(1);
		}
	}

	// Rational agents
	else
	{
		double speed, angle, xPart, yPart;
		int index = 0;
		int TANK_COUNT = 1;
		for(index = 0; index < TANK_COUNT; index = (index + 1) % TANK_COUNT)
		{
			vector<tank_t> myTanks;
			while(!MyTeam.get_mytanks(&myTanks));
			tank_t tank = myTanks[index];

			fieldGen.reloadFields();
			cout << "Tank pos: " << tank.pos[0] << ", " << tank.pos[1] << endl;
			getVectorComponents(&fields, tank.pos[0], tank.pos[1], speed, angle, xPart, yPart);

			while(!MyTeam.speed(index, controller.TurnTank(index, angle)));
			MyTeam.shoot(index);
		}
	}

	// Clean up
	fieldGen.freeFields();
	MyTeam.Close();

	return 0;
}

/*
 * Init world
 */
void initWorld(BZRC* myTeam)
{
	vector<constant_t> constants;
	while(!myTeam->get_constants(&constants));

	unsigned int i;
	for(i = 0; i < constants.size(); i++)
		cout << "[" << constants[i].name << "] = " << constants[i].value << endl;
	cout << endl;
}

/*
 * Get the speed and the angle of the vector acting on an agent
 * at postion (x, y), taking all potential fiends into account
 */
void getVectorComponents(vector<PotentialField*>* fields, double x, double y, double& speed,
			double& angle, double& xPart, double& yPart)
{
	xPart = 0;
	yPart = 0;
	unsigned int i;
	for(i = 0; i < fields->size(); i++)
	{
		double xComp, yComp;
		fields->at(i)->CalculateVectorComponents(x, y, xComp, yComp);
		xPart += xComp;
		yPart += yComp;
	}

	speed = sqrt(pow(xPart, 2.0) + pow(yPart, 2.0));
	speed = min(speed, 1.0);

	angle = atan2(yPart, xPart);
	if(angle < 0) angle += 2*PI;
}

/*
 * Write GNU plot file
 */
void writeGnuFile(ostream& os, BZRC* myTeam, vector<PotentialField*>* fields)
{
	double worldSize = atof(myTeam->getConstantByName("worldsize").c_str());
	double halfSize = worldSize / 2.0;

	// Setup
	os << "set xrange [-" << halfSize << ": " << halfSize << "]" << endl;
	os << "set yrange [-" << halfSize << ": " << halfSize << "]" << endl;
	os << "unset key" << endl;
	os << "set size square" << endl << endl;

	// Obstacles
	os << "unset arrow" << endl;

	vector<obstacle_t> obstacles;
	myTeam->get_obstacles(&obstacles);

	unsigned int i;
	int j;
	for(i = 0; i < obstacles.size(); i++)
	{
		obstacle_t obs = obstacles[i];
		for(j = 1; j <= MAX_OBSTACLE_CORNERS; j++)
		{
			if(obs.o_corner[j][0] < 1 && obs.o_corner[j][0] > 0 &&
				obs.o_corner[j][1] < 1 && obs.o_corner[j][1] > 0)
			{
				os << "set arrow from " << obs.o_corner[j-1][0] << ", " << obs.o_corner[j-1][1];
				os << " to " << obs.o_corner[0][0] << ", " << obs.o_corner[0][1];
				os << " nohead lt 3" << endl;
				break;
			}

			os << "set arrow from " << obs.o_corner[j-1][0] << ", " << obs.o_corner[j-1][1];
			if(j != MAX_OBSTACLE_CORNERS)
				os << " to " << obs.o_corner[j][0] << ", " << obs.o_corner[j][1];
			else
				os << " to " << obs.o_corner[0][0] << ", " << obs.o_corner[0][1];
			os << " nohead lt 3" << endl;
		}
	}
	os << endl;

	// Potential Fields
	os << "plot '-' with vectors head" << endl;
	
	double speed, angle, xPart, yPart;
	int x, y;
	for(x = (halfSize * -1); x < halfSize; x += 20)
	{
		for(y = (halfSize * -1); y < halfSize; y += 20)
		{
			getVectorComponents(fields, x, y, speed, angle, xPart, yPart);
			xPart *= 20;
			yPart *= 20;
			os << x << " " << y << " " << xPart << " " << yPart << endl;
		}
	}
	os << "e" << endl;
}
