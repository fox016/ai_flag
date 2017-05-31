
#include <iostream>
#include <fstream>
#include <ostream>
#include "470bot.h"
#include "gridFilter.h"
#include "pdController.h"
#include "potentialField.h"
#include "fieldGenerator.h"

using namespace std;

void initWorld(BZRC* myTeam);
void getVectorComponents(vector<PotentialField*>* fields, double x, double y, double& angle);

const char *kDefaultServerName = "localhost";
const int kDefaultServerPort = 4000;
const int defaultIsDumb = 0;

int main(int argc, char *argv[])
{
	const char *pcHost;
	int nPort;

	// Get server name and port from arguments
	if(argc < 2)
		pcHost = kDefaultServerName;
	else
		pcHost = argv[1];
	if(argc < 3)
		nPort = kDefaultServerPort;
	else
		nPort = atoi(argv[2]);

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

	// Initialize field generator
	vector<PotentialField*> fields;
	FieldGenerator fieldGen = FieldGenerator(&MyTeam, &fields);
	fieldGen.reloadFields();

	// Initialize grid filter
	double truePositive = atof(MyTeam.getConstantByName("truepositive").c_str());
	double trueNegative = atof(MyTeam.getConstantByName("truenegative").c_str());
	GridFilter gridFitler = GridFilter(truePositive, trueNegative);

	// TODO search around
	int index = 0;
	occgrid_t occgrid;
	MyTeam.get_occgrid(index, &occgrid);

	int TANK_COUNT = 1;
	for(index = 0; index < TANK_COUNT; index = (index + 1) % TANK_COUNT)
	{
		double angle;
		vector<tank_t> myTanks;
		while(!MyTeam.get_mytanks(&myTanks));
		tank_t tank = myTanks[index];

		fieldGen.reloadFields();
		cout << "Tank pos: " << tank.pos[0] << ", " << tank.pos[1] << endl;
		getVectorComponents(&fields, tank.pos[0], tank.pos[1], angle);

		while(!MyTeam.speed(index, controller.TurnTank(index, angle)));
		MyTeam.shoot(index);
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
void getVectorComponents(vector<PotentialField*>* fields, double x, double y, double& angle)
{
	double xPart = 0;
	double yPart = 0;
	unsigned int i;
	for(i = 0; i < fields->size(); i++)
	{
		double xComp, yComp;
		fields->at(i)->CalculateVectorComponents(x, y, xComp, yComp);
		xPart += xComp;
		yPart += yComp;
	}

	angle = atan2(yPart, xPart);
	if(angle < 0) angle += 2*PI;
}
