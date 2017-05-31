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

void printConstants(BZRC* myTeam);

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

	// Initialize world
	World world = World(&MyTeam);

	// Initialize PD controller
	PdController controller = PdController(&MyTeam);

	// Print constants from server
	printConstants(&MyTeam);

	// Initialize grid filter
	GridFilter gridFilter = GridFilter(&world);

	// Initialize tank objects
	int TANK_COUNT = 4;
	int index = 0;
	vector<Tank*> tanks;
	for(index = 0; index < TANK_COUNT; index++)
	{
		tanks.push_back(new Tank(index, &MyTeam, &world));
	}
	
	// Search around and update grid filter
	for(index = 0; index < TANK_COUNT; index = (index + 1) % TANK_COUNT)
	{
		// Get tank to control
		double angle;
		vector<tank_t> myTanks;
		while(!MyTeam.get_mytanks(&myTanks));
		tank_t tank = myTanks[index];

		// Search occgrid and update grid filter
		occgrid_t occgrid;
		if(MyTeam.get_occgrid(index, &occgrid))
		{
			gridFilter.updateGrid(&occgrid);
		}

		// Reload potential fields and get target angle
		cout << "Tank pos: " << tank.pos[0] << ", " << tank.pos[1] << endl;
		tanks[index]->addHistory(tank.pos[0], tank.pos[1]);
		tanks[index]->reloadFields(&gridFilter);
		angle = tanks[index]->getTargetAngle(tank.pos[0], tank.pos[1]);

		// Set tank angle and speed
		while(!MyTeam.speed(index, controller.TurnTank(index, angle)));

		// Shoot randomly
//		MyTeam.shoot(index);
	}

	return 0;
}

/*
 * Print constants from server
 */
void printConstants(BZRC* myTeam)
{
	vector<constant_t> constants;
	while(!myTeam->get_constants(&constants));

	unsigned int i;
	for(i = 0; i < constants.size(); i++)
		cout << "[" << constants[i].name << "] = " << constants[i].value << endl;
	cout << endl;
}
