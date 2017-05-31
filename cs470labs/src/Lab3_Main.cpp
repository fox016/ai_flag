#include <iostream>
#include <fstream>
#include <ostream>
#include "myClock.h"
#include "470bot.h"
#include "world.h"
#include "pdController.h"
#include "tank.h"

using namespace std;

void printConstants(BZRC* myTeam);

int main(int argc, char *argv[])
{
	int nPort;
	int tank_type;

	// Get server name and port from arguments
	if(argc != 2)
	{
		cout << "Usage: lab3 <tank_type>" << endl;
		exit(1);
	}

	// Get type from args
	tank_type = atoi(argv[1]);

	// Determine port
	if(tank_type == HUNTER)
		nPort = 22222;
	else
		nPort = 11111;

	// Connect to BZRC server
	BZRC MyTeam = BZRC("localhost", nPort, false);
	if(!MyTeam.GetStatus())
	{
		cout << "Can't connect to BZRC server." << endl;
		exit(1);
	}

	// Initialize world
	World world = World(&MyTeam);

	// Print constants from server
	printConstants(&MyTeam); 

	// Initialize tank
	Tank(0, &MyTeam, &world, tank_type);

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
