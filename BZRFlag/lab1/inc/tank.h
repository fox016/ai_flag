
void printTank(tank_t* tank);
void moveTank(BZRC* myTeam, int index, double maxSpeed, int waitTime);

void printTank(tank_t* tank)
{
	cout << "Index: " << tank->index << endl;
	cout << "Position: " << tank->pos[0] << ", " << tank->pos[1] << endl;
	cout << "Velocity: " << tank->velocity[0] << ", " << tank->velocity[1] << endl;
	cout << "Angle: " << tank->angle << endl;
	cout << "Ang Velocity: " << tank->angvel << endl;
}

void moveTankTime(BZRC* myTeam, int index, double maxSpeed, useconds_t waitTime)
{
	myTeam->speed(index, maxSpeed);
	usleep(waitTime);
	myTeam->speed(index, 0);
}

void turnTankTime(BZRC* myTeam, int index, double maxAngVelocity, useconds_t waitTime)
{
	myTeam->angvel(index, maxAngVelocity);
	usleep(waitTime);
	myTeam->angvel(index, 0);
}
