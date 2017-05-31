class World
{
private:
	vector <constant_t> constants;

	// Get value of a single constant
	string getConstantByName(string name)
	{
		unsigned int i;
		for(i = 0; i < constants.size(); i++)
			if(constants[i].name == name)
				return constants[i].value;
		return NULL;
	}
public:
	int tankRadius;
	string teamColor;	

	int worldSize;

	double truePositive;
	double trueNegative;

	int shotSpeed;

	World(BZRC* myTeam)
	{
		while(!myTeam->get_constants(&constants));
		tankRadius = atoi(getConstantByName("tankradius").c_str());
		teamColor = getConstantByName("team");
		worldSize = atoi(getConstantByName("worldsize").c_str());
		truePositive = atof(getConstantByName("truepositive").c_str());
		trueNegative = atof(getConstantByName("truenegative").c_str());
		shotSpeed = atoi(getConstantByName("shotspeed").c_str());
	}
};
