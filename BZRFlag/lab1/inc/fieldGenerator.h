
class FieldGenerator
{
private:

	BZRC* myTeam;
	vector<PotentialField*>* fields;

	/*
	 * Make repulsive and tangential fields for all obstacles
	 */
	void createObstacleRepel()
	{
		vector<obstacle_t> obstacles;
		myTeam->get_obstacles(&obstacles);

		unsigned int i;
		int j;
		for(i = 0; i < obstacles.size(); i++)
		{
			obstacle_t obs = obstacles[i];
			for(j = 0; j < MAX_OBSTACLE_CORNERS; j++)
			{
				if(obs.o_corner[j][0] < 1 && obs.o_corner[j][0] > 0 &&
					obs.o_corner[j][1] < 1 && obs.o_corner[j][1] > 0)
				{
					break;
				}
				fields->push_back(new PotentialField(obs.o_corner[j][0],
					obs.o_corner[j][1], 50, 0, 0.5, REPEL));
				fields->push_back(new PotentialField(obs.o_corner[j][0],
					obs.o_corner[j][1], 60, 0, 0.5, TANGENT_LEFT));
			}
		}
	}

	/*
	 * Make repulsive and tangential fields for all tanks
	 */
	void createTankRepel()
	{
		double tankRadius = atof(myTeam->getConstantByName("tankradius").c_str());

		vector<tank_t> myTanks;
		vector<otank_t> otherTanks;
		myTeam->get_mytanks(&myTanks);
		myTeam->get_othertanks(&otherTanks);

		// Repel other tanks
		unsigned int i;
		for(i = 0; i < otherTanks.size(); i++)
		{
			otank_t avoidTank = otherTanks[i];
			fields->push_back(new PotentialField(avoidTank.pos[0],
				avoidTank.pos[1], tankRadius*2, 0, 0.1, TANGENT_LEFT));
		}
		
		// Repel own tanks
		for(i = 0; i < myTanks.size(); i++)
		{
			tank_t avoidTank = myTanks[i];
			fields->push_back(new PotentialField(avoidTank.pos[0],
				avoidTank.pos[1], tankRadius*2, tankRadius, 0.1, TANGENT_LEFT));
		}
	}

	/*
	 * If I do not have an enemy flag, create attraction to enemy flag
	 */
	void createFlagAttract()
	{
		string myColor = myTeam->getConstantByName("team");
		double worldSize = atof(myTeam->getConstantByName("worldsize").c_str());
		vector <flag_t> flags;
		myTeam->get_flags(&flags);
		flag_t enemyFlag;

		// Find enemy flag
		unsigned int i;
		for(i = 0; i < flags.size(); i++)
		{
			if(flags[i].color != myColor)
			{
				enemyFlag = flags[i];
				break;
			}
		}

		fields->push_back(new PotentialField(enemyFlag.pos[0],
			enemyFlag.pos[1], worldSize*2, 0.0, 2.0, ATTRACT));
	}

	/*
	 * Creates attractive field for home base
	 */
	void createBaseAttract()
	{
		string myColor = myTeam->getConstantByName("team");
		double worldSize = atof(myTeam->getConstantByName("worldsize").c_str());
		vector <base_t> bases;
		myTeam->get_bases(&bases);

		unsigned int i, j;
		for(i = 0; i < bases.size(); i++)
		{
			if(bases[i].color == myColor)
			{
				for(j = 0; j < 4; j++)
				{ 
					fields->push_back(new PotentialField(bases[i].base_corner[j][0],
						bases[i].base_corner[j][1], worldSize*2, 0.0, 0.25, ATTRACT));
				}
				return;
			}
		}
	}

	/*
	 * Returns true iff I have an enemy flag
	 */
	bool hasEnemyFlag()
	{
		string myColor = myTeam->getConstantByName("team");
		vector <flag_t> flags;
		myTeam->get_flags(&flags);

		unsigned int i;
		for(i = 0; i < flags.size(); i++)
		{
			if(flags[i].poss_color == myColor)
			{
				return true;
			}
		}
		return false;
	}

public:

	/*
	 * Constructor
	 * @param team - used to communicate with server
	 * @param fieldList - list of fields this generator affects
	 */
	FieldGenerator(BZRC* team, vector<PotentialField*>* fieldList)
	{
		myTeam = team;
		fields = fieldList;
	}

	/*
	 * Destructor
	 */
	~FieldGenerator()
	{
		freeFields();
	}

	/*
	 * Reload all potential fields
	 */
	void reloadFields()
	{
		freeFields();
		fields->clear();

		// Create repulsive/gantential fields for all obstacles
		createObstacleRepel();
		
		// Create repulsive/tangential fields for all tanks
		createTankRepel();
		
		// Create attractive field for an enemy flag (or own base if I have an enemy flag)
		if(!hasEnemyFlag())
			createFlagAttract();
		else
			createBaseAttract();
	}

	/*
	 * Free memory for all fields
	 */
	void freeFields()
	{
		unsigned int i;
		for(i = 0; i < fields->size(); i++)
			delete fields->at(i);
	}
};
