#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <unistd.h>
#include "math.h"
using namespace std;

const int MAX_OBSTACLE_CORNERS = 4;
const int kBufferSize = 1024;

// Team
typedef struct team_t
{
	string color;
	int count;
} team_t;

// Base
typedef struct base_t
{
	string color;
	double base_corner[4][2];
} base_t;

// Obstacle
typedef struct obstacle_t
{
	double o_corner[MAX_OBSTACLE_CORNERS][2];
} obstacle_t;

// Flag
typedef struct flag_t
{
	string color;
	string poss_color;
	double pos[2];
} flag_t;

// Shot
typedef struct shot_t
{
	double pos[2];
	double velocity[2];
} shot_t;

// Tank
typedef struct tank_t
{
	int index;
	string callsign;
	string status;
	int shots_avail;
	double time_to_reload;
	string flag;
	double pos[2];
	double angle;
	double velocity[2];
	double angvel;
} tank_t;

// Opponent Tank
typedef struct otank_t
{
	string callsign;
	string color;
	string status;
	string flag;
	double pos[2];
	double angle;
} otank_t;

// Constant
typedef struct constant_t
{
	string name;
	string value;
} constant_t;

/**
 * Class used to split a string into a vector of strings
 * Delimeter is " "
 */
class SplitString
{
	vector <string> MyVector;
	string MyString;

public:

	SplitString(string str)
	{
		MyString=str;
		MyVector.begin();
	}

	vector <string> Split()
	{
		MyVector.clear();
		size_t LastLoc = -1;
		size_t CurLoc = MyString.find(" ", 0);
		while (CurLoc != string::npos) {
			MyVector.push_back(MyString.substr(LastLoc+1, CurLoc-LastLoc-1));
			LastLoc=CurLoc;
			CurLoc = MyString.find(" ", LastLoc+1);
		}
		MyVector.push_back(MyString.substr(LastLoc+1, MyString.size()-LastLoc));
		return MyVector;
	}
};

/*
 * Class used to communicate with server
 */
class BZRC
{
	const char *pcHost;
	int nPort;
	bool debug;
	bool InitStatus;
	char ReplyBuffer[kBufferSize];
	int LineFeedPos;
	int Start;
	int sd;

	int Init()
	{
		ResetReplyBuffer();
		Start = 0;
		struct addrinfo *infop = NULL;
		struct addrinfo hint;

		memset(&hint, 0, sizeof(hint));
		hint.ai_family = AF_INET;
		hint.ai_socktype = SOCK_STREAM;

		char port[10];
		snprintf(port, 10, "%d", nPort);

		if(getaddrinfo(pcHost, port, &hint, &infop) != 0)
		{
			perror("Couldn't lookup host.");
			return 1;
		}

		if((sd = socket(infop->ai_family, infop->ai_socktype, infop->ai_protocol)) < 0)
		{
			perror("Couldn't create socket.");
			return 1;
		}

		if(connect(sd, infop->ai_addr, infop->ai_addrlen) < 0)
		{
			perror("Couldn't connect.");
			close(sd);
		}

		freeaddrinfo(infop);

		if(HandShake() == 1)
		{
			cerr << "Handshake Failed!" << endl;
			return 1;
		}

		return 0;
	}

	// Send line to server
	int SendLine(const char *LineText)
	{
		int Length=(int)strlen(LineText);
		char Command[kBufferSize];
		strcpy(Command, LineText);
		Command[Length]='\n';
		Command[Length+1]='\0';
		if(debug) cout << Command;
		if(send(sd, Command, Length+1, 0) >= 0)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
	
	// Read line back from server
	int ReadReply(char *Reply)
	{
		char acReadBuffer[kBufferSize];

		int nNewBytes = recv(sd, acReadBuffer, kBufferSize, 0);
		if (nNewBytes < 0)
		{
			return -1;
		}
		else if (nNewBytes == 0)
		{
			cerr << "Connection closed by peer." << endl;
			return 0;
		}
		
		memcpy(Reply, &acReadBuffer, nNewBytes);
		if(nNewBytes!=kBufferSize)
		{
			Reply[nNewBytes]='\0';
		}

		return nNewBytes;
	}

	// Only read one line of text from ReplyBuffer
	void ReadLine(char *LineText)
	{
		memset(LineText, '\0', kBufferSize);
		if(strlen(ReplyBuffer) == 0)
		{
			char *Reply;
			Reply = ReplyBuffer;
			ReadReply(Reply);
		}
		int i=0;
		bool done=false;
		while(!done)
		{
			for(i=LineFeedPos+1; (i<kBufferSize && ReplyBuffer[i]); i++)
			{
				if(ReplyBuffer[i]=='\n')
				{
					LineText[i-LineFeedPos-1+Start]='\0';
					LineFeedPos=i;
					Start=0;
					done=true;
					break;
				}
				LineText[i-LineFeedPos-1+Start]=ReplyBuffer[i];
			}
			if(!done)
			{
					Start = (int)strlen(LineText);
					ResetReplyBuffer();	
					char *Reply;
					Reply = ReplyBuffer;
					ReadReply(Reply);
			}
			else
			{
				if(ReplyBuffer[i]=='\0')
				{
					done=true;
					Start=0;
					ResetReplyBuffer();
				}
			}
		}
	}

	// Reset the reply buffer
	void ResetReplyBuffer()
	{
		memset(ReplyBuffer, '\0', kBufferSize);
		LineFeedPos=-1;
	}
	
	// Perform handshake with server
	int HandShake() {
		char str[kBufferSize];
		char *LineText;
		LineText=str;
		ReadLine(LineText);
		if(debug) cout << LineText << endl;
		if (!strcmp(LineText, "bzrobots 1")) {
			const char * Command="agent 1";
			int temp=SendLine(Command);
			if(temp==1) 
				return 1;
			else
				ResetReplyBuffer();
				return 0;
		}
		else
			return 1;
	}

	// Read line into vector
	vector <string> ReadArr() {
		char str[kBufferSize];
		char *LineText=str;
		ReadLine(LineText);
		if(strlen(LineText)!=0) {
			if(debug) cout << LineText << endl;
		}
		while(strlen(LineText)==0) {
			ReadLine(LineText);
			if(debug) cout << LineText << endl;
		}
		SplitString ss=SplitString(LineText);
		return ss.Split();
	}

	// Read acknowledgement
	void ReadAck() {
		vector <string> v=ReadArr();
		if(v.at(0)!="ack") {
			cout << "Didn't receive ack! Exit!" << endl;
			exit(1);
		}
	}

	// Read "ok"	
	bool ReadBool() {
		vector <string> v=ReadArr();
		if(v.at(0)=="ok") {
			return true;
		}
		else if(v.at(0)=="fail"){
			if(debug) cout << "Received fail. Exiting!" << endl;
			return false;
		}
		else {
			if(debug) cout << "Something went wrong. Exiting!" << endl;
			return false;
		}
	}

	// Receive and print another line
	void PrintLine() {
		char str[kBufferSize];
		char *LineText=str;
		ReadLine(LineText);
		if(debug) cout << LineText << endl;
	}

public:

	// Constructor
	BZRC(const char *host, int port, bool debug_mode)
	{
		pcHost = host;
		nPort = port;
		debug = debug_mode;
		if(Init())
		{
			cout << "BZRC initialization failed." << endl;
			InitStatus=false;
			Close();
		}
		else
		{
			InitStatus=true;
		}
	}

	// Get port
	int GetPort(){return nPort;}

	// Get host
	const char *GetHost() {return pcHost;}

	// Get status
	bool GetStatus() {return InitStatus;}

	// Shoot from tank[index]
	bool shoot(int index)
	{
		char char_buff[20];
		sprintf(char_buff, " %d", index);	
		string str_buff="shoot";
		str_buff.append(char_buff);
		const char *Command = str_buff.c_str();
		SendLine(Command);
		ReadAck();
		if(ReadBool()) {
			return true;
		}
		else {
			return false;
		}
	}

	// Adjust speed for tank[index]
	bool speed(int index, double value) {
		char char_buff[20];
		sprintf(char_buff, " %d", index);	
		string str_buff="speed";
		str_buff.append(char_buff);
		sprintf(char_buff, " %f", value);
		str_buff.append(char_buff);
		const char *Command = str_buff.c_str();
		SendLine(Command);
		ReadAck();
		if(ReadBool()) {
			return true;
		}
		else {
			return false;
		}
	}

	// Adjust angular velocity for tank[index]
	bool angvel(int index, double value) {
		char char_buff[20];
		sprintf(char_buff, " %d", index);	
		string str_buff="angvel";
		str_buff.append(char_buff);
		sprintf(char_buff, " %f", value);
		str_buff.append(char_buff);
		const char *Command = str_buff.c_str();
		SendLine(Command);
		ReadAck();
		if(ReadBool()) {
			return true;
		}
		else {
			return false;
		}
	}

	// Get all bases
	bool get_bases(vector <base_t> *AllBases)
	{
		SendLine("bases");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		while(v.at(0)=="base")
		{
			base_t base;
			base.color = v.at(1);
			base.base_corner[0][0] = atof(v.at(2).c_str());
			base.base_corner[0][1] = atof(v.at(3).c_str());
			base.base_corner[1][0] = atof(v.at(4).c_str());
			base.base_corner[1][1] = atof(v.at(5).c_str());
			base.base_corner[2][0] = atof(v.at(6).c_str());
			base.base_corner[2][1] = atof(v.at(7).c_str());
			base.base_corner[3][0] = atof(v.at(8).c_str());
			base.base_corner[3][1] = atof(v.at(9).c_str());
			AllBases->push_back(base);
			v.clear();
			v=ReadArr();
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
	}

	// Get all teams
	bool get_teams(vector <team_t> *AllTeams) {
		SendLine("teams");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		while(v.at(0)=="team") {
			team_t MyTeam;
			MyTeam.color=v.at(1);
			MyTeam.count=atoi(v.at(2).c_str());
			AllTeams->push_back(MyTeam);
			v.clear();
			v=ReadArr();
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
	}

	// Get all obstacles
	bool get_obstacles(vector <obstacle_t> *AllObstacles) {
		SendLine("obstacles");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="obstacle") {
			obstacle_t MyObstacle;
			int j=0;
			while(j+2<(int)v.size()) {
				MyObstacle.o_corner[j/2][0]=atof(v.at(j+1).c_str());
				MyObstacle.o_corner[j/2][1]=atof(v.at(j+2).c_str());
				j=j+2;
			}
			AllObstacles->push_back(MyObstacle);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
	}

	// Get all flags
	bool get_flags(vector <flag_t> *AllFlags) {
		SendLine("flags");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="flag") {
			flag_t MyFlag;
			MyFlag.color=v.at(1);
			MyFlag.poss_color=v.at(2);
			MyFlag.pos[0]=atof(v.at(3).c_str());
			MyFlag.pos[1]=atof(v.at(4).c_str());
			AllFlags->push_back(MyFlag);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
	}

	// Get all shots
	bool get_shots(vector <shot_t> *AllShots) {
		SendLine("shots");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="shot") {
			shot_t MyShot;
			MyShot.pos[0]=atof(v.at(1).c_str());
			MyShot.pos[1]=atof(v.at(2).c_str());
			MyShot.velocity[0]=atof(v.at(3).c_str());
			MyShot.velocity[1]=atof(v.at(4).c_str());
			AllShots->push_back(MyShot);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
	}

	// Get all my tanks
	bool get_mytanks(vector <tank_t> *AllMyTanks) {
		SendLine("mytanks");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="mytank") {
			tank_t MyTank;
			MyTank.index=atoi(v.at(1).c_str());
			MyTank.callsign=v.at(2);
			MyTank.status=v.at(3);
			MyTank.shots_avail=atoi(v.at(4).c_str());
			MyTank.time_to_reload=atof(v.at(5).c_str());
			MyTank.flag=v.at(6);
			MyTank.pos[0]=atof(v.at(7).c_str());
			MyTank.pos[1]=atof(v.at(8).c_str());
			MyTank.angle=atof(v.at(9).c_str());
			MyTank.velocity[0]=atof(v.at(10).c_str());
			MyTank.velocity[1]=atof(v.at(11).c_str());
			MyTank.angvel=atof(v.at(12).c_str());
			AllMyTanks->push_back(MyTank);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			if(debug) cout << v.at(0) << endl;
			return false;
		}
		return true;
	}

	// Get all other tanks
	bool get_othertanks(vector <otank_t> *AllOtherTanks) {
		SendLine("othertanks");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="othertank") {
			otank_t OtherTank;
			OtherTank.callsign=v.at(1);
			OtherTank.color=v.at(2);
			OtherTank.status=v.at(3);
			OtherTank.flag=v.at(4);
			OtherTank.pos[0]=atof(v.at(5).c_str());
			OtherTank.pos[1]=atof(v.at(6).c_str());
			OtherTank.angle=atof(v.at(7).c_str());
			AllOtherTanks->push_back(OtherTank);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
	}

	// Get all constants
	bool get_constants(vector <constant_t> *AllConstants) {
		SendLine("constants");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="constant") {
			constant_t MyConstant;
			MyConstant.name=v.at(1);
			MyConstant.value=v.at(2);
			AllConstants->push_back(MyConstant);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
	}

	// Get value of a single constant
	string getConstantByName(string name)
	{
		vector <constant_t> constants;
		get_constants(&constants);
		unsigned int i;
		for(i = 0; i < constants.size(); i++)
			if(constants[i].name == name)
				return constants[i].value;
		return NULL;
	}

	// Close socket
	int Close()
	{
		close(sd);
		return 0;
	}
};
