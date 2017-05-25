#ifndef GOALTUNNEL_H
#define GOALTUNNEL_H

#define HORIZONTAL		'0'
#define VERTICAL		'1'
#define SEARCH_STATE_0		0
#define SEARCH_STATE_1		1
#define SEARCH_STATE_2		2
#define SEARCH_STATE_3		3
#define SEARCH_STATE_4		4
#define SEARCH_STATE_5		5

#include <vector>
#include "Grid.h"
#include "SokobanAI.h"

using namespace std;

/* GoalTunnel: straight line of not-wall grids only in which can a box reach a set of goals */
struct GoalTunnel
{
	char type_;			//GoalTunnel type(horizontal or vertical)
	Grid start_;		//start grid of the GoalTunnel
	int length_;		//length of the GoalTunnel
};

class GoalTunnelSolver
{
public:
	GoalTunnelSolver(SokobanSolver*);
	bool check_goal_tunnel_deadlock();		//check goal-tunnel pattern dead-locked or not
private:
	SokobanSolver* sokoban_solver_;
	vector<GoalTunnel> goal_tunnel_set_;	//set of GoalTunnels
};

#endif