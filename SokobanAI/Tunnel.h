#ifndef TUNNEL_H
#define TUNNEL_H

#define LEFT_RIGHT_STRAIGHT		'1'
#define UP_DOWN_STRAIGHT		'2'
#define LEFT_UP_TURNING			'3'
#define LEFT_DOWN_TURNING		'4'
#define RIGHT_UP_TURNING		'5'
#define RIGHT_DOWN_TURNING		'6'

#include "Grid.h"
#include "SokobanAI.h"

/***
 * Tunnel: line of not-wall grids each of which having 2 and only 2 not-wall neighbor grids
 * more requirements of a Tunnel: the outer not-wall neighbor grids of the Tunnel have more than 2 not-wall neighbor grids
***/
struct Tunnel
{
	int length_;		//length of the Tunnel
	bool straight_;		//Tunnel straight or not
	char direction_;	//direction of the Tunnel(if straight)
	bool start_out_block_, end_out_block_;		//outer neighbor grid blocking the Tunnel or not	
	Grid start_, end_;			//Tunnel terminal grids
	Grid start_out_, end_out_;	//outer neighbor grids
};

class TunnelSolver
{
public:
	TunnelSolver(SokobanSolver*);
	~TunnelSolver();
	bool check_tunnel_deadlock(StateNode*);					//check tunnel pattern dead-locked or not
	int get_pass_tunnel_distance(StateNode*, int, char);	//get distance for a box to move through a Tunnel
private:
	SokobanSolver* sokoban_solver_;
	vector<Tunnel> tunnel_set_;			//set of Tunnels
	bool** tunnel_block_entrance_map_;	//map of grids blocking a Tunnel's entrance
	int** tunnel_index_map_;			//map of Tunnels' indexes
	bool check_tunnel_block(StateNode*, int);	//check Tunnel blocked or not in condition that one specified outer neighbor grid is under a box
};

#endif