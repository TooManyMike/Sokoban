#ifndef TRAP_H
#define TRAP_H

#include <vector>
#include "Grid.h"
#include "SokobanAI.h"

using namespace std;

/***
 * Trap: rectangular of not-wall grids with 3 edges surrounded by wall grids, the other edge(called entrance) surrounded by not-wall grids
 * the opposite edge of entrance is called bottom, the length of entrance and bottom is called width, the length of another two edges is called depth
 * the location of not-wall grids surrounded entrance is called opening direction
 * straight line of grids in the Trap vertical to entrance is called axial line
 * more requirements of a Trap: no goal at bottom and not containing goal at each axial line
***/
struct Trap
{
	char type_;		//opening direction
	Grid start_;	//start grid of the entrance 
	int width_;		//width of the trap£¨possible values: 2, 3£©
	int depth_;		//depth of the trap£¨possible values: 1, 2, 3£©
};

class TrapSolver
{
public:
	TrapSolver(SokobanSolver*);
	~TrapSolver();
	bool check_trap_deadlock(StateNode*);	//check trap pattern dead-locked or not
private:
	SokobanSolver* sokoban_solver_;
	vector<Trap> trap_set_;		//set of traps
	int** trap_index_map_;		//map of traps' indexes
	void add_trap(vector<Grid>, int, int, char, int&);	//add valid trap
};

#endif