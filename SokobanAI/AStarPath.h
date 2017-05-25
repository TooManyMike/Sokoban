#ifndef ASTARPATH_H
#define ASTARPATH_H

#define UNVISITED		0
#define OPEN			1
#define CLOSED			2

#include <vector>
#include "Grid.h"

using namespace std;

struct PosNode
{
	char x_, y_;				//coordinate components of the grid
	char f_, g_, h_;			//f, g, h values in heuristic search
	char state_;				//state of the PosNode: unvisited, in open list or in closed list
	PosNode* predecessor_;		//predecessor PosNode
};

class AStarPath
{
public:
	AStarPath(bool**, int, int);
	~AStarPath();
	void set(Grid, Grid);		//set start_ and end_
	bool find();				//check path existing
	int get(char*&);			//get path steps		
private:
	Grid start_, end_;			//start and end grids
	int width_, height_;		//width and height of map_
	bool** map_;				//the map where path searching is conducted. 'true' represents passable grid, 'false' represents blocked grid
	PosNode** pos_node_;		//matrix of PosNodes, same size with map
	vector<PosNode*> open_list_;		//open list in heuristic search
	void try_expand(PosNode*, char);	//try expanding neighbor grids
};

#endif