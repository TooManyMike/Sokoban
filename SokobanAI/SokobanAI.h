#ifndef SOKOBANAI_H
#define SOKOBANAI_H

#define DEEPMAX		200

#include <vector>
#include <stack>
#include "Grid.h"
#include "CombIndex.h"
#include "AStarPath.h"
#include "MinAssign.h"

using namespace std;

struct ClosedNode;
struct LockedGoalsNode;
class LockedBoxSolver;
class TrapSolver;
class TunnelSolver;
class GoalTunnelSolver;

/* search state node */
struct StateNode
{
	vector<Grid> box_;			//boxes' positions(in ascending order, with x-coordinate in preference)
	__int64 boxes_pos_index_;	//index of boxes' position
	int locked_goals_index_;	//index of locked goals state
	Grid man_;					//man's position
	char moved_box_index_;		//index of last moved box
	char direction_;			//motion direction of last moved box
	char box_move_distance_;	//motion distance of last moved box
	char man_move_distance_;	//distance of man's last motion(not pushing box)
	char alive_child_num_;		//number of child StateNodes(neither dead-locked nor loop-locked)
	char g_;					//g value in heuristic search(distance of all boxes' motion)
	short f_, h_;				//f, h values in heuristic search
	unsigned char moved_box_h_;	//h value of last moved box
	ClosedNode* closed_node_;	//the image of StateNode in closed list
};

/* image of StateNode in closed list */
struct ClosedNode
{
	__int64 boxes_pos_index_;
	Grid man_;
	char g_;
	bool dead_locked_;		//corresponding StateNode dead-locked or not
	bool visited_;			//corresponding StateNode visited or not in current layer of IDA search
	ClosedNode* next_;		//pointer of next ClosedNode in the same bucket of closed list
	ClosedNode(StateNode* node):	boxes_pos_index_(node->boxes_pos_index_),
									man_(node->man_),
									g_(node->g_),
									dead_locked_(false),
									visited_(true),
									next_(NULL){}
};

/* node in dead_locked_boxes_pos_list_ */
struct BoxesPosNode
{
	__int64 boxes_pos_index_;
	BoxesPosNode* next_;	//pointer of next BoxesPosNode in the same bucket of dead_locked_boxes_pos_list_
	BoxesPosNode(StateNode* node): boxes_pos_index_(node->boxes_pos_index_), next_(NULL){}
};


/*******************************************************************************************
*
* SokobanSolver can solve a small-scale sokoban problem(maximum map size: 16 x 16)
*
* constructor: (char** map, int width, int height)
* parameters:
* map[x][y]: xsb-format of grid(x, y) of the map, with original point on the top-left corner
* width: width of the map
* height: height of the map
*
* use SokobanSolver.solve(char* &step) function to get solution steps
* parameter:
* [out] step: stores lurd-format solution steps
* return value:
* non-negative: number of solution steps
* MAP_ERROR(-1): input invalid width, height(<=0 or >16) or map
* NO_SOLUTION(-2): solution doesn't exist
* CANT_FIND_SOLUTION(-3): incapable to find a solution
*
*******************************************************************************************/

class SokobanSolver
{
public:
	SokobanSolver(char**, int, int);
	~SokobanSolver();
	int	width_;					//width of map
	int height_;				//height of map
	int box_num_;				//number of boxes
	int goal_num_;				//number of goals
	bool** no_box_maze_;		//maze without boxes
	bool** all_box_maze_;		//maze with boxes(regard box as wall) [dynamic]
	bool** box_map_;			//map of boxes [dynamic]
	int** box_pos_index_map_;	//map of box-not-dead-locked positions' indexes
	int** goal_index_map_;		//map of goals' indexes
	vector<Grid> goals_;		//goals' positions
	int solve(char*&);			//get solution steps and return the number of steps
	void find_pattern(vector<Grid>&, char**, int, int);	//search for pattern in no_box_maze_
private:
	bool size_error_;			//width_ and height_ invalid or not
	bool map_error_;			//input map has error or not
	bool solvable_;				//capable to solve the problem or not
	bool no_solution_;			//detected no solution or not from start state
	bool find_;					//found solution or not
	int depth_limit_;			//search depth limit [dynamic]
	int box_pos_num_;			//number of not-dead-locked-box's positions
	int hash_list_len_;			//length of closed_list_ and dead_locked_boxes_pos_list_
	int** man_reachable_map_;	//map of man reachable area(regard box as wall) [dynamic]
	unsigned char*** goals_h_map_;		//maps of minimum steps for a box to reach specified goals
	StateNode start_state_;				//start StateNode
	vector<StateNode*> expanded_list_;	//sorted list of new expanded StateNodes [dynamic]
	stack<StateNode*> open_list_;		//open list of StateNodes [dynamic]
	stack<StateNode*> ancestor_list_;	//ancestor StateNodes of the current StateNode [dynamic]
	ClosedNode** closed_list_;					//hash list of ClosedNodes [dynamic]
	BoxesPosNode** dead_locked_boxes_pos_list_;	//hash list of dead-locked boxes' position [dynamic]
	/******************************** basic computation solvers ***********************************/
	CombIndex* comb_index_solver_;			//combination index calculation(to get boxes_pos_index_)
	AStarPath* path_solver_;				//A* path solution(to get man's path) [dynamic]
	MinAssign* min_assignment_solver_;		//minimum assignment(to get h value) [dynamic]
	/**********************************************************************************************/
	/******************************* dead-lock detection solvers **********************************/
	LockedBoxSolver* locked_box_solver_;	//locked box pattern dead-lock
	TrapSolver* trap_solver_;				//trap pattern dead-lock
	TunnelSolver* tunnel_solver_;			//tunnel pattern dead-lock
	GoalTunnelSolver* goal_tunnel_solver_;	//goal-tunnel pattern dead-lock
	/**********************************************************************************************/
	void try_expand(StateNode*, int, char);		//try to expand a StateNode
	bool check_dup(StateNode*);					//check a StateNode is duplicated or not
	void add_dead_locked_boxes_pos(StateNode*);	//insert corresponding BoxesPosNode into dead_locked_boxes_pos_list_
	void calc_boxes_pos_index(StateNode*);		//calculate index of boxes' position
	void calc_start_h();						//calculate h value of start_state_
	void calc_h(StateNode*, LockedGoalsNode*);	//calculate h value of a StateNode and update its f value
	void map_load_box(StateNode*);				//set maps with boxes of the current StateNode
	void map_clear_box(StateNode*);				//restore maps with no box
	void map_move_box(StateNode*);				//modify maps with boxes of the expanded StateNode
	void map_inv_move_box(StateNode*);			//restore maps with boxes of the current StateNode
	StateNode* make_start_node();				//generate start StateNode
	StateNode* make_child_node(StateNode*, int, int, char);		//generate child StateNode
};

#endif
