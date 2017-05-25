#ifndef LOCKEDBOX_H
#define LOCKEDBOX_H

#define NO_NEW_LOCKED_BOX			'0'
#define NEW_LOCKED_BOX_ON_GOAL		'1'
#define NEW_LOCKED_BOX_OFF_GOAL		'2'

#include <vector>
#include "SokobanAI.h"

using namespace std;

/* node in locked_goals_state_list_ */
struct LockedGoalsNode
{
	int locked_goals_index_;		//index of locked goals state
	char locked_num_;				//number of locked goals
	char** connected_area_map_;		//map of connectivity(regard locked goal as wall)
	unsigned char*** goals_h_map_;	//maps of minimum steps for a box to reach specified unlocked goals
	LockedGoalsNode* next_;			//pointer of next LockedGoalsNode in the same bucket of locked_goals_state_list_
	LockedGoalsNode(StateNode* node):	locked_goals_index_(node->locked_goals_index_),
										connected_area_map_(NULL),
										goals_h_map_(NULL),
										next_(NULL){
		int t = locked_goals_index_;
		locked_num_ = 0;
		while (t)
		{
			if (t & 1)
			{
				locked_num_++;
			}
			t = t >> 1;
		}
	}
};

class LockedBoxSolver
{
public:
	LockedBoxSolver(SokobanSolver*);
	~LockedBoxSolver();
	bool** locked_goal_map_;				//map of locked goals [dynamic]
	char check_locked_box(StateNode*);		//check new locked boxes all on-goal or not
	bool check_box_accessiblity(StateNode*, LockedGoalsNode*);	//check man can reach all not-on-goal boxes or not
	LockedGoalsNode* get_locked_goals_node(StateNode*);			//get the corresponding LockedGoalsNode
private:
	SokobanSolver* sokoban_solver_;
	int hash_list_len_;								//length of locked_goals_state_list_
	bool** locked_box_maze_;						//maze with locked boxes(regard locked box as wall) [dynamic]
	vector<int> new_locked_goal_indexes;			//indexes of new locked goals [dynamic]
	LockedGoalsNode** locked_goals_state_list_;		//hash list of locked-goals-state [dynamic]
	bool check_locked_square(int, int);				//check boxes in 2 x 2 locked square and its connected zig-zag locked traces(if exist) all on goal or not
	char find_locked_trace(int, int, char, char);	//search for zig-zag locked trace started from a specified locked grid
};

#endif