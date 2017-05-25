#define HASH_LIST_LENGTH_MAX	10000000

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <queue>
#include <deque>
#include <algorithm>
#include "Sokoban.h"
#include "SokobanAI.h"
#include "SmartMap.h"
#include "LockedBox.h"
#include "Trap.h"
#include "Tunnel.h"
#include "GoalTunnel.h"

using namespace std;

/* dll export function */
int __stdcall SokobanSolve(char** map, int width, int height, char* &step)
{
	SokobanSolver problem(map, width, height);
	return problem.solve(step);
}

SokobanSolver::SokobanSolver(char** map, int width, int height)
{
	int i, j, x, y;
	/* check size error, set width_, height_ */
	size_error_ = false;
	if (width <= 0 || width > 16 || height <= 0 || height > 16)
	{
		size_error_ = true;
		return;
	}
	else
	{
		width_ = width;
		height_ = height;
	}
	/* allocate memory for maze_temp, goal_index_map_ */
	bool** maze_temp = (bool**)malloc(width * sizeof(bool*));
	goal_index_map_ = (int**)malloc(width * sizeof(int*));
	maze_temp[0] = (bool*)malloc(width * height);
	goal_index_map_[0] = (int*)malloc(width * height * sizeof(int));
	for (i = 1; i < width; i++)
	{
		maze_temp[i] = maze_temp[i - 1] + height;
		goal_index_map_[i] = goal_index_map_[i - 1] + height;
	}
	/* check input map error, set start_state_, box_num_, goal_number_, goals_, goal_index_map_, maze_temp */
	Grid temp;
	bool found_man = false;
	map_error_ = false;
	box_num_ = 0;
	goal_num_ = 0;
	for (x = 0; x < width; x++)
	{
		for (y = 0; y < height; y++)
		{
			temp.x_ = x;
			temp.y_ = y;
			switch (map[x][y])
			{
			case MAN:
				maze_temp[x][y] = true;
				if (!found_man)
				{
					found_man = true;
					start_state_.man_ = temp;
				}
				else
				{
					map_error_ = true;
					break;
				}
				goal_index_map_[x][y] = -1;
				break;
			case MAN_ON_GOAL:
				maze_temp[x][y] = true;
				if (!found_man)
				{
					found_man = true;
					start_state_.man_ = temp;
				}
				else
				{
					map_error_ = true;
					break;
				}
				goals_.push_back(temp);
				goal_index_map_[x][y] = goal_num_++;
				break;
			case BOX:
				maze_temp[x][y] = true;
				start_state_.box_.push_back(temp);
				box_num_++;
				goal_index_map_[x][y] = -1;
				break;
			case BOX_ON_GOAL:
				maze_temp[x][y] = true;
				start_state_.box_.push_back(temp);
				box_num_++;
				goals_.push_back(temp);
				goal_index_map_[x][y] = goal_num_++;
				break;
			case GOAL:
				maze_temp[x][y] = true;
				goals_.push_back(temp);
				goal_index_map_[x][y] = goal_num_++;
				break;
			case FLOOR:
				maze_temp[x][y] = true;
				goal_index_map_[x][y] = -1;
				break;
			case WALL:
				maze_temp[x][y] = false;
				goal_index_map_[x][y] = -1;
				break;
			default:
				map_error_ = true;
			}
			if (map_error_)
			{
				break;
			}
		}
		if (map_error_)
		{
			break;
		}
	}
	if (!found_man || box_num_ == 0 || box_num_ != goal_num_ || !check_map_closure(maze_temp, width, height, start_state_.man_))
	{
		map_error_ = true;
	}
	if (map_error_)
	{
		free(maze_temp[0]);
		free(goal_index_map_[0]);
		free(maze_temp);
		free(goal_index_map_);
		return;
	}
	/* revise maze_temp */
	for (x = 1; x < width - 1; x++)
	{
		for (y = 1; y < height - 1; y++)
		{
			if (map[x][y] == FLOOR)
			{
				i = 0;
				if (map[x - 1][y] == WALL)
				{
					i++;
				}
				if (map[x + 1][y] == WALL)
				{
					i++;
				}
				if (map[x][y - 1] == WALL)
				{
					i++;
				}
				if (map[x][y + 1] == WALL)
				{
					i++;
				}
				if (i >= 3)
				{
					maze_temp[x][y] = false;
				}
			}
		}
	}
	/* allocate memory for maps of SokobanSolver */
	no_box_maze_ = (bool**)malloc(width * sizeof(bool*));
	all_box_maze_ = (bool**)malloc(width * sizeof(bool*));
	box_map_ = (bool**)malloc(width * sizeof(bool*));
	box_pos_index_map_ = (int**)malloc(width * sizeof(int*));
	man_reachable_map_ = (int**)malloc(width * sizeof(int*));
	no_box_maze_[0] = (bool*)malloc(width * height);
	all_box_maze_[0] = (bool*)malloc(width * height);
	box_map_[0] = (bool*)malloc(width * height);
	box_pos_index_map_[0] = (int*)malloc(width * height * sizeof(int));
	man_reachable_map_[0] = (int*)malloc(width * height * sizeof(int));
	for (i = 1; i < width; i++)
	{
		no_box_maze_[i] = no_box_maze_[i - 1] + height;
		all_box_maze_[i] = all_box_maze_[i - 1] + height;
		box_map_[i] = box_map_[i - 1] + height;
		box_pos_index_map_[i] = box_pos_index_map_[i - 1] + height;
		man_reachable_map_[i] = man_reachable_map_[i - 1] + height;
	}
	/* set no_box_maze_, initialize all_box_maze_, box_map_ */
	get_reachability_map(no_box_maze_, maze_temp, width, height, start_state_.man_);
	free(maze_temp[0]);
	free(maze_temp);
	memcpy(all_box_maze_[0], no_box_maze_[0], width * height);
	memset(box_map_[0], 0, width_ * height_);
	/* allocate memory and set goals_h_map_ */
	goals_h_map_ = (unsigned char***)malloc(goal_num_ * sizeof(unsigned char**));
	goals_h_map_[0] = (unsigned char**)malloc(goal_num_ * width * sizeof(unsigned char*));
	goals_h_map_[0][0] = (unsigned char*)malloc(goal_num_ * width * height);
	for (i = 1; i < goal_num_; i++)
	{
		goals_h_map_[i] = goals_h_map_[i - 1] + width;
		goals_h_map_[i][0] = goals_h_map_[i - 1][0] + width * height;
	}
	for (i = 0; i < goal_num_; i++)
	{
		for (j = 1; j < width; j++)
		{
			goals_h_map_[i][j] = goals_h_map_[i][j - 1] + height;
		}
		get_goal_h_map(goals_h_map_[i], no_box_maze_, width, height, goals_[i]);
	}
	/* set box_pos_num_, box_pos_index_map_ */
	bool check;
	box_pos_num_ = 0;
	for (x = 0; x < width; x++)
	{
		for (y = 0; y < height; y++)
		{
			check = false;
			for (i = 0; i < goal_num_; i++)
			{
				if (goals_h_map_[i][x][y] != LARGE)
				{
					check = true;
					break;
				}
			}
			if (check)
			{
				box_pos_index_map_[x][y] = box_pos_num_++;
			}
			else
			{
				box_pos_index_map_[x][y] = -1;
			}
		}
	}
	/* check solvable */
	solvable_ = true;
	int box_pos_max[31] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
							188, 151, 127, 110, 98, 89, 82, 77, 73, 70,
							68, 66, 64, 63, 62, 61, 61, 61, 60, 60 };
	if (box_num_ > 30 || (box_num_ > 10 && box_pos_num_ > box_pos_max[box_num_]))
	{
		solvable_ = false;
		return;
	}
	/* initialize solvers */
	comb_index_solver_ = new CombIndex(box_pos_num_, box_num_);
	path_solver_ = new AStarPath(all_box_maze_, width, height);
	min_assignment_solver_ = new MinAssign(box_num_);
	locked_box_solver_ = new LockedBoxSolver(this);
	trap_solver_ = new TrapSolver(this);
	tunnel_solver_ = new TunnelSolver(this);
	goal_tunnel_solver_ = new GoalTunnelSolver(this);
	/* further set start_state_, set no_solution_ */
	start_state_.locked_goals_index_ = 0;
	start_state_.g_ = 0;
	no_solution_ = false;
	for (i = 0; i < box_num_; i++)
	{
		if (box_pos_index_map_[start_state_.box_[i].x_][start_state_.box_[i].y_] == -1)
		{
			no_solution_ = true;
			break;
		}
	}
	if (!no_solution_)
	{
		calc_boxes_pos_index(&start_state_);
		calc_start_h();
		if (start_state_.h_ >= LARGE)
		{
			for (i = 0; i < box_num_; i++)
			{
				if (min_assignment_solver_->cost_[i][min_assignment_solver_->bigraph_->left_link_[i]] == LARGE)
				{
					no_solution_ = true;
					break;
				}
			}
		}
	}
	/* set hash_list_len_, allocate memory and initialize closed_list_, dead_locked_boxes_pos_list_ */
	__int64 boxes_pos_num = comb_index_solver_->comb(box_pos_num_, box_num_);
	hash_list_len_ = int((boxes_pos_num < HASH_LIST_LENGTH_MAX) ? boxes_pos_num : HASH_LIST_LENGTH_MAX);
	closed_list_ = (ClosedNode**)malloc(hash_list_len_ * sizeof(ClosedNode*));
	dead_locked_boxes_pos_list_ = (BoxesPosNode**)malloc(hash_list_len_ * sizeof(BoxesPosNode*));
	memset(closed_list_, 0, hash_list_len_ * sizeof(ClosedNode*));
	memset(dead_locked_boxes_pos_list_, 0, hash_list_len_ * sizeof(BoxesPosNode*));
}

SokobanSolver::~SokobanSolver()
{
	if (size_error_ || map_error_)
	{
		return;
	}
	/* release memory of maps */
	free(no_box_maze_[0]);
	free(all_box_maze_[0]);
	free(box_map_[0]);
	free(box_pos_index_map_[0]);
	free(goal_index_map_[0]);
	free(man_reachable_map_[0]);
	free(no_box_maze_);
	free(all_box_maze_);
	free(box_map_);
	free(box_pos_index_map_);
	free(goal_index_map_);
	free(man_reachable_map_);
	free(goals_h_map_[0][0]);
	free(goals_h_map_[0]);
	free(goals_h_map_);
	if (solvable_)
	{
		/* release memory of solvers */
		delete comb_index_solver_;
		delete path_solver_;
		delete min_assignment_solver_;
		delete locked_box_solver_;
		delete trap_solver_;
		delete tunnel_solver_;
		delete goal_tunnel_solver_;
		/* release memory of dead_locked_boxes_pos_list_, closed_list_ */
		ClosedNode *temp1, *next1;
		BoxesPosNode *temp2, *next2;
		for (int i = 0; i < hash_list_len_; i++)
		{
			temp1 = closed_list_[i];
			temp2 = dead_locked_boxes_pos_list_[i];
			while (temp1 != NULL)
			{
				next1 = temp1->next_;
				delete temp1;
				temp1 = next1;
			}
			while (temp2 != NULL)
			{
				next2 = temp2->next_;
				delete temp2;
				temp2 = next2;
			}
		}
		free(closed_list_);
		free(dead_locked_boxes_pos_list_);
	}
}

void SokobanSolver::find_pattern(vector<Grid>& findings, char** pattern, int pattern_width, int pattern_height)
{
	int x, y, i, j;
	bool check;
	Grid temp;
	findings.clear();
	for (x = 0; x <= width_ - pattern_width; x++)
	{
		for (y = 0; y <= height_ - pattern_height; y++)
		{
			check = true;
			for (i = 0; i < pattern_width && check; i++)
			{
				for (j = 0; j < pattern_height; j++)
				{
					if (*((char*)pattern + i * pattern_height + j) != 'N' && char(no_box_maze_[x + i][y + j]) != *((char*)pattern + i * pattern_height + j))
					{
						check = false;
						break;
					}
				}
			}
			if (check)
			{
				temp.x_ = x;
				temp.y_ = y;
				findings.push_back(temp);
			}
		}
	}
}

/* compare StateNodes */
bool comp_state(StateNode* node1, StateNode* node2)
{
	if (node1->f_ != node2->f_)
	{
		return node1->f_ > node2->f_;
	}
	if (node1->man_move_distance_ != node2->man_move_distance_)
	{
		return node1->man_move_distance_ > node2->man_move_distance_;
	}
	return node1->moved_box_h_ > node2->moved_box_h_;
}

int SokobanSolver::solve(char* &step)
{
	int i, j, step_num;
	vector<char> reverse_step;
	StateNode *current, *last;
	ClosedNode *temp;
	Grid man;
	if (size_error_ || map_error_)
	{
		return MAP_ERROR;
	}
	if (!solvable_)
	{
		return CANT_FIND_SOLUTION;
	}
	if (no_solution_)
	{
		return NO_SOLUTION;
	}
	find_ = false;
	/* IDA-star search */
	for (depth_limit_ = start_state_.h_; depth_limit_ <= DEEPMAX; depth_limit_ += 2)
	{
		current = make_start_node();
		map_load_box(current);
		if (check_dup(current))		//no solution
		{
			return NO_SOLUTION;
		}
		map_clear_box(current);
		open_list_.push(current);
		while(!open_list_.empty())         
		{
			current = open_list_.top();		//get current StateNode from open list
			open_list_.pop();
			/* update ancestor list */
			while(!ancestor_list_.empty())
			{
				last = ancestor_list_.top();
				if(last->g_ >= current->g_)
				{
					ancestor_list_.pop();
					delete last;
				}
				else
				{
					break;
				}
			}
			if(current->h_ == 0)		//find a solution
			{
				find_ = true;
				break;
			}
			/* get maps */
			map_load_box(current);
			get_reachability_map(man_reachable_map_, all_box_maze_, width_, height_, current->man_);
			/* try to expand child StateNodes by moving a box */
			for(i = 0; i < box_num_; i++)
	    	{	
				j = (i + current->moved_box_index_) % box_num_;	//shift box index to process last moved box first
				if (all_box_maze_[current->box_[j].x_ - 1][current->box_[j].y_] && man_reachable_map_[current->box_[j].x_ + 1][current->box_[j].y_] >= 0 && box_pos_index_map_[current->box_[j].x_ - 1][current->box_[j].y_] != -1)
				{
					try_expand(current, j, LEFT_MOVE_BOX);
				}
				if (all_box_maze_[current->box_[j].x_ + 1][current->box_[j].y_] && man_reachable_map_[current->box_[j].x_ - 1][current->box_[j].y_] >= 0 && box_pos_index_map_[current->box_[j].x_ + 1][current->box_[j].y_] != -1)
				{
					try_expand(current, j, RIGHT_MOVE_BOX);
				}
				if (all_box_maze_[current->box_[j].x_][current->box_[j].y_ - 1] && man_reachable_map_[current->box_[j].x_][current->box_[j].y_ + 1] >= 0 && box_pos_index_map_[current->box_[j].x_][current->box_[j].y_ - 1] != -1)
				{
					try_expand(current, j, UP_MOVE_BOX);
				}
				if (all_box_maze_[current->box_[j].x_][current->box_[j].y_ + 1] && man_reachable_map_[current->box_[j].x_][current->box_[j].y_ - 1] >= 0 && box_pos_index_map_[current->box_[j].x_][current->box_[j].y_ + 1] != -1)
				{
					try_expand(current, j, DOWN_MOVE_BOX);
				}
			}
			map_clear_box(current);

			/************************************************************************************************
			check current StateNode dead-locked(all child StateNodes dead-locked or duplicated) or not
			if dead-locked, mark it in closed list, and check ancestor StateNodes dead-locked or not
			************************************************************************************************/
			if(current->alive_child_num_ == 0)
			{
				current->closed_node_->dead_locked_ = true;
				delete current;
				while(!ancestor_list_.empty())
				{
					last = ancestor_list_.top();
					last->alive_child_num_--;
					if(last->alive_child_num_ == 0)
					{
						last->closed_node_->dead_locked_ = true;
						ancestor_list_.pop();
						delete last;
					}
					else
					{
						break;
					}
				}
			}
			else
			{
				ancestor_list_.push(current); 
				/* put expanded child StateNodes into open list */
				sort_heap(expanded_list_.begin(), expanded_list_.end(), comp_state);
				for (i = 0; i < expanded_list_.size(); i++)
				{
					open_list_.push(expanded_list_[i]);
				}
				expanded_list_.clear();
			}
		}
		if(find_)
		{
			map_load_box(current);
			/* get solution steps by back tracking ancestor list */
			while(!ancestor_list_.empty())
			{
				last = ancestor_list_.top();
				ancestor_list_.pop();
				map_inv_move_box(current);
				man = current->man_;
				switch(current->direction_)
				{
				case LEFT_MOVE_BOX:
					for (i = 0; i < current->box_move_distance_; i++)
					{
						reverse_step.push_back(LEFT_MOVE_BOX);
					}
					man.x_ += current->box_move_distance_;
					break;
				case RIGHT_MOVE_BOX:
					for (i = 0; i < current->box_move_distance_; i++)
					{
						reverse_step.push_back(RIGHT_MOVE_BOX);
					}
					man.x_ -= current->box_move_distance_;
					break;
				case UP_MOVE_BOX:
					for (i = 0; i < current->box_move_distance_; i++)
					{
						reverse_step.push_back(UP_MOVE_BOX);
					}
					man.y_ += current->box_move_distance_;
					break;
				case DOWN_MOVE_BOX:
					for (i = 0; i < current->box_move_distance_; i++)
					{
						reverse_step.push_back(DOWN_MOVE_BOX);
					}
					man.y_ -= current->box_move_distance_;
					break; 
				}
				path_solver_->set(last->man_, man);
				step_num = path_solver_->get(step);
				for (i = step_num - 1; i >= 0; i--)
				{
					reverse_step.push_back(step[i]);
				}
				if (step_num > 0)
				{
					free(step);
				}
				current = last;
			}
			step_num = int(reverse_step.size());
			step = new char[step_num + 1];
			i = 0;
			while(!reverse_step.empty())
			{
				step[i++] = reverse_step.back();
				reverse_step.pop_back();
			}
			step[step_num] = 0;
			return step_num;
		}
		//set StateNodes in closed list to unvisited
		for (i = 0; i < hash_list_len_; i++)
		{
			temp = closed_list_[i];
			while(temp != NULL)
			{
				temp->visited_ = false;
				temp = temp->next_;
			}
		}
	}
	return CANT_FIND_SOLUTION;	//no solution under limited depth search
}

void SokobanSolver::try_expand(StateNode* node, int moved_box_index, char direction)
{
	bool expand = false;
	char new_dead_box_check;
	StateNode* child_node;
	int distance = tunnel_solver_->get_pass_tunnel_distance(node, moved_box_index, direction);
	if (distance == -1)
	{
		return;
	}
	child_node = make_child_node(node, moved_box_index, distance, direction);
	map_move_box(child_node);
	if(!check_dup(child_node))
	{
		new_dead_box_check = locked_box_solver_->check_locked_box(child_node);
		if (new_dead_box_check == NEW_LOCKED_BOX_OFF_GOAL)
		{
			add_dead_locked_boxes_pos(child_node);
		}
		else
		{
			LockedGoalsNode* locked_goals_node = locked_box_solver_->get_locked_goals_node(child_node);
			calc_h(child_node, locked_goals_node);
			if (child_node->h_ > DEEPMAX || goal_tunnel_solver_->check_goal_tunnel_deadlock())
			{
				add_dead_locked_boxes_pos(child_node);
			}
			else if ((new_dead_box_check == NEW_LOCKED_BOX_ON_GOAL && !locked_box_solver_->check_box_accessiblity(child_node, locked_goals_node))
					|| trap_solver_->check_trap_deadlock(child_node)
					|| tunnel_solver_->check_tunnel_deadlock(child_node))
			{
				child_node->closed_node_->dead_locked_ = true;
			}
			else
			{
				node->alive_child_num_++;
				if (child_node->f_ <= depth_limit_)
				{
					expand = true;
					expanded_list_.push_back(child_node);
					push_heap(expanded_list_.begin(), expanded_list_.end(), comp_state);
				}
			}
		}
	}
	map_inv_move_box(child_node);
	if (!expand)
	{
		delete child_node;
	}
}

StateNode* SokobanSolver::make_start_node()
{
	StateNode* node = new StateNode;
	node->man_ = start_state_.man_;
	node->box_ = start_state_.box_;
	node->moved_box_index_ = 0;
	node->g_ = 0;
	node->h_ = start_state_.h_;
	node->f_ = start_state_.f_;
	node->locked_goals_index_ = 0;
	node->boxes_pos_index_ = start_state_.boxes_pos_index_;
	node->alive_child_num_ = 0;
	return node;
}

StateNode* SokobanSolver::make_child_node(StateNode* node, int moved_box_index, int distance, char direction)
{
	int next;
	StateNode* child_node = new StateNode;
	child_node->man_ = node->box_[moved_box_index];
	for (int i = 0; i < box_num_; i++)
	{
		child_node->box_.push_back(node->box_[i]);
	}
	switch(direction)
	{
	case LEFT_MOVE_BOX:
		child_node->man_move_distance_ = man_reachable_map_[node->box_[moved_box_index].x_ + 1][node->box_[moved_box_index].y_];
		child_node->man_.x_ -= distance - 1;
		child_node->box_[moved_box_index].x_ -= distance;
		/* boxes' indexes might change if move box left */
		next = moved_box_index - 1;
		while (next >= 0 && (child_node->box_[moved_box_index].x_ < child_node->box_[next].x_ || (child_node->box_[moved_box_index].x_ == child_node->box_[next].x_ && child_node->box_[moved_box_index].y_ < child_node->box_[next].y_)))
		{
			swap(child_node->box_[moved_box_index], child_node->box_[next]);
			moved_box_index--;
			next--;
		}
		break;
	case RIGHT_MOVE_BOX:
		child_node->man_move_distance_ = man_reachable_map_[node->box_[moved_box_index].x_ - 1][node->box_[moved_box_index].y_];
		child_node->man_.x_ += distance - 1;
		child_node->box_[moved_box_index].x_ += distance;
		/* boxes' indexes might change if move box right */
		next = moved_box_index + 1;
		while (next < box_num_ && (child_node->box_[moved_box_index].x_ > child_node->box_[next].x_ || (child_node->box_[moved_box_index].x_ == child_node->box_[next].x_ && child_node->box_[moved_box_index].y_ > child_node->box_[next].y_)))
		{
			swap(child_node->box_[moved_box_index], child_node->box_[next]);
			moved_box_index++;
			next++;
		}
		break;
	case UP_MOVE_BOX:
		child_node->man_move_distance_ = man_reachable_map_[node->box_[moved_box_index].x_][node->box_[moved_box_index].y_ + 1];
		child_node->man_.y_ -= distance - 1;
		child_node->box_[moved_box_index].y_ -= distance;
		break;
	case DOWN_MOVE_BOX:
		child_node->man_move_distance_ = man_reachable_map_[node->box_[moved_box_index].x_][node->box_[moved_box_index].y_ - 1];
		child_node->man_.y_ += distance - 1;
		child_node->box_[moved_box_index].y_ += distance;
	}
	child_node->moved_box_index_ = moved_box_index;
	child_node->direction_ = direction;
	child_node->box_move_distance_ = distance;
	child_node->g_ = node->g_ + distance;
	child_node->locked_goals_index_ = node->locked_goals_index_;
	child_node->alive_child_num_ = 0;
	calc_boxes_pos_index(child_node);
	return child_node;
}

bool SokobanSolver::check_dup(StateNode* node)
{
	__int64 boxes_pos_index = node->boxes_pos_index_;
	int boxes_pos_hash = boxes_pos_index % hash_list_len_;
	/* check boxes' position dead-locked or not */
	BoxesPosNode* temp1 = dead_locked_boxes_pos_list_[boxes_pos_hash];
	while (temp1)
	{
		if (temp1->boxes_pos_index_ == boxes_pos_index)
		{
			break;
		}
		temp1 = temp1->next_;
	}
	if (temp1)
	{
		return true;
	}
	/* check equal StateNode in closed list */
	ClosedNode*& start = closed_list_[boxes_pos_hash];
	ClosedNode* temp2;
	temp2 = start;	
	while (temp2)
	{
		if (temp2->boxes_pos_index_ == boxes_pos_index)
		{
			path_solver_->set(node->man_, temp2->man_);
			if (path_solver_->find())		//equal with a StateNode in closed list(two StateNodes are equal: same boxes' position and man's position connected)
			{
				//if not dead-locked, and with smaller g value or first visited with same g value, then the StateNode is not duplicated
				if (!temp2->dead_locked_ && (node->g_ < temp2->g_ || (node->g_ == temp2->g_ && !temp2->visited_)))
				{
					temp2->g_ = node->g_;
					temp2->visited_ = true;
					node->closed_node_ = temp2;
					return false;
				}
				return true;
			}
		}
		temp2 = temp2->next_;
	}
	/* add the StateNode into closed list if there's no equal StateNode */
	temp2 = new ClosedNode(node);
	temp2->next_ = start;
	start = temp2;
	node->closed_node_ = start;
	return false;
}

void SokobanSolver::add_dead_locked_boxes_pos(StateNode* node)
{
	BoxesPosNode*& start = dead_locked_boxes_pos_list_[node->boxes_pos_index_ % hash_list_len_];
	BoxesPosNode* temp = new BoxesPosNode(node);
	temp->next_ = start;
	start = temp;
}

void SokobanSolver::calc_boxes_pos_index(StateNode* node)
{
	int* a = (int*)malloc(box_num_ * sizeof(int));
	for (int i = 0; i < box_num_; i++)
	{
		a[i] = box_pos_index_map_[node->box_[i].x_][node->box_[i].y_];
	}
	node->boxes_pos_index_ = comb_index_solver_->get_comb_index(a);
	free(a);
}

void SokobanSolver::calc_start_h()
{
	int i, j;
	for (i = 0; i < box_num_; i++)
	{
		for (j = 0; j < goal_num_; j++)
		{
			min_assignment_solver_->cost_[i][j] = goals_h_map_[j][start_state_.box_[i].x_][start_state_.box_[i].y_];
		}
	}
	min_assignment_solver_->num_ = box_num_;
	start_state_.h_ = min_assignment_solver_->get_min_cost();
	start_state_.f_ = start_state_.g_ + start_state_.h_;
}

void SokobanSolver::calc_h(StateNode* node, LockedGoalsNode* locked_goals_node)
{
	int i, j;
	unsigned char*** goals_h_map;
	/* get unlocked_num, unlocked_box_index, unlocked_goal_index */
	int unlocked_num = box_num_ - locked_goals_node->locked_num_;
	int* unlocked_box_index = (int*)malloc(unlocked_num * sizeof(int));
	int* unlocked_goal_index = (int*)malloc(unlocked_num * sizeof(int));
	int unlocked_count = 0;
	bool** locked_goal_map = locked_box_solver_->locked_goal_map_;
	goals_h_map = locked_goals_node->goals_h_map_;
	for (i = 0; unlocked_count < unlocked_num; i++)
	{
		if (!locked_goal_map[node->box_[i].x_][node->box_[i].y_])
		{
			unlocked_box_index[unlocked_count++] = i;
		}
	}
	unlocked_count = 0;
	for (i = 0; unlocked_count < unlocked_num; i++)
	{
		if (!locked_goal_map[goals_[i].x_][goals_[i].y_])
		{
			unlocked_goal_index[unlocked_count++] = i;
		}
	}
	/* set min_assignment_solver_ */
	for (i = 0; i < unlocked_num; i++)
	{
		for (j = 0; j < unlocked_num; j++)
		{
			min_assignment_solver_->cost_[i][j] = goals_h_map[unlocked_goal_index[j]][node->box_[unlocked_box_index[i]].x_][node->box_[unlocked_box_index[i]].y_];
		}
	}
	min_assignment_solver_->num_ = unlocked_num;
	/* set h, f values of node */
	node->h_ = min_assignment_solver_->get_min_cost();
	node->f_ = node->g_ + node->h_;
	/* set moved_box_h_ of node */
	int moved_box_index = node->moved_box_index_;
	int moved_box_match_goal_index;
	if (node->locked_goals_index_ != 0)
	{
		bool moved_box_locked = true;
		for (i = 0; i < unlocked_num; i++)
		{
			if (unlocked_box_index[i] == moved_box_index)
			{
				moved_box_locked = false;
				break;
			}
			else if (unlocked_box_index[i] > moved_box_index)
			{
				break;
			}
		}
		if (moved_box_locked)
		{
			node->moved_box_h_ = 0;
		}
		else
		{
			moved_box_match_goal_index = unlocked_goal_index[min_assignment_solver_->bigraph_->left_link_[i]];
			node->moved_box_h_ = goals_h_map[moved_box_match_goal_index][node->box_[moved_box_index].x_][node->box_[moved_box_index].y_];
		}
	}
	else
	{
		moved_box_match_goal_index = min_assignment_solver_->bigraph_->left_link_[moved_box_index];
		node->moved_box_h_ = goals_h_map[moved_box_match_goal_index][node->box_[moved_box_index].x_][node->box_[moved_box_index].y_];
	}
	free(unlocked_box_index);
	free(unlocked_goal_index);
}

void SokobanSolver::map_load_box(StateNode* node)
{
	for(int i = 0; i < box_num_; i++)
	{
		all_box_maze_[node->box_[i].x_][node->box_[i].y_] = false;
		box_map_[node->box_[i].x_][node->box_[i].y_] = true;
	}
}

void SokobanSolver::map_clear_box(StateNode* node)
{
	for(int i = 0; i < box_num_; i++)
	{
		all_box_maze_[node->box_[i].x_][node->box_[i].y_] = true;
		box_map_[node->box_[i].x_][node->box_[i].y_] = false;
	}
}

void SokobanSolver::map_move_box(StateNode* node)
{
	int i, t;
	int x = node->box_[node->moved_box_index_].x_;
	int y = node->box_[node->moved_box_index_].y_;
	all_box_maze_[x][y] = false;
	box_map_[x][y] = true;
	switch(node->direction_)
	{
	case LEFT_MOVE_BOX:
		x += node->box_move_distance_;
		break;
	case RIGHT_MOVE_BOX:
		x -= node->box_move_distance_;
		break;
	case UP_MOVE_BOX:
		y += node->box_move_distance_;
		break;
	case DOWN_MOVE_BOX:
		y -= node->box_move_distance_;
		break;
	}
	all_box_maze_[x][y] = true;
	box_map_[x][y] = false;
	t = node->locked_goals_index_;
	for(i = 0; i < goal_num_; i++)
	{
		if (t & 1)
		{
			locked_box_solver_->locked_goal_map_[goals_[i].x_][goals_[i].y_] = true;
		}
		t = t >> 1;
	}
}

void SokobanSolver::map_inv_move_box(StateNode* node)
{
	int i, t;
	int x = node->box_[node->moved_box_index_].x_;
	int y = node->box_[node->moved_box_index_].y_;
	all_box_maze_[x][y] = true;
	box_map_[x][y] = false;
	switch(node->direction_)
	{
	case LEFT_MOVE_BOX:
		x += node->box_move_distance_;
		break;
	case RIGHT_MOVE_BOX:
		x -= node->box_move_distance_;
		break;
	case UP_MOVE_BOX:
		y += node->box_move_distance_;
		break;
	case DOWN_MOVE_BOX:
		y -= node->box_move_distance_;
		break;
	}
	all_box_maze_[x][y] = false;
	box_map_[x][y] = true;
	t = node->locked_goals_index_;
	for(i = 0; i < goal_num_; i++)
	{
		if (t & 1)
		{
			locked_box_solver_->locked_goal_map_[goals_[i].x_][goals_[i].y_] = false;
		}
		t = t >> 1;
	}
}