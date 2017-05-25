#define HASH_LIST_LENGTH_MAX	1000

#include "LockedBox.h"
#include "Sokoban.h"
#include "Grid.h"
#include "SmartMap.h"

LockedBoxSolver::LockedBoxSolver(SokobanSolver* sokoban_solver)
{
	sokoban_solver_ = sokoban_solver;
	int width = sokoban_solver_->width_;
	int height = sokoban_solver_->height_;
	int box_num = sokoban_solver_->box_num_;
	int goal_num = sokoban_solver_->goal_num_;
	int i;
	locked_goal_map_ = (bool**)malloc(width * sizeof(bool*));
	locked_goal_map_[0] = (bool*)malloc(width * height);
	locked_box_maze_ = (bool**)malloc(width * sizeof(bool*));
	locked_box_maze_[0] = (bool*)malloc(width * height);
	memset(locked_goal_map_[0], 0, width * height);
	memcpy(locked_box_maze_[0], sokoban_solver_->no_box_maze_[0], width * height);		//initialize locked_box_maze_ with no_box_maze_
	for (i = 1; i < width; i++)
	{
		locked_goal_map_[i] = locked_goal_map_[i - 1] + height;
		locked_box_maze_[i] = locked_box_maze_[i - 1] + height;
	}
	/* set hash_list_len_, and allocate memory for locked_goals_state_list_ */
	int locked_goals_state_num = 1 << (sokoban_solver_->goal_num_);
	hash_list_len_ = (locked_goals_state_num < HASH_LIST_LENGTH_MAX) ? locked_goals_state_num : HASH_LIST_LENGTH_MAX;
	locked_goals_state_list_ = (LockedGoalsNode**)malloc(hash_list_len_ * sizeof(LockedGoalsNode*));
	memset(locked_goals_state_list_, 0, hash_list_len_ * sizeof(LockedGoalsNode*));
}

LockedBoxSolver::~LockedBoxSolver()
{
	free(locked_goal_map_[0]);
	free(locked_box_maze_[0]);
	free(locked_goal_map_);
	free(locked_box_maze_);
	/* release memory of locked_goals_state_list_ */
	int i, j, t;
	int goal_num = sokoban_solver_->goal_num_;
	LockedGoalsNode *temp, *next;
	for (i = 0; i < hash_list_len_; i++)
	{
		temp = locked_goals_state_list_[i];
		while (temp)
		{
			free(temp->connected_area_map_[0]);
			free(temp->connected_area_map_);
			t = temp->locked_goals_index_;
			for (j = 0; j < goal_num; j++)
			{
				if (!(t & 1))
				{
					free(temp->goals_h_map_[j][0]);
					free(temp->goals_h_map_[j]);
				}
				t = t >> 1;
			}
			free(temp->goals_h_map_);
			next = temp->next_;
			free(temp);
			temp = next;
		}
	}
	free(locked_goals_state_list_);
}

LockedGoalsNode* LockedBoxSolver::get_locked_goals_node(StateNode* node)
{
	int i, j, t;
	int locked_goals_index = node->locked_goals_index_;
	int locked_goals_hash = locked_goals_index % hash_list_len_;
	LockedGoalsNode*& start = locked_goals_state_list_[locked_goals_hash];
	LockedGoalsNode* temp;
	temp = start;
	while (temp)
	{
		if (temp->locked_goals_index_ == locked_goals_index)
		{
			break;
		}
		temp = temp->next_;
	}
	if (!temp)
	{
		temp = new LockedGoalsNode(node);	//create a LockedGoalsNode
		temp->next_ = start;
		start = temp;
		int width = sokoban_solver_->width_;
		int height = sokoban_solver_->height_;
		int goal_num = sokoban_solver_->goal_num_;
		vector<Grid> goals = sokoban_solver_->goals_;
		/* set locked_box_maze_ */
		t = locked_goals_index;
		for (i = 0; i < goal_num; i++)
		{
			if (t & 1)
			{
				locked_box_maze_[goals[i].x_][goals[i].y_] = false;
			}
			t = t >> 1;
		}
		/* set connected_area_map_ */
		temp->connected_area_map_ = (char**)malloc(width * sizeof(char*));
		char** connected_area_map = temp->connected_area_map_;
		connected_area_map[0] = (char*)malloc(width * height);
		for (i = 1; i < width; i++)
		{
			connected_area_map[i] = connected_area_map[i - 1] + height;
		}
		get_connectivity_map(connected_area_map, locked_box_maze_, width, height);
		/* set goals_h_map_ */
		temp->goals_h_map_ = (unsigned char***)malloc(goal_num * sizeof(unsigned char**));
		unsigned char*** goals_h_map = temp->goals_h_map_;
		t = locked_goals_index;
		for (i = 0; i < goal_num; i++)
		{
			/* allocate memory only for unlocked goals' h-map */
			if (!(t & 1))
			{
				goals_h_map[i] = (unsigned char**)malloc(width * sizeof(unsigned char*));
				goals_h_map[i][0] = (unsigned char*)malloc(width * height);
				for (j = 1; j < width; j++)
				{
					goals_h_map[i][j] = goals_h_map[i][j - 1] + height;
				}
				get_goal_h_map(goals_h_map[i], locked_box_maze_, width, height, goals[i]);
			}
			t = t >> 1;
		}
		/* restore locked_box_maze_ */
		t = locked_goals_index;
		for (i = 0; i < goal_num; i++)
		{
			if (t & 1)
			{
				locked_box_maze_[goals[i].x_][goals[i].y_] = true;
			}
			t = t >> 1;
		}
	}
	return temp;
}

bool LockedBoxSolver::check_box_accessiblity(StateNode* node, LockedGoalsNode* locked_goals_node)
{
	int i, x, y;
	int man_x = node->man_.x_;
	int man_y = node->man_.y_;
	int box_num = sokoban_solver_->box_num_;
	int** goal_index_map = sokoban_solver_->goal_index_map_;
	char** connected_area_map = locked_goals_node->connected_area_map_;
	for (i = 0; i < box_num; i++)
	{
		x = node->box_[i].x_;
		y = node->box_[i].y_;
		if (connected_area_map[x][y] != connected_area_map[man_x][man_y] && goal_index_map[x][y] == -1)
		{
			return false;
		}
	}
	return true;
}

char LockedBoxSolver::check_locked_box(StateNode* node)
{
	int i;
	int x = node->box_[node->moved_box_index_].x_;
	int y = node->box_[node->moved_box_index_].y_;
	char a, b, c, d, result;
	bool premise_check, square_check;
	bool** all_box_maze = sokoban_solver_->all_box_maze_;
	int** goal_index_map = sokoban_solver_->goal_index_map_;
	/* premise of new locked box: last moved box have both horizontal and vertical neighbor box or wall */
	switch (node->direction_)
	{
	case LEFT_MOVE_BOX:
		premise_check = !all_box_maze[x - 1][y] && (!all_box_maze[x][y - 1] || !all_box_maze[x][y + 1]);
		break;
	case RIGHT_MOVE_BOX:
		premise_check = !all_box_maze[x + 1][y] && (!all_box_maze[x][y - 1] || !all_box_maze[x][y + 1]);
		break;
	case UP_MOVE_BOX:
		premise_check = !all_box_maze[x][y - 1] && (!all_box_maze[x - 1][y] || !all_box_maze[x + 1][y]);
		break;
	case DOWN_MOVE_BOX:
		premise_check = !all_box_maze[x][y + 1] && (!all_box_maze[x - 1][y] || !all_box_maze[x + 1][y]);
		break;
	}
	if (!premise_check)
	{
		return NO_NEW_LOCKED_BOX;
	}
	/* search for 2 x 2 locked square and its connected zig-zag locked traces */
	result = NO_NEW_LOCKED_BOX;
	square_check = false;
	switch (node->direction_)
	{
	case LEFT_MOVE_BOX:
		if (!all_box_maze[x][y - 1] && !all_box_maze[x - 1][y - 1])
		{
			square_check = true;
			if (check_locked_square(x - 1, y - 1))
			{
				result = NEW_LOCKED_BOX_ON_GOAL;
			}
			else
			{
				result = NEW_LOCKED_BOX_OFF_GOAL;
			}
		}
		if (result != NEW_LOCKED_BOX_OFF_GOAL && !all_box_maze[x][y + 1] && !all_box_maze[x - 1][y + 1])
		{
			square_check = true;
			if (check_locked_square(x - 1, y))
			{
				result = NEW_LOCKED_BOX_ON_GOAL;
			}
			else
			{
				result = NEW_LOCKED_BOX_OFF_GOAL;
			}
		}
		break;
	case RIGHT_MOVE_BOX:
		if (!all_box_maze[x][y - 1] && !all_box_maze[x + 1][y - 1])
		{
			square_check = true;
			if (check_locked_square(x, y - 1))
			{
				result = NEW_LOCKED_BOX_ON_GOAL;
			}
			else
			{
				result = NEW_LOCKED_BOX_OFF_GOAL;
			}

		}
		if (result != NEW_LOCKED_BOX_OFF_GOAL && !all_box_maze[x][y + 1] && !all_box_maze[x + 1][y + 1])
		{
			square_check = true;
			if (check_locked_square(x, y))
			{
				result = NEW_LOCKED_BOX_ON_GOAL;
			}
			else
			{
				result = NEW_LOCKED_BOX_OFF_GOAL;
			}
		}
		break;
	case UP_MOVE_BOX:
		if (!all_box_maze[x - 1][y] && !all_box_maze[x - 1][y - 1])
		{
			square_check = true;
			if (check_locked_square(x - 1, y - 1))
			{
				result = NEW_LOCKED_BOX_ON_GOAL;
			}
			else
			{
				result = NEW_LOCKED_BOX_OFF_GOAL;
			}
		}
		if (result != NEW_LOCKED_BOX_OFF_GOAL && !all_box_maze[x + 1][y] && !all_box_maze[x + 1][y - 1])
		{
			square_check = true;
			if (check_locked_square(x, y - 1))
			{
				result = NEW_LOCKED_BOX_ON_GOAL;
			}
			else
			{
				result = NEW_LOCKED_BOX_OFF_GOAL;
			}
		}
		break;
	case DOWN_MOVE_BOX:
		if (!all_box_maze[x - 1][y] && !all_box_maze[x - 1][y + 1])
		{
			square_check = true;
			if (check_locked_square(x - 1, y))
			{
				result = NEW_LOCKED_BOX_ON_GOAL;
			}
			else
			{
				result = NEW_LOCKED_BOX_OFF_GOAL;
			}
		}
		if (result != NEW_LOCKED_BOX_OFF_GOAL && !all_box_maze[x + 1][y] && !all_box_maze[x + 1][y + 1])
		{
			square_check = true;
			if (check_locked_square(x, y))
			{
				result = NEW_LOCKED_BOX_ON_GOAL;
			}
			else
			{
				result = NEW_LOCKED_BOX_OFF_GOAL;
			}
		}
		break;
	}
	/* if 2 x 2 locked square not found£¬search for opposite directions of zig-zag locked traces */
	if (!square_check)
	{
		switch (node->direction_)
		{
		case LEFT_MOVE_BOX:
			a = find_locked_trace(x, y, LEFT, UP);
			b = find_locked_trace(x, y, DOWN, RIGHT);
			c = find_locked_trace(x, y, LEFT, DOWN);
			d = find_locked_trace(x, y, UP, RIGHT);
			if ((a != NO_NEW_LOCKED_BOX && b != NO_NEW_LOCKED_BOX) || (c != NO_NEW_LOCKED_BOX && d != NO_NEW_LOCKED_BOX))
			{
				if (goal_index_map[x][y] == -1 || a == NEW_LOCKED_BOX_OFF_GOAL || b == NEW_LOCKED_BOX_OFF_GOAL || c == NEW_LOCKED_BOX_OFF_GOAL || d == NEW_LOCKED_BOX_OFF_GOAL)
				{
					result = NEW_LOCKED_BOX_OFF_GOAL;
				}
				else
				{
					new_locked_goal_indexes.push_back(goal_index_map[x][y]);
					locked_goal_map_[x][y] = true;
					result = NEW_LOCKED_BOX_ON_GOAL;
				}
			}
			break;
		case RIGHT_MOVE_BOX:
			a = find_locked_trace(x, y, RIGHT, UP);
			b = find_locked_trace(x, y, DOWN, LEFT);
			c = find_locked_trace(x, y, RIGHT, DOWN);
			d = find_locked_trace(x, y, UP, LEFT);
			if ((a != NO_NEW_LOCKED_BOX && b != NO_NEW_LOCKED_BOX) || (c != NO_NEW_LOCKED_BOX && d != NO_NEW_LOCKED_BOX))
			{
				if (goal_index_map[x][y] == -1 || a == NEW_LOCKED_BOX_OFF_GOAL || b == NEW_LOCKED_BOX_OFF_GOAL || c == NEW_LOCKED_BOX_OFF_GOAL || d == NEW_LOCKED_BOX_OFF_GOAL)
				{
					result = NEW_LOCKED_BOX_OFF_GOAL;
				}
				else
				{
					new_locked_goal_indexes.push_back(goal_index_map[x][y]);
					locked_goal_map_[x][y] = true;
					result = NEW_LOCKED_BOX_ON_GOAL;
				}
			}
			break;
		case UP_MOVE_BOX:
			a = find_locked_trace(x, y, UP, LEFT);
			b = find_locked_trace(x, y, RIGHT, DOWN);
			c = find_locked_trace(x, y, UP, RIGHT);
			d = find_locked_trace(x, y, LEFT, DOWN);
			if ((a != NO_NEW_LOCKED_BOX && b != NO_NEW_LOCKED_BOX) || (c != NO_NEW_LOCKED_BOX && d != NO_NEW_LOCKED_BOX))
			{
				if (goal_index_map[x][y] == -1 || a == NEW_LOCKED_BOX_OFF_GOAL || b == NEW_LOCKED_BOX_OFF_GOAL || c == NEW_LOCKED_BOX_OFF_GOAL || d == NEW_LOCKED_BOX_OFF_GOAL)
				{
					result = NEW_LOCKED_BOX_OFF_GOAL;
				}
				else
				{
					new_locked_goal_indexes.push_back(goal_index_map[x][y]);
					locked_goal_map_[x][y] = true;
					result = NEW_LOCKED_BOX_ON_GOAL;
				}
			}
			break;
		case DOWN_MOVE_BOX:
			a = find_locked_trace(x, y, DOWN, LEFT);
			b = find_locked_trace(x, y, RIGHT, UP);
			c = find_locked_trace(x, y, DOWN, RIGHT);
			d = find_locked_trace(x, y, LEFT, UP);
			if ((a != NO_NEW_LOCKED_BOX && b != NO_NEW_LOCKED_BOX) || (c != NO_NEW_LOCKED_BOX && d != NO_NEW_LOCKED_BOX))
			{
				if (goal_index_map[x][y] == -1 || a == NEW_LOCKED_BOX_OFF_GOAL || b == NEW_LOCKED_BOX_OFF_GOAL || c == NEW_LOCKED_BOX_OFF_GOAL || d == NEW_LOCKED_BOX_OFF_GOAL)
				{
					result = NEW_LOCKED_BOX_OFF_GOAL;
				}
				else
				{
					new_locked_goal_indexes.push_back(goal_index_map[x][y]);
					locked_goal_map_[x][y] = true;
					result = NEW_LOCKED_BOX_ON_GOAL;
				}
			}
			break;
		}
	}
	if (result == NEW_LOCKED_BOX_ON_GOAL)
	{
		while (!new_locked_goal_indexes.empty())
		{
			node->locked_goals_index_ += 1 << new_locked_goal_indexes.back();
			new_locked_goal_indexes.pop_back();
		}
	}
	else
	{
		while (!new_locked_goal_indexes.empty())
		{
			i = new_locked_goal_indexes.back();
			new_locked_goal_indexes.pop_back();
			locked_goal_map_[sokoban_solver_->goals_[i].x_][sokoban_solver_->goals_[i].y_] = false;
		}
	}
	return result;
}

bool LockedBoxSolver::check_locked_square(int x, int y)
{
	char c;
	bool** box_map = sokoban_solver_->box_map_;
	int** goal_index_map = sokoban_solver_->goal_index_map_;
	if (box_map[x][y] && !locked_goal_map_[x][y])
	{
		if (goal_index_map[x][y] == -1)
		{
			return false;
		}
		new_locked_goal_indexes.push_back(goal_index_map[x][y]);
		locked_goal_map_[x][y] = true;
		c = find_locked_trace(x, y, LEFT, UP);
		if (c == NEW_LOCKED_BOX_OFF_GOAL)
		{
			return false;
		}
		if (c == NO_NEW_LOCKED_BOX && find_locked_trace(x, y, UP, LEFT) == NEW_LOCKED_BOX_OFF_GOAL)
		{
			return false;
		}
	}
	if (box_map[x][y + 1] && !locked_goal_map_[x][y + 1])
	{
		if (goal_index_map[x][y + 1] == -1)
		{
			return false;
		}
		new_locked_goal_indexes.push_back(goal_index_map[x][y + 1]);
		locked_goal_map_[x][y + 1] = true;
		c = find_locked_trace(x, y + 1, LEFT, DOWN);
		if (c == NEW_LOCKED_BOX_OFF_GOAL)
		{
			return false;
		}
		if (c == NO_NEW_LOCKED_BOX && find_locked_trace(x, y + 1, DOWN, LEFT) == NEW_LOCKED_BOX_OFF_GOAL)
		{
			return false;
		}
	}
	if (box_map[x + 1][y] && !locked_goal_map_[x + 1][y])
	{
		if (goal_index_map[x + 1][y] == -1)
		{
			return false;
		}
		new_locked_goal_indexes.push_back(goal_index_map[x + 1][y]);
		locked_goal_map_[x + 1][y] = true;
		c = find_locked_trace(x + 1, y, RIGHT, UP);
		if (c == NEW_LOCKED_BOX_OFF_GOAL)
		{
			return false;
		}
		if (c == NO_NEW_LOCKED_BOX && find_locked_trace(x + 1, y, UP, RIGHT) == NEW_LOCKED_BOX_OFF_GOAL)
		{
			return false;
		}
	}
	if (box_map[x + 1][y + 1] && !locked_goal_map_[x + 1][y + 1])
	{
		if (goal_index_map[x + 1][y + 1] == -1)
		{
			return false;
		}
		new_locked_goal_indexes.push_back(goal_index_map[x + 1][y + 1]);
		locked_goal_map_[x + 1][y + 1] = true;
		c = find_locked_trace(x + 1, y + 1, RIGHT, DOWN);
		if (c == NEW_LOCKED_BOX_OFF_GOAL)
		{
			return false;
		}
		if (c == NO_NEW_LOCKED_BOX && find_locked_trace(x + 1, y + 1, DOWN, RIGHT) == NEW_LOCKED_BOX_OFF_GOAL)
		{
			return false;
		}
	}
	return true;
}

char LockedBoxSolver::find_locked_trace(int x, int y, char first_direction, char second_direction)
{
	bool odd = true;
	bool off_goal_check = false;
	char direction;
	int i;
	vector<int> locked_box_buffer;
	while (true)
	{
		if (odd)
		{
			direction = first_direction;
		}
		else
		{
			direction = second_direction;
		}
		switch (direction)
		{
		case LEFT:
			x--;
			break;
		case RIGHT:
			x++;
			break;
		case UP:
			y--;
			break;
		case DOWN:
			y++;
			break;
		}
		if (sokoban_solver_->all_box_maze_[x][y])
		{
			return NO_NEW_LOCKED_BOX;
		}
		if (locked_goal_map_[x][y] || !sokoban_solver_->no_box_maze_[x][y])
		{
			if (off_goal_check)
			{
				return NEW_LOCKED_BOX_OFF_GOAL;
			}
			else
			{
				while (!locked_box_buffer.empty())
				{
					i = locked_box_buffer.back();
					locked_box_buffer.pop_back();
					new_locked_goal_indexes.push_back(i);
					locked_goal_map_[sokoban_solver_->goals_[i].x_][sokoban_solver_->goals_[i].y_] = true;
				}
				return NEW_LOCKED_BOX_ON_GOAL;
			}
		}
		if (sokoban_solver_->goal_index_map_[x][y] == -1)
		{
			off_goal_check = true;
		}
		else
		{
			locked_box_buffer.push_back(sokoban_solver_->goal_index_map_[x][y]);
		}
		odd = !odd;
	}
}