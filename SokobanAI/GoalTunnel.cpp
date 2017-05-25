#include "GoalTunnel.h"

GoalTunnelSolver::GoalTunnelSolver(SokobanSolver* sokoban_solver)
{
	sokoban_solver_ = sokoban_solver;
	int x, y;
	int width = sokoban_solver_->width_;
	int height = sokoban_solver_->height_;
	int goal_num;
	int start, length;
	char state;
	bool** no_box_maze = sokoban_solver_->no_box_maze_;
	int** goal_index_map = sokoban_solver_->goal_index_map_;
	GoalTunnel temp_tunnel;
	temp_tunnel.type_ = HORIZONTAL;
	for (y = 1; y < height - 1; y++)
	{
		temp_tunnel.start_.y_ = y;
		state = SEARCH_STATE_0;
		for (x = 0; x < width; x++)
		{
			switch (state)
			{
			case SEARCH_STATE_0:
				if (!no_box_maze[x][y])
				{
					state++;
				}
				break;
			case SEARCH_STATE_1:
				goal_num = 0;
				if (no_box_maze[x][y])
				{
					state++;
					start = x;
				}
				break;
			case SEARCH_STATE_2:
				if (no_box_maze[x][y])
				{
					if ((!no_box_maze[x][y - 1] || (y > 1 && !no_box_maze[x][y - 2])) && (!no_box_maze[x][y + 1] || (y < height - 2 && !no_box_maze[x][y + 2])))
					{
						state++;
					}
					else
					{
						state = SEARCH_STATE_0;
					}
				}
				else
				{
					state = SEARCH_STATE_1;
				}
				break;
			case SEARCH_STATE_3:
				if (!no_box_maze[x][y])
				{
					state = SEARCH_STATE_5;
				}
				else if ((no_box_maze[x][y - 1] && y > 1 && no_box_maze[x][y - 2]) || (no_box_maze[x][y + 1] && y < height - 2 && no_box_maze[x][y + 2]))
				{
					state++;
				}
				break;
			case SEARCH_STATE_4:
				if (no_box_maze[x][y])
				{
					state = SEARCH_STATE_0;
				}
				else
				{
					state++;
				}
				break;
			}
			if (goal_index_map[x][y] >= 0)
			{
				goal_num++;
			}
			if (state == SEARCH_STATE_5)
			{
				if (goal_index_map[start][y] == -1)
				{
					start++;
				}
				else if ((no_box_maze[start][y - 1] && y > 1 && no_box_maze[start][y - 2]) || (no_box_maze[start][y + 1] && y < height - 2 && no_box_maze[start][y + 2]))
				{
					start++;
					goal_num--;
				}
				length = x - start;
				if (goal_index_map[x - 1][y] == -1)
				{
					length--;
				}
				else if ((no_box_maze[x - 1][y - 1] && y > 1 && no_box_maze[x - 1][y - 2]) || (no_box_maze[x - 1][y + 1] && y < height - 2 && no_box_maze[x - 1][y + 2]))
				{
					length--;
					goal_num--;
				}
				if (length > goal_num && goal_num >= 2)
				{
					temp_tunnel.start_.x_ = start;
					temp_tunnel.length_ = length;
					goal_tunnel_set_.push_back(temp_tunnel);
				}
				state = SEARCH_STATE_1;
			}
		}
	}
	temp_tunnel.type_ = VERTICAL;
	for (x = 1; x < width - 1; x++)
	{
		temp_tunnel.start_.x_ = x;
		state = SEARCH_STATE_0;
		for (y = 0; y < height; y++)
		{
			switch (state)
			{
			case SEARCH_STATE_0:
				if (!no_box_maze[x][y])
				{
					state++;
				}
				break;
			case SEARCH_STATE_1:
				goal_num = 0;
				if (no_box_maze[x][y])
				{
					state++;
					start = y;
				}
				break;
			case SEARCH_STATE_2:
				if (no_box_maze[x][y])
				{
					if ((!no_box_maze[x - 1][y] || (x > 1 && !no_box_maze[x - 2][y])) && (!no_box_maze[x + 1][y] || (x < width - 2 && !no_box_maze[x + 2][y])))
					{
						state++;
					}
					else
					{
						state = SEARCH_STATE_0;
					}
				}
				else
				{
					state = SEARCH_STATE_1;
				}
				break;
			case SEARCH_STATE_3:
				if (!no_box_maze[x][y])
				{
					state = SEARCH_STATE_5;
				}
				else if ((no_box_maze[x - 1][y] && x > 1 && no_box_maze[x - 2][y]) || (no_box_maze[x + 1][y] && x < width - 2 && no_box_maze[x + 2][y]))
				{
					state++;
				}
				break;
			case SEARCH_STATE_4:
				if (no_box_maze[x][y])
				{
					state = SEARCH_STATE_0;
				}
				else
				{
					state++;
				}
				break;
			}
			if (goal_index_map[x][y] >= 0)
			{
				goal_num++;
			}
			if (state == SEARCH_STATE_5)
			{
				if (goal_index_map[x][start] == -1)
				{
					start++;
				}
				else if ((no_box_maze[x - 1][start] && x > 1 && no_box_maze[x - 2][start]) || (no_box_maze[x + 1][start] && x < width - 2 && no_box_maze[x + 2][start]))
				{
					start++;
					goal_num--;
				}
				length = y - start;
				if (goal_index_map[x][y - 1] == -1)
				{
					length--;
				}
				else if ((no_box_maze[x - 1][y - 1] && x > 1 && no_box_maze[x - 2][y - 1]) || (no_box_maze[x + 1][y - 1] && x < width - 2 && no_box_maze[x + 2][y - 1]))
				{
					length--;
					goal_num--;
				}
				if (length > goal_num && goal_num >= 2)
				{
					temp_tunnel.start_.y_ = start;
					temp_tunnel.length_ = length;
					goal_tunnel_set_.push_back(temp_tunnel);
				}
				state = SEARCH_STATE_1;
			}
		}
	}
}

bool GoalTunnelSolver::check_goal_tunnel_deadlock()
{
	int i, j, x, y;
	int goal_num, box_num;
	bool block;
	bool** box_map = sokoban_solver_->box_map_;
	int** goal_index_map = sokoban_solver_->goal_index_map_;
	for (i = 0; i < goal_tunnel_set_.size(); i++)
	{
		x = goal_tunnel_set_[i].start_.x_;
		y = goal_tunnel_set_[i].start_.y_;
		goal_num = 0;
		box_num = 0;
		block = false;
		if (goal_tunnel_set_[i].type_ == HORIZONTAL)
		{
			for (j = 0; j < goal_tunnel_set_[i].length_; j++)
			{
				if (box_map[x + j][y])
				{
					if (goal_index_map[x + j][y] >= 0)
					{
						if (!block || (goal_index_map[x + j - 1][y] && box_num == goal_num))
						{
							block = true;
							box_num++;
						}
						else
						{
							block = false;
						}
						goal_num++;
					}
					else
					{
						if (!block)
						{
							box_num++;
						}
						block = !block;
					}
				}
				else
				{
					if (goal_index_map[x + j][y] >= 0)
					{
						goal_num++;
					}
					block = false;
				}
			}
		}
		else
		{
			for (j = 0; j < goal_tunnel_set_[i].length_; j++)
			{
				if (box_map[x][y + j])
				{
					if (goal_index_map[x][y + j] >= 0)
					{
						if (!block || (goal_index_map[x][y + j - 1] && box_num == goal_num))
						{
							block = true;
							box_num++;
						}
						else
						{
							block = false;
						}
						goal_num++;
					}
					else
					{
						if (!block)
						{
							box_num++;
						}
						block = !block;
					}
				}
				else
				{
					if (goal_index_map[x][y + j] >= 0)
					{
						goal_num++;
					}
					block = false;
				}
			}
		}
		/* if the number of boxes remaining in a goal-tunnel after removing blocked ones is less than the number of goals in the goal-tunnel,
		then goal-tunnel pattern dead-lock detected */
		if (box_num < goal_num)
		{
			return true;
		}
	}
	return false;
}