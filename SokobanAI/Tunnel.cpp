#include <algorithm>
#include "Tunnel.h"
#include "Sokoban.h"

TunnelSolver::TunnelSolver(SokobanSolver* sokoban_solver)
{
	sokoban_solver_ = sokoban_solver;
	int width = sokoban_solver_->width_;
	int height = sokoban_solver_->height_;
	int** goal_index_map = sokoban_solver_->goal_index_map_;
	bool** no_box_maze = sokoban_solver_->no_box_maze_;
	bool valid;
	char type;
	Grid temp;
	vector<Grid> findings, path;
	Tunnel tunnel;
	int i, x, y, goal_num, neighbour_num, tunnel_num = 0;
	tunnel_index_map_ = (int**)malloc(width * sizeof(int*));
	tunnel_block_entrance_map_ = (bool**)malloc(width * sizeof(bool*));
	char** tunnel_check = (char**)malloc(width * sizeof(char*));
	bool** visited = (bool**)malloc(width * sizeof(bool*));
	tunnel_index_map_[0] = (int*)malloc(width * height * sizeof(int));
	tunnel_block_entrance_map_[0] = (bool*)malloc(width * height);
	tunnel_check[0] = (char*)malloc(width * height);
	visited[0] = (bool*)malloc(width * height);
	memset(tunnel_index_map_[0], 0xFF, width * height * sizeof(int));
	memset(tunnel_block_entrance_map_[0], 0, width * height);
	memset(tunnel_check[0], 0, width * height);
	memset(visited[0], 0, width * height);
	for (i = 1; i < width; i++)
	{
		tunnel_index_map_[i] = tunnel_index_map_[i - 1] + height;
		tunnel_block_entrance_map_[i] = tunnel_block_entrance_map_[i - 1] + height;
		tunnel_check[i] = tunnel_check[i - 1] + height;
		visited[i] = visited[i - 1] + height;
	}
	/* patterns of a grid in a Tunnel */
	char left_right_straight[3][3] = { 'N', 1, 'N', 0, 1, 0, 'N', 1, 'N' };
	char up_down_straight[3][3] = { 'N', 0, 'N', 1, 1, 1, 'N', 0, 'N' };
	char left_up_turning[3][3] = { 0, 1, 'N', 1, 1, 0, 'N', 0, 'N' };
	char left_down_turning[3][3] = { 'N', 1, 0, 0, 1, 1, 'N', 0, 'N' };
	char right_up_turning[3][3] = { 'N', 0, 'N', 1, 1, 0, 0, 1, 'N' };
	char right_down_turning[3][3] = { 'N', 0, 'N', 0, 1, 1, 'N', 1, 0 };
	/* search for grids in a Tunnel */
	sokoban_solver_->find_pattern(findings, (char**)left_right_straight, 3, 3);
	for (i = 0; i < findings.size(); i++)
	{
		tunnel_check[findings[i].x_ + 1][findings[i].y_ + 1] = LEFT_RIGHT_STRAIGHT;
	}
	sokoban_solver_->find_pattern(findings, (char**)up_down_straight, 3, 3);
	for (i = 0; i < findings.size(); i++)
	{
		tunnel_check[findings[i].x_ + 1][findings[i].y_ + 1] = UP_DOWN_STRAIGHT;
	}
	sokoban_solver_->find_pattern(findings, (char**)left_up_turning, 3, 3);
	for (i = 0; i < findings.size(); i++)
	{
		tunnel_check[findings[i].x_ + 1][findings[i].y_ + 1] = LEFT_UP_TURNING;
	}
	sokoban_solver_->find_pattern(findings, (char**)right_up_turning, 3, 3);
	for (i = 0; i < findings.size(); i++)
	{
		tunnel_check[findings[i].x_ + 1][findings[i].y_ + 1] = RIGHT_UP_TURNING;
	}
	sokoban_solver_->find_pattern(findings, (char**)left_down_turning, 3, 3);
	for (i = 0; i < findings.size(); i++)
	{
		tunnel_check[findings[i].x_ + 1][findings[i].y_ + 1] = LEFT_DOWN_TURNING;
	}
	sokoban_solver_->find_pattern(findings, (char**)right_down_turning, 3, 3);
	for (i = 0; i < findings.size(); i++)
	{
		tunnel_check[findings[i].x_ + 1][findings[i].y_ + 1] = RIGHT_DOWN_TURNING;
	}
	/* screen valid Tunnels */
	for (x = 1; x < width; x++)
	{
		for (y = 1; y < height; y++)
		{
			if (tunnel_check[x][y] != 0 && !visited[x][y])
			{
				neighbour_num = 0;
				if (tunnel_check[x - 1][y] != 0)
				{
					neighbour_num++;
				}
				if (tunnel_check[x + 1][y] != 0)
				{
					neighbour_num++;
				}
				if (tunnel_check[x][y - 1] != 0)
				{
					neighbour_num++;
				}
				if (tunnel_check[x][y + 1] != 0)
				{
					neighbour_num++;
				}
				if (neighbour_num <= 1)		//terminal of a Tunnel
				{
					path.clear();
					valid = true;
					goal_num = 0;
					tunnel.straight_ = true;
					tunnel.length_ = 0;
					temp.x_ = x;
					temp.y_ = y;
					tunnel.start_ = temp;	//get start terminal
					/* process grids in a Tunnel, check Tunnel straight or not, count number of goals in the Tunnel */
					while (1)
					{
						path.push_back(temp);
						tunnel.length_++;
						visited[temp.x_][temp.y_] = true;
						if (!(tunnel_check[temp.x_][temp.y_] == LEFT_RIGHT_STRAIGHT || tunnel_check[temp.x_][temp.y_] == UP_DOWN_STRAIGHT))
						{
							tunnel.straight_ = false;
						}
						if (goal_index_map[temp.x_][temp.y_] != -1)
						{
							goal_num++;
						}
						if (tunnel_check[temp.x_ - 1][temp.y_] != 0 && !visited[temp.x_ - 1][temp.y_])
						{
							temp.x_--;
						}
						else if (tunnel_check[temp.x_ + 1][temp.y_] != 0 && !visited[temp.x_ + 1][temp.y_])
						{
							temp.x_++;
						}
						else if (tunnel_check[temp.x_][temp.y_ - 1] != 0 && !visited[temp.x_][temp.y_ - 1])
						{
							temp.y_--;
						}
						else if (tunnel_check[temp.x_][temp.y_ + 1] != 0 && !visited[temp.x_][temp.y_ + 1])
						{
							temp.y_++;
						}
						else
						{
							break;
						}
					}
					tunnel.end_ = temp;		//get end terminal
					/* get Tunnel direction */
					if (tunnel.straight_ && tunnel.length_ > 1)
					{
						if (tunnel_check[x][y] == LEFT_RIGHT_STRAIGHT && tunnel_check[x - 1][y] != 0)
						{
							tunnel.direction_ = LEFT;
						}
						else if (tunnel_check[x][y] == LEFT_RIGHT_STRAIGHT && tunnel_check[x + 1][y] != 0)
						{
							tunnel.direction_ = RIGHT;
						}
						else if (tunnel_check[x][y] == UP_DOWN_STRAIGHT && tunnel_check[x][y - 1] != 0)
						{
							tunnel.direction_ = UP;
						}
						else
						{
							tunnel.direction_ = DOWN;
						}
					}
					else
					{
						tunnel.direction_ = 0;
					}
					/* process the outer neighbor grid of the Tunnel's start terminal */
					type = tunnel_check[x][y];
					if ((type == LEFT_RIGHT_STRAIGHT || type == LEFT_UP_TURNING || type == LEFT_DOWN_TURNING) && tunnel_check[x - 1][y] == 0)
					{
						tunnel.start_out_.x_ = x - 1;
						tunnel.start_out_.y_ = y;
						if (!no_box_maze[x - 2][y])
						{
							valid = false;
						}
					}
					else if ((type == LEFT_RIGHT_STRAIGHT || type == RIGHT_UP_TURNING || type == RIGHT_DOWN_TURNING) && tunnel_check[x + 1][y] == 0)
					{
						tunnel.start_out_.x_ = x + 1;
						tunnel.start_out_.y_ = y;
						if (!no_box_maze[x + 2][y])
						{
							valid = false;
						}
					}
					else if ((type == UP_DOWN_STRAIGHT || type == LEFT_UP_TURNING || type == RIGHT_UP_TURNING) && tunnel_check[x][y - 1] == 0)
					{
						tunnel.start_out_.x_ = x;
						tunnel.start_out_.y_ = y - 1;
						if (!no_box_maze[x][y - 2])
						{
							valid = false;
						}
					}
					else
					{
						tunnel.start_out_.x_ = x;
						tunnel.start_out_.y_ = y + 1;
						if (!no_box_maze[x][y + 2])
						{
							valid = false;
						}
					}
					if (type == LEFT_UP_TURNING || type == LEFT_DOWN_TURNING || type == RIGHT_UP_TURNING || type == RIGHT_DOWN_TURNING)
					{
						tunnel.start_out_block_ = true;
					}
					else if (type == LEFT_RIGHT_STRAIGHT && (!no_box_maze[tunnel.start_out_.x_][tunnel.start_out_.y_ - 1] || !no_box_maze[tunnel.start_out_.x_][tunnel.start_out_.y_ + 1]))
					{
						tunnel.start_out_block_ = true;
					}
					else if (type == UP_DOWN_STRAIGHT && (!no_box_maze[tunnel.start_out_.x_ - 1][tunnel.start_out_.y_] || !no_box_maze[tunnel.start_out_.x_ + 1][tunnel.start_out_.y_]))
					{
						tunnel.start_out_block_ = true;
					}
					else
					{
						tunnel.start_out_block_ = false;
					}
					if (tunnel.start_out_block_ && goal_index_map[tunnel.start_out_.x_][tunnel.start_out_.y_] != -1)
					{
						goal_num++;
					}
					/* process the outer neighbor grid of the Tunnel's end terminal */
					type = tunnel_check[tunnel.end_.x_][tunnel.end_.y_];
					if ((type == LEFT_RIGHT_STRAIGHT || type == LEFT_UP_TURNING || type == LEFT_DOWN_TURNING) && tunnel_check[tunnel.end_.x_ - 1][tunnel.end_.y_] == 0 && (tunnel.length_ > 1 || tunnel.start_out_.x_ != x - 1))
					{
						tunnel.end_out_.x_ = tunnel.end_.x_ - 1;
						tunnel.end_out_.y_ = tunnel.end_.y_;
						if (!no_box_maze[tunnel.end_.x_ - 2][tunnel.end_.y_])
						{
							valid = false;
						}
					}
					else if ((type == LEFT_RIGHT_STRAIGHT || type == RIGHT_UP_TURNING || type == RIGHT_DOWN_TURNING) && tunnel_check[tunnel.end_.x_ + 1][tunnel.end_.y_] == 0 && (tunnel.length_ > 1 || tunnel.start_out_.x_ != x + 1))
					{
						tunnel.end_out_.x_ = tunnel.end_.x_ + 1;
						tunnel.end_out_.y_ = tunnel.end_.y_;
						if (!no_box_maze[tunnel.end_.x_ + 2][tunnel.end_.y_])
						{
							valid = false;
						}
					}
					else if ((type == UP_DOWN_STRAIGHT || type == LEFT_UP_TURNING || type == RIGHT_UP_TURNING) && tunnel_check[tunnel.end_.x_][tunnel.end_.y_ - 1] == 0 && (tunnel.length_ > 1 || tunnel.start_out_.y_ != y - 1))
					{
						tunnel.end_out_.x_ = tunnel.end_.x_;
						tunnel.end_out_.y_ = tunnel.end_.y_ - 1;
						if (!no_box_maze[tunnel.end_.x_][tunnel.end_.y_ - 2])
						{
							valid = false;
						}
					}
					else
					{
						tunnel.end_out_.x_ = tunnel.end_.x_;
						tunnel.end_out_.y_ = tunnel.end_.y_ + 1;
						if (!no_box_maze[tunnel.end_.x_][tunnel.end_.y_ + 2])
						{
							valid = false;
						}
					}
					if (type == LEFT_UP_TURNING || type == LEFT_DOWN_TURNING || type == RIGHT_UP_TURNING || type == RIGHT_DOWN_TURNING)
					{
						tunnel.end_out_block_ = true;
					}
					else if (type == LEFT_RIGHT_STRAIGHT && (!no_box_maze[tunnel.end_out_.x_][tunnel.end_out_.y_ - 1] || !no_box_maze[tunnel.end_out_.x_][tunnel.end_out_.y_ + 1]))
					{
						tunnel.end_out_block_ = true;
					}
					else if (type == UP_DOWN_STRAIGHT && (!no_box_maze[tunnel.end_out_.x_ - 1][tunnel.end_out_.y_] || !no_box_maze[tunnel.end_out_.x_ + 1][tunnel.end_out_.y_]))
					{
						tunnel.end_out_block_ = true;
					}
					else
					{
						tunnel.end_out_block_ = false;
					}
					if (tunnel.end_out_block_ && goal_index_map[tunnel.end_out_.x_][tunnel.end_out_.y_] != -1)
					{
						goal_num++;
					}
					/* if the Tunnel is valid, put it into tunnel_set_, and modify tunnel_index_map_ */
					if (valid && (tunnel.length_ > 1 || (tunnel.start_out_block_ && tunnel.end_out_block_)) && goal_num < 2)
					{
						tunnel_set_.push_back(tunnel);
						if (tunnel.start_out_block_)
						{
							tunnel_block_entrance_map_[tunnel.start_out_.x_][tunnel.start_out_.y_] = true;
						}
						if (tunnel.end_out_block_)
						{
							tunnel_block_entrance_map_[tunnel.end_out_.x_][tunnel.end_out_.y_] = true;
						}
						while (!path.empty())
						{
							tunnel_index_map_[path.back().x_][path.back().y_] = tunnel_num;
							path.pop_back();
						}
						tunnel_num++;
					}
				}
			}
		}
	}
	free(tunnel_check[0]);
	free(tunnel_check);
	free(visited[0]);
	free(visited);
}

TunnelSolver::~TunnelSolver()
{
	free(tunnel_index_map_[0]);
	free(tunnel_index_map_);
	free(tunnel_block_entrance_map_[0]);
	free(tunnel_block_entrance_map_);
}

bool TunnelSolver::check_tunnel_deadlock(StateNode* node)
{
	int x, y, tunnel_index;
	bool** box_map = sokoban_solver_->box_map_;
	x = node->box_[node->moved_box_index_].x_;
	y = node->box_[node->moved_box_index_].y_;
	tunnel_index = tunnel_index_map_[x][y];
	if (tunnel_index != -1)		//box moved into a Tunnel
	{
		Tunnel &tunnel = tunnel_set_[tunnel_index];
		if (tunnel.length_ == 1 || tunnel_index_map_[node->man_.x_][node->man_.y_] == tunnel_index)
		{
			return false;
		}
		if (x == tunnel.start_.x_ && y == tunnel.start_.y_)
		{
			if ((box_map[tunnel.end_out_.x_][tunnel.end_out_.y_] && tunnel.end_out_block_) || box_map[tunnel.end_.x_][tunnel.end_.y_])
			{
				return true;
			}
		}
		else
		{
			if ((box_map[tunnel.start_out_.x_][tunnel.start_out_.y_] && tunnel.start_out_block_) || box_map[tunnel.start_.x_][tunnel.start_.y_])
			{
				return true;
			}
		}
		return false;
	}
	if (tunnel_block_entrance_map_[x][y])	//box moved to an outer neighbor grid of a Tunnel
	{
		tunnel_index = tunnel_index_map_[x - 1][y];
		if (tunnel_index != -1 && check_tunnel_block(node, tunnel_index))
		{
			return true;
		}
		tunnel_index = tunnel_index_map_[x + 1][y];
		if (tunnel_index != -1 && check_tunnel_block(node, tunnel_index))
		{
			return true;
		}
		tunnel_index = tunnel_index_map_[x][y - 1];
		if (tunnel_index != -1 && check_tunnel_block(node, tunnel_index))
		{
			return true;
		}
		tunnel_index = tunnel_index_map_[x][y + 1];
		if (tunnel_index != -1 && check_tunnel_block(node, tunnel_index))
		{
			return true;
		}
	}
	return false;
}

bool TunnelSolver::check_tunnel_block(StateNode* node, int tunnel_index)
{
	int x, y;
	bool** box_map = sokoban_solver_->box_map_;
	if (tunnel_index_map_[node->man_.x_][node->man_.y_] == tunnel_index)
	{
		return false;
	}
	x = node->box_[node->moved_box_index_].x_;
	y = node->box_[node->moved_box_index_].y_;
	Tunnel &tunnel = tunnel_set_[tunnel_index];
	if (x == tunnel.start_out_.x_ && y == tunnel.start_out_.y_)
	{
		if ((box_map[tunnel.end_out_.x_][tunnel.end_out_.y_] && tunnel.end_out_block_) || box_map[tunnel.end_.x_][tunnel.end_.y_])
		{
			return true;
		}
	}
	else
	{
		if ((box_map[tunnel.start_out_.x_][tunnel.start_out_.y_] && tunnel.start_out_block_) || box_map[tunnel.start_.x_][tunnel.start_.y_])
		{
			return true;
		}
	}
	return false;
}

int TunnelSolver::get_pass_tunnel_distance(StateNode* node, int moved_box_index, char direction)
{
	int x, y, tunnel_index;
	bool** box_map = sokoban_solver_->box_map_;
	x = node->box_[moved_box_index].x_;
	y = node->box_[moved_box_index].y_;
	tunnel_index = tunnel_index_map_[x][y];
	if (tunnel_index == -1)
	{
		return 1;
	}
	Tunnel &tunnel = tunnel_set_[tunnel_index];
	if (tunnel.length_ == 1 || !tunnel_set_[tunnel_index].straight_)
	{
		return 1;
	}
	if (direction + 0x20 == tunnel.direction_)
	{
		if (box_map[tunnel.end_out_.x_][tunnel.end_out_.y_])
		{
			return -1;
		}
		return max(abs(x - tunnel.end_out_.x_), abs(y - tunnel.end_out_.y_));
	}
	else
	{
		if (box_map[tunnel.start_out_.x_][tunnel.start_out_.y_])
		{
			return -1;
		}
		return max(abs(x - tunnel.start_out_.x_), abs(y - tunnel.start_out_.y_));
	}
}