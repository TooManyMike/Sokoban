#include <algorithm>
#include "AStarPath.h"
#include "Sokoban.h"

AStarPath::AStarPath(bool** map, int width, int height)
{
	map_ = map;
	width_ = width;
	height_ = height;
	pos_node_ = (PosNode**)malloc(width * sizeof(PosNode*));
	pos_node_[0] = (PosNode*)malloc(width * height * sizeof(PosNode));
	for (int i = 1; i < width; i++)
	{
		pos_node_[i] = pos_node_[i - 1] + height;
	}
}

AStarPath::~AStarPath()
{
	free(pos_node_[0]);
	free(pos_node_);
}

void AStarPath::set(Grid start, Grid end)
{
	start_ = start;
	end_ = end;
}

bool comp_pos(PosNode* node1, PosNode* node2)
{
	return node1->f_ > node2->f_;
}

void AStarPath::try_expand(PosNode* current, char direction)
{
	int neighbour_x, neighbour_y;
	switch (direction)
	{
	case LEFT:
		neighbour_x = current->x_ - 1;
		neighbour_y = current->y_;
		break;
	case RIGHT:
		neighbour_x = current->x_ + 1;
		neighbour_y = current->y_;
		break;
	case UP:
		neighbour_x = current->x_;
		neighbour_y = current->y_ - 1;
		break;
	case DOWN:
		neighbour_x = current->x_;
		neighbour_y = current->y_ + 1;
		break;
	}
	PosNode* neighbour = &pos_node_[neighbour_x][neighbour_y];
	if (neighbour->state_ == UNVISITED)
	{
		neighbour->x_ = neighbour_x;
		neighbour->y_ = neighbour_y;
		neighbour->g_ = current->g_ + 1;
		neighbour->h_ = abs(neighbour_x - end_.x_) + abs(neighbour_y - end_.y_);
		neighbour->f_ = neighbour->g_ + neighbour->h_;
		neighbour->state_ = OPEN;
		neighbour->predecessor_ = current;
		open_list_.push_back(neighbour);
		push_heap(open_list_.begin(), open_list_.end(), comp_pos);
	}
	else if (neighbour->state_ == OPEN)
	{
		if (current->g_ + 1 < neighbour->g_)
		{
			/* update neighbor PosNode */
			neighbour->g_ = current->g_ + 1;
			neighbour->f_ = neighbour->g_ + neighbour->h_;
			neighbour->predecessor_ = current;
			make_heap(open_list_.begin(), open_list_.end(), comp_pos);
		}
	}
}

bool AStarPath::find()
{
	PosNode* current;
	memset(pos_node_[0], 0, width_ * height_ * sizeof(PosNode));
	open_list_.clear();
	if (map_[start_.x_][start_.y_] && map_[end_.x_][end_.y_])
	{
		current = &pos_node_[start_.x_][start_.y_];
		current->x_ = start_.x_;
		current->y_ = start_.y_;
		current->g_ = 0;
		current->h_ = abs(start_.x_ - end_.x_) + abs(start_.y_ - end_.y_);
		current->f_ = current->h_;
		current->state_ = OPEN;
		open_list_.push_back(current);
		while (!open_list_.empty())
		{
			/* get PosNode with minimum f value from the open list */
			pop_heap(open_list_.begin(), open_list_.end(), comp_pos);
			current = open_list_.back();
			open_list_.pop_back();
			if (current->h_ == 0)
			{
				return true;
			}
			else
			{
				if (current->x_ > 0 && map_[current->x_ - 1][current->y_])
				{
					try_expand(current, LEFT);
				}
				if (current->x_ < width_ - 1 && map_[current->x_ + 1][current->y_])
				{
					try_expand(current, RIGHT);
				}
				if (current->y_ > 0 && map_[current->x_][current->y_ - 1])
				{
					try_expand(current, UP);
				}
				if (current->y_ < height_ - 1 && map_[current->x_][current->y_ + 1])
				{
					try_expand(current, DOWN);
				}
				current->state_ = CLOSED;
			}
		}
	}      
	return false;
}

int AStarPath::get(char* &step)
{
	int step_num;
	PosNode* current;
	memset(pos_node_[0], 0, width_ * height_ * sizeof(PosNode));
	open_list_.clear();
	if (map_[start_.x_][start_.y_] && map_[end_.x_][end_.y_])
	{
		current = &pos_node_[start_.x_][start_.y_];
		current->x_ = start_.x_;
		current->y_ = start_.y_;
		current->g_ = 0;
		current->h_ = abs(start_.x_ - end_.x_) + abs(start_.y_ - end_.y_);
		current->f_ = current->h_;
		current->state_ = OPEN;
		open_list_.push_back(current);
		while (!open_list_.empty())
		{
			pop_heap(open_list_.begin(), open_list_.end(), comp_pos);
			current = open_list_.back();
			open_list_.pop_back();
			if (current->h_ == 0)
			{
				step_num = current->g_;
				if (step_num > 0)
				{
					step = (char*)malloc(step_num);
					/* get path steps by tracing predecessor */
					for (int i = step_num - 1; i >= 0; i--)
					{
						if (current->x_ == current->predecessor_->x_ - 1)
						{
							step[i] = LEFT;
						}
						else if (current->x_ == current->predecessor_->x_ + 1)
						{
							step[i] = RIGHT;
						}
						else if (current->y_ == current->predecessor_->y_ - 1)
						{
							step[i] = UP;
						}
						else
						{
							step[i] = DOWN;
						}
						current = current->predecessor_;
					}
				}
				return step_num;
			}
			else
			{
				if (current->x_ > 0 && map_[current->x_ - 1][current->y_])
				{
					try_expand(current, LEFT);
				}
				if (current->x_ < width_ - 1 && map_[current->x_ + 1][current->y_])
				{
					try_expand(current, RIGHT);
				}
				if (current->y_ > 0 && map_[current->x_][current->y_ - 1])
				{
					try_expand(current, UP);
				}
				if (current->y_ < height_ - 1 && map_[current->x_][current->y_ + 1])
				{
					try_expand(current, DOWN);
				}
				current->state_ = CLOSED;
			}
		}
	}        
	return -1;	//can't find path
}