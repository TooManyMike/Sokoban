#include "Trap.h"
#include "Sokoban.h"

TrapSolver::TrapSolver(SokobanSolver* sokoban_solver)
{
	sokoban_solver_ = sokoban_solver;
	/* initialize trap_index_map_ */
	int width = sokoban_solver_->width_;
	int height = sokoban_solver_->height_;
	trap_index_map_ = (int**)malloc(width * sizeof(int*));
	trap_index_map_[0] = (int*)malloc(width * height * sizeof(int));
	for (int i = 1; i < width; i++)
	{
		trap_index_map_[i] = trap_index_map_[i - 1] + height;
	}
	memset(trap_index_map_[0], 0xFF, width * height * sizeof(int));
	/* patterns of trap */
	char trap_set_2_3_LEFT[5][4] = { 'N', 1, 1, 'N', 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0 };
	char trap_set_2_3_RIGHT[5][4] = { 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 'N', 1, 1, 'N' };
	char trap_set_2_3_UP[4][5] = { 'N', 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 'N', 0, 0, 0, 0 };
	char trap_set_2_3_DOWN[4][5] = { 0, 0, 0, 0, 'N', 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 'N' };
	char trap_set_2_2_LEFT[4][4] = { 'N', 1, 1, 'N', 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0 };
	char trap_set_2_2_RIGHT[4][4] = { 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 'N', 1, 1, 'N' };
	char trap_set_2_2_UP[4][4] = { 'N', 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 'N', 0, 0, 0 };
	char trap_set_2_2_DOWN[4][4] = { 0, 0, 0, 'N', 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 'N' };
	char trap_set_2_1_LEFT[3][4] = { 'N', 1, 1, 'N', 0, 1, 1, 0, 0, 0, 0, 0 };
	char trap_set_2_1_RIGHT[3][4] = { 0, 0, 0, 0, 0, 1, 1, 0, 'N', 1, 1, 'N' };
	char trap_set_2_1_UP[4][3] = { 'N', 0, 0, 1, 1, 0, 1, 1, 0, 'N', 0, 0 };
	char trap_set_2_1_DOWN[4][3] = { 0, 0, 'N', 0, 1, 1, 0, 1, 1, 0, 0, 'N' };
	char trap_set_3_3_LEFT[5][5] = { 'N', 1, 1, 1, 'N', 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0 };
	char trap_set_3_3_RIGHT[5][5] = { 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 'N', 1, 1, 1, 'N' };
	char trap_set_3_3_UP[5][5] = { 'N', 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 'N', 0, 0, 0, 0 };
	char trap_set_3_3_DOWN[5][5] = { 0, 0, 0, 0, 'N', 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 'N' };
	char trap_set_3_2_LEFT[4][5] = { 'N', 1, 1, 1, 'N', 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0 };
	char trap_set_3_2_RIGHT[4][5] = { 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 'N', 1, 1, 1, 'N' };
	char trap_set_3_2_UP[5][4] = { 'N', 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 'N', 0, 0, 0 };
	char trap_set_3_2_DOWN[5][4] = { 0, 0, 0, 'N', 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 'N' };
	char trap_set_3_1_LEFT[3][5] = { 'N', 1, 1, 1, 'N', 0, 1, 1, 1, 0, 0, 0, 0, 0, 0 };
	char trap_set_3_1_RIGHT[3][5] = { 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 'N', 1, 1, 1, 'N' };
	char trap_set_3_1_UP[5][3] = { 'N', 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 'N', 0, 0 };
	char trap_set_3_1_DOWN[5][3] = { 0, 0, 'N', 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 'N' };
	/* search for traps */
	int trap_num = 0;
	vector<Grid> findings;
	sokoban_solver_->find_pattern(findings, (char**)trap_set_2_3_LEFT, 5, 4);
	add_trap(findings, 2, 3, LEFT, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_2_3_RIGHT, 5, 4);
	add_trap(findings, 2, 3, RIGHT, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_2_3_UP, 4, 5);
	add_trap(findings, 2, 3, UP, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_2_3_DOWN, 4, 5);
	add_trap(findings, 2, 3, DOWN, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_2_2_LEFT, 4, 4);
	add_trap(findings, 2, 2, LEFT, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_2_2_RIGHT, 4, 4);
	add_trap(findings, 2, 2, RIGHT, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_2_2_UP, 4, 4);
	add_trap(findings, 2, 2, UP, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_2_2_DOWN, 4, 4);
	add_trap(findings, 2, 2, DOWN, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_2_1_LEFT, 3, 4);
	add_trap(findings, 2, 1, LEFT, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_2_1_RIGHT, 3, 4);
	add_trap(findings, 2, 1, RIGHT, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_2_1_UP, 4, 3);
	add_trap(findings, 2, 1, UP, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_2_1_DOWN, 4, 3);
	add_trap(findings, 2, 1, DOWN, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_3_3_LEFT, 5, 5);
	add_trap(findings, 3, 3, LEFT, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_3_3_RIGHT, 5, 5);
	add_trap(findings, 3, 3, RIGHT, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_3_3_UP, 5, 5);
	add_trap(findings, 3, 3, UP, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_3_3_DOWN, 5, 5);
	add_trap(findings, 3, 3, DOWN, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_3_2_LEFT, 4, 5);
	add_trap(findings, 3, 2, LEFT, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_3_2_RIGHT, 4, 5);
	add_trap(findings, 3, 2, RIGHT, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_3_2_UP, 5, 4);
	add_trap(findings, 3, 2, UP, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_3_2_DOWN, 5, 4);
	add_trap(findings, 3, 2, DOWN, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_3_1_LEFT, 3, 5);
	add_trap(findings, 3, 1, LEFT, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_3_1_RIGHT, 3, 5);
	add_trap(findings, 3, 1, RIGHT, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_3_1_UP, 5, 3);
	add_trap(findings, 3, 1, UP, trap_num);
	sokoban_solver_->find_pattern(findings, (char**)trap_set_3_1_DOWN, 5, 3);
	add_trap(findings, 3, 1, DOWN, trap_num);
}

TrapSolver::~TrapSolver()
{
	free(trap_index_map_[0]);
	free(trap_index_map_);
}

void TrapSolver::add_trap(vector<Grid> findings, int width, int depth, char direction, int &trap_num)
{
	bool check;
	int i, j, k, x, y;
	Trap trap;
	int** goal_index_map = sokoban_solver_->goal_index_map_;
	int** box_pos_index_map = sokoban_solver_->box_pos_index_map_;
	for (k = 0; k < findings.size(); k++)
	{
		x = findings[k].x_ + 1;
		y = findings[k].y_ + 1;
		if (trap_index_map_[x][y] != -1)	//trap repeated
		{
			continue;
		}
		/* check trap bottom containing goal or not */
		check = false;
		switch (direction)
		{
		case LEFT:
			for (i = 0; i < width; i++)
			{
				if (goal_index_map[x + depth - 1][y + i] != -1)
				{
					check = true;
					break;
				}
			}
			break;
		case RIGHT:
			for (i = 0; i < width; i++)
			{
				if (goal_index_map[x][y + i] != -1)
				{
					check = true;
					break;
				}
			}
			break;
		case UP:
			for (i = 0; i < width; i++)
			{
				if (goal_index_map[x + i][y + depth - 1] != -1)
				{
					check = true;
					break;
				}
			}
			break;
		case DOWN:
			for (i = 0; i < width; i++)
			{
				if (goal_index_map[x + i][y] != -1)
				{
					check = true;
					break;
				}
			}
			break;
		}
		if (check)
		{
			continue;
		}
		/* check axial lines of the trap all containing goal or not */
		check = true;
		switch (direction)
		{
		case LEFT:
			for (i = 0; i < width; i++)
			{
				if ((depth == 1 && goal_index_map[x - 1][y + i] == -1) || (depth == 2 && goal_index_map[x][y + i] == -1) || (depth == 3 && goal_index_map[x][y + i] == -1 && goal_index_map[x + 1][y + i] == -1))
				{
					check = false;
					break;
				}
			}
			break;
		case RIGHT:
			for (i = 0; i < width; i++)
			{
				if ((depth == 1 && goal_index_map[x + 1][y + i] == -1) || (depth == 2 && goal_index_map[x + 1][y + i] == -1) || (depth == 3 && goal_index_map[x + 1][y + i] == -1 && goal_index_map[x + 2][y + i] == -1))
				{
					check = false;
					break;
				}
			}
			break;
		case UP:
			for (i = 0; i < width; i++)
			{
				if ((depth == 1 && goal_index_map[x + i][y - 1] == -1) || (depth == 2 && goal_index_map[x + i][y] == -1) || (depth == 3 && goal_index_map[x + i][y] == -1 && goal_index_map[x + i][y + 1] == -1))
				{
					check = false;
					break;
				}
			}
			break;
		case DOWN:
			for (i = 0; i < width; i++)
			{
				if ((depth == 1 && goal_index_map[x + i][y + 1] == -1) || (depth == 2 && goal_index_map[x + i][y + 1] == -1) || (depth == 3 && goal_index_map[x + i][y + 1] == -1 && goal_index_map[x + i][y + 2] == -1))
				{
					check = false;
					break;
				}
			}
			break;
		}
		if (check)
		{
			continue;
		}
		/* check grids at trap entrance all unlocked or not */
		check = true;
		switch (direction)
		{
		case LEFT:
			for (i = 0; i < width; i++)
			{
				if ((depth == 1 && box_pos_index_map[x - 1][y + i] == -1) || (depth >= 2 && box_pos_index_map[x][y + i] == -1))
				{
					check = false;
					break;
				}
			}
			break;
		case RIGHT:
			for (i = 0; i < width; i++)
			{
				if ((depth == 1 && box_pos_index_map[x + 1][y + i] == -1) || (depth >= 2 && box_pos_index_map[x + depth - 1][y + i] == -1))
				{
					check = false;
					break;
				}
			}
			break;
		case UP:
			for (i = 0; i < width; i++)
			{
				if ((depth == 1 && box_pos_index_map[x + i][y - 1] == -1) || (depth >= 2 && box_pos_index_map[x + i][y] == -1))
				{
					check = false;
					break;
				}
			}
			break;
		case DOWN:
			for (i = 0; i < width; i++)
			{
				if ((depth == 1 && box_pos_index_map[x + i][y + 1] == -1) || (depth >= 2 && box_pos_index_map[x + i][y + depth - 1] == -1))
				{
					check = false;
					break;
				}
			}
			break;
		}
		if (!check)
		{
			continue;
		}
		//add trap into trap_set_ */
		trap.type_ = direction;
		trap.width_ = width;
		trap.depth_ = depth;
		trap.start_.x_ = x;
		trap.start_.y_ = y;
		if (direction == RIGHT)
		{
			trap.start_.x_ += depth - 1;
		}
		else if (direction == DOWN)
		{
			trap.start_.y_ += depth - 1;
		}
		trap_set_.push_back(trap);
		/* update trap_index_map_ */
		if (direction == LEFT || direction == RIGHT)
		{
			for (i = 0; i < depth; i++)
			{
				for (j = 0; j < width; j++)
				{
					trap_index_map_[x + i][y + j] = trap_num;
				}
			}
		}
		else
		{
			for (i = 0; i < width; i++)
			{
				for (j = 0; j < depth; j++)
				{
					trap_index_map_[x + i][y + j] = trap_num;
				}
			}
		}
		trap_num++;
	}
}

bool TrapSolver::check_trap_deadlock(StateNode* node)
{
	int i, x, y, start_x, start_y, trap_index;
	bool check;
	bool** box_map = sokoban_solver_->box_map_;
	x = node->box_[node->moved_box_index_].x_;
	y = node->box_[node->moved_box_index_].y_;
	trap_index = trap_index_map_[x][y];
	if (trap_index != -1)	//box moved into a trap
	{
		start_x = trap_set_[trap_index].start_.x_;
		start_y = trap_set_[trap_index].start_.y_;
		check = true;
		switch (trap_set_[trap_index].type_)
		{
		case LEFT:
			for (i = 0; i < trap_set_[trap_index].width_; i++)
			{
				if (!box_map[start_x][start_y + i] && (trap_set_[trap_index].depth_ == 2 || !box_map[start_x + 1][start_y + i]))
				{
					check = false;
					break;
				}
			}
			break;
		case RIGHT:
			for (i = 0; i < trap_set_[trap_index].width_; i++)
			{
				if (!box_map[start_x][start_y + i] && (trap_set_[trap_index].depth_ == 2 || !box_map[start_x - 1][start_y + i]))
				{
					check = false;
					break;
				}
			}
			break;
		case UP:
			for (i = 0; i < trap_set_[trap_index].width_; i++)
			{
				if (!box_map[start_x + i][start_y] && (trap_set_[trap_index].depth_ == 2 || !box_map[start_x + i][start_y + 1]))
				{
					check = false;
					break;
				}
			}
			break;
		case DOWN:
			for (i = 0; i < trap_set_[trap_index].width_; i++)
			{
				if (!box_map[start_x + i][start_y] && (trap_set_[trap_index].depth_ == 2 || !box_map[start_x + i][start_y - 1]))
				{
					check = false;
					break;
				}
			}
			break;
		}
		if (!check)
		{
			return false;
		}
		/* check man in a trap(sealed up by boxes) or not */
		switch (trap_set_[trap_index].type_)
		{
		case LEFT:
			if (node->man_.x_ <= start_x)
			{
				return true;
			}
			return false;
		case RIGHT:
			if (node->man_.x_ >= start_x)
			{
				return true;
			}
			return false;
		case UP:
			if (node->man_.y_ <= start_y)
			{
				return true;
			}
			return false;
		case DOWN:
			if (node->man_.y_ >= start_y)
			{
				return true;
			}
			return false;
		}
	}
	trap_index = trap_index_map_[x - 1][y];
	if (trap_index != -1 && trap_set_[trap_index].depth_ == 1)	//box moved to entrance of a trap with depth 1
	{
		start_y = trap_set_[trap_index].start_.y_;
		check = true;
		for (i = 0; i < trap_set_[trap_index].width_; i++)
		{
			if (!box_map[x][start_y + i])
			{
				check = false;
				break;
			}
		}
		if (check)
		{
			return true;
		}
	}
	trap_index = trap_index_map_[x + 1][y];
	if (trap_index != -1 && trap_set_[trap_index].depth_ == 1)
	{
		start_y = trap_set_[trap_index].start_.y_;
		check = true;
		for (i = 0; i < trap_set_[trap_index].width_; i++)
		{
			if (!box_map[x][start_y + i])
			{
				check = false;
				break;
			}
		}
		if (check)
		{
			return true;
		}
	}
	trap_index = trap_index_map_[x][y - 1];
	if (trap_index != -1 && trap_set_[trap_index].depth_ == 1)
	{
		start_x = trap_set_[trap_index].start_.x_;
		check = true;
		for (i = 0; i < trap_set_[trap_index].width_; i++)
		{
			if (!box_map[start_x + i][y])
			{
				check = false;
				break;
			}
		}
		if (check)
		{
			return true;
		}
	}
	trap_index = trap_index_map_[x][y + 1];
	if (trap_index != -1 && trap_set_[trap_index].depth_ == 1)
	{
		start_x = trap_set_[trap_index].start_.x_;
		check = true;
		for (i = 0; i < trap_set_[trap_index].width_; i++)
		{
			if (!box_map[start_x + i][y])
			{
				check = false;
				break;
			}
		}
		if (check)
		{
			return true;
		}
	}
	return false;
}
