#include <queue>
#include "SmartMap.h"

using namespace std;

bool check_map_closure(bool** map, int width, int height, Grid man)
{
	bool result = true;
	int temp_x, temp_y;
	Grid temp;
	queue<Grid> open_list;
	bool** closed_map = (bool**)malloc(width * sizeof(bool*));
	closed_map[0] = (bool*)malloc(width * height);
	memset(closed_map[0], 0, width * height);
	for (int i = 1; i < width; i++)
	{
		closed_map[i] = closed_map[i - 1] + height;
	}
	open_list.push(man);
	while (!open_list.empty())
	{
		temp_x = open_list.front().x_;
		if (temp_x == 0 || temp_x == width - 1)
		{
			result = false;
			break;
		}
		temp_y = open_list.front().y_;
		if (temp_y == 0 || temp_y == height - 1)
		{
			result = false;
			break;
		}
		open_list.pop();
		if (map[temp_x - 1][temp_y] && !closed_map[temp_x - 1][temp_y])
		{
			closed_map[temp_x - 1][temp_y] = true;
			temp.x_ = temp_x - 1;
			temp.y_ = temp_y;
			open_list.push(temp);
		}
		if (map[temp_x + 1][temp_y] && !closed_map[temp_x + 1][temp_y])
		{
			closed_map[temp_x + 1][temp_y] = true;
			temp.x_ = temp_x + 1;
			temp.y_ = temp_y;
			open_list.push(temp);
		}
		if (map[temp_x][temp_y - 1] && !closed_map[temp_x][temp_y - 1])
		{
			closed_map[temp_x][temp_y - 1] = true;
			temp.x_ = temp_x;
			temp.y_ = temp_y - 1;
			open_list.push(temp);
		}
		if (map[temp_x][temp_y + 1] && !closed_map[temp_x][temp_y + 1])
		{
			closed_map[temp_x][temp_y + 1] = true;
			temp.x_ = temp_x;
			temp.y_ = temp_y + 1;
			open_list.push(temp);
		}
	}
	free(closed_map[0]);
	free(closed_map);
	return result;
}

void get_connectivity_map(char** answer, bool** map, int width, int height)
{
	int x, y, temp_x, temp_y;
	Grid temp;
	queue<Grid> open_list;
	for (x = 0; x < width; x++)
	{
		for (y = 0; y < height; y++)
		{
			if (map[x][y])
			{
				answer[x][y] = '0';
			}
			else
			{
				answer[x][y] = '#';
			}
		}
	}
	char c = '0';
	for (x = 0; x < width; x++)
	{
		for (y = 0; y < height; y++)
		{
			if (answer[x][y] == '0')
			{
				c++;
				answer[x][y] = c;
				temp.x_ = x;
				temp.y_ = y;
				open_list.push(temp);
				while (!open_list.empty())
				{
					temp_x = open_list.front().x_;
					temp_y = open_list.front().y_;
					open_list.pop();
					if (map[temp_x - 1][temp_y] && answer[temp_x - 1][temp_y] == '0')
					{
						answer[temp_x - 1][temp_y] = c;
						temp.x_ = temp_x - 1;
						temp.y_ = temp_y;
						open_list.push(temp);
					}
					if (map[temp_x + 1][temp_y] && answer[temp_x + 1][temp_y] == '0')
					{
						answer[temp_x + 1][temp_y] = c;
						temp.x_ = temp_x + 1;
						temp.y_ = temp_y;
						open_list.push(temp);
					}
					if (map[temp_x][temp_y - 1] && answer[temp_x][temp_y - 1] == '0')
					{
						answer[temp_x][temp_y - 1] = c;
						temp.x_ = temp_x;
						temp.y_ = temp_y - 1;
						open_list.push(temp);
					}
					if (map[temp_x][temp_y + 1] && answer[temp_x][temp_y + 1] == '0')
					{
						answer[temp_x][temp_y + 1] = c;
						temp.x_ = temp_x;
						temp.y_ = temp_y + 1;
						open_list.push(temp);
					}
				}
			}
		}
	}
}

void get_reachability_map(bool** answer, bool** map, int width, int height, Grid start)
{
	int temp_x, temp_y;
	Grid temp;
	queue<Grid> open_list;
	memset(answer[0], 0, width * height);
	answer[start.x_][start.y_] = 0;
	open_list.push(start);
	while (!open_list.empty())
	{
		temp_x = open_list.front().x_;
		temp_y = open_list.front().y_;
		open_list.pop();
		if (map[temp_x - 1][temp_y] && !answer[temp_x - 1][temp_y])
		{
			answer[temp_x - 1][temp_y] = true;
			temp.x_ = temp_x - 1;
			temp.y_ = temp_y;
			open_list.push(temp);
		}
		if (map[temp_x + 1][temp_y] && !answer[temp_x + 1][temp_y])
		{
			answer[temp_x + 1][temp_y] = true;
			temp.x_ = temp_x + 1;
			temp.y_ = temp_y;
			open_list.push(temp);
		}
		if (map[temp_x][temp_y - 1] && !answer[temp_x][temp_y - 1])
		{
			answer[temp_x][temp_y - 1] = true;
			temp.x_ = temp_x;
			temp.y_ = temp_y - 1;
			open_list.push(temp);
		}
		if (map[temp_x][temp_y + 1] && !answer[temp_x][temp_y + 1])
		{
			answer[temp_x][temp_y + 1] = true;
			temp.x_ = temp_x;
			temp.y_ = temp_y + 1;
			open_list.push(temp);
		}
	}
}

void get_reachability_map(int** answer, bool** map, int width, int height, Grid start)
{
	int temp_x, temp_y;
	Grid temp;
	queue<Grid> open_list;
	memset(answer[0], 0xFF, width * height * sizeof(int));
	answer[start.x_][start.y_] = 0;
	open_list.push(start);
	while (!open_list.empty())
	{
		temp_x = open_list.front().x_;
		temp_y = open_list.front().y_;
		open_list.pop();
		if (map[temp_x - 1][temp_y] && answer[temp_x - 1][temp_y] == -1)
		{
			answer[temp_x - 1][temp_y] = answer[temp_x][temp_y] + 1;
			temp.x_ = temp_x - 1;
			temp.y_ = temp_y;
			open_list.push(temp);
		}
		if (map[temp_x + 1][temp_y] && answer[temp_x + 1][temp_y] == -1)
		{
			answer[temp_x + 1][temp_y] = answer[temp_x][temp_y] + 1;
			temp.x_ = temp_x + 1;
			temp.y_ = temp_y;
			open_list.push(temp);
		}
		if (map[temp_x][temp_y - 1] && answer[temp_x][temp_y - 1] == -1)
		{
			answer[temp_x][temp_y - 1] = answer[temp_x][temp_y] + 1;
			temp.x_ = temp_x;
			temp.y_ = temp_y - 1;
			open_list.push(temp);
		}
		if (map[temp_x][temp_y + 1] && answer[temp_x][temp_y + 1] == -1)
		{
			answer[temp_x][temp_y + 1] = answer[temp_x][temp_y] + 1;
			temp.x_ = temp_x;
			temp.y_ = temp_y + 1;
			open_list.push(temp);
		}
	}
}

void get_goal_h_map(unsigned char** answer, bool** map, int width, int height, Grid goal)
{
	int temp_x, temp_y;
	Grid temp;
	queue<Grid> open_list;
	memset(answer[0], 0xFF, width * height);
	answer[goal.x_][goal.y_] = 0;
	open_list.push(goal);
	while (!open_list.empty())
	{
		temp_x = open_list.front().x_;
		temp_y = open_list.front().y_;
		open_list.pop();
		if (map[temp_x - 1][temp_y] && map[temp_x - 2][temp_y] && answer[temp_x - 1][temp_y] > answer[temp_x][temp_y] + 1)
		{
			answer[temp_x - 1][temp_y] = answer[temp_x][temp_y] + 1;
			temp.x_ = temp_x - 1;
			temp.y_ = temp_y;
			open_list.push(temp);
		}
		if (map[temp_x + 1][temp_y] && map[temp_x + 2][temp_y] && answer[temp_x + 1][temp_y] > answer[temp_x][temp_y] + 1)
		{
			answer[temp_x + 1][temp_y] = answer[temp_x][temp_y] + 1;
			temp.x_ = temp_x + 1;
			temp.y_ = temp_y;
			open_list.push(temp);
		}
		if (map[temp_x][temp_y - 1] && map[temp_x][temp_y - 2] && answer[temp_x][temp_y - 1] > answer[temp_x][temp_y] + 1)
		{
			answer[temp_x][temp_y - 1] = answer[temp_x][temp_y] + 1;
			temp.x_ = temp_x;
			temp.y_ = temp_y - 1;
			open_list.push(temp);
		}
		if (map[temp_x][temp_y + 1] && map[temp_x][temp_y + 2] && answer[temp_x][temp_y + 1] > answer[temp_x][temp_y] + 1)
		{
			answer[temp_x][temp_y + 1] = answer[temp_x][temp_y] + 1;
			temp.x_ = temp_x;
			temp.y_ = temp_y + 1;
			open_list.push(temp);
		}
	}
}