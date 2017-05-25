#include <queue>
#include "MinAssign.h"

using namespace std;

BiGraph::BiGraph(int num)
{
	left_link_ = (int*)malloc(sizeof(int) * num);
	right_link_ = (int*)malloc(sizeof(int) * num);
	left_cover_ = (bool*)malloc(num);
	right_cover_ = (bool*)malloc(num);
	visited_ = (bool*)malloc(num);
	edge_ = (bool**)malloc(sizeof(bool*) * num);
	edge_[0] = (bool*)malloc(num * num);
	for (int i = 1; i < num; i++)
	{
		edge_[i] = edge_[i - 1] + num;
	}
}

BiGraph::~BiGraph()
{
	free(left_link_);
	free(right_link_);
	free(left_cover_);
	free(right_cover_);
	free(visited_);
	free(edge_[0]);
	free(edge_);
}

bool BiGraph::find_augmented_rail(int v)
{
	for (int i = 0; i < num_; i++)
	{
		if (edge_[v][i] && !visited_[i])
		{
			visited_[i] = true;
			if (right_link_[i] == -1 || find_augmented_rail(right_link_[i]))
			{
				right_link_[i] = v;
				return true;
			}
		}
	}
	return false;
}

int BiGraph::max_match()
{
	int i, match_num = 0;
	memset(right_link_, 0xFF, sizeof(int) * num_);	//set right_link_ all to -1
	/* get right_link_ */
	for (i = 0; i < num_; i++)
	{
		memset(visited_, 0, num_);
		if (find_augmented_rail(i))
		{
			match_num++;
		}
	}
	memset(left_link_, 0xFF, sizeof(int) * num_);	//set left_link_ all to -1
	/* get left_link_ */
	for (i = 0; i < num_; i++)
	{
		if (right_link_[i] >= 0)
		{
			left_link_[right_link_[i]] = i;
		}
	}
	return match_num;
}

/***
* minimum vertex covering problem of bipartite graph
***/
/* NOTICE£ºto run min_cover requires max_match executed */
void BiGraph::min_cover()
{
	int i, j, k, temp;
	queue<int> q;
	memset(left_cover_, 0, num_);
	memset(right_cover_, 0, num_);
	/* traverse all edges */
	for (i = 0; i < num_; i++)
	{
		for (j = 0; j < num_; j++)
		{
			if (edge_[i][j])
			{
				/* if the right vertex of an edge is not matched, then cover the left vertex of this edge. */
				if (right_link_[j] == -1 && !left_cover_[i])
				{
					left_cover_[i] = true;
					q.push(i);
					while (!q.empty())
					{
						temp = left_link_[q.front()];
						q.pop();
						for (k = 0; k < num_; k++)
						{
							/* if the right vertex of an edge is matched with a covered vertex, then cover the left vertex of this edge. */
							if (edge_[k][temp] && !left_cover_[k])
							{
								left_cover_[k] = true;
								q.push(k);
							}
						}
					}
				}
				/* if the left vertex of an edge is not matched, then cover the right vertex of this edge. */
				if (left_link_[i] == -1 && !right_cover_[j])
				{
					right_cover_[j] = true;
					q.push(j);
					while (!q.empty())
					{
						temp = right_link_[q.front()];
						q.pop();
						for (k = 0; k < num_; k++)
						{
							/* if the left vertex of an edge is matched with a covered vertex, then cover the right vertex of this edge. */
							if (edge_[temp][k] && !right_cover_[k])
							{
								right_cover_[k] = true;
								q.push(k);
							}
						}
					}
				}
			}
		}
	}
	/* cover the left vertexes of isolated edges */
	for (i = 0; i < num_; i++)
	{
		if (left_link_[i] >= 0 && !left_cover_[i] && !right_cover_[left_link_[i]])
		{
			left_cover_[i] = true;
		}
	}
}

MinAssign::MinAssign(int num)
{
	cost_ = (int**)malloc(sizeof(int*) * num);
	trans_ = (int**)malloc(sizeof(int*) * num);
	cost_[0] = (int*)malloc(sizeof(int) * num * num);
	trans_[0] = (int*)malloc(sizeof(int) * num * num);
	for (int i = 1; i < num; i++)
	{
		cost_[i] = cost_[i - 1] + num;
		trans_[i] = trans_[i - 1] + num;
	}
	bigraph_ = new BiGraph(num);
}

MinAssign::~MinAssign()
{
	free(cost_[0]);
	free(trans_[0]);
	free(cost_);
	free(trans_);
	delete bigraph_;
}

/***
* minimum assignment problem
***/
/* NOTICE: to run get_min_cost requires initialization of num_ and cost_ */
int MinAssign::get_min_cost()
{
	int i, j, min, cover_num, ans = 0;
	/* subtract minimum element per row */
	for (i = 0; i < num_; i++)
	{
		min = LARGE;
		for (j = 0; j < num_; j++)
		{
			if (cost_[i][j] < min)
			{
				min = cost_[i][j];
			}
		}
		for (j = 0; j < num_; j++)
		{
			trans_[i][j] = cost_[i][j] - min;
		}
		ans += min;
	}
	/* subtract minimum element per column */
	for (i = 0; i < num_; i++)
	{
		min = LARGE;
		for (j = 0; j < num_; j++)
		{
			if (trans_[j][i] < min)
			{
				min = trans_[j][i];
			}
		}
		for (j = 0; j < num_; j++)
		{
			trans_[j][i] -= min;
		}
		ans += min;
	}
	bigraph_->num_ = num_;
	do
	{
		for (i = 0; i < num_; i++)
		{
			for (j = 0; j < num_; j++)
			{
				bigraph_->edge_[i][j] = trans_[i][j] == 0;
			}
		}
		cover_num = bigraph_->max_match();
		if (cover_num != num_)	//the number of independent-zeroes is less than the number of rows
		{
			/* get minimum zero-covering-lines */
			bigraph_->min_cover();
			/* get minimum uncovered element */
			min = LARGE;
			for (i = 0; i < num_; i++)
			{
				for (j = 0; j < num_; j++)
				{
					if (!bigraph_->left_cover_[i] && !bigraph_->right_cover_[j] && trans_[i][j] < min)
					{
						min = trans_[i][j];
					}
				}
			}
			/* subtract this element from transformed cost-matrix, and add this element to each zero-covering-line */
			for (i = 0; i < num_; i++)
			{
				for (j = 0; j < num_; j++)
				{
					if (!bigraph_->left_cover_[i] && !bigraph_->right_cover_[j])
					{
						trans_[i][j] -= min;
					}
					else if (bigraph_->left_cover_[i] && bigraph_->right_cover_[j])
					{
						trans_[i][j] += min;
					}
				}
			}
			ans += (num_ - cover_num) * min;
		}
	} while (cover_num != num_);	//repeat till the number of independent-zeroes and rows are equal
	return ans;
}