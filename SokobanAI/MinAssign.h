#ifndef MINASSIGN_H
#define MINASSIGN_H

#define LARGE		0xFF

class BiGraph
{
public:
	BiGraph(int);
	~BiGraph();
	int		num_;			//number of left-side-vertexes(equals to right-side-vertexes)
	int*	left_link_;		//index of right-side-vertex matched with left-side-vertex
	int*	right_link_;	//index of left-side-vertex matched with right-side-vertex
	bool*	left_cover_;	//covered status of left-side-vertex
	bool*	right_cover_;	//covered status of right-side-vertex
	bool**	edge_;			//adjacency matrix
	int		max_match();	//maximum edge matching
	void	min_cover();	//minimum vertex covering
private:
	bool*	visited_;		//visited status of right-side-vertex while searching augmented rail
	bool	find_augmented_rail(int);		//search augmented rail originated from left-side-vertex	
};

class MinAssign
{
public:
	MinAssign(int);
	~MinAssign();
	int		num_;			//number of people(tasks)
	int**	cost_;			//cost-matrix
	BiGraph* bigraph_;
	int		get_min_cost();
private:
	int**	trans_;			//transformed cost-matrix
};

#endif