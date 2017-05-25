#ifndef COMBINDEX_H
#define COMBINDEX_H

class CombIndex
{
public:
	CombIndex(int, int);
	~CombIndex();
	/* calculate the number of combinations selecting n elements from size-m-set */
	__int64 comb(int n, int m);
	/***
	* calculate the index of a combination
	* parameter c is a combination of m_ elements selecting from {0, 1,..., n_ - 1} in ascending order
	* return the lexicographic order of c in all same size combinations
	***/
	__int64 get_comb_index(int* c);
private:
	int n_, m_;
	__int64** comb_;
};

#endif