#include <cstdlib>
#include "CombIndex.h"

CombIndex::CombIndex(int n, int m)
{
	int i, j;
	n_ = n;
	m_ = m;
	comb_ = (__int64**)malloc((n + 1) * sizeof(__int64*));
	comb_[0] = (__int64*)malloc((n + 1) * (m + 1) * sizeof(__int64));
	for (i = 1; i <= n; i++)
	{
		comb_[i] = comb_[i - 1] + (m + 1);
	}
	for (i = 0; i <= m; i++)
	{
		for (j = i; j <= n; j++)
		{
			comb_[j][i] = comb(j, i);
		}
	}
}

CombIndex::~CombIndex()
{
	free(comb_[0]);
	free(comb_);
}

__int64 CombIndex::comb(int n, int m)
{
	if (m > n / 2)
	{
		m = n - m;
	}
	__int64 ans = 1;
	for (int i = 1; i <= m; i++)
	{
		ans *= (n - i + 1);
		ans /= i;
	}
	return ans;
}

__int64 CombIndex::get_comb_index(int* c)
{
	__int64 ans = 0;
	int l = n_;
	for (int i = 0; i < m_; i++)
	{
		ans += comb_[l][m_ - i] - comb_[n_ - c[i]][m_ - i];
		l = n_ - c[i] - 1;
	}
	return ans;
}