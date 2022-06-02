#pragma once
#include <LBFGS\Non_Linear_Optimization.h>
#include <PQP\Distance_OBB.h>
using namespace std;

class Path_smooth :public Non_Linear_Optimization
{
protected:
	vector<CPoint3D> Sources;
	vector<CPoint3D> Targets;
	vector<CPoint3D> Initial_path;
	double lamda = 1000;
	int m_;
	int n_;
public:
	Path_smooth(const vector<CPoint3D>& path ,int m, int n) :Initial_path(path),m_(m),n_(n) {
		m_parameters.epsilon = 1e-8;//约束条件
		int size = Initial_path.size();
		m_variables.clear();
		for (int i = 0; i < size - 1; i++)
		{
			if (i % n == 0)
			{
				Sources.push_back(Initial_path[i]);
			}
			else
				if (i % n == n - 1)
				{
					Targets.push_back(Initial_path[i]);
				}
				else
				{
					m_variables.push_back(Initial_path[i].x);
					m_variables.push_back(Initial_path[i].y);
					m_variables.push_back(Initial_path[i].z);
				}
		}
	};
	virtual lbfgsfloatval_t evaluate(
		const lbfgsfloatval_t* x,
		lbfgsfloatval_t* g,
		const int n,
		const lbfgsfloatval_t step//步长
	)
	{
		double fx = 0;

		return fx;
	}
};
