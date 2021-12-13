#pragma once
#include "Non_Linear_Optimization.h"
#include <Eigen\LU>
#include "Distance_OBB.h"
#include "path_project.h"
using namespace std;
using namespace Eigen;

class Geodesic2_3 :public Non_Linear_Optimization
{
protected:
	CPoint3D Source;
	CPoint3D Target;
	vector<CPoint3D> Initial_path;
	vector<CPoint3D> normals;
	CBaseModel model;
	Distance_OBB obb;
	double length;
	vector<double> surface;
public:
	Geodesic2_3(const vector<CPoint3D> &path, const CBaseModel& mesh) :
		Initial_path(path), model(mesh), obb(model){
		m_parameters.epsilon = 1e-8;//约束条件
		int size = Initial_path.size();
		m_variables.resize((size - 2) * 3);
		surface.resize((size - 2) * 3);
		normals.resize(size - 2);
		for (int i = 1; i < size - 1; i++)
		{
			m_variables[3 * (i - 1)] = Initial_path[i].x;
			m_variables[3 * (i - 1) + 1] = Initial_path[i].y;
			m_variables[3 * (i - 1) + 2] = Initial_path[i].z;
		}
		Source = path[0];
		Target = path[size - 1];
		length = 0;
	};
	virtual lbfgsfloatval_t evaluate(
		const lbfgsfloatval_t *x,
		lbfgsfloatval_t *g,
		const int n,
		const lbfgsfloatval_t step//步长
	)
	{
		//投影
		for (int i = 0; i < n; i = i + 3)
		{
			CPoint3D p(x[i], x[i + 1], x[i + 2]);
			auto query = obb.Query(p);
			p = query.closestPnt;
			m_variables[i] = p.x;
			m_variables[i + 1] = p.y;
			m_variables[i + 2] = p.z;

			int tri_id = query.triangleID;
			CPoint3D p0 = model.m_Verts[model.m_Faces[tri_id].verts[0]];
			CPoint3D p1 = model.m_Verts[model.m_Faces[tri_id].verts[1]];
			CPoint3D p2 = model.m_Verts[model.m_Faces[tri_id].verts[2]];
			CPoint3D n = CPoint_Cross(p1 - p0, p2 - p0);
			normals[i / 3] = n;
		}
		//计算sum:+=e ^ (lamda *(||x_(i+1) - x_i||^2))
		double fx = 0;
		for (int i = 0; i < n; i = i + 3)
		{
			if (i == 0)
			{
				CPoint3D p(m_variables[0], m_variables[1], m_variables[2]);
				fx += exp((p - Source) ^ (p - Source));
			}
			if (i >= 0 && i < n - 3)
			{
				CPoint3D p1(m_variables[i], m_variables[i + 1], m_variables[i + 2]);
				CPoint3D p2(m_variables[i + 3], m_variables[i + 4], m_variables[i + 5]);
				fx += exp((p2 - p1) ^ (p2 - p1));
			}
			if (i == n - 3)
			{
				CPoint3D p1(m_variables[i], m_variables[i + 1], m_variables[i + 2]);
				fx += exp((Target - p1) ^ (Target - p1));
			}
		}
		//计算梯度: g = 2 * lamda * (x_(i+1) - x_i) * e ^ (lamda *(||x_(i+1) - x_i||^2))
		for (int i = 0; i < n; i = i + 3)
		{
			CPoint3D grad;
			CPoint3D p(m_variables[i], m_variables[i + 1], m_variables[i + 2]);
			if (i == 0)
			{
				CPoint3D p_after(m_variables[3], m_variables[4], m_variables[5]);
				grad = 2 * (p - Source) * exp((p - Source) ^ (p - Source))
					+ 2 * (p - p_after) * exp((p - p_after) ^ (p - p_after));
			}
			if (i >= 3 && i < n - 3)
			{
				CPoint3D p_befor(m_variables[i - 3], m_variables[i - 2], m_variables[i - 1]);
				CPoint3D p_after(m_variables[i + 3], m_variables[i + 4], m_variables[i + 5]);
				grad = 2  * (p - p_befor) * exp((p - p_befor) ^ (p - p_befor))
					+ 2  * (p - p_after) * exp((p - p_after) ^ (p - p_after));
			}
			if (i == n - 3)
			{
				CPoint3D p_befor(m_variables[i - 3], m_variables[i - 2], m_variables[i - 1]);
				grad = 2  * (p - p_befor) * exp( (p - p_befor) ^ (p - p_befor))
					+ 2  * (p - Target) * exp((p - Target) ^ (p - Target));
			}
			grad = grad - CPoint_Dot(grad, normals[i / 3]) / (normals[i / 3].Len() * normals[i / 3].Len()) * normals[i / 3];
		//	cout << grad << endl;
			g[i] = grad.x;
			g[i + 1] = grad.y;
			g[i + 2] = grad.z;
		}

		return fx;
	}
	vector<CPoint3D> getpath()
	{
		vector<CPoint3D> Final_path;
		Final_path.push_back(Source);
		for (int i = 0; i < m_variables.size(); i = i + 3)
		{
			CPoint3D p(m_variables[i], m_variables[i + 1], m_variables[i + 2]);
			p = obb.Query(p).closestPnt;
			Final_path.push_back(p);
		}
		Final_path.push_back(Target);
		return Final_path;
	}
	void Save_path(string outpath)
	{
		ofstream out(outpath);
		out << "v " << Source;
		for (int i = 0; i < m_variables.size(); i = i + 3)
		{
			CPoint3D p(m_variables[i], m_variables[i + 1], m_variables[i + 2]);
			p = obb.Query(p).closestPnt;
			out << "v " << p;
		}
		out << "v " << Target;
		for (int i = 0; i <= m_variables.size() / 3; i++)
		{
			out << "l " << i+1 << " " << i + 2 << endl;
		}
	}
};
