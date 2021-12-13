#pragma once
#include "Point3D.h"
#include "BaseModel.h"
//z: from 0 to l
class Cylinder
{
public:
	static CBaseModel CreateCylinder(double r, double l, int segNumOfSection)
	{
		vector<CPoint3D> verts;
		vector<CBaseModel::CFace> faces;

		int segNumOfAxis = int(segNumOfSection / (2 * 3.1415926 * r) * l + 0.5);
		for (int i = 0; i < segNumOfAxis; ++i)
		{
			double zless = i * l / (double)segNumOfAxis;
			double zlarger = (i + 1) * l / (double)segNumOfAxis;
			for (int j = 0; j < segNumOfSection; ++j)
			{
				double theta_less = j * 2 * 3.14159265 / segNumOfSection;
				double theta_larger = (j + 1) * 2 * 3.14159265 / segNumOfSection;
				double x_less = r * cos(theta_less);
				double x_larger = r * cos(theta_larger);
				double y_less = r * sin(theta_less);
				double y_larger = r * sin(theta_larger);
				CPoint3D pt1(x_less, y_less, zless);
				CPoint3D pt2(x_larger, y_larger, zless);
				CPoint3D pt3(x_less, y_less, zlarger);
				CPoint3D pt4(x_larger, y_larger, zlarger);
				int id = verts.size();
				verts.push_back(pt1);
				verts.push_back(pt2);
				verts.push_back(pt3);
				verts.push_back(pt4);
				faces.push_back(CBaseModel::CFace(id, id + 1, id + 2));
				faces.push_back(CBaseModel::CFace(id + 2, id + 1, id + 3));
			}
		}

		CBaseModel model("");
		model.m_Verts = verts;
		model.m_Faces = faces;
		return model;
	}
};