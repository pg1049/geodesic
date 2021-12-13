#include "RichModel.h"



CPoint3D GetBasicNormal(const CRichModel& model)
{
	CPoint3D result(0, 0, 0);
	for (int i = 0; i < model.GetNumOfVerts(); ++i)
		result = result + model.Normal(i);
	result.Normalize();
	return result;
}

bool NeedFlip(const vector<EdgePoint>& loop, CPoint3D normal, const CRichModel& model)
{
	CPoint3D crossProduct_sum(0, 0, 0);
	for (int i = 0; i < loop.size(); ++i)
	{
		CPoint3D pre = loop[(i - 1 + loop.size()) % loop.size()].Get3DPoint(model);
		CPoint3D nxt = loop[(i + 1 + loop.size()) % loop.size()].Get3DPoint(model);
		CPoint3D cur = loop[(i + loop.size()) % loop.size()].Get3DPoint(model);
		crossProduct_sum = crossProduct_sum + VectorCross(pre, cur, nxt);
	}
	return (crossProduct_sum ^ normal) < 0;
}