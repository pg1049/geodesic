#pragma once
#include "BaseModel.h"
#include "PQP.h"
class Distance_OBB
{
	const CBaseModel& m_model;
	PQP_Model m_pqp_model;
public:
	struct QueryResult
	{
		int triangleID;
		double distance;
		CPoint3D closestPnt;
		QueryResult(){}
		QueryResult(int triangleID, double distance, const CPoint3D &closestPnt) 
			: triangleID(triangleID), distance(distance), closestPnt(closestPnt)
		{
		}
		bool operator<(const QueryResult& other) const
		{
			if (distance < other.distance)
				return true;
			if (distance > other.distance)
				return false;
			if (triangleID < other.triangleID)
				return true;
			if (triangleID > other.triangleID)
				return false;
			return false;
		}
	};
public:
	Distance_OBB(const CBaseModel& model);
	QueryResult Query(const CPoint3D& pt);
	vector<QueryResult> Query(const CPoint3D& pt, const CPoint3D& dir);
};

