#include "path_project.h"
#include <algorithm>
vector<CPoint3D>Distance_projection(vector<CPoint3D> thread1, string filename)
{
	const double PI = 3.141592653589793;//π
	vector<CPoint3D>Points, new_thread;//new_thread用来存放新的路径
	vector<int> Faces;
	CBaseModel model(filename);
	model.LoadModel();
	CRichModel model2(filename);
	model2.LoadModel();
	//for (int i = 0; i < model.m_Verts.size(); i++)
	//{
	//	model.m_Verts[i].z = model.m_Verts[i].z * 2;
	//}
	//for (int i = 0; i < model2.m_Verts.size(); i++)
	//{
	//	model2.m_Verts[i].z = model2.m_Verts[i].z * 2;
	//}
	Distance_OBB obb(model);
	//模型所有点、面的信息
	auto points = model.m_Verts;
	auto faces = model.m_Faces;
	Points.resize(points.size());
	Faces.resize(3 * faces.size());
	for (int i = 0; i < Points.size(); i++) {
		Points[i] = points[i];
	}
	for (int i = 0; i < Faces.size() / 3; i++) {
		Faces[3 * i] = faces[i].verts[0];
		Faces[3 * i + 1] = faces[i].verts[1];
		Faces[3 * i + 2] = faces[i].verts[2];
	}
	new_thread.clear();

	CPoint3D pt1 = thread1[0];
	CPoint3D pt2 = thread1[1];
	new_thread.push_back(pt1);
	int FaceID1, FaceID2;

	double t1, t2;
	CPoint3D guodu, jiaodian;
	CEdge oppo_edge, edge;
	int reverse_oppo_edge_id, next_edge_id, next_edge2_id;
	int oppo_edge_id, left_point_ID, right_point_ID;
	CPoint3D left_point, right_point;

	int edgeid = -1;
	int FaceID21, FaceID22;
	vector<CPoint3D> thread2 = thread1;
	for (int i = 0; i < thread2.size() - 1; i++) {
		//cout << "i= " << i << endl;
		int count = 0;
		pt1 = thread2[i];
		pt2 = thread2[i + 1];

		FaceID1 = obb.Query(pt1).triangleID;
		FaceID2 = obb.Query(pt2).triangleID;
		FaceID21 = FaceID2;
		FaceID22 = FaceID2;
		CPoint3D vert_11, vert_12, vert_13;//(一个面的三个点)
		vert_11 = (Points[Faces[3 * FaceID1]]);
		vert_12 = (Points[Faces[3 * FaceID1 + 1]]);
		vert_13 = (Points[Faces[3 * FaceID1 + 2]]);
		vector<CPoint3D> tri1 = { vert_11, vert_12, vert_13 };

		CPoint3D vert_21, vert_22, vert_23;//(一个面的三个点)
		vert_21 = (Points[Faces[3 * FaceID2]]);
		vert_22 = (Points[Faces[3 * FaceID2 + 1]]);
		vert_23 = (Points[Faces[3 * FaceID2 + 2]]);
		vector<CPoint3D> tri2 = { vert_21, vert_22, vert_23 };
		int style1 = Point_style(pt1, tri1);
		int style2 = Point_style(pt2, tri2);
		if (style1 == 10)
		{
			new_thread.push_back(pt1);
			int point_id = model2.GetVertexID(pt1);
			FaceID1 = Check_vertex(model, model2, pt1, pt2, point_id);
		}

		if (style1 == 20)
		{
			new_thread.push_back(pt1);
			FaceID1 = edgeid;
		}
		if (style2 == 10)
		{
			int point_id = model2.GetVertexID(pt2);
			FaceID21 = Check_vertex(model, model2, pt2, pt1, point_id);
			FaceID22 = FaceID21;
		}
		if (style2 == 20)
		{
			vector<int> IDs = Check_edge(model, model2, pt2);
			FaceID21 = IDs.at(0);
			FaceID22 = IDs.at(1);
		}
		if (!(style1 == 10 && style2 == 10))
		{
			vector<CPoint3D> new_thread1;
			if (style2 == 20 && FaceID1 == FaceID21)
			{
				edgeid = FaceID22;
			}
			if (style2 == 20 && FaceID1 == FaceID22)
			{
				edgeid = FaceID21;
			}
			bool flag1 = 0;
			while (!((FaceID1 == FaceID21) || (FaceID1 == FaceID22))){
				//前面一个点所在三角面片 所在的平面

				CPoint3D vert_1, vert_2, vert_3;//(一个面的三个点)
				CPoint3D vec_1, vec_2, vec_3, vec_;//两个向量(用来求三角面片的法向)
		//		cout << Points.size() << " "<< Faces.size() << " "<< Faces[3 * FaceID1 + 1]  << endl;
				vert_1 = (Points[Faces[3 * FaceID1]]);
				vert_2 = (Points[Faces[3 * FaceID1 + 1]]);
				vert_3 = (Points[Faces[3 * FaceID1 + 2]]);
				vec_1 = vert_2 - vert_1;
				vec_2 = vert_3 - vert_2;
				vec_3 = vert_1 - vert_3;
				//求叉乘向量（得到平面法向）
				vec_.x = vec_1.y*vec_2.z - vec_1.z*vec_2.y;
				vec_.y = vec_2.x*vec_1.z - vec_1.x*vec_2.z;
				vec_.z = vec_1.x*vec_2.y - vec_1.y*vec_2.x;
				//平面方程Ax+By+Cz+D=0;A=vec_.x,B=vec_.y,C=vec_.z
				double D = -vec_.x*vert_1.x - vec_.y*vert_1.y - vec_.z*vert_1.z;
				CPoint3D point_pro;//第二个端点在第一个点所在平面上的投影点
				double t = (vec_.x * pt2.x + vec_.y * pt2.y + vec_.z * pt2.z + D) / (vec_.x * vec_.x + vec_.y * vec_.y + vec_.z * vec_.z);
				point_pro.x = pt2.x - vec_.x * t;
				point_pro.y = pt2.y - vec_.y * t;
				point_pro.z = pt2.z - vec_.z * t;
				CPoint3D derta = point_pro - pt1;
				//第一条线段
				t1 = (vert_1.x*(vert_2.y - vert_1.y) - vert_1.y*(vert_2.x - vert_1.x) + pt1.y*(vert_2.x - vert_1.x)
					- pt1.x*(vert_2.y - vert_1.y)) / (derta.x*(vert_2.y - vert_1.y) - derta.y*(vert_2.x - vert_1.x));
				guodu = pt1 + t1 * derta;
				if (vert_2.x - vert_1.x != 0) {
					t2 = (guodu.x - vert_1.x) / (vert_2.x - vert_1.x);
				}
				else if (vert_2.y - vert_1.y != 0) {
					t2 = (guodu.y - vert_1.y) / (vert_2.y - vert_1.y);
				}
				else if (vert_2.z - vert_1.z != 0) {
					t2 = (guodu.z - vert_1.z) / (vert_2.z - vert_1.z);
				}
			//	cout << t1 << " " << t2 << " " << endl;
				if (t1 > 0 && t2 > 0 && t2 < 1) {
					jiaodian = guodu;
					pt1 = jiaodian;
					int index = model2.GetEdgeIndexFromTwoVertices(Faces[3 * FaceID1], Faces[3 * FaceID1 + 1]);
					edge = model2.Edge(index);
					flag1 = 1;
				}
				else {//第二条线段
					t1 = (vert_2.x*(vert_3.y - vert_2.y) - vert_2.y*(vert_3.x - vert_2.x) + pt1.y*(vert_3.x - vert_2.x)
						- pt1.x*(vert_3.y - vert_2.y)) / (derta.x*(vert_3.y - vert_2.y) - derta.y*(vert_3.x - vert_2.x));
					//cout << "第二段 " << derta.x*(vert_3.y - vert_2.y) - derta.y*(vert_3.x - vert_2.x) << endl;
					guodu = pt1 + t1 * derta;

					if (vert_3.x - vert_2.x != 0) {
						t2 = (guodu.x - vert_2.x) / (vert_3.x - vert_2.x);
					}
					else if (vert_3.y - vert_2.y != 0) {
						t2 = (guodu.y - vert_2.y) / (vert_3.y - vert_2.y);
					}
					else if (vert_3.z - vert_2.z != 0) {
						t2 = (guodu.z - vert_2.z) / (vert_3.z - vert_2.z);
					}
				//	cout << "t1: " << t1 << " t2: " << t2 << endl;
					if (t1 > 0 && t2 > 0 && t2 < 1) {
						jiaodian = guodu;
						pt1 = jiaodian;
						int index = model2.GetEdgeIndexFromTwoVertices(Faces[3 * FaceID1 + 1], Faces[3 * FaceID1 + 2]);
						edge = model2.Edge(index);
						flag1 = 1;
					//	cout << "success" << endl;
					}
					else {//第三条线段
						t1 = (vert_3.x*(vert_1.y - vert_3.y) - vert_3.y*(vert_1.x - vert_3.x) + pt1.y*(vert_1.x - vert_3.x)
							- pt1.x*(vert_1.y - vert_3.y)) / (derta.x*(vert_1.y - vert_3.y) - derta.y*(vert_1.x - vert_3.x));
						//cout << "第三段 " << derta.x*(vert_1.y - vert_3.y) - derta.y*(vert_1.x - vert_3.x) << endl;
						guodu = pt1 + t1 * derta;

						if (vert_1.x - vert_3.x != 0) {
							t2 = (guodu.x - vert_3.x) / (vert_1.x - vert_3.x);
						}
						else if (vert_1.y - vert_3.y != 0) {
							t2 = (guodu.y - vert_3.y) / (vert_1.y - vert_3.y);
						}
						else if (vert_1.z - vert_3.z != 0) {
							t2 = (guodu.z - vert_3.z) / (vert_1.z - vert_3.z);
						}
					//	cout << "t1: " << t1 << " t2: " << t2 << endl;
						if (t1 > 0 && t2 > 0 && t2 < 1) {
							jiaodian = guodu;
							pt1 = jiaodian;
							int index = model2.GetEdgeIndexFromTwoVertices(Faces[3 * FaceID1 + 2], Faces[3 * FaceID1]);
							edge = model2.Edge(index);
							flag1 = 1;
					//		cout << "success" << endl;
						}
					}
				}


				int rever_index = edge.indexOfReverseEdge;
				if (rever_index > 0 && rever_index < model2.GetNumOfEdges() * 2)
				{
					int x1 = model2.Edge(rever_index).indexOfLeftVert;
					int x2 = model2.Edge(rever_index).indexOfRightVert;
					CPoint3D x11 = Points[x1];
					CPoint3D x12 = Points[x2];
					edge = model2.Edge(rever_index);
				}
				count++;

				if (count > 10) {

					CPoint3D center = (pt1 + pt2) / 2;
					center = obb.Query(center).closestPnt;
					thread2.insert(thread2.begin() + i + 1, center);
					--i;

					flag1 = 0;

					break;
				}
				else
				{
					FaceID1 = edge.indexOfFrontFace;
					new_thread1.push_back(jiaodian);
				}
			}
			if (style2 == 20 && FaceID1 == FaceID21&&flag1 == 1)
			{
				edgeid = FaceID22;
			}
			if (style2 == 20 && FaceID1 == FaceID22 && flag1 == 1)
			{
				edgeid = FaceID21;
			}
			for (int j = 0; j < new_thread1.size(); ++j)
			{
				new_thread.push_back(new_thread1.at(j));
			}
		}
	}
	new_thread.push_back(thread2[thread2.size() - 1]);
	for (int i = 0; i < new_thread.size() - 1; ++i)
	{
		for (int j = new_thread.size() - 1; j >= i + 1; --j)
		{
			double length = sqrt((new_thread[j] - new_thread[i]) ^ (new_thread[j] - new_thread[i]));
			if (length < 10e-12)
			{
				int t = i + 1;
				for (int k = i + 1; k <= j; k++)
				{
					new_thread.erase(new_thread.begin() + t);
				}
				i--;
				break;
			}
		}
	}
	for (int i = 0; i < new_thread.size() - 2; ++i)
	{
		double lenght1 = sqrt((new_thread[i + 1] - new_thread[i]) ^ (new_thread[i + 1] - new_thread[i]));
		double lenght2 = sqrt((new_thread[i + 2] - new_thread[i]) ^ (new_thread[i + 2] - new_thread[i]));
		if (lenght1 > lenght2)
		{
			new_thread.erase(new_thread.begin() + i + 1);
			i--;
		}
	}
	return new_thread;
}
int Check_vertex(CBaseModel &model, CRichModel& model2, CPoint3D pt, CPoint3D pt2, int pt1_id)
{
	Distance_OBB obb(model);
	vector<CPoint3D>Points;
	vector<int> Faces;
	auto points = model.m_Verts;
	auto faces = model.m_Faces;
	Points.resize(points.size());
	Faces.resize(3 * faces.size());
	for (int i = 0; i < Points.size(); i++) {
		Points[i] = points[i];
	}
	for (int i = 0; i < Faces.size() / 3; i++) {
		Faces[3 * i] = faces[i].verts[0];
		Faces[3 * i + 1] = faces[i].verts[1];
		Faces[3 * i + 2] = faces[i].verts[2];
	}
	vector<pair<int, double> > ID_edge = model2.Neigh(pt1_id);
	double FaceID1;
	double t1, t2;
	CPoint3D guodu, jiaodian;
	CEdge oppo_edge, edge;
	int reverse_oppo_edge_id, next_edge_id, next_edge2_id;
	int oppo_edge_id, left_point_ID, right_point_ID;
	for (int j = 0; j < ID_edge.size(); j++)
	{
		oppo_edge_id = model2.Edge(ID_edge[j].first).indexOfRightEdge;
		oppo_edge = model2.Edge(oppo_edge_id);
		reverse_oppo_edge_id = oppo_edge.indexOfReverseEdge;//对边的反向边的ID
		oppo_edge = model2.Edge(reverse_oppo_edge_id);
		FaceID1 = oppo_edge.indexOfFrontFace;
		//前面一个点所在三角面片 所在的平面
		CPoint3D vert_1, vert_2, vert_3;//(一个面的三个点)
		CPoint3D vec_1, vec_2, vec_3, vec_;//两个向量(用来求三角面片的法向)
		vert_1 = (Points[Faces[3 * FaceID1]]);
		vert_2 = (Points[Faces[3 * FaceID1 + 1]]);
		vert_3 = (Points[Faces[3 * FaceID1 + 2]]);
		vec_1 = vert_2 - vert_1;
		vec_2 = vert_3 - vert_2;
		vec_3 = vert_1 - vert_3;
		//求叉乘向量（得到平面法向）
		vec_.x = vec_1.y*vec_2.z - vec_1.z*vec_2.y;
		vec_.y = vec_2.x*vec_1.z - vec_1.x*vec_2.z;
		vec_.z = vec_1.x*vec_2.y - vec_1.y*vec_2.x;
		//平面方程Ax+By+Cz+D=0;A=vec_.x,B=vec_.y,C=vec_.z
		double D = -vec_.x*vert_1.x - vec_.y*vert_1.y - vec_.z*vert_1.z;
		CPoint3D point_pro;//第二个端点在第一个点所在平面上的投影点
		double t = (vec_.x * pt2.x + vec_.y * pt2.y + vec_.z * pt2.z + D) / (vec_.x * vec_.x + vec_.y * vec_.y + vec_.z * vec_.z);

		point_pro.x = pt2.x - vec_.x * t;
		point_pro.y = pt2.y - vec_.y * t;
		point_pro.z = pt2.z - vec_.z * t;

		CPoint3D derta = point_pro - pt;
		CPoint3D left_point, right_point;
		left_point_ID = model2.Edge(oppo_edge_id).indexOfLeftVert;
		right_point_ID = model2.Edge(oppo_edge_id).indexOfRightVert;
		left_point = Points[left_point_ID];
		right_point = Points[right_point_ID];
		t1 = (left_point.x*(right_point.y - left_point.y) - left_point.y*(right_point.x - left_point.x) + pt.y*(right_point.x - left_point.x)
			- pt.x*(right_point.y - left_point.y)) / (derta.x*(right_point.y - left_point.y) - derta.y*(right_point.x - left_point.x));
		guodu = pt + t1 * derta;

		if (t1 >= 0) {
			if (right_point.x - left_point.x != 0) {
				t2 = (guodu.x - left_point.x) / (right_point.x - left_point.x);
			}
			else if (right_point.y - left_point.y != 0) {
				t2 = (guodu.y - left_point.y) / (right_point.y - left_point.y);
			}
			else if (right_point.z - left_point.z != 0) {
				t2 = (guodu.z - left_point.z) / (right_point.z - left_point.z);
			}
			if (t2 >= 0 && t2 <= 1)
			{
				break;
			}
		}
	}
	return FaceID1;
}
vector<int> Check_edge(CBaseModel &model, CRichModel& model2, CPoint3D pt) {
	Distance_OBB obb(model);
	vector<CPoint3D>Points;
	vector<int> Faces;
	vector<int> FaceIDs;
	int FaceID;
	auto points = model.m_Verts;
	auto faces = model.m_Faces;
	Points.resize(points.size());
	Faces.resize(3 * faces.size());
	for (int i = 0; i < Points.size(); i++) {
		Points[i] = points[i];
	}
	for (int i = 0; i < Faces.size() / 3; i++) {
		Faces[3 * i] = faces[i].verts[0];
		Faces[3 * i + 1] = faces[i].verts[1];
		Faces[3 * i + 2] = faces[i].verts[2];
	}
	FaceID = obb.Query(pt).triangleID;
	CPoint3D vert_1, vert_2, vert_3;//(一个面的三个点)
	vert_1 = (Points[Faces[3 * FaceID]]);
	vert_2 = (Points[Faces[3 * FaceID + 1]]);
	vert_3 = (Points[Faces[3 * FaceID + 2]]);
	int id1, id2, id3;
	id1 = Faces[3 * FaceID];
	id2 = Faces[3 * FaceID + 1];
	id3 = Faces[3 * FaceID + 2];
	vector<double> edgevec_1 = { vert_1.x - vert_2.x,vert_1.y - vert_2.y,vert_1.z - vert_2.z };
	vector<double> edgevec_2 = { vert_2.x - vert_3.x,vert_2.y - vert_3.y,vert_2.z - vert_3.z };
	vector<double> edgevec_3 = { vert_3.x - vert_1.x,vert_3.y - vert_1.y,vert_3.z - vert_1.z };

	CPoint3D edgepoint_1(vert_1.x - vert_2.x, vert_1.y - vert_2.y, vert_1.z - vert_2.z);
	CPoint3D edgepoint_2(vert_2.x - vert_3.x, vert_2.y - vert_3.y, vert_2.z - vert_3.z);
	CPoint3D edgepoint_3(vert_3.x - vert_1.x, vert_3.y - vert_1.y, vert_3.z - vert_1.z);
	double  angle;
	vector <double> vec;
	vec = { pt.x - vert_2.x,pt.y - vert_2.y,pt.z - vert_2.z };
	CPoint3D p1(pt.x - vert_2.x, pt.y - vert_2.y, pt.z - vert_2.z);
	double dot1 = Dot(edgevec_1, vec);
	double a1 = sqrt(edgepoint_1^edgepoint_1);
	double b1 = sqrt(p1^p1);
	angle = dot1 / a1 / b1;
	if (abs(angle - 1) < 10e-6 || abs(angle + 1) < 10e-6)
	{
		int edgeid1 = model2.GetEdgeIndexFromTwoVertices(id1, id2);
		auto oppo_edge1 = model2.Edge(edgeid1);
		int faceid1 = oppo_edge1.indexOfFrontFace;
		int edgeid2 = model2.GetEdgeIndexFromTwoVertices(id2, id1);
		auto oppo_edge2 = model2.Edge(edgeid2);
		int faceid2 = oppo_edge2.indexOfFrontFace;
		FaceIDs = { faceid1 ,faceid2 };
	}

	vec = { pt.x - vert_3.x,pt.y - vert_3.y,pt.z - vert_3.z };
	CPoint3D p2(pt.x - vert_3.x, pt.y - vert_3.y, pt.z - vert_3.z);
	double dot2 = Dot(edgevec_2, vec);
	double a2 = sqrt(edgepoint_2^edgepoint_2);
	double b2 = sqrt(p2^p2);
	angle = dot2 / a2 / b2;
	if (abs(angle - 1) < 10e-6 || abs(angle + 1) < 10e-6)
	{
		int edgeid1 = model2.GetEdgeIndexFromTwoVertices(id2, id3);
		auto oppo_edge1 = model2.Edge(edgeid1);
		int faceid1 = oppo_edge1.indexOfFrontFace;
		int edgeid2 = model2.GetEdgeIndexFromTwoVertices(id3, id2);
		auto oppo_edge2 = model2.Edge(edgeid2);
		int faceid2 = oppo_edge2.indexOfFrontFace;
		FaceIDs = { faceid1 ,faceid2 };

	}

	vec = { pt.x - vert_1.x,pt.y - vert_1.y,pt.z - vert_1.z };
	CPoint3D p3(pt.x - vert_1.x, pt.y - vert_1.y, pt.z - vert_1.z);
	double dot3 = Dot(edgevec_3, vec);
	double a3 = sqrt(edgepoint_3^edgepoint_3);
	double b3 = sqrt(p3^p3);
	angle = dot3 / a3 / b3;
	if (abs(angle - 1) < 10e-6 || abs(angle + 1) < 10e-6)
	{
		int edgeid1 = model2.GetEdgeIndexFromTwoVertices(id3, id1);
		auto oppo_edge1 = model2.Edge(edgeid1);
		int faceid1 = oppo_edge1.indexOfFrontFace;
		int edgeid2 = model2.GetEdgeIndexFromTwoVertices(id1, id3);
		auto oppo_edge2 = model2.Edge(edgeid2);
		int faceid2 = oppo_edge2.indexOfFrontFace;
		FaceIDs = { faceid1 ,faceid2 };
	}
	return FaceIDs;
}
int Point_style(CPoint3D p, vector<CPoint3D> tri)
{
	// 1:vertices;
	// 2:edge;
	// 3:in face;
	if (tri.size() != 3)
		return -1;
	for(int i = 0; i< 3; i++)
	{
		if ((p - tri[i]).Len() < 1e-6)
			return 10;
	}
	CPoint3D vec = p - tri[0];
	CPoint3D edge1, edge2, edge3;//(一个面的三个点)
	edge1 = tri[0] - tri[1];
	edge2 = tri[1] - tri[2];
	edge3 = tri[0] - tri[3];
	double angle1 = CPoint_Dot(vec, edge1) / (Normal(vec), Normal(edge1));
	double angle2 = CPoint_Dot(vec, edge2) / (Normal(vec), Normal(edge2));
	double angle3 = CPoint_Dot(vec, edge3) / (Normal(vec), Normal(edge3));
	if (abs(angle1 - 1) < 10e-6 || abs(angle1 + 1) < 10e-6)
		return 20;
	if (abs(angle2 - 1) < 10e-6 || abs(angle2 + 1) < 10e-6)
		return 20;
	if (abs(angle3 - 1) < 10e-6 || abs(angle3 + 1) < 10e-6)
		return 20;
	return 30;
}
double CPoint_Dot(CPoint3D v1, CPoint3D v2)
{
	return v1.x*v2.x + v1.y * v2.y + v1.z * v2.z;
}
CPoint3D CPoint_Cross(CPoint3D v1, CPoint3D v2)
{
	return CPoint3D(v1.y*v2.z - v2.y*v1.z, v2.x*v1.z - v1.x*v2.z, v1.x*v2.y - v2.x*v1.y);
}
void SaveThread2(const vector<CPoint3D>& thread1, string filename) //保存纱线的点的值
{
	ofstream out(filename);
	out << "g thread\n";
	for (int i = 0; i < thread1.size(); ++i)
		out << "v " << thread1[i].x << " " << thread1[i].y << " " << thread1[i].z << endl;
	for (int i = 0; i < thread1.size() - 1; ++i)
		out << "l " << i + 1 << " " << i + 2 << endl;
	out.close();
}
double Distance(const vector<CPoint3D> &path)
{
	double sum = 0;
	for (int i = 0; i < path.size() - 1; i++)
	{
		sum += (path[i + 1] - path[i]).Len();
	}
	return sum;
}

CBaseModel sub_model(const CBaseModel &model, vector<int> inital_path, int num)
{
	set<CFace> m_faces;
	for (int i = 0; i < model.GetNumOfFaces(); i++)
	{
	  auto face = model.m_Faces[i];
	  for (int j = 0; j < 3; j++)
	  {
		  int id = face.verts[j];
		  auto it = find(inital_path.begin(), inital_path.end(),id);
		  if (it != inital_path.end())
			  m_faces.insert(face);
	  }
	}
	vector<CFace> sub_faces;
	for (auto it = m_faces.begin(); it != m_faces.end(); it++)
	{
		sub_faces.push_back(*it);
	}
	CBaseModel sub_mesh(model.m_Verts, sub_faces);
	return sub_mesh;
}
double Normal(CPoint3D vec)
{
	return sqrt(vec^vec);
}