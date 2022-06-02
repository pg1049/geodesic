#pragma once
#include "Distance_OBB.h"
#include "VectorOperations.h"
#include "RichModel.h"
#include <iostream>
#include <fstream>
typedef CRichModel::CEdge CEdge;
typedef CBaseModel::CFace CFace;
using namespace std;
vector<CPoint3D>Distance_projection(vector<CPoint3D> thread1, string filename);
int Check_vertex(CBaseModel &model, CRichModel& model2, CPoint3D pt, CPoint3D pt2, int pt1_id);
vector<int> Check_edge(CBaseModel &model, CRichModel& model2, CPoint3D pt);
int Point_style(CPoint3D p, vector<CPoint3D> tri);
CPoint3D CPoint_Cross(CPoint3D v1, CPoint3D v2);
double CPoint_Dot(CPoint3D v1, CPoint3D v2);
double Normal(CPoint3D vec);
void SaveThread2(const vector<CPoint3D>& thread1,  string filename);
double Distance(const vector<CPoint3D> &path);
CBaseModel sub_model(const CBaseModel &model, vector<int> inital_path, int num);
