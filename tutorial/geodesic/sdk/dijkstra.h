#pragma once
#include <vector>
#include <set>
#include <map>
#include <tuple>
#include <string>
#include <queue>
#include <fstream>
#include<functional>
#include "BaseModel.h"
using namespace std;
vector<int> GetShortestPath(const map<int, set<pair<int, double>>>& graph, int begin, int end);
vector<int> Dijkstra(const CBaseModel& mesh, int begin, int end, vector<CPoint3D> &path);
vector<int> Dijkstra(const CBaseModel& mesh, vector<int> points, vector<CPoint3D> &path);
void Save_path(const string filename, const vector<CPoint3D>& path);