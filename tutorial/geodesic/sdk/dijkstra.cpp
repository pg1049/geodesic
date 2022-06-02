#include "dijkstra.h"
//dijkstraÀ„∑®
vector<int> GetShortestPath(const map<int, set<pair<int, double>>>& graph, int begin, int end)
{
	if (begin == end)
		return vector<int>();
	//total weight; self; parent;
	struct Item : public tuple<double, int, int>
	{
		Item() {}
		Item(double a, int b, int c) : tuple<double, int, int>(a, b, c) {}
		bool operator<(const Item& other) const
		{
			return make_pair(get<0>(*this), make_pair(get<1>(*this), get<2>(*this))) < make_pair(get<0>(other), make_pair(get<1>(other), get<2>(other)));
		}
		bool operator>(const Item& other) const
		{
			return make_pair(get<0>(*this), make_pair(get<1>(*this), get<2>(*this))) > make_pair(get<0>(other), make_pair(get<1>(other), get<2>(other)));
		}
	};
	priority_queue<Item, vector<Item>, greater<Item>> pending;
	set<int> fixed;
	map<int, int> self2parent;
	pending.push(Item(0, begin, -1));
	while (!pending.empty())
	{
		auto top = pending.top();
		pending.pop();
		int self = get<1>(top);
		int parent = get<2>(top);
		if (fixed.find(self) != fixed.end())
			continue;
		fixed.insert(self);
		self2parent[self] = parent;
		if (self == end)
			break;
		for (auto neigh : graph.find(self)->second)
		{
			if (fixed.find(neigh.first) != fixed.end())
				continue;
			int neighID = neigh.first;
			double edgeW = neigh.second;
			pending.push(Item(edgeW + get<0>(top), neighID, self));
		}
	}
	vector<int> path;
	path.push_back(end);
	while (true)
	{
		int parent = self2parent[path.back()];
		if (parent == -1)
			break;
		path.push_back(parent);
	}
	reverse(path.begin(), path.end());
	return path;
}
vector<int> Dijkstra(const CBaseModel& mesh, int begin, int end, vector<CPoint3D> &path)
{
	map<int, set<pair<int, double>>> graph;
	for (auto face : mesh.m_Faces)
	{
		double weight1 = (mesh.m_Verts[face[0]] - mesh.m_Verts[face[1]]).Len();
		graph[face[0]].insert(make_pair(face[1], weight1));
		graph[face[1]].insert(make_pair(face[0], weight1));

		double weight2 = (mesh.m_Verts[face[0]] - mesh.m_Verts[face[2]]).Len();
		graph[face[0]].insert(make_pair(face[2], weight2));
		graph[face[2]].insert(make_pair(face[0], weight2));

		double weight3 = (mesh.m_Verts[face[1]] - mesh.m_Verts[face[2]]).Len();
		graph[face[1]].insert(make_pair(face[2], weight3));
		graph[face[2]].insert(make_pair(face[1], weight3));
	}
	vector<int> pathid;
	pathid.clear();
	pathid = GetShortestPath(graph, begin, end);
	path.resize(pathid.size());
	for (int i = 0; i < pathid.size(); i++)
	{
		path[i] = mesh.m_Verts[pathid[i]];
	}
	return pathid;
}
void Save_path(const string filename, const vector<CPoint3D>& path)
{
	ofstream out(filename);
	for (int i = 0; i < path.size(); i++)
	{
		out << "v " << path[i];
	}
	for (int i = 0; i < path.size() - 1; i++)
	{
		out << "l " << i + 1 << " " << i + 2 << endl;
	}
	out.close();
}
vector<int> Dijkstra(const CBaseModel& mesh, vector<int> points, vector<CPoint3D>& path)
{
	vector<int> path_dijk_id;
	vector<CPoint3D> path_dijk;
	for (int i = 0; i < points.size()-1; i++)
	{
		Dijkstra(mesh, points[i], points[i+1], path_dijk);
		for (int j = 0; j < path_dijk.size() - 1; j++)
		{
			path.push_back(path_dijk[j]);
		}
		if (i == points.size() - 2)
		{
			path.push_back(path_dijk[path_dijk.size() - 1]);
		}
	}
	return path_dijk_id;
}