#include <igl/readOBJ.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <iostream>
#include <igl/unproject_onto_mesh.h>
#include <set>
#include <map>
#include <tuple>
#include <string>
#include <queue>
#include <fstream>
#include "geodesic/sdk/geodesic2_3.h"
#include"geodesic/sdk/dijkstra.h"
using namespace std;
#define V viewer.data().V
#define F viewer.data().F
bool flag = false;

int main(int argc, char *argv[])
{
	Eigen::MatrixXd C;
	Eigen::MatrixXd VV;
	Eigen::MatrixXi FF;
	CBaseModel model;
	vector<CFace> BaseModel_faces;
	vector<CPoint3D> BaseModel_points;
	vector<int> Select_Point_id;
	vector<CPoint3D> path_dijk;
	vector<CPoint3D> _path_dijk;
	vector<CPoint3D> path;
	int addpoint_num;
	// Init the viewer
	igl::opengl::glfw::Viewer viewer;
	viewer.data().point_size=10;
	// Attach a menu plugin

	//V = viewer.data().V;
	//F = viewer.data().F;
	igl::opengl::glfw::imgui::ImGuiMenu menu;
	viewer.plugins.push_back(&menu);

	// Customize the menu
	double doubleVariable = 0.1f; // Shared between two menus
	int step=0;
	Eigen::MatrixXd p1(1, 3);
	Eigen::MatrixXd p2(1, 3);
	// Add content to the default menu window
	menu.callback_draw_viewer_menu = [&]()
	{
		// Draw parent menu content
		if (ImGui::CollapsingHeader("Mesh", ImGuiTreeNodeFlags_DefaultOpen))
		{
			float w = ImGui::GetContentRegionAvailWidth();
			float p = ImGui::GetStyle().FramePadding.x;
			if (ImGui::Button("Load##Mesh", ImVec2((w - p) / 2.f, 0)))
			{
	
				viewer.open_dialog_load_mesh();
				BaseModel_points.resize(V.rows());
				for (int i = 0; i < V.rows(); i++)
				{
					CPoint3D p(V.row(i)[0], V.row(i)[1], V.row(i)[2]);
					BaseModel_points[i] = p;
				}
				BaseModel_faces.resize(F.rows());
				for (int j = 0; j < F.rows(); j++)
				{
					CFace f(F.row(j)[0], F.row(j)[1], F.row(j)[2]);
					BaseModel_faces[j] = f;
				}
				model.m_Verts = BaseModel_points;
				model.m_Faces = BaseModel_faces;
				model.LoadModel();
			}


			ImGui::SameLine(0, p);
			if (ImGui::Button("Save##Mesh", ImVec2((w - p) / 2.f, 0)))
			{
				viewer.open_dialog_save_mesh();
			}
		}

		// Add new group
		if (ImGui::CollapsingHeader("Geodesic", ImGuiTreeNodeFlags_DefaultOpen))
		{
			// Expose variable directly ...
		//	ImGui::InputDouble("double", &doubleVariable, 0, 0, "%.4f");

			// ... or using a custom callback
			//static bool boolVariable = true;
			//if (ImGui::Checkbox("bool", &boolVariable))
			//{
			//	// do something
			//	std::cout << "boolVariable: " << std::boolalpha << boolVariable << std::endl;
			//}
			//选择点
			if (ImGui::CollapsingHeader("Select Point", ImGuiTreeNodeFlags_DefaultOpen))
			{
				float w = ImGui::GetContentRegionAvailWidth();
				float p = ImGui::GetStyle().FramePadding.x;
				if (ImGui::Button("begin", ImVec2((w - p) / 2.f, 0)))
				{
					flag = true;
				}
				ImGui::SameLine(0, p);
				if (ImGui::Button("end", ImVec2((w - p) / 2.f, 0)))
				{
					flag = false;
					addpoint_num = Select_Point_id.size();
				}
			}
			if (ImGui::CollapsingHeader("Path", ImGuiTreeNodeFlags_DefaultOpen))
			{
				ImGui::InputInt("insert point", &step, 1, 100);
				if (ImGui::Button("initial", ImVec2(-1, 0)))
				{
			
				
					for (int i = 0; i < addpoint_num; i++)
					{
						viewer.data().del_last_points();
					}
					Dijkstra(model, Select_Point_id, path_dijk);



					_path_dijk.clear();
					for (int i = 0; i < path_dijk.size() - 1; i++)
					{
						CPoint3D p1 = path_dijk[i];
						CPoint3D p2 = path_dijk[i + 1];
						for (int j = 0; j < step+1; j++)
						{
							CPoint3D p = p1 + (p2 - p1) / (step + 1) * j;
							_path_dijk.push_back(p);
						}
					}
					_path_dijk.push_back(path_dijk[path_dijk.size()-1]);
					for (int i = 0; i < _path_dijk.size(); i++)
					{
						p1 << _path_dijk[i].x, _path_dijk[i].y, _path_dijk[i].z;
						viewer.data().add_points(p1, Eigen::RowVector3d(0, 0, 1));
					}	
					addpoint_num = _path_dijk.size();
					cout << _path_dijk.size() << endl;
				}
			
				if (ImGui::Button("optimization", ImVec2(-1, 0)))
				{
					for (int i = 0; i < addpoint_num; i++)
					{
						viewer.data().del_last_points();
					}
					cout << _path_dijk.size() << endl;
					Geodesic2_3 our_method(_path_dijk, model);
					int iter = our_method.run();
					path = our_method.getpath();
					cout << path.size() << endl;
					for (int i = 0; i < path.size(); i++)
					{
						p1 << path[i].x, path[i].y, path[i].z;
						viewer.data().add_points(p1, Eigen::RowVector3d(0, 0, 1));
					}
					addpoint_num = path.size();
	
				}
				if (ImGui::Button("save path", ImVec2(-1, 0)))
				{
					std::string fname = igl::file_dialog_save();
					ofstream out(fname);
					for (int i = 0; i < path.size(); i ++)
					{
						out << "v " << path[i];
					}
					out.close();
				}
				
			}
			// Expose an enumeration type
			enum Orientation { Up = 0, Down, Left, Right };
			static Orientation dir = Up;
			ImGui::Combo("Direction", (int *)(&dir), "Up\0Down\0Left\0Right\0\0");

			// We can also use a std::vector<std::string> defined dynamically
			static int num_choices = 3;
			static std::vector<std::string> choices;
			static int idx_choice = 0;

			//全部清除（包括模型）
			if (ImGui::Button("clear", ImVec2(-1, 0))) {
				viewer.data().clear();
				viewer.data().set_mesh(V, F);
				BaseModel_faces.clear();
				BaseModel_points.clear();
				Select_Point_id.clear();
				path_dijk.clear();
				_path_dijk.clear();
			}






			if (num_choices != (int)choices.size())
			{
				choices.resize(num_choices);
				for (int i = 0; i < num_choices; ++i)
					choices[i] = std::string(1, 'A' + i);
				if (idx_choice >= num_choices)
					idx_choice = num_choices - 1;
			}
			ImGui::Combo("Letter", &idx_choice, choices);

		}
	};

	C = Eigen::MatrixXd::Constant(F.rows(), 3, 1);
	//	igl::opengl::glfw::Viewer viewer;
	viewer.callback_mouse_down =
		[&VV, &FF, &C, &Select_Point_id ](igl::opengl::glfw::Viewer& viewer,int, int)->bool
	{

		int fid;
		Eigen::Vector3f bc;
		// Cast a ray in the view direction starting from the mouse position
		double x = viewer.current_mouse_x;
		double y = viewer.core().viewport(3) - viewer.current_mouse_y;

		
		if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core().view,
			viewer.core().proj, viewer.core().viewport, V, F, fid, bc)) {
			if (flag)
			{
				Eigen::MatrixXd point(1, 3);
				Eigen::MatrixXd point_selected(1, 3);
				point << V.row(F.row(fid)[0]) * bc(0) + V.row(F.row(fid)[1]) * bc(1) + V.row(F.row(fid)[2]) * bc(2);
				double dis1 = ((V.row(F.row(fid)[0])[0]) - point.row(0)[0]) * ((V.row(F.row(fid)[0])[0]) - point.row(0)[0])
					+ ((V.row(F.row(fid)[0])[1]) - point.row(0)[1]) * ((V.row(F.row(fid)[0])[1]) - point.row(0)[1])
					+ ((V.row(F.row(fid)[0])[2]) - point.row(0)[2]) * ((V.row(F.row(fid)[0])[2]) - point.row(0)[2]);
				double dis2 = ((V.row(F.row(fid)[1])[0]) - point.row(0)[0]) * ((V.row(F.row(fid)[1])[0]) - point.row(0)[0])
					+ ((V.row(F.row(fid)[1])[1]) - point.row(0)[1]) * ((V.row(F.row(fid)[1])[1]) - point.row(0)[1])
					+ ((V.row(F.row(fid)[1])[2]) - point.row(0)[2]) * ((V.row(F.row(fid)[1])[2]) - point.row(0)[2]);
				double dis3 = ((V.row(F.row(fid)[2])[0]) - point.row(0)[0]) * ((V.row(F.row(fid)[2])[0]) - point.row(0)[0])
					+ ((V.row(F.row(fid)[2])[1]) - point.row(0)[1]) * ((V.row(F.row(fid)[2])[1]) - point.row(0)[1])
					+ ((V.row(F.row(fid)[2])[2]) - point.row(0)[2]) * ((V.row(F.row(fid)[2])[2]) - point.row(0)[2]);
				if (dis1 <= dis2 && dis1 <= dis3) {
					point_selected << V.row(F.row(fid)[0]);
					Select_Point_id.push_back(F.row(fid)[0]);
				}
				if (dis2 <= dis1 && dis2 <= dis3) {
					point_selected << V.row(F.row(fid)[1]);
					Select_Point_id.push_back(F.row(fid)[1]);
				}
				if (dis3 <= dis1 && dis3 <= dis2) {
					point_selected << V.row(F.row(fid)[2]);
					Select_Point_id.push_back(F.row(fid)[2]);
				}
			
				viewer.data().add_points(point_selected, Eigen::RowVector3d(0, 0, 1));
				/**********************************/
				return true;
			}
			return false;
		}
		return false;
	};

		
	
	// Plot the mesh
	viewer.data().set_mesh(V, F);
	viewer.launch();
}
