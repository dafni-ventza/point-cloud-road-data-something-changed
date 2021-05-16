#include "SceneMesh3D.h"

#include <SDKDDKVer.h>
#include <Windows.h>
#include <stdio.h>
#include <string.h>


#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/core/core.hpp>



#define SPLIT_INSTEAD_OF_INTERSECT 1
#define FLAG_SHOW_CANVAS 1

using namespace std;
using namespace vvr;
using namespace cv;

Mesh3DScene::Mesh3DScene()
{
	//! Load settings.
	vvr::Shape::DEF_LINE_WIDTH = 4;
	vvr::Shape::DEF_POINT_SIZE = 3;
	m_perspective_proj = true;
	m_bg_col = Colour("768E77");
	
	reset();
}


//load_point_cloud
// void Mesh3DScene::load_point_cloud() {
// 
//	// load point cloud
//	const string pcldir = getBasePath() + "resources/extracted_las_files/0_5D4KVPBP/2016/";
//	
//	vector<std::string> pclFiles;
//	//Number of point cloud bin files in the folder
//	const size_t numberOfFiles = 11;
//
//	for (size_t i = 0; i < 1; i++)
//	{
//		pclFiles.push_back(pcldir + "0_5D4KVPBP_" + to_string(i) + "_rgb_d.bin");
//
//		vec cm(0, 0, 0);
//		fstream input(pclFiles[i].c_str(), ios::in | ios::binary);
//		if (!input.good()) {
//			cerr << "Could not read file: " << pclFiles[i] << endl;
//			exit(EXIT_FAILURE);
//		}
//		input.seekg(0, ios::beg);
//		int j;
//		for (j = 0; input.good() && !input.eof(); j++) {
//			vec point;
//			float intensity;
//			input.read((char *)&point.x, 3 * sizeof(float));
//			input.read((char *)&intensity, sizeof(float));
//			cm += point;
//			point_cloud.push_back(point);
//		}
//		input.close();
//	}
//	
//
//	
//	///You can also work with the point of interest
//	///for something changed remove comments
//	//cm/=point_cloud.size();
//	//for (int i=0;i<point_cloud.size();i++) {
//	  //    point_cloud[i][0]=point_cloud[i][0]-cm[0];
//	    //  point_cloud[i][1]=point_cloud[i][1]-cm[1];
//		  //point_cloud[i][2]=point_cloud[i][2]-cm[2];
//
////    }
//}

void Mesh3DScene::load_point_cloud_comparison() {
	const string pcldir16 = getBasePath() + "resources/extracted_las_files/0_5D4KVPBP/2016/";
	const string pcldir20 = getBasePath() + "resources/extracted_las_files/0_5D4KVPBP/2020/";
	
	const string pclFiles16 = pcldir16 + "0_5D4KVPBP_2_rgb_d.bin";
	const string pclFiles20 = pcldir20 + "0_5D4KVPBP_2_rgb_d.bin";
	vec cm16(0, 0, 0), cm20(0, 0, 0);

	// !!!!!!!!!!!!!!! 2016 !!!!!!!!!!!!!!!//
	fstream input16(pclFiles16.c_str(), ios::in | ios::binary);
	if (!input16.good()) {
		cerr << "Could not read file: " << pclFiles16 << endl;
		exit(EXIT_FAILURE);
	}
	input16.seekg(0, ios::beg);
	for (size_t i = 0; input16.good() && !input16.eof(); i++) {
		vec point;
		float intensity;
		input16.read((char *)&point.x, 3 * sizeof(float));
		input16.read((char *)&intensity, sizeof(float));
		cm16 += point;
		point_cloud16.push_back(point);
	}
	input16.close();

	cm16 /= point_cloud16.size();
	for (int i = 0; i < point_cloud16.size(); i++) {
		point_cloud16[i][0] = point_cloud16[i][0] - cm16[0];
		point_cloud16[i][1] = point_cloud16[i][1] - cm16[1];
		point_cloud16[i][2] = point_cloud16[i][2] - cm16[2];

	}

	// !!!!!!!!!!!!!!! 2020 !!!!!!!!!!!!!!!//
	fstream input20(pclFiles20.c_str(), ios::in | ios::binary);
	if (!input20.good()) {
		cerr << "Could not read file: " << pclFiles20 << endl;
		exit(EXIT_FAILURE);
	}
	input20.seekg(0, ios::beg);
	for (size_t i = 0; input20.good() && !input20.eof(); i++) {
		vec point;
		float intensity;
		input20.read((char *)&point.x, 3 * sizeof(float));
		input20.read((char *)&intensity, sizeof(float));
		cm20 += point;
		point_cloud20.push_back(point);
	}
	input20.close();


	cm20/= point_cloud20.size();
	for (int i=0;i< point_cloud20.size();i++) {
		point_cloud20[i][0]= point_cloud20[i][0]- cm20[0];
		point_cloud20[i][1]= point_cloud20[i][1]- cm20[1];
		point_cloud20[i][2]= point_cloud20[i][2]- cm20[2];

    }	

}

void Mesh3DScene::reset()
{
	Scene::reset();

	//! Define plane
	m_plane_d = 0;
	m_plane = Plane(vec(0, 1, 1).Normalized(), m_plane_d);

	//! Define what will be vissible by default
	m_style_flag = 0;
	m_style_flag |= FLAG_SHOW_SOLID;
	m_style_flag |= FLAG_SHOW_WIRE;
	m_style_flag |= FLAG_SHOW_AXES;
	m_style_flag |= FLAG_SHOW_AABB;
	m_style_flag |= FLAG_SHOW_PLANE;


}

void Mesh3DScene::resize()
{
	//! By Making `first_pass` static and initializing it to true,
	//! we make sure that the if block will be executed only once.
	static bool first_pass = true;

	if (first_pass)
	{
		m_model_original.setBigSize(getSceneWidth() / 2);
		m_model_original.update();
		m_model = m_model_original;
		Tasks();
		first_pass = false;
	}
}





void Mesh3DScene::arrowEvent(ArrowDir dir, int modif)
{
	math::vec n = m_plane.normal;
	if (dir == UP) m_plane_d += 1;
	if (dir == DOWN) m_plane_d -= 1;
	else if (dir == LEFT) n = math::float3x3::RotateY(DegToRad(1)).Transform(n);
	else if (dir == RIGHT) n = math::float3x3::RotateY(DegToRad(-1)).Transform(n);
	m_plane = Plane(n.Normalized(), m_plane_d);

	if (SPLIT_INSTEAD_OF_INTERSECT == false) {
		m_intersections.clear();
		 
	}
	else {
		m_model = Mesh(m_model_original);
		 
	}
}



void Mesh3DScene::keyEvent(unsigned char key, bool up, int modif)
{
	Scene::keyEvent(key, up, modif);
	key = tolower(key);

	switch (key)
	{
	case 's': m_style_flag ^= FLAG_SHOW_SOLID; break;
	case 'w': m_style_flag ^= FLAG_SHOW_WIRE; break;
	case 'n': m_style_flag ^= FLAG_SHOW_NORMALS; break;
	case 'a': m_style_flag ^= FLAG_SHOW_AXES; break;
	case 'p': m_style_flag ^= FLAG_SHOW_PLANE; break;
	case 'b': m_style_flag ^= FLAG_SHOW_AABB; break;
	/*case 'y': load_cv_test(); break;*/
	}
}

void Mesh3DScene::Tasks()
{

	//load_point_cloud();
	load_point_cloud_comparison();

}

void Mesh3DScene::processPoint(C2DPoint* const p)
{
	C2DPoint* v_opposite = nullptr;
	unsigned tri_adjacent_index = NULL;

	if (!m_run_task_3 && !m_run_task_4 && !m_run_task_5) {
		//! [Check 1]
		//! Check whether a point already exists in the same coords.
		for (size_t i = 0; i < m_pts.size(); i++)
		{
			if (m_pts.GetAt(i)->x == p->x &&
				m_pts.GetAt(i)->y == p->y)
			{
				delete p;
				return;
			}
		}

		//! Find enclosing triangle.
		unsigned i_enclosing = -1;
		unsigned count_enclosing = 0;
		for (int i = 0; i < m_triangles.size(); i++) {
			if (m_triangles[i].to_C2D().Contains(*p)) {
				count_enclosing++;
				i_enclosing = i;
			}
		}

		////! [Check 2]
		////! If no enclosing triangle was found.
		////! Or if more than one were found.
		//if (count_enclosing != 1) {
		//	delete p;
		//	return;
		//}
		//m_canvas.clear();
		//m_pts.Add(p);

		// The enclosing triangle
		Tri &tri_enclosing = m_triangles[i_enclosing];

		//!//////////////////////////////////////////////////////////////////////////////////
		//! TASK 0:
		//!  - Create the 3 subdivision triangles and store them to `new_triangles`.
		//!  - Remove the enclosing triangle and store the new ones with different colour.
		//!
		//! HINTS:
		//!  - To delete a triangle from `m_triangles` :
		//!      m_triangles.erase(m_triangles.begin() + index_trigwnou_pros_diagrafh);
		//!
		//!
		//! VARIABLES:
		//!
		//!  - C2DPoint* p;                // New point
		//!  - Tri &tri_enclosing          // Enclosing triangle of the new point
		//!  - Tri tris_new[3];            // The 3 triangles that will replace the enclosing.
		//!
		//!//////////////////////////////////////////////////////////////////////////////////

		vector<Tri> new_triangles;

		/// store 3 new triangles in new_triangles
		/// p, 3 vertices of i_enclosing triangle , m_triangles[i_enclosing]
		C2DPoint *p1 = m_triangles[i_enclosing].v1;
		C2DPoint *p2 = m_triangles[i_enclosing].v2;
		C2DPoint *p3 = m_triangles[i_enclosing].v3;

		new_triangles.push_back(Tri(p, p1, p2, vvr::Colour::red));
		new_triangles.push_back(Tri(p, p1, p3, vvr::Colour::red));
		new_triangles.push_back(Tri(p, p2, p3, vvr::Colour::red));

		/// erase i_enclosing triangle from m_triangles
		m_triangles.erase(m_triangles.begin() + i_enclosing);

		// add new_triangles to m_triangles
		m_triangles.insert(m_triangles.end(), new_triangles.begin(), new_triangles.end());


		//!//////////////////////////////////////////////////////////////////////////////////
		//! TASK 2:
		//!  - Check the 3 new triangles for Delaunay violations.
		//!  - If not Delaunay, add it with different color and show CircumCircle
		//!
		//! HINTS:
		//!  - use isDelaunay()
		//!
		//!  - To add circumCircle of `t`: canvas.add(GetCircumCircle(t), col, false)
		//!
		//!//////////////////////////////////////////////////////////////////////////////////


		//TODO
		/*for (auto &new_triangle : new_triangles) {
			if (findViolations(m_triangles, new_triangle, v_opposite, tri_adjacent_index)) {
				showViolations(new_triangle, m_canvas, vvr::Colour::white);
			}
		}*/

	}

	//!//////////////////////////////////////////////////////////////////////////////////
	//! TASK 3:
	//!  - In case of Delaunay violation, find the adjacent triangle and perform the flip.
	//!  - store the new triangles to `m_triangles`
	//!
	//! HINTS:
	//!  - Kathe trigwnno exei eite 3 geitonika, eite 2 an prokeitai gia trigwno
	//!    pou vrisketai sta oria (=> anikei merikws sto Convex Hull)
	//!
	//!  - bool adj_exists = FindAdjacentTriangle(m_triangles, edge_start, edge_end, &tri_adjacent, &v_opposite);
	//!
	//! VARIABLES:
	//!
	//!  - unsigned tri_adjacent_index // The adjacent triangle index, i.e. m_triangles[tri_adjacent_index] is what we want
	//!  - C2DPoint *v_opposite        // The opposite vertex of the adjacent triangle
	//!
	//!//////////////////////////////////////////////////////////////////////////////////


}


void Mesh3DScene::draw()
{
	

	/*for (int i = 0; i < point_cloud16.size(); i++) {
		Point3D(point_cloud16[i].x, point_cloud16[i].y, point_cloud16[i].z, vvr::Colour::blue).draw();
	}
	if (m_style_flag & FLAG_SHOW_SOLID) m_model_original.draw(m_obj_col, SOLID);
	if (m_style_flag & FLAG_SHOW_WIRE) m_model_original.draw(Colour::black, WIRE);
	if (m_style_flag & FLAG_SHOW_NORMALS) m_model_original.draw(Colour::black, NORMALS);
	if (m_style_flag & FLAG_SHOW_AXES) m_model.draw(Colour::black, AXES);*/

	enterPixelMode();

	//! Draw violations and anything else added to canvas
	if (m_style_flag & FLAG_SHOW_CANVAS) {
		Shape::DEF_LINE_WIDTH = m_lw_canvas;
		m_canvas.draw();
	}

	//! Draw triangles
	Shape::DEF_LINE_WIDTH = m_lw_tris;

	vector<Tri>::const_iterator tri_iter;
	for (tri_iter = m_triangles.begin(); tri_iter != m_triangles.end(); ++tri_iter) {
		tri_iter->to_vvr().draw();
	}

	//! Draw points
	Shape::DEF_POINT_SIZE = m_sz_pt;
	vvr::draw(m_pts, Colour::red);

	returnFromPixelMode();
}

//void Mesh3DScene::draw()
//{
//	//! Draw center mass
//	for (int i = 0; i < point_cloud16.size(); i++) {
//		Point3D(point_cloud16[i].x, point_cloud16[i].y, point_cloud16[i].z, vvr::Colour::yellow).draw();
//	}
//	for (int i = 0; i < point_cloud20.size(); i++) {
//		Point3D(point_cloud20[i].x, point_cloud20[i].y, point_cloud20[i].z, vvr::Colour::red).draw();
//	}
//
//	if (m_style_flag & FLAG_SHOW_SOLID) m_model.draw(m_obj_col, SOLID);
//	if (m_style_flag & FLAG_SHOW_WIRE) m_model.draw(Colour::black, WIRE);
//	if (m_style_flag & FLAG_SHOW_NORMALS) m_model.draw(Colour::black, NORMALS);
//	if (m_style_flag & FLAG_SHOW_AXES) m_model.draw(Colour::black, AXES);
//
//}
 
int main(int argc, char* argv[])
{
	try {
		return vvr::mainLoop(argc, argv, new Mesh3DScene);
	}
	catch (std::string exc) {
		cerr << exc << endl;
		return 1;
	}
	catch (...)
	{
		cerr << "Unknown exception" << endl;
		return 1;
	}
}

 