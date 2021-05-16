#ifndef SCENE_MESH_3D_H
#define SCENE_MESH_3D_H

#include <VVRScene/canvas.h>
#include <VVRScene/mesh.h>
#include <VVRScene/settings.h>
#include <VVRScene/utils.h>
#include <MathGeoLib.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include <string>
#include <set>
#include "symmetriceigensolver3x3.h"

#define FLAG_SHOW_AXES       1
#define FLAG_SHOW_WIRE       2
#define FLAG_SHOW_SOLID      4
#define FLAG_SHOW_NORMALS    8
#define FLAG_SHOW_PLANE     16
#define FLAG_SHOW_AABB      32

 
 

struct Tri;

class Mesh3DScene : public vvr::Scene
{
public:
	Mesh3DScene();
	const char* getName() const { return "3D Scene"; }
	void keyEvent(unsigned char key, bool up, int modif) override;
	void arrowEvent(vvr::ArrowDir dir, int modif) override;
	//void load_point_cloud();
	void load_point_cloud_comparison();

private:
	void draw() override;
	void reset() override;
	void resize() override;
	//void drawCompare();
	void Tasks();	
	 

private:
	int m_style_flag;
	float m_plane_d;
	vvr::Canvas2D m_canvas;
	vvr::Colour m_obj_col;
	vvr::Mesh m_model_original, m_model;
	vvr::Box3D m_aabb;
	math::vec m_center_mass;
	math::vec m_pca_cen;
	math::vec m_pca_dir;
	math::Plane m_plane;
	std::vector<int> m_intersections;
	std::vector<vec> point_cloud, point_cloud16, point_cloud20;
	 

private:
	void processPoint(C2DPoint* const p);

private:
	
	C2DPointSet m_pts;
	std::vector<Tri> m_triangles;	
	float m_lw_canvas;
	float m_lw_tris;
	float m_sz_pt;
	bool m_run_task_3, m_run_task_4, m_run_task_5;
	C2DPoint* point_pressed;

};
/**
* Struct representing a triangle with pointers to its 3 vertices
*/
struct Tri
{
	C2DPoint *v1;
	C2DPoint *v2;
	C2DPoint *v3;
	float area;

	vvr::Colour colour;

	Tri(C2DPoint *v1, C2DPoint *v2, C2DPoint *v3, vvr::Colour colour_ = vvr::Colour::black) : v1(v1), v2(v2), v3(v3), colour(colour_) {
		area = to_C2D().GetArea();
	}

	C2DTriangle to_C2D() const { return C2DTriangle(*v1, *v2, *v3); }

	vvr::Triangle2D to_vvr(bool filled = false) const {
		vvr::Triangle2D t(v1->x, v1->y, v2->x, v2->y, v3->x, v3->y, colour);
		t.setSolidRender(filled);
		return t;
	}

	bool operator < (const Tri& other) const {
		if (area != other.area) return (area < other.area);
		else if (v1 != other.v1) return v1 < other.v1;
		else if (v2 != other.v2) return v2 < other.v2;
		else if (v3 != other.v3) return v3 < other.v3;
	}
};

//C2DCircle GetCircumCircle(const C2DTriangle &t);
//
//bool IsDelaunay(const Tri& t, const C2DPointSet &pset);
//
//bool FindAdjacentTriangle(std::vector<Tri> &tris, C2DPoint *p1, C2DPoint *p2, unsigned &tri_adj_index, C2DPoint* &opposite_vertex);
//
//bool findViolations(const std::vector<Tri> &all_triangles, const Tri &tris, C2DPoint *&v_opposite, unsigned &tri_adjacent_index);
//void showViolations(const Tri &tris, vvr::Canvas2D &canvas, const vvr::Colour &col);
#endif // SCENE_MESH_3D_H