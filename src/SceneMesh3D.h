#ifndef SCENE_MESH_3D_H
#define SCENE_MESH_3D_H

//#include <GeoLib.h>
#include <VVRScene/scene.h>
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
#include "math.h"
#include "canvas.h"
#include <ctime>
#include <algorithm>
//OpenCV library
#include <opencv2/opencv.hpp>

//Eigen Library
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "Eigen/SparseQR"
#include "Eigen/Eigenvalues"
#include "Eigen/SparseCholesky"


//! MACROS used for toggling and testing bitwise flags.
#define FLAG(x) (1<<(x))
#define FLAG_ON(v,f) (v & FLAG(f))
#define FLAG_TOGGLE(v,c,f) case c: v ^= FLAG(f); std::cout \
    << #f << " = " << (FLAG_ON(v,f) ? "OFF" : "ON") \
    << std::endl; break


#define FLAG_SHOW_CANVAS     1
#define FLAG_SHOW_WIRE       2
#define FLAG_SHOW_SOLID      4
#define FLAG_SHOW_NORMALS    8
#define FLAG_SHOW_PLANE     16
#define FLAG_SHOW_AABB      32

//From KD Trees lab (5)
/**
 * Array with 6 predefined colours.
 */
static const vvr::Colour Pallete[11] = {
	vvr::Colour::red, vvr::Colour::green, vvr::Colour::blue, vvr::Colour::magenta,
	vvr::Colour::orange, vvr::Colour::yellow,
	vvr::Colour::darkGreen,vvr::Colour::darkOrange,vvr::Colour::darkRed,
	vvr::Colour::white, vvr::Colour::cyan,
};





/**
 * A node of a KD-Tree
 */
struct KDNode
{
	vec split_point;
	int axis;
	int level;
	AABB aabb;
	KDNode *child_left;
	KDNode *child_right;
	KDNode() : child_left(NULL), child_right(NULL) {}
	~KDNode() { delete child_left; delete child_right; }
};


/**
 * Function object to compare 2 3D-vecs in the specified axis.
 */
struct VecComparator {
	unsigned axis;
	VecComparator(unsigned axis) : axis(axis % 3) {}
	virtual inline bool operator() (const vec& v1, const vec& v2) {
		return (v1.ptr()[axis] < v2.ptr()[axis]);
	}
};

//! Task function prototypes
struct compare {
	double* values;
	compare(double *input) : values(input) {}
	bool operator()(int i, int j) const { return values[i] < values[j]; }
};


//......
struct Tri;
typedef struct randomIndices;



#define DEFAULT 0.535353

struct Level {
	vvr::Mesh* meshl = nullptr;
	float zlevel = 0.0;
};
struct vLevel {
	std::vector<vec> lvertices;
	float zlevel = 0.0;
};



/**
 * KD-Tree wrapper. Holds a ptr to tree root.
 */ 
//
//class KDTree
//{
//public:
//	KDTree(VecArray &pts);
//	~KDTree();
//	std::vector<KDNode*> getNodesOfLevel(int level);
//	int depth() const { return m_depth; }
//	const KDNode* root() const { return m_root; }
//	const VecArray &pts;
//
//private:
//	static int makeNode(KDNode *node, VecArray &pts, const int level);
//	static void getNodesOfLevel(KDNode *node, std::vector<KDNode*> &nodes, int level);
//
//private:
//	KDNode *m_root;
//	int m_depth;
//};


//Scene - declarations, members, functions etc
class Mesh3DScene : public vvr::Scene
{

	enum {
		BRUTEFORCE, POINTS_ON_SURFACE, SHOW_AXES, SHOW_FPS, SHOW_NN, SHOW_KNN, SHOW_SPHERE,
		SHOW_KDTREE, SHOW_PTS_ALL, SHOW_PTS_KDTREE,
		SHOW_PTS_IN_SPHERE, SHOW_TIME, FLAG_1A, FLAG_1B, FLAG_2A,
	};



public:
	

	//Block Matching parameters;
	//SAD-> Sum of absolute differences
	int bm_sad = 10; int bm_sad_max = 20; //*2 + 1
	int bm_num_disparities = 3; int bm_num_disparities_max = 10; //*16
	int bm_uniqueness = 0;	int bm_uniqueness_max = 100;

	
	//Stereo Matrices
	cv::Mat image2016;
	cv::Mat image2020;
	cv::Mat trueDisp;
	cv::Mat disp;
	cv::Mat disp8; //8-bit disparity to display
	

	//Stereo to 3D params  ( Z = f(B/d) ) (f = ResX / (2 * tan(FOV/2.0) ) (X = (V-Vc) * (Z/f))
	float stereo_baseline = 0.2f; // 20cm
	float fieldOfView = 1.2f; // rad
	float focal_length_constant = 1.0f / (2.0f * tan(fieldOfView / 2.0f));


public:
	Mesh3DScene();
	const char* getName() const { return "3D Scene"; }
	void keyEvent(unsigned char key, bool up, int modif) override;
	void arrowEvent(vvr::ArrowDir dir, int modif) override;
	void mouseWheel(int dir, int modif) override;
	void printKeyboardShortcuts();

	void load_point_cloud_comparison();
	void Task1a();
	void Task1b(int pointCloudSize, std::vector<vec> point_cloud);
	//void Task1b(int pointCloudSize, std::array point_cloud);
	void Task2a(int pointCloudSize);


	

	double findMin(double a, double b);
	double findMax(double a, double b);

private:
	void draw() override;

	randomIndices generateRandomIndicesPair(std::vector<vec> point_cloud);

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

	int subfoldersIndex, fileIndex;
	math::float3 allXvalues;
	double xValues[2], yValues[2], zValues[2];



private: // Data
	C2DLine         m_bound_horizontal;
	C2DLine         m_bound_vertical;
	vvr::Canvas2D   m_canvas_0;
	vvr::Canvas2D   m_canvas_1;
	

	//From KD Trees Lab5
	//KDTree *m_KDTree;
	VecArray m_pts;
	//vvr::Sphere3D m_sphere;
	//vvr::Animation m_anim;
	int m_flag;
	math::LCG m_lcg;
	int m_current_tree_level;
	//float m_tree_invalidation_sec;
	//int m_kn;

private:
	//void processPoint(C2DPoint* const p);
	void processPoint();
	void readUserInput();

	

private:
	
	/*C2DPoint *m_pts;*/
	std::vector<Tri> m_triangles;	
	float m_lw_canvas;
	float m_lw_tris;
	float m_sz_pt;
	bool m_run_task_3, m_run_task_4, m_run_task_5;
	
};


struct randomIndices {
	int index1, index2;
};

template<typename S>
auto selectRandom(const S &s, size_t n) {
	auto it = std::begin(s);
	// 'advance' the iterator n times
	std::advance(it, n);
	return it;
}

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





/**
 * Find all the points under `root` node of the tree.
 */
//void Task_01_FindPtsOfNode(const KDNode* root, VecArray &pts);



/**
 * Find the nearest neighbour of `test_pt` inside `root`.
 */
//void Task_02_Nearest(const vec& test_pt, const KDNode* root, const KDNode **nn, float *best_dist);

/**
 * Find the points of `kdtree` that are contained inside `sphere`.
 */
//void Task_03_InSphere(const Sphere &sphere, const KDNode *root, VecArray &pts);

/**
 * Find the `k` nearest neighbours of `test_pt` inside `root`.
 */
//void Task_04_NearestK(const int k, const vec& test_pt, const KDNode* root, const KDNode **knn, float *best_dist);

//C2DCircle GetCircumCircle(const C2DTriangle &t);
//
//bool IsDelaunay(const Tri& t, const C2DPointSet &pset);
//
//bool FindAdjacentTriangle(std::vector<Tri> &tris, C2DPoint *p1, C2DPoint *p2, unsigned &tri_adj_index, C2DPoint* &opposite_vertex);
//
//bool findViolations(const std::vector<Tri> &all_triangles, const Tri &tris, C2DPoint *&v_opposite, unsigned &tri_adjacent_index);
//void showViolations(const Tri &tris, vvr::Canvas2D &canvas, const vvr::Colour &col);
#endif // SCENE_MESH_3D_H