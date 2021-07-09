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
#include <chrono>


#define SPLIT_INSTEAD_OF_INTERSECT 1


using namespace std::chrono;

#define DIMENSIONS 3
//Those were default from lab5 - maybe change?
#define SEC_PER_FLOOR 10
#define GND_WIDTH 40
#define GND_TOP 10
#define GND_BOTTOM -10
#define GND_DEPTH 10
#define SPHERE_RAD 0.5
#define POINT_SIZE 6
#define AUTOPLAY false


using namespace std;
using namespace vvr;
using namespace cv;

using namespace Eigen;


int levels_calculated = 0;

Mesh3DScene::Mesh3DScene()
{	
	
	//! Load settings.
	vvr::Shape::DEF_LINE_WIDTH = 1;
	vvr::Shape::DEF_POINT_SIZE = 4;
	m_perspective_proj = true;
	m_bg_col = Colour("768E77");
	

	const float canvas_w = getSceneWidth() /2.0f;
	const float canvas_h = getSceneHeight() / 2.0f;
	//m_canvas_0.add(C2DLine());

	m_create_menus = true;
	m_lw_canvas = 2;
	m_lw_tris = 3;
	m_sz_pt = 4;

	
	m_hide_log = false;
	m_hide_sliders = false;
	m_fullscreen = true;
	/*if (AUTOPLAY) m_anim.update(true);
	m_KDTree = NULL;
*/
	reset();
}



void Mesh3DScene::readUserInput() {
	
	do{
		// Type a number and press enter
		cout << "--------------WELCOME--------------" <<endl;
		cout << "Please type a number from 0-60." << endl << "You will get access to the folder with files first:"; 
		cin >> subfoldersIndex; // Get user input from the keyboard

		if (!cin.fail()) {
			break;
		}

	} while ((subfoldersIndex > 0 || subfoldersIndex < 71));
	
	do{
		cout << "Please select a specific file inside the selected folder:"; // Type a number and press enter
		cin >> fileIndex; // Get user input from the keyboard

		if (!cin.fail()) {
			break;
		}
	}while ( (fileIndex >0  || fileIndex<20));


	cout << "Please wait..." << endl << "Point Cloud Data --------> LOADING... <--------" << endl;


}

/* HEATMAP: 
// create an array of squares, link the positions of the players and check if they are within the squares. 
//Then change the colour within the squares based on how many times each square is visited!
*/

//input: ratio is between 0 to 1
//output: rgb color
unsigned int rgb(double ratio)
{
	//we want to normalize ratio so that it fits in to 6 regions
	//where each region is 256 units long
	int normalized = int(ratio * 256 * 6);

	//find the distance to the start of the closest region
	int x = normalized % 256;

	int red = 0, grn = 0, blu = 0;
	switch (normalized / 256)
	{
	case 0: red = 255;      grn = x;        blu = 0;       break;//red
	case 1: red = 255 - x;  grn = 255;      blu = 0;       break;//yellow
	case 2: red = 0;        grn = 255;      blu = x;       break;//green
	case 3: red = 0;        grn = 255 - x;  blu = 255;     break;//cyan
	case 4: red = x;        grn = 0;        blu = 255;     break;//blue
	case 5: red = 255;      grn = 0;        blu = 255 - x; break;//magenta
	}

	return red + (grn << 8) + (blu << 16);
}



void Mesh3DScene::load_point_cloud_comparison() {

	const string subfolders[78] ={ "0_5D4KVPBP",
"10_5D4KVPXD",
"11_5D4KVPYD",
"12_5D4KVQ0P",
"13_5D4KVQ9U",
"14_5D4KVRER",
"16_5D4KVT48",
"17_5D4KVT9X",
"18_5D4KX2ZN",
"19_5D4KX38L",
"20_5D4KX3EC",
"21_5D4KX3LW",
"22_5D4KX3PZ",
"23_5D4KX3RR",
"24_5D4KX3TQ",
"25_5D4KX3VN",
"2_5D4KVPDO",
"26_5D4KX40Y",
"27_5D4KX4HE",
"28_5D4KX4QC",
"29_5D4KX4ZE",
"30_5D4KX56H",
"31_5D4KX5G9",
"32_5D4KX5NM",
"33_5D4KX5WV",
"34_5D4KX66R",
"35_5D4KX6L3",
"3_5D4KVPFI",
"36_5D4KX6T5",
"37_5D4KX76F",
"38_5D4KX7FN",
"39_5D4KX7IA",
"40_5D4KX7KT",
"41_5D4KX7RD",
"42_5D4KX826",
"43_5D4KX8IR",
"44_5D4KX8UQ",
"45_5D4KX8Y6",
"4_5D4KVPG4",
"46_5D4KX993",
"47_5D4KX9N2",
"48_5D4KX9SD",
"49_5D4KX9SY",
"50_5D4KX9ZE",
"51_5D4KXA0G",
"52_5D4KXAW7",
"53_5D4KXB8D",
"54_5D4KXBC8",
"55_5D4KXBTC",
"5_5D4KVPIN",
"56_5D4L1GZR",
"57_5D4L1IQ4",
"58_5D4L1JIE",
"59_5D4L1M3I",
"60_5D4L1MGO",
"61_5D4L1P8E",
"62_5D4L1QP2",
"63_5D4L1RDR",
"64_5D4L1RW5",
"65_5D4L1TDI",
"6_5D4KVPJE",
"66_5D4L1TH9",
"67_5D4L1TX7",
"68_5D4L1TYC",
"69_5D4L1WHI",
"70_5D4L1XPJ",
"71_5D4L1Y38",
"72_5D4L1YDX",
"73_5D4L2BFI",
"74_5D4L2C9B",
"75_5D4L2DGW",
"7_5D4KVPNC",
"76_5D4L2DTM",
"77_5D4L2FRJ",
"78_5D4L2G9K",
"79_5D4LHQUX",
"8_5D4KVPT5",
"9_5D4KVPX2"
	};
	
	string pclDir = getBasePath() + "resources/extracted_las_files/";
	const string pcldir16 = pclDir + subfolders[subfoldersIndex] + "/2016/";
	
	string strFile = to_string(fileIndex);
	const string pclFiles16 =  pcldir16 + subfolders[subfoldersIndex]+ "_" + strFile + "_rgb_d.bin";


	pclDir = getBasePath() + "resources/extracted_las_files/";
	const string pcldir20 = pclDir + subfolders[subfoldersIndex] + "/2020/";

	strFile = to_string(fileIndex);
	const string pclFiles20 = pcldir16 + subfolders[subfoldersIndex] + "_" + strFile + "_rgb_d.bin";

	//const string pcldir20 = getBasePath() + "resources/extracted_las_files/0_5D4KVPBP";
	//const string pclFiles20 = pcldir20 + "/2020/" + "/0_5D4KVPBP_5_rgb_d.bin";
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
		input16.read((char *)&point, 3 * sizeof(float));
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

void Mesh3DScene::Task1a()
{
	//2016
	double min16X, max16X, min16Y, max16Y, min16Z, max16Z;
	
	min16X = point_cloud16[0][0];
	max16X = min16X;

	min16Y = point_cloud16[0][1];
	max16Y = min16Y;
	
	min16Z = point_cloud16[0][2];
	max16Z = min16Z;

	for (int i = 0; i < point_cloud16.size(); i++) {
	
		min16X = findMin(point_cloud16[i][0], min16X);
		max16X = findMax(point_cloud16[i][0], max16X);

		min16Y = findMin(point_cloud16[i][1], min16Y);
		max16Y = findMax(point_cloud16[i][1], max16Y);

		min16Z = findMin(point_cloud16[i][2], min16Z);		
		max16Z = findMax(point_cloud16[i][2], max16Z);
		

	}
	/*cout << "2016" << endl;
	cout << "min16Z:" << min16Z <<endl;
	cout << "max16Z:" << max16Z << endl;
	cout << "z threshold:" << (max16Z - min16Z)/ point_cloud16.size() << endl;*/
	
	//2020

	double min20X, max20X, min20Y, max20Y, min20Z, max20Z;

	min20X = point_cloud20[0][0];
	max20X = min20X;

	min20Y = point_cloud20[0][1];
	max20Y = min20Y;

	min20Z = point_cloud20[0][2];
	max20Z = min20Z;

	for (int i = 0; i < point_cloud20.size(); i++) {

		min20X = findMin(point_cloud20[i][0], min20X);
		max20X = findMax(point_cloud20[i][0], max20X);

		min20Y = findMin(point_cloud20[i][1], min20Y);
		max20Y = findMax(point_cloud20[i][1], max20Y);

		min20Z = findMin(point_cloud20[i][2], min20Z);
		max20Z = findMax(point_cloud20[i][2], max20Z);


	}


	/*cout << "2020" << endl;
	cout << "min20Z:" << min20Z <<endl;
	cout << "max20Z:" << max20Z << endl;
	cout << "z threshold:" << (max20Z - min20Z)/ point_cloud20.size() << endl;*/

	//searching the median - not the medium since we have outliers
	/*
	1. Min value
	2. Max value
	3. Q1
	4. Q3
	5. Median
	*/
	math::float3 median16, median20;
	double positionMed16, positionMed20;
	positionMed16 = ceil(point_cloud16.size() / 2.0);
	positionMed20 = ceil(point_cloud20.size() / 2.0);
	median16 = point_cloud16[positionMed16];
	median20 = point_cloud20[positionMed20];
	

	/*cout << "2016" << endl;
	cout << "median16:" << median16 << endl;
	
	cout << "2020" << endl;
	cout << "median20:" << median20 << endl;*/
	
	float3 Q1_16, Q3_16, Q1_20, Q3_20;
	double positionQ1_16, positionQ1_20, positionQ3_16, positionQ3_20;
	
	positionQ1_16 = ceil((positionMed16 - 1) / 2.0);
	positionQ1_20 = ceil((positionMed20 - 1)/ 2.0);
	Q1_16 = point_cloud16[positionQ1_16];
	Q1_20 = point_cloud20[positionQ1_20];

	/*cout << "2016" << endl;
	cout << "Q1_16:" << Q1_16 << endl;

	cout << "2020" << endl;
	cout << "Q1_20:" << Q1_20 << endl;*/

	positionQ3_16 = ceil((positionMed16 - 1) / 2.0);
	positionQ3_20 = ceil((positionMed20 - 1) / 2.0);
	Q3_16 = point_cloud16[positionMed16+positionQ3_16];
	Q3_20 = point_cloud20[positionMed20+positionQ3_16];

	/*cout << "2016" << endl;
	cout << "Q3_16:" << Q3_16 << endl;

	cout << "2020" << endl;
	cout << "Q3_20:" << Q3_20 << endl;*/
	//Following regarding outliers - thresholds --> https://www.youtube.com/watch?v=Pfk9tlWy500
	//Inner Quartile Range (BOX)
	float3 IQR16, IQR20;
	const float factorIQR = 1.5f;
	IQR16 = (Q3_16 - Q1_16) * factorIQR;
	IQR20 = (Q3_20 - Q1_20) * factorIQR;

	//add iqr * factor to Q3 subtract from Q1;
	//UPPER Boundary or equally Upper limit
	Q3_16 += IQR16;
	Q3_20 += IQR20;
	//LOWER Boundary or equally Lower limit
	Q1_16 -= IQR16;
	Q1_20 -= IQR20;

	//cout << "2016" << endl;
	//cout << "Q1_16:" << Q1_16 << endl;
	//cout << "Q3_16:" << Q3_16 << endl;

	//cout << "2020" << endl;
	//cout << "Q1_20:" << Q1_20 << endl;
	//cout << "Q3_20:" << Q3_20 << endl;

	cout << "point_cloud16.size():" << point_cloud16.size() << endl;
	int counter = 0;
	unsigned position16 = -1;
	for (int t = 0; t < point_cloud16.size()-1; t++) {
		//COMPARE ALL WITH ALL DIMENSIONS
		if ((point_cloud16[t].x < Q1_16.x || point_cloud16[t].x > Q3_16.x)
			&& (point_cloud16[t].y < Q1_16.y || point_cloud16[t].y > Q3_16.y)
			&& (point_cloud16[t].z < Q1_16.z || point_cloud16[t].z > Q3_16.z)){
			//deleting from a vector the t-th element
			//point_cloud16.erase(point_cloud16.begin() + (t - 2));
			position16 = t;
			cout << "position16:" << position16 << endl;
			//deleting from a vector the t-th element
			point_cloud16.erase(point_cloud16.begin() + position16);
			
			counter++;
		}
	}
	
	cout << "point_cloud16 removed:" << counter << endl;


	cout << "point_cloud20.size():" << point_cloud20.size() << endl;
	counter = 0;
	unsigned position20 = -1;
	for (int t = 0; t < point_cloud20.size()-1; t++) {
		if ((point_cloud20[t].x < Q1_20.x || point_cloud20[t].x > Q3_20.x)
			 && (point_cloud20[t].y < Q1_20.y || point_cloud20[t].y > Q3_20.y)
			 && (point_cloud20[t].z < Q1_20.z || point_cloud20[t].z > Q3_20.z))  {
			//point_cloud20.pop_back();
			
			//deleting from a vector the t-th element
			position20 = t;
			//cout << "position20:" << position16 << endl;

			point_cloud20.erase(point_cloud20.begin() + position20);
			m_canvas.draw();
			//delete &point_cloud20[t];

			counter++;
		}
	}

	cout << "point_cloud20 removed:" << counter << endl;


}

void Mesh3DScene::Task1b()
{
	//RANSAC Implementation actually
	double dist;
	double A, B, C;
	// LINE: A*x+B*y+C=0
	/*C2DPoint *p1, *p2;*/
	
	//find the distance of all points from this line
	//the points which lie within tolerance range are the inliers
	

	//the line with the maximum number of inliers becomes the line equation
	double variationX = 0, variationY = 0, variationZ = 0, variation=0;
	double sum, sumX = 0, sumY=0, sumZ=0;
	int numPoints = point_cloud16.size();
	double mean;
	int n = 0;
	
	for (size_t i = 0; i < numPoints; i++)
	{
		sumX += point_cloud16[i].x;
		sumY += point_cloud16[i].y;
		sumZ += point_cloud16[i].z;
	}
	sum = sumX + sumY + sumZ;
	mean = sum / numPoints;
	while (n < numPoints) {
		variationX = variationX + (pow((point_cloud16[n].x - mean), (point_cloud16[n].x - mean))) ;
		variationY = variationY + (pow((point_cloud16[n].y - mean), (point_cloud16[n].y - mean)));
		variationZ = variationZ + (pow((point_cloud16[n].z - mean), (point_cloud16[n].z - mean)));
		n++;
	}
	variation = variationX + variationY + variationZ;
	variation /= numPoints;
	
	double sd = sqrt(variation);

	//test to decide
	// numberOfIterations shouldn't be too large
	int numberOfIterations=1000;
	//tolerance is based upon the spread of the dataset
	double tolerance = 5.0f;
	int maxInliersCounter = 0;
	double coefficients[3];

	for (size_t i = 0; i < numberOfIterations; i++)
	{
		randomIndices s;
		//finding two random points
		s = generateRandomIndicesPair();
		//cout << "s: " << s.index1 << "AND" << s.index2 << endl;
		//find the coefficients for the line
		//equation of line passing through two points is given by(y1 - y2)*X + (x2 - x1)*Y + (x1*y2 - y1 * x2) = 0
		A = point_cloud16[s.index1][1] - point_cloud16[s.index2][1];
		B = point_cloud16[s.index2][0] - point_cloud16[s.index1][0];
		C = point_cloud16[s.index1][0] * point_cloud16[s.index2][1] - point_cloud16[s.index1][1] * point_cloud16[s.index2][0];
	

		//find the count of points lying within tolerance of this line
		int inlierscount = 0;
		for (size_t i = 0; i < point_cloud16.size(); i++)
		{
			//perpendicular distance of point x1, y1 from line Ax + By + C = 0 is | Ax1 + By1 + C|/sqrt(A ^ 2 + B ^ 2)
			dist = abs(A*point_cloud16[i][0] + B * point_cloud16[i][1] + C) / (pow(A*A + B * B, 0.5));
			if (dist < tolerance)
			{
				inlierscount += 1;
			}
		}
		if (inlierscount > maxInliersCounter) {
			coefficients[0] = A;
			coefficients[1] = B;
			coefficients[2] = C;
			maxInliersCounter = inlierscount;
		}
		
	}
	
	for (size_t i = 0; i < point_cloud16.size(); i++) 
	{
		A = coefficients[0];
		B = coefficients[1];
		C = coefficients[2];
	}
		
	for (int p = 0; p < point_cloud16.size(); p++) {
		allXvalues = point_cloud16[p];
	}

	double minimumVal, maximumVal;
	minimumVal = 0;
	maximumVal = minimumVal;

	for (int p = 0; p < point_cloud16.size(); p++) {
		if (point_cloud16[p][0] < minimumVal) {
			minimumVal = point_cloud16[p][0];
		}
		if (point_cloud16[p][0] > maximumVal) {
			maximumVal = point_cloud16[p][0];
		}
	}
	

	
	xValues[0] = minimumVal;
	xValues[1] = maximumVal;


	yValues[0] = (-C - A * xValues[0]) / B;
	yValues[1] = (-C - A * xValues[1]) / B;


	cout << "xValues:" << xValues[0] << " AND " << xValues[1] << endl;
	cout << "yValues:" << yValues[0] << " AND " << yValues[1] << endl;


	double minXValues, maxXValues, minYValues, maxYValues, minZValues, maxZValues;

	minXValues = 0;
	maxXValues = minXValues;
	
	minYValues = 0;
	maxYValues = minYValues;
	
	minZValues = 0;
	maxZValues = minZValues;

	for (size_t i = 0; i < point_cloud16.size(); i++)
	{
		allXvalues.x = point_cloud16[0].x;
		allXvalues.y = point_cloud16[0].y;
		allXvalues.z = point_cloud16[0].z;
	}
	//min max for x
	for (size_t i = 0; i < point_cloud16.size(); i++)
	{
		if (allXvalues.x < minXValues) {
			minXValues = allXvalues.x;
		}
		if (allXvalues.x > maxXValues) {
			maxXValues = allXvalues.x;
		}
	}
	//min max for y
	for (size_t i = 0; i < point_cloud16.size(); i++)
	{
		if (allXvalues.y < minYValues) {
			minYValues = allXvalues.y;
		}
		if (allXvalues.y > maxYValues) {
			maxYValues = allXvalues.y;
		}
	}
	//min max for Z
	for (size_t i = 0; i < point_cloud16.size(); i++)
	{
		if (allXvalues.z < minZValues) {
			minZValues = allXvalues.z;
		}
		if (allXvalues.z > maxZValues) {
			maxZValues = allXvalues.z;
		}
	}

	

	xValues[0] = minXValues;
	xValues[1] = maxXValues;

	yValues[0] = minYValues;
	yValues[1] = maxYValues;

	zValues[0] = minZValues;
	zValues[1] = maxZValues;

	xValues[0] = (-C - A * xValues[0]) / B;
	xValues[1] = (-C - A * xValues[1]) / B;
	yValues[0] = (-C - A * yValues[0]) / B;
	yValues[1] = (-C - A * yValues[1]) / B;
	zValues[0] = (-C - A * zValues[0]) / B;
	zValues[1] = (-C - A * zValues[1]) / B;
	
}

void Mesh3DScene::Task2a()
{
	//Implementation of K-Means for unlabeled data
	/* https://github.com/aditya1601/kmeans-clustering-cpp/blob/master/kmeans.cpp
	1. Find the centroid of all point dataset
	2. Create an array of clusters
	3. Delete/Clear the clusters before processing them
	4. Find the NearestCluster()
	5. Add points to their nearest cluster
	6. Reassign the points to their newest cluster
	7. Implement PARALLEL code with  #pragma omp parallel for reduction(&&: done) num_threads(16) or similarly
	*/

}

double Mesh3DScene::findMin(double a, double b)
{
	if (a < b){
		return a;
	}
	else {
		return b;
	}

}

double Mesh3DScene::findMax(double a, double b)
{
	if (a > b) {
		return a;
	}
	else {
		return b;
	}

}


randomIndices Mesh3DScene::generateRandomIndicesPair() {
	
	randomIndices s;
	s.index1 = rand() % (point_cloud16.size() - 1);
	s.index2 = rand() % (point_cloud16.size() - 1);
	while (s.index1 == s.index2) {
		s.index2 = rand() % (point_cloud16.size() - 1);
	}

	return s;
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
	m_style_flag |= FLAG_SHOW_CANVAS;
	m_style_flag |= FLAG_SHOW_AABB;
	m_style_flag |= FLAG_SHOW_PLANE;
	

	//for working with canvas and triangulization process
	m_canvas.clear();
	m_triangles.clear();
	m_pts.clear();
	//m_pts.DeleteAll();
	

	//// For making a split view - m_canvas.add(SHAPE) --> triangulization is needed
	//m_canvas_0.clear();
	//m_canvas_1.clear();
	//

	//// Divide window to Tasks
	//m_bound_vertical.Set(C2DPoint(0, -3000), C2DPoint(0, 3000));
	//m_canvas_0.add(m_bound_vertical, Colour::black);

	/*m_kn = 10;
	m_current_tree_level = 0;*/

	//! Position camera
	//auto pos = getFrustum().Pos();
	//pos.y += 20;
	//pos.z -= 40;
	//setCameraPos(pos);

	//! Define what will be vissible by default
	//m_flag = 0;
	//m_flag |= FLAG(SHOW_NN);
	//m_flag |= FLAG(SHOW_PTS_KDTREE);
	//m_flag |= FLAG(SHOW_KDTREE);
	//m_flag |= FLAG(SHOW_PTS_ALL);
	//m_flag |= FLAG(SHOW_PTS_IN_SPHERE);
	//m_flag |= FLAG(BRUTEFORCE);
	//m_flag |= FLAG(SHOW_TIME);

	//! Define scene objects
	//m_sphere = vvr::Sphere3D(-GND_WIDTH / 2, 0, 0, SPHERE_RAD, vvr::Colour::white);

	//! Create random points
	const float mw = getSceneWidth() * 0.3;
	const float mh = getSceneHeight() * 0.3;
	const float mz = std::min(mw, mh);

	//if (FLAG_ON(m_flag, POINTS_ON_SURFACE)) {
	//	createSurfacePts(m_pts.empty() ? NUM_PTS_DEFAULT : m_pts.size());
	//}
	//else {
	//	createRandomPts(m_pts.empty() ? NUM_PTS_DEFAULT : m_pts.size());
	//}

	//delete m_KDTree;
	
	//m_pts in Lab5 vs m_pts in point cloud appalaktiki different types, what's that???
	//m_KDTree = new KDTree(m_pts);
	//m_tree_invalidation_sec = -1;

	//! Reset animation
	//m_anim.setTime(0);

	//Taken from triangulization lab
	//scene vs viewport
	const int W = getViewportWidth() * 0.9;
	const int H = getViewportHeight()  * 0.9;

	C2DPoint* A = new C2DPoint(-W / 2, -H / 2);
	C2DPoint* B = new C2DPoint(W / 2, -H / 2);
	C2DPoint* C = new C2DPoint(0, H / 2);

	/*m_pts.Add(A);
	m_pts.Add(B);
	m_pts.Add(C);*/
	m_triangles.push_back(Tri(A, B, C));

}

void Mesh3DScene::resize()
{
	//! By Making `first_pass` static and initializing it to true,
	//! we make sure that the if block will be executed only once.
	static bool first_pass = true;

	if (first_pass)
	{
		/*m_model_original.setBigSize(getSceneWidth() / 2);
		m_model_original.update();
		m_model = m_model_original;*/
		
		readUserInput();
		
		Tasks();
		first_pass = false;
	}
}


void Mesh3DScene::printKeyboardShortcuts()
{
	std::cout << "Keyboard shortcuts:"
		<< std::endl << "'?' => This shortcut list:"
		<< std::endl << "'b' => Task1b"
		<< std::endl << "'n' => SHOW_NN"
		<< std::endl << "'k' => SHOW_KNN"
		<< std::endl << "'f' => SHOW_FPS"
		/*<< std::endl << "'a' => SHOW_AXES"*/
		<< std::endl << "'s' => SHOW_SPHERE"
		<< std::endl << "'t' => SHOW_KDTREE"
		<< std::endl << "'p' => SHOW_PTS_ALL"
		<< std::endl << "'d' => SHOW_PTS_KDTREE"
		<< std::endl << "'c' => SHOW_PTS_IN_SPHERE"
		<< std::endl << "'u' => POINTS_ON_SURFACE"
		<< std::endl << std::endl;
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

	//From Lab5
	/*if (dir == vvr::UP)
	{
		++m_current_tree_level;
		if (m_current_tree_level > m_KDTree->depth())
			m_current_tree_level = m_KDTree->depth();
	}
	else if (dir == vvr::DOWN)
	{
		--m_current_tree_level;
		if (m_current_tree_level < 0)
			m_current_tree_level = 0;
	}*/
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
	
	case 'p': m_style_flag ^= FLAG_SHOW_PLANE; break;
	//case 'b': m_style_flag ^= FLAG_SHOW_AABB; break;
	case 'a': Task1a();  break;
	case 'b': Task1b();  break;
	case 'q': load_point_cloud_comparison();  break;

		//FLAG_TOGGLE(m_flag, 'b', BRUTEFORCE);
		//FLAG_TOGGLE(m_flag, 'n', SHOW_NN);
		//FLAG_TOGGLE(m_flag, 'k', SHOW_KNN);
		//FLAG_TOGGLE(m_flag, 'f', SHOW_FPS);
		FLAG_TOGGLE(m_flag, 'c', FLAG_SHOW_CANVAS);
		

		//FLAG_TOGGLE(m_flag, 't', SHOW_KDTREE);
		//FLAG_TOGGLE(m_flag, 'p', SHOW_PTS_ALL);
		//FLAG_TOGGLE(m_flag, 'd', SHOW_PTS_KDTREE);
		//FLAG_TOGGLE(m_flag, 'c', SHOW_PTS_IN_SPHERE);
		///*FLAG_TOGGLE(m_flag, 'r', POINTS_ON_SURFACE);*/
		//FLAG_TOGGLE(m_flag, 'o', SHOW_TIME);
	}
}

void Mesh3DScene::mouseWheel(int dir, int modif)
{
	if (shiftDown(modif))
	{
		m_lw_canvas *= pow(1.2f, dir);
		m_lw_tris *= pow(1.2f, dir);
		m_sz_pt *= pow(1.2f, dir);
	}
	else
	{
		Scene::mouseWheel(dir, modif);
	}
}

void Mesh3DScene::Tasks()
{


	//load_point_cloud_comparison();
	/*Task_3_TriangulateMesh(m_vertices, m_model);
	if (!levels_calculated) {
		if (REGION_GROWING) Task_4_DivideLayers_RR(m_vertices, v_all_levels);
		else Task_4_DivideLayers(m_model, &all_levels);
	}*/
	/*Task1a();
	Task1b();*/
	
	//processPoint();

}

void Mesh3DScene::processPoint()
{
	

	C2DPoint* v_opposite = nullptr;
	unsigned tri_adjacent_index = NULL;

	if (!m_run_task_3 && !m_run_task_4 && !m_run_task_5) {
		//! [Check 1]
		//! Check whether a point already exists in the same coords.
		for (size_t i = 0; i < m_pts.size(); i++)
		{
			//!!!!!!!!!!!!!!TODO: Convert from a vector::math::float3 to a C3DPoint

			/*if (m_pts.GetAt(i)->x == point_cloud16[i].x &&
				m_pts.GetAt(i)->y == point_cloud16[i].x)
			{
				delete &point_cloud16[i];
				return;
			}*/
		}

		//! Find enclosing triangle.
		/*unsigned i_enclosing = -1;
		unsigned count_enclosing = 0;
		for (int i = 0; i < m_triangles.size(); i++) {
			if (m_triangles[i].to_C2D().Contains(point_cloud16[i])) {
				count_enclosing++;
				i_enclosing = i;
			}
		}*/

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
		//Tri &tri_enclosing = m_triangles[i_enclosing];

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
		/*C2DPoint *p1 = m_triangles[i_enclosing].v1;
		C2DPoint *p2 = m_triangles[i_enclosing].v2;
		C2DPoint *p3 = m_triangles[i_enclosing].v3;*/

		//p->x = point_cloud16[0].x;
		//p->y = point_cloud16[0].y;
		////p->z = point_cloud16[0].y;

		//C2DPoint *p1 = m_triangles[i_enclosing].v1;
		//C2DPoint *p2 = m_triangles[i_enclosing].v2;
		//C2DPoint *p3 = m_triangles[i_enclosing].v3;

		//new_triangles.push_back(Tri(p, p1, p2, vvr::Colour::red));
		//new_triangles.push_back(Tri(p, p1, p3, vvr::Colour::red));
		//new_triangles.push_back(Tri(p, p2, p3, vvr::Colour::red));

		///// erase i_enclosing triangle from m_triangles
		//m_triangles.erase(m_triangles.begin() + i_enclosing);

		//// add new_triangles to m_triangles
		//m_triangles.insert(m_triangles.end(), new_triangles.begin(), new_triangles.end());


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
	
	const float POINT_SIZE_SAVE = vvr::Shape::DEF_POINT_SIZE;

	//! Draw violations and anything else added to canvas
	if (m_style_flag & FLAG_SHOW_CANVAS) {
		Shape::DEF_LINE_WIDTH = m_lw_canvas;
		m_canvas.draw();
	}

	
	//! Draw triangles
	Shape::DEF_LINE_WIDTH = m_lw_tris;

	//for triangulization - need to adjust
	/*vector<Tri>::const_iterator tri_iter;
	for (tri_iter = m_triangles.begin(); tri_iter != m_triangles.end(); ++tri_iter) {
		tri_iter->to_vvr().draw();
	}
*/
	////! Draw any point
	//Shape::DEF_POINT_SIZE = m_sz_pt;
	//for (int i = 0; i < m_pts.size(); i++) {
	//	Point3D(m_pts[i].x, m_pts[i].y, m_pts[i].z, Colour::yellow).draw();
	//}


	////! Draw point clouds
	//for (int i = 0; i < point_cloud16.size(); i++) {
	//	Point3D(point_cloud16[i].x, point_cloud16[i].y, point_cloud16[i].z, vvr::Colour::blue).draw();
	//}
	//for (int i = 0; i < point_cloud20.size(); i++) {
	//	Point3D(point_cloud20[i].x, point_cloud20[i].y, point_cloud20[i].z, vvr::Colour::darkRed).draw();
	//}
	
	

	//! Draw point clouds
	for (int i = 0; i < point_cloud16.size(); i++) {
		Point3D(point_cloud16[i].x, point_cloud16[i].y, point_cloud16[i].z,vvr::Colour::black).draw();
	}
	for (int i = 0; i < point_cloud20.size(); i++) {
		Point3D(point_cloud20[i].x, point_cloud20[i].y, point_cloud20[i].z, vvr::Colour::white).draw();
	}

	//draw for RANSAC
	// SOMETHING'S NOT NICE....
	
	//Plane(xValues[0], yValues[0], xValues[1], yValues[1], vvr::Colour::green).draw();
	vvr::Colour colPlane=vvr::Colour::yellowGreen;
	//size of the plane
	float u = 10, v = 10;
	math::vec pp0(m_plane.Point(-u, -v, math::vec(xValues[0], yValues[0], zValues[0])));
	math::vec pp1(m_plane.Point(-u, v, math::vec(xValues[0], yValues[0], zValues[0])));
	math::vec pp2(m_plane.Point(u, -v, math::vec(xValues[0], yValues[0], zValues[0])));
	math::vec pp3(m_plane.Point(u, v, math::vec(xValues[0], yValues[0], zValues[0])));
	math::vec pp4(m_plane.Point(-u, -v, math::vec(xValues[1], yValues[1], zValues[1])));
	math::vec pp5(m_plane.Point(-u, v, math::vec(xValues[1], yValues[1], zValues[1])));
	math::vec pp6(m_plane.Point(u, -v, math::vec(xValues[1], yValues[1], zValues[1])));
	math::vec pp7(m_plane.Point(u, v, math::vec(xValues[1], yValues[1], zValues[1])));
	
	
	math2vvr(math::Triangle(pp0, pp1, pp2), colPlane).draw();
	math2vvr(math::Triangle(pp2, pp1, pp3), colPlane).draw();

	math2vvr(math::Triangle(pp4, pp5, pp6), colPlane).draw();
	math2vvr(math::Triangle(pp6, pp4, pp5), colPlane).draw();

	////! Draw plane
	//if (m_style_flag & FLAG_SHOW_PLANE) {
	//	vvr::Colour colPlane(0x0f, 0xf1, 0xce);
	//	//WXH for the plane - increase?
	//	float u = 10, v = 10;
	//	math::vec p0(m_plane.Point(-u, -v, math::vec(0, 0, 0)));
	//	math::vec p1(m_plane.Point(-u, v, math::vec(0, 0, 0)));
	//	math::vec p2(m_plane.Point(u, -v, math::vec(0, 0, 0)));
	//	math::vec p3(m_plane.Point(u, v, math::vec(0, 0, 0)));
	//	math2vvr(math::Triangle(p0, p1, p2), colPlane).draw();
	//	math2vvr(math::Triangle(p2, p1, p3), colPlane).draw();
	//}


	
	if (m_style_flag & FLAG_SHOW_SOLID) m_model.draw(m_obj_col, SOLID);
	if (m_style_flag & FLAG_SHOW_WIRE) m_model.draw(Colour::black, WIRE);
	if (m_style_flag & FLAG_SHOW_NORMALS) m_model.draw(Colour::black, NORMALS);
	
	Task1a()) m_model.draw(Colour::black, AXES);

	//from lab5
	//! Find and Draw Nearest Neighbour

	//if (FLAG_ON(m_flag, SHOW_NN)) {
	//	float dist;
	//	const KDNode *nearest = NULL;

	//	//!!!! COPY PASTE FROM MY YEAR THE TASK 02 AND UNCOMMENT BELOW
	//	Task_02_Nearest(sc, m_KDTree->root(), &nearest, &dist);

	//	vec nn = nearest->split_point;
	//	vvr::Shape::DEF_POINT_SIZE = vvr::Shape::DEF_POINT_SIZE = POINT_SIZE;
	//	math2vvr(sc, vvr::Colour::blue).draw();
	//	math2vvr(nn, vvr::Colour::green).draw();
	//	vvr::Shape::DEF_POINT_SIZE = POINT_SIZE_SAVE;
	//}

	//! Find and Draw K Nearest Neighbour using bruteforce
	/*if (FLAG_ON(m_flag, SHOW_KNN) && FLAG_ON(m_flag, BRUTEFORCE)) {
		vec *nearests = new vec[m_KDTree->pts.size()];
		double* allDistances = new double[m_KDTree->pts.size()];
		memset(nearests, NULL, m_KDTree->pts.size() * sizeof(vec));

		for (int i = 0; i < m_KDTree->pts.size(); i++) {
			*(nearests + i) = m_KDTree->pts[i];
			allDistances[i] = sc.Distance(m_KDTree->pts[i]);
		}

		int *indices = new int[m_KDTree->pts.size()];
		for (int i = 0; i < m_KDTree->pts.size(); i++) {
			*(indices + i) = i;
		}
		std::sort(indices, indices + m_KDTree->pts.size(), compare::compare(allDistances));
		for (int i = 0; i < m_kn; i++) {
			vec nn = m_KDTree->pts[*(indices + i)];
			vvr::Shape::DEF_POINT_SIZE = vvr::Shape::DEF_POINT_SIZE = POINT_SIZE;
			math2vvr(sc, vvr::Colour::blue).draw();
			math2vvr(nn, vvr::Colour::green).draw();
			vvr::Shape::DEF_POINT_SIZE = POINT_SIZE_SAVE;
		}

		delete[] nearests;
		delete[] indices;
	}
*/
	//! Find and Draw K Nearest Neighbour using recursion

	//if (FLAG_ON(m_flag, SHOW_KNN) && !FLAG_ON(m_flag, BRUTEFORCE)) {
	//	float dist;
	//	const KDNode **nearests = new const KDNode*[m_kn];
	//	memset(nearests, NULL, m_kn * sizeof(KDNode*));

	//	/*
	//	Needs adaption - Task 4 in Lab05 is about the KNN and NNN when moving the sphere.
	//	Here the KNN and NNN is regarding the mesh of the scene.
	//	Hence, CHECK AGAIN!
	//	*/
	//	/*for (int i = 0; i < m_kn; i++) {
	//		Task_04_NearestK(i, sc, m_KDTree->root(), nearests, &dist);
	//	}

	//	for (int i = 0; i < m_kn; i++) {
	//		if (!nearests[i]) continue;
	//		vec nn = nearests[i]->split_point;
	//		vvr::Shape::DEF_POINT_SIZE = vvr::Shape::DEF_POINT_SIZE = POINT_SIZE;
	//		math2vvr(sc, vvr::Colour::blue).draw();
	//		math2vvr(nn, vvr::Colour::green).draw();
	//		vvr::Shape::DEF_POINT_SIZE = POINT_SIZE_SAVE;
	//	}
	//	delete[] nearests;*/
	//}

	//////! Draw KDTree
	//if (FLAG_ON(m_flag, SHOW_KDTREE)) {
	//	for (int level = m_current_tree_level; level <= m_current_tree_level; level++) {
	//		std::vector<KDNode*> levelNodes = m_KDTree->getNodesOfLevel(level);
	//		for (int i = 0; i < levelNodes.size(); i++) {
	//			if (m_flag & FLAG(SHOW_PTS_KDTREE)) {
	//				VecArray pts;
	//				Task_01_FindPtsOfNode(levelNodes[i], pts);
	//				vvr::Shape::DEF_POINT_SIZE = vvr::Shape::DEF_POINT_SIZE = POINT_SIZE;
	//				for (int pi = 0; pi < pts.size(); pi++) {
	//					math2vvr(pts[pi], Pallete[i % 6]).draw();
	//				}
	//				vvr::Shape::DEF_POINT_SIZE = POINT_SIZE_SAVE;
	//			}
	//			vec c1 = levelNodes[i]->aabb.minPoint - math::vec(0.1);
	//			vec c2 = levelNodes[i]->aabb.maxPoint + math::vec(0.1);
	//			vvr::Box3D box(c1.x, c1.y, c1.z, c2.x, c2.y, c2.z);
	//			box.setTransparency(0.9);
	//			box.setColour(vvr::Colour::cyan);
	//			box.draw();
	//		}
	//	}
	//}

	////! Compute & Display FPS
	//static float last_update = 0;
	//static float last_show = 0;
	//const float sec = vvr::getSeconds();
	//const float dt = sec - last_update;
	//const float dt_show = sec - last_show;
	//int FPS = 1.0 / dt;
	//last_update = sec;
	//if (FLAG_ON(m_flag, SHOW_FPS) && dt_show >= 1) {
	//	echo(FPS);
	//	last_show = sec;
	//}

}
 
////! KDTree::
//
//KDTree::KDTree(VecArray &pts)
//	: pts(pts)
//{
//	const float t = vvr::getSeconds();
//	m_root = new KDNode();
//	m_depth = makeNode(m_root, pts, 0);
//	const float KDTree_construction_time = vvr::getSeconds() - t;
//	echo(KDTree_construction_time);
//	echo(m_depth);
//}
//
//KDTree::~KDTree()
//{
//	const float t = vvr::getSeconds();
//	delete m_root;
//	const float KDTree_destruction_time = vvr::getSeconds() - t;
//	echo(KDTree_destruction_time);
//}
//
//int KDTree::makeNode(KDNode *node, VecArray &pts, const int level)
//{
//	//! Sort along the appropriate axis, find median point and split.
//	const int axis = level % DIMENSIONS;
//	std::sort(pts.begin(), pts.end(), VecComparator(axis));
//	const int i_median = pts.size() / 2;
//
//	//! Set node members
//	node->level = level;
//	node->axis = axis;
//	node->split_point = pts[i_median];
//	node->aabb.SetFrom(&pts[0], pts.size());
//
//	//! Continue recursively or stop.
//	if (pts.size() <= 1)
//	{
//		return level;
//	}
//	else
//	{
//		int level_left = 0;
//		int level_right = 0;
//		VecArray pts_left(pts.begin(), pts.begin() + i_median);
//		VecArray pts_right(pts.begin() + i_median + 1, pts.end());
//
//		if (!pts_left.empty())
//		{
//			node->child_left = new KDNode();
//			level_left = makeNode(node->child_left, pts_left, level + 1);
//
//		}
//		if (!pts_right.empty())
//		{
//			node->child_right = new KDNode();
//			level_right = makeNode(node->child_right, pts_right, level + 1);
//		}
//
//		int max_level = std::max(level_left, level_right);
//		return max_level;
//	}
//}

////!!!!!!!!!!!!!!!! 
//void Task_01_FindPtsOfNode(const KDNode* root, VecArray &pts)
//{
//	pts.push_back(root->split_point);
//
//	if (root->child_left) {
//		Task_01_FindPtsOfNode(root->child_left, pts);
//
//	}
//
//	if (root->child_right) {
//
//		Task_01_FindPtsOfNode(root->child_right, pts);
//	}
//
//
//}
////
//void Task_02_Nearest(const vec& test_pt, const KDNode* root, const KDNode **nn, float *best_dist)
//{
//	if (!root)
//		return;
//
//	const double d = test_pt.Distance(root->split_point);
//	const double d_split = root->split_point.ptr()[root->axis] - test_pt.ptr()[root->axis];
//	const bool right_of_split = d_split <= 0;
//
//	if (*nn == NULL || d < *best_dist) {
//		*best_dist = d;
//		*nn = root;
//	}
//
//	//searching
//	Task_02_Nearest(test_pt, right_of_split ? root->child_right : root->child_left, nn, best_dist);
//
//	//pruning
//	if (d_split * d_split >= *best_dist) return;
//
//	Task_02_Nearest(test_pt, right_of_split ? root->child_left : root->child_right, nn, best_dist);
//
//
//}
////
//void Task_03_InSphere(const Sphere &sphere, const KDNode *root, VecArray &pts)
//{
//	if (!root)
//		return;
//
//	const double d = sphere.pos.DistanceSq(root->split_point);
//	const double d_split = root->split_point.ptr()[root->axis] - sphere.pos.ptr()[root->axis];
//	const bool right_of_split = d_split <= 0;
//
//	if (d < sphere.r * sphere.r) {
//		pts.push_back(root->split_point);
//	}
//
//	Task_03_InSphere(sphere, right_of_split ? root->child_right : root->child_left, pts);
//
//	//pruning
//	if (d_split * d_split > sphere.r* sphere.r)return;
//
//	Task_03_InSphere(sphere, right_of_split ? root->child_left : root->child_right, pts);
//}
//
//
//void Task_04_NearestK(const int k, const vec& test_pt, const KDNode* root, const KDNode **knn, float *best_dist)
//{
//
//}
//
//void KDTree::getNodesOfLevel(KDNode *node, std::vector<KDNode*> &nodes, int level)
//{
//	if (!level)
//	{
//		nodes.push_back(node);
//	}
//	else
//	{
//		if (node->child_left) getNodesOfLevel(node->child_left, nodes, level - 1);
//		if (node->child_right) getNodesOfLevel(node->child_right, nodes, level - 1);
//	}
//}
//
//std::vector<KDNode*> KDTree::getNodesOfLevel(const int level)
//{
//	std::vector<KDNode*> nodes;
//	if (!m_root) return nodes;
//	getNodesOfLevel(m_root, nodes, level);
//	return nodes;
//}

//
//void Task_3_TriangulateMesh(const std::vector<vec>& vertices, vvr::Mesh*& mesh)
//{
//	//!//////////////////////////////////////////////////////////////////////////////////
//	//! TASK:
//	//!
//	//!  - Create 2 triangles for every 4-vertex block
//	//!  - Create mesh "m_model" (dont forget new Mesh()) 
//	//!
//	//!//////////////////////////////////////////////////////////////////////////////////
//
//	Mat curdisp = trueDisp;
//
//	mesh = new Mesh();
//	std::vector<vec>& modelVerts = mesh->getVertices();
//	std::vector<vvr::Triangle>& tris = mesh->getTriangles();
//
//	int i_lim = curdisp.rows - 1;
//	int j_lim = curdisp.cols % 2 ? curdisp.cols - 1 : curdisp.cols;
//
//	for (int i = 0; i < i_lim; i++) {
//		for (int j = 0; j < j_lim; j++) {
//
//			int upperleft = i * i_lim + j;
//			int upperright = i * i_lim + j + 1;
//			int lowerleft = (i + 1)*i_lim + j;
//			int lowerright = (i + 1) * i_lim + j + 1;
//
//			modelVerts.push_back(vertices.at(upperleft));
//			modelVerts.push_back(vertices.at(lowerleft));
//			modelVerts.push_back(vertices.at(upperright));
//			tris.push_back(vvr::Triangle(&modelVerts, modelVerts.size() - 3, modelVerts.size() - 2, modelVerts.size() - 1));
//
//			modelVerts.push_back(vertices.at(upperright));
//			modelVerts.push_back(vertices.at(lowerleft));
//			modelVerts.push_back(vertices.at(lowerright));
//			tris.push_back(vvr::Triangle(&modelVerts, modelVerts.size() - 3, modelVerts.size() - 2, modelVerts.size() - 1));
//
//		}
//	}
//}
//
//bool valid(vec v) {
//	if ((isnormal(v.x) || v.x == 0) && (isnormal(v.y) || v.y == 0) && (isnormal(v.z) || v.z == 0.0)) return true;
//}
//
//void vvr::Task_4_DivideLayers_RR(const std::vector<vec>& vertices, std::vector<vLevel>& all_levels) {
//
//	int new_index, i, j, preindex;
//	char* vchart = (char *)malloc(vertices.size() * sizeof(char));
//	for (int k = 0; k < vertices.size(); k++) vchart[k] = '0';
//
//	while (levels < 4) {
//
//		new_index = rand() % (vertices.size());
//		if (vchart[new_index] == '0' && valid(vertices.at(new_index))) {
//
//			j = div(new_index, trueDisp.cols).quot + 1;
//			i = div(new_index, trueDisp.cols).rem;
//
//			if (!haveLevel(vertices.at(new_index).z, all_levels, preindex)) {
//
//				levels++;
//				vLevel newlevel;
//				newlevel.zlevel = vertices.at(new_index).z;
//				recur(i, j, vertices, &newlevel, vchart);
//				all_levels.push_back(newlevel);
//			}
//
//			else {
//				all_levels[preindex].lvertices.push_back(vertices[new_index]);
//				vchart[new_index] = '1';
//
//			}
//		}
//
//	}
//}
//
//void vvr::recur(int i, int j, const std::vector<vec>& vertices, vLevel* level, char* vchart) {
//
//	i %= trueDisp.rows;
//	j %= trueDisp.cols;
//	if (vertices.at((j - 1)*trueDisp.cols + i).z == level->zlevel  && valid(vertices.at((j - 1) * trueDisp.cols + i)) && vchart[(j - 1) * trueDisp.cols + i] == '0') {
//
//		vchart[(j - 1) * trueDisp.cols + i] = '1';
//		recur(i, j - 1, vertices, level, vchart);
//		recur(i - 1, j, vertices, level, vchart);
//		recur(i + 1, j, vertices, level, vchart);
//		recur(i, j + 1, vertices, level, vchart);
//
//		//add to level
//		level->lvertices.push_back(vertices.at((j - 1) * trueDisp.cols + i));
//
//	}
//
//	return;
//}
//
//
//
//
//bool haveLevel(float level, std::vector<vLevel>& all_levels, int &index) {
//
//	for (int i = 0; i < levels; i++) {
//		if (all_levels[i].zlevel == level) {
//			index = i; return true;
//		}
//	}
//	return false;
//
//}
//
//
//void vvr::Task_4_DivideLayers(vvr::Mesh*& mesh, Level** all_levels_ptr) {
//	levels_calculated = 1;
//	std::vector<vvr::Triangle>& tris = mesh->getTriangles();
//
//	int place;
//	//initialize levels
//	for (place = 0; place < tris.size(); place++) if (triValidity(tris[place])) break; //find first valid triangle
//
//	levels++;
//	(*all_levels_ptr)[0].zlevel = tris[place].v1().z;
//	(*all_levels_ptr)[0].meshl = new Mesh();
//	qsert((*all_levels_ptr)[0].meshl, tris[place]);
//	place++;
//	//handle all triangles
//	for (; place < tris.size(); place++) {
//		Triangle tri = tris[place];
//
//		if (!triValidity(tris[place])) continue; //ignore non valid triangles
//		if (tri.v1().z != tri.v2().z || tri.v1().z != tri.v3().z || tri.v3().z != tri.v2().z) continue;	//ignore triangles between levels		
//		if (insert_in_layer(tri, *all_levels_ptr)) continue; //inserted in pre-existing layer
//		//need to create new layer:
//		levels++;
//		int newsize = levels * (sizeof(**all_levels_ptr));
//		*all_levels_ptr = (Level *)realloc((*all_levels_ptr), newsize);
//		(*all_levels_ptr)[levels - 1].zlevel = tris[place].v1().z;
//		(*all_levels_ptr)[levels - 1].meshl = new Mesh();
//		qsert((*all_levels_ptr)[levels - 1].meshl, tris[place]);
//	}
//
//}
//
//
//bool triValidity(vvr::Triangle tri) {
//	vec v1 = tri.v1(), v2 = tri.v2(), v3 = tri.v3();
//
//	if (!isnormal(v1.x) || !isnormal(v1.y) || !isnormal(v1.z)) return false;
//	if (!isnormal(v2.x) || !isnormal(v2.y) || !isnormal(v2.z)) return false;
//	if (!isnormal(v3.x) || !isnormal(v3.y) || !isnormal(v3.z)) return false;
//	return true;
//}
//
////
//bool insert_in_layer(vvr::Triangle tri, Level* all_levels) {
//
//	for (int i = 0; i < levels; i++) {
//		if (tri.v1().z == all_levels[i].zlevel) {
//			qsert(all_levels[i].meshl, tri);
//			return true;
//		}
//	}
//	return false;
//}
//
//void qsert(vvr::Mesh*& mesh, vvr::Triangle tri) {
//
//	std::vector<vec>& verts = mesh->getVertices();
//	std::vector<vvr::Triangle>& tris = mesh->getTriangles();
//	verts.push_back(tri.v1());
//	verts.push_back(tri.v2());
//	verts.push_back(tri.v3());
//
//	mesh->getTriangles().push_back(vvr::Triangle(&verts, verts.size() - 3, verts.size() - 2, verts.size() - 1));
//
//
//}
//
//


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




 