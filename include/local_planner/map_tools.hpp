
/*************************************************************************//**
 *****************************************************************************
 * @file        sweep.hpp
 * @brief       Provide methods for sweeping a partion
 * @author      Frank Imeson
 * @date        2012-01-02
 *****************************************************************************
 ****************************************************************************/

#ifndef MAPTOOLS_H		// guard
#define MAPTOOLS_H



/*****************************************************************************
 * INCLUDE
 ****************************************************************************/

#include <ros/ros.h>
#include <math.h>
#include <sstream>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

//#include <cv.h>
//#include <highgui.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

#include "general.hpp"

#include <boost/geometry.hpp>

// #include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/linestring.hpp> 
 #include <boost/geometry/multi/geometries/multi_polygon.hpp>
 
// #include <boost/geometry/io/wkt/wkt.hpp>

typedef boost::geometry::model::d2::point_xy<double> point2dboost;
typedef boost::geometry::model::polygon<point2dboost> polygonboost;
typedef boost::geometry::model::multi_polygon<polygonboost> multipolygonboost;
typedef boost::geometry::model::linestring<point2dboost > linestringboost;



using namespace cv;
using namespace std;
using namespace Eigen;


/*****************************************************************************
 * DEFINE
 ****************************************************************************/

#define PERCENT_TOL		0.0001
enum COLLISIONS {UP_TO, THRU, AROUND};


#define BLACK_RGBA    std_msgs::ColorRGBA(0,0,0,0)
#define RED_RGBA    std_msgs::ColorRGBA(1,0,0,1)
#define GREEN_RGBA    std_msgs::ColorRGBA( vector<float> (0.0,1.0,0.0,1.0) )
#define BLUE_RGBA    std_msgs::ColorRGBA(0,0,1,1)
#define WHITE_RGBA    std_msgs::ColorRGBA(1,1,1,1)

//FOR ARUN'S MAP. DELETE THESE LATER WHEN WE FIX IT.
#define RESOLUTION 0.25
#define MSIZE 400
#define NGRID (MSIZE/RESOLUTION)

/*****************************************************************************
 * Structures
 ****************************************************************************/



struct VectorAns {
	bool ans;
	Vector2f vector;
	VectorAns() : ans(false), vector(0,0) {}
};

struct IndexAns {
	bool ans;
	Vector2i index;
	IndexAns() : ans(false), index(0,0) {}
};

struct ObstacleAns {
	double area;
	bool collision, discovered, exit;
	Vector2f left, right, in, out;
	ObstacleAns() {
		collision = false;
		discovered = false;
		exit = false;
		left = Vector2f(0,0);
		right = left;
		in = left;
		out = left;
	}
};





/*****************************************************************************
 * Class Definitions
 ****************************************************************************/

class GridMap{
	private:
		vector<Vector2i> getTestIndex01 (Vector2i index, bool left);
		vector<Vector2i> getTestIndex02 (Vector2i index, bool left);
		IndexAns gotoNextNeighbour (Vector2i index, bool left);
	public:
		nav_msgs::OccupancyGrid* map;
		int width, height;
		double resolution;

		GridMap ();
		GridMap (nav_msgs::OccupancyGrid* map);
		~GridMap ();

		Vector2f getPos (Vector2i index);
		Vector2i getIndex (Vector2f pos);
		Mat getImage ();

		bool testMap (Vector2f pos);
		bool testMap (Vector2i index);
		bool testMap (int region, Vector2f pos);
		bool testMap (int region, Vector2i index);

		void setMap (int region, Vector2i index);
		void setMap (int region, Vector2f index);

		int getMap (Vector2i index);
		int getMap (Vector2f pos);

		ObstacleAns collide (Vector2f start, Vector2f stop, COLLISIONS mode);
};




class ImageMap{
	private:
		bool initialized;
	public:
		Mat image;
		Vector2f origin;
		double resolution, tolerance;


		ImageMap ();
		ImageMap (Mat _image, Vector2f _origin, double res);
		ImageMap (GridMap map);
		ImageMap (const ImageMap& image_map);
		nav_msgs::OccupancyGrid getMapMsg ();

		Point2i getIndex (Vector2f index);
		Vector2f getPos (Vector2i index);

		void initilize (Mat _image, Vector2f _origin, double res);
		int getMap (Point2i index);
		int getMap (Vector2f pos);
		void setMap (uchar value, Vector2f pos);
		void mergeMap(GridMap grid);
		void mergeMap(ImageMap image);
		void polyFill(vector<Vector2f> poly);
		double compare(ImageMap _compare);
		void clear();
		vector< vector<Vector2f> > findPolys ();
};




class ConvexPolygon{
	private:
		Vector2f centroid;
		Vector2f findCentroid ();
	public:
		vector<Vector2f> point_list;
		
		ConvexPolygon ();
		ConvexPolygon (vector<Vector2f> point_list);

		Vector2f getCentroid ();
		double getArea();

		Vector2f collide (Vector2f start, Vector2f direction);
		bool test (Vector2f position);
};




class VisualMsg{
	private:
	    int id;
	public:
        visualization_msgs::MarkerArray msg;
   		std_msgs::ColorRGBA black, red, green, blue, white;

    	VisualMsg ();
    	
        void addLine (const vector<Vector2f> poly, int colour, int size);
        void addPoly (const vector< vector<Vector2f> > &polys, const std_msgs::ColorRGBA &colour, const double size, const char* name_space);
        void addText (const vector<Vector2f> poly, int colour, int size);
        void addDot (const vector<Vector2f> poly, int colour, int size);
};


/*****************************************************************************
 * Prototypes
 ****************************************************************************/

VectorAns DoLineSegmentsIntersect(Vector2f start01, Vector2f finish01, Vector2f start02, Vector2f finish02);
VectorAns vectorIntersect (Vector2f start01, Vector2f finish01, Vector2f start02, Vector2f finish02, bool include);
VectorAns intersectVector (Vector2f start01, Vector2f finish01, Vector2f start02, Vector2f finish02);
bool pointOnVector (Vector2f point, Vector2f start, Vector2f finish);
bool testVectors (Vector2f point, Vector2f start);

VectorAns polyLeft (vector<Vector2f> poly);
double calcPathCost (const vector<Vector2f> path);

bool findObstacleIntersections_boost(Vector2f p1, Vector2f p2, const vector< Vector2f > &obstacle, bool isEnvBndry, vector<linestringboost> *otpt=NULL);
bool pointOnSegment(Vector2f point, Vector2f start, Vector2f finish);
Vector2f closestPointOnSegment(Vector2f point, Vector2f start, Vector2f finish);
vector<Vector2f> convertPolyBoostToVector2f (polygonboost boostVect);
polygonboost convertPolyVector2fToBoost (const vector<Vector2f> & vector2fVect);
point2dboost vector2fToBoost(Vector2f p);
std::pair<unsigned int, Vector2f> closestBoundaryPoint(Vector2f p, const vector<Vector2f> & obsPts);
vector<vector<Vector2f> > convertPolyWithHolesBoostToVector2f(polygonboost boostVect);
int find_boundary_indices2(vector<vector<cv::Point> > contour, vector<Vec4i> neighbours, Point2f cur_pose, float MAP_RESOLUTION, int BOUND_THRES, int BOUND_DIST);
/*************************************************************************//**
 * FOR DEBUGGING
 ****************************************************************************/
string vector2Str(Vector2f v);
string linestringboostvector2Str(vector<linestringboost> &l);

template < class T >
string VECTOR_STR ( vector<T> row_vector ) {
    stringstream text;
    if (row_vector.size() > 0) {
        text << "[ " << row_vector[0];
        for(unsigned int i=1; i<row_vector.size(); i++)
            text << " , " << row_vector[i];
        text << " ]";
    }
    return text.str();
}

template < class T >
string MATRIX_STR ( vector< vector<T> > matrix ) {
    stringstream text;
    if (matrix.size() > 0 && matrix[0].size() > 0) {
        text << "[ ";
        for(unsigned int i=0; i< matrix.size(); ++i)
        {
            text << matrix[i][0]; // allow proper placement of comma
            for(unsigned int j=1; j<matrix.size(); j++)
                text << " , " << matrix[i][j];
            text << "\n";
        }
        text << " ]";
    }
    return text.str();
}

//////////////////////////// >>>>>>>>>>>>>>>> ARCHIVE/DEPRECATED <<<<<<<<<<<<<<<<<<<
bool doesLineIntersectObstacle(Vector2f p1, Vector2f p2, const vector<Vector2f> &obstacle, bool isEnvBndry);



#endif	// guard

