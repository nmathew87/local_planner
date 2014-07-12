
/*************************************************************************//**
 *****************************************************************************
 * @file        localplan.hpp
 * @brief       Provide methods creating (and executing?) a local plan
 * @author      Talha Jawaid
 * @date        2013-02-13
 *****************************************************************************
 ****************************************************************************/

#ifndef LOCAL_PLAN_H      // guard
#define LOCAL_PLAN_H

 #define ALGO_TYPE_BUG 0
 #define ALGO_TYPE_VIS 1
 #define ALGO_TYPE_LINVIS 2

/*****************************************************************************
 * INCLUDE
 ****************************************************************************/

#include <ros/ros.h>
#include <math.h>
#include <sstream>
#include <iostream>
#include <Eigen/Core>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>

// #include <nav_msgs/Path.h>
// #include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// #include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include "map_tools.hpp"
// #include "general.hpp"
// #include "greedycut.hpp"

#include <boost/geometry.hpp>
// #include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/linestring.hpp> 
// #include <boost/geometry/io/wkt/wkt.hpp>

typedef boost::geometry::model::d2::point_xy<double> point2dboost;
typedef boost::geometry::model::polygon<point2dboost> polygonboost;
typedef boost::geometry::model::linestring<point2dboost > linestringboost;

using namespace std;
using namespace Eigen;

/****************************************************************************/

/*****************************************************************************
 * Struct Definitions
 ****************************************************************************/
typedef struct aStarNodeStruct
{
    int nodeIdx; // unique index of node
    double sDist; // distance from start node
    double fDist; // heuristc distance to finish node
    int parent; // index of parent node
    bool visited; // had the node been visited

    aStarNodeStruct():
        nodeIdx(-1),sDist(INFINITY),fDist(INFINITY),parent(-1),visited(false){}
} staStarNode;

typedef struct aStarDataStruct
{
    // vector< staStarNode > aStarNodes;
    deque< staStarNode > nextNodes; // corresponds to indices in aStarNodes
    deque< staStarNode > visitedNodes; // corresponds to indices in aStarNodes
    int startVertex;
    int finishVertex;

    aStarDataStruct(int s=0, int f=0):startVertex(s),finishVertex(f){}
} staStarData;

/*****************************************************************************
 * Class Definition
 ****************************************************************************/
class Localplan {
private:
    vector< vector<Vector2f> > obstacle_list; // list of obstacles
    vector< Vector2f > vis_vertices; // list of vertices of obstacles ... idx 0 and 1 reserved for start and finish points
    vector< vector<double> > vis_dist_matrix; // distance matrix for visibilty garph

    vector< unsigned int > temp_vertices;
    vector< vector<unsigned int> > obstacle_vertices; // grouping indices vis_vertices into obstales

    vector< int > aStarPath; // vertex indices indicating sequential waypoints for a path

    //------ Private Class Methods -------------------------------------------
    vector< Vector2f > getObstaclePoints(unsigned int o);
    unsigned int addTempVertex(Vector2f v);
    void clearTempVertices();
    bool createTempVertices();
    bool obstacle_collision(Vector2f p1, Vector2f p2, bool findAll, vector< vector<linestringboost> > &otpt);
    bool obstacle_collision_sub(Vector2f p1, Vector2f p2, const vector<unsigned int> &obsList, bool findAll, vector< vector<linestringboost> > &otpt);
    bool fixStartAndEnd();

    String edgeListStr() const; // get a string of all edges
    String vertexListStr() const; // get a string of all vertices

    // AStar algorithm functions
    double asHeuristicCost(int fromVertex, int toVertex);
    unsigned int asNodeVisitedIdx(int idx, const staStarData &data);
    staStarNode asFindAndRemoveBestNode(staStarData &d);
    vector<int> asReconstructBestPath(const staStarData &d);

public:
    static int ALGO_TYPE; // see defines above
    bool ignoreObs; // flag determining whether to ignore obstacles when planning
    bool ignoreOrientation; //flag to ignore orientation when planning

    geometry_msgs::PoseStamped goalPose; //goal pose sent from FSM 

    //------ Public Class Methods --------------------------------------------
    void reset();
    
    void setGoalInfo(Vector2f start, Vector2f finish);
    void setObstacleInfo(const vector< vector<Vector2f> > &obstacles);
    void updateGraphObstacles();
    void updateGraphGoalAndPlan();

    void asRunAStarSearch(int startVertex, int finishVertex); // Dont need to pass in vertices...
    bool pathExists() const;
    vector< Vector2f > getPath() const;
    bool checkPathCollision(const vector< vector<Vector2f> > &obstacles);

    visualization_msgs::MarkerArray getVisualMsg();
};

#endif // guard
