/*************************************************************************//**
 *****************************************************************************
 * @file        localctrlr.hpp
 * @brief       Provide methods for outputting velocity commands given a path
 * @author      Talha Jawaid
 * @date        2013-02-13
 *****************************************************************************
 ****************************************************************************/

#ifndef LOCAL_CONTROLLER_H      // guard
#define LOCAL_CONTROLLER_H

// #define ACKERMANN_DRIVE
//#define USE_PRESET_YAW

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
 #include <geometry_msgs/TwistStamped.h> 
#include <tf/tf.h>

//#include <cv_bridge/cv_bridge.h>
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include "opencv2/highgui/highgui.hpp"

#include "map_tools.hpp"
// #include "general.hpp"
// #include "greedycut.hpp"

//#include <boost/geometry.hpp>
// #include <boost/geometry/geometries/geometries.hpp>
//#include <boost/geometry/geometries/point_xy.hpp>
//#include <boost/geometry/geometries/polygon.hpp>
//#include <boost/geometry/geometries/linestring.hpp> 
// #include <boost/geometry/io/wkt/wkt.hpp>
//
//typedef boost::geometry::model::d2::point_xy<double> point2dboost;
//typedef boost::geometry::model::polygon<point2dboost> polygonboost;
//typedef boost::geometry::model::linestring<point2dboost > linestringboost;

using namespace std;
using namespace Eigen;

/*****************************************************************************
 * Struct Definitions
 ****************************************************************************/
typedef struct robot_state_struct {
    double x;
    double y;
    double yaw;
    robot_state_struct(double xi=INFINITY,double yi=INFINITY,double ai=INFINITY)
        :x(xi),y(yi),yaw(ai){}
} robot_state;

typedef struct vel_cmd_struct {
    geometry_msgs::TwistStamped cmd;
    bool finished;
    vel_cmd_struct():finished(false){}
} vel_cmd;

/*****************************************************************************
 * Class Definition
 ****************************************************************************/
class Localctrlr {
private:
    vector< Vector2f > waypoints;
    unsigned int curSegment;
    double initialYaw;
    
public:
    static double MAX_LNR_SPEED; // max robot linear speed
    static double MAX_ANG_SPEED; // max robot angular speed
	double STOP_TOL; // inflation of the goal points
    static double PATH_TOL; // inflation of the path
    static double DES_YAW; // desired yaw (if using preset yaw, 99 for initial yaw)
    static double YAW_TOL; // heading angle tolerance
    static double YAW_MOVECONE; // cone in which to move forward and turn
    static double PERP_RETURN; // distance to require perp return to path
    static double K_STEER; // gain for angular velocity
    static double K_CROSSTRACK; // ratio of moving towards path to moving forward
	static double D_LINEAR;
	static double D_ROT;
	static geometry_msgs::TwistStamped OLD_TWIST;

    geometry_msgs::PoseStamped goalPose; //goal from FSM
    bool ignoreOrientation; //heading lock?
    
    Localctrlr():initialYaw(INFINITY) {
		STOP_TOL = 0.5; //initial 
	}
    double getDesiredYaw();
    void clearPath();
    void setPath(const vector<Vector2f> &newPath);
    bool hasPath();
    
    vel_cmd getVelCmd(robot_state rs);
    
    // visualization_msgs::MarkerArray getVisualMsg();
};
/*****************************************************************************
 * Functions
 ****************************************************************************/

int sign(double num);
	
#endif // guard
