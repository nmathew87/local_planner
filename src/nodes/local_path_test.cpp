
/*************************************************************************//**
 *****************************************************************************
 * @file        local_path.cpp
 * @brief       Crreate Local Plan
 * @author      Talha Jawaid
 * @date        2013-02-12
 *****************************************************************************
 ****************************************************************************/

// http://www.ros.org/wiki/CppStyleGuide


#define DEBUG

#include <ros/ros.h>
#include <math.h>
#include <time.h>
#include <sstream>
#include <map>
#include <Eigen/Core>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <nav_msgs/Path.h>
// #include <nav_msgs/Odometry.h>
#include <geometry_msgs/Polygon.h>
// #include <geometry_msgs/PolygonStamped.h>
#include "nasa_msgs/PolyArray.h"
#include "nasa_msgs/MapMonitor.h"
#include "local_planner/map_tools.hpp"

using namespace std;
using namespace Eigen;

/*****************************************************************************
 * gloabal vars
 ****************************************************************************/
ros::Publisher publisher01;
ros::Publisher publisher02;
ros::Publisher publisher03;

/*****************************************************************************
 * Function prototypes
 ****************************************************************************/
void publishAndVisualize(nasa_msgs::PolyArray);
geometry_msgs::Point32 getPoint32(float x, float y);
geometry_msgs::PoseStamped getGoalMsg(float x, float y);
nasa_msgs::PolyArray getObsMsg(int );
vector< vector<Vector2f> > polyList2pointLists(const vector<geometry_msgs::Polygon> &polys);

/*****************************************************************************
 * Main
 ****************************************************************************/
int main(int argc, char **argv)
{
    ROS_INFO("Initializing local_path");
    /*****************************************************************************
     * INIT
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     ****************************************************************************/
    ros::init(argc, argv, "local_path_test");

    /*****************************************************************************
     * NODEHANDLE
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     ****************************************************************************/
    ros::NodeHandle node_handle("");

    /*****************************************************************************
     * PUBLISHER
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. 
     * advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The first parameter is the name of the topic
     * The second parameter is the size of the queue used for publishing messages.
     * The third parameter specifies whether to latch the message. If true,
     * the last message published on this topic will be saved and sent to new
     * subscribers when they connect
     ****************************************************************************/
    ROS_INFO("Creating Publishers");
    // publisher01 = node_handle.advertise<nasa_msgs::PolyArray>("/global/obstacles", 1, true);
    publisher01 = node_handle.advertise<nasa_msgs::MapMonitor>("/global/local_monitor", 1, true);
    // publisher02 = node_handle.advertise<geometry_msgs::PolygonStamped>("/local/test/obstacles", 100, true); // for RViz
    publisher02 = node_handle.advertise<visualization_msgs::MarkerArray>("/local/test/obstacles", 1, true); // for RViz
    publisher03 = node_handle.advertise<geometry_msgs::PoseStamped>("/local/test/goal", 1, true);

    /*****************************************************************************
     * PARAMS
     ****************************************************************************/
    int testnum = 0;
    if ( node_handle.getParam("/local/test/testnum", testnum) ) {}
    else node_handle.setParam("/local/test/testnum", testnum);

    /*****************************************************************************
     * Main Loop
     ****************************************************************************/
    ROS_INFO("Done initializaiton.");

    while(ros::ok())
    {
        // ROS_INFO("First pass");
        // ROS_INFO("Publishing goal message");
        // publisher03.publish( getGoalMsg(23,12) );
        // publishAndVisualize( getObsMsg(3) );
        // ros::Duration(5).sleep();

        // if(!ros::ok()) return 0;

        // ROS_INFO("Second pass");
        // ROS_INFO("Publishing goal message");
        // publisher03.publish( getGoalMsg(24,12) );
        // ros::Duration(5).sleep();

        // if(!ros::ok()) return 0;

        // ROS_INFO("Third pass");
        // publishAndVisualize( getObsMsg(2) );
        // ros::Duration(5).sleep();

        // if(!ros::ok()) return 0;

        // ROS_INFO("Fourth pass");
        // ROS_INFO("Publishing goal message");
        // publisher03.publish( getGoalMsg(23,12) );
        // publishAndVisualize( getObsMsg(4) );
        // ros::Duration(5).sleep();

        ROS_INFO("Fourth pass");
        ROS_INFO("Publishing goal message");
        publisher03.publish( getGoalMsg(23,12) );
        publishAndVisualize( getObsMsg(testnum) );
        ros::Duration(5).sleep();
    }

    ROS_INFO("Exit!");
    return 0;
}

/*****************************************************************************
 * Publishes obstacles and RViz markers for obstacles
 ****************************************************************************/
void publishAndVisualize(nasa_msgs::PolyArray testMsg)
{
    ROS_DEBUG("testMsg.polygons.size()=%lu",testMsg.polygons.size());

    ROS_INFO("Publishing obstacles message");

    // Publish obstacles
    nasa_msgs::MapMonitor newMsg;
    newMsg.obstacles = testMsg.polygons;
    newMsg.obstacles.erase(newMsg.obstacles.begin());
    newMsg.boundaries.push_back(testMsg.polygons[0]);
    publisher01.publish(newMsg);

    // Publish Obstacle markers
    VisualMsg visMsg;
    visMsg.addPoly(polyList2pointLists(testMsg.polygons), visMsg.blue, 0.1, "obs");
    publisher02.publish(visMsg.msg);
}

/*****************************************************************************
 * Creates a geometry_msgs::Point32 structure
 ****************************************************************************/
geometry_msgs::Point32 getPoint32(float x, float y)
{
    geometry_msgs::Point32 p;
    p.x = x;
    p.y = y;
    p.z = 0;
    return p;
}

/*****************************************************************************
 * Creates a geometry_msgs::PoseStamped structure
 ****************************************************************************/
geometry_msgs::PoseStamped getGoalMsg(float x, float y)
{
    geometry_msgs::PoseStamped goalMsg;
    goalMsg.header.frame_id = "/odom";
    goalMsg.pose.position.x = x;
    goalMsg.pose.position.y = y;
    return goalMsg;
}

/*****************************************************************************
 * Converts a geometry_msgs::polygon array into vertex lists
 ****************************************************************************/
vector< vector<Vector2f> > polyList2pointLists(const vector<geometry_msgs::Polygon> &polys)
{
    vector< vector<Vector2f> > v;
    for(unsigned int i = 0; i< polys.size(); ++i) // for each polygon
    {
        v.push_back(vector<Vector2f>());
        for (unsigned int j = 0; j < polys[i].points.size(); ++j) // add all the points
        {
            v.back().push_back(Vector2f(polys[i].points[j].x, polys[i].points[j].y));
        }
    }

    return v;
}

/*****************************************************************************
 * Test Messages
 *
 * nasa_msgs/PolyArray:
 * geometry_msgs/Polygon[] polygons
 *   geometry_msgs/Point32[] points
 *     float32 x
 *     float32 y
 *     float32 z
 ****************************************************************************/
nasa_msgs::PolyArray getObsMsg(int testnum)
{
    nasa_msgs::PolyArray msg;

    // Environment Boundary (first obstacle)
    msg.polygons.push_back(geometry_msgs::Polygon());
    msg.polygons.back().points.push_back( getPoint32(0,0) );
    msg.polygons.back().points.push_back( getPoint32(0,50) );
    msg.polygons.back().points.push_back( getPoint32(50,50) );
    msg.polygons.back().points.push_back( getPoint32(50,0) );

    if(testnum == 1) {
        // Obstacle 1
        msg.polygons.push_back(geometry_msgs::Polygon());
        msg.polygons.back().points.push_back( getPoint32(5,5) );
        msg.polygons.back().points.push_back( getPoint32(5,10) );
        msg.polygons.back().points.push_back( getPoint32(10,10) );
        msg.polygons.back().points.push_back( getPoint32(10,5) );

        // Obstacle 2
        msg.polygons.push_back(geometry_msgs::Polygon());
        msg.polygons.back().points.push_back( getPoint32(5,15) );
        msg.polygons.back().points.push_back( getPoint32(10,20) );
        msg.polygons.back().points.push_back( getPoint32(10,15) );

        // Obstacle 3
        msg.polygons.push_back(geometry_msgs::Polygon());
        msg.polygons.back().points.push_back( getPoint32(15,15) );
        msg.polygons.back().points.push_back( getPoint32(17,18) );
        msg.polygons.back().points.push_back( getPoint32(20,17) );
        msg.polygons.back().points.push_back( getPoint32(20,8) );
        msg.polygons.back().points.push_back( getPoint32(18,2) );
        msg.polygons.back().points.push_back( getPoint32(13,10) );
        msg.polygons.back().points.push_back( getPoint32(17,12) );
    } else if(testnum == 2) {
        // Obstacle 1
        msg.polygons.push_back(geometry_msgs::Polygon());
        msg.polygons.back().points.push_back( getPoint32(5,5) );
        msg.polygons.back().points.push_back( getPoint32(5,10) );
        msg.polygons.back().points.push_back( getPoint32(10,10) );
        msg.polygons.back().points.push_back( getPoint32(10,5) );

        // Obstacle 2
        msg.polygons.push_back(geometry_msgs::Polygon());
        msg.polygons.back().points.push_back( getPoint32(15,7) );
        msg.polygons.back().points.push_back( getPoint32(15,12) );
        msg.polygons.back().points.push_back( getPoint32(20,12) );
        msg.polygons.back().points.push_back( getPoint32(20,7) );
    } else if(testnum == 3) {
        // Obstacle 1
        msg.polygons.push_back(geometry_msgs::Polygon());
        msg.polygons.back().points.push_back( getPoint32(3,4) );
        msg.polygons.back().points.push_back( getPoint32(3,7) );
        msg.polygons.back().points.push_back( getPoint32(4,8) );
        msg.polygons.back().points.push_back( getPoint32(6,6) );
        msg.polygons.back().points.push_back( getPoint32(6,5) );
        msg.polygons.back().points.push_back( getPoint32(4,4) );
        msg.polygons.back().points.push_back( getPoint32(4,2) );

        // Obstacle 2
        msg.polygons.push_back(geometry_msgs::Polygon());
        msg.polygons.back().points.push_back( getPoint32(10,6) );
        msg.polygons.back().points.push_back( getPoint32(11,7) );
        msg.polygons.back().points.push_back( getPoint32(11,9) );
        msg.polygons.back().points.push_back( getPoint32(13,9) );
        msg.polygons.back().points.push_back( getPoint32(13,7) );
        msg.polygons.back().points.push_back( getPoint32(11,5) );

        // Obstacle 3
        msg.polygons.push_back(geometry_msgs::Polygon());
        msg.polygons.back().points.push_back( getPoint32(18,10) );
        msg.polygons.back().points.push_back( getPoint32(18,11) );
        msg.polygons.back().points.push_back( getPoint32(18.5,12) );
        msg.polygons.back().points.push_back( getPoint32(20,13) );
        msg.polygons.back().points.push_back( getPoint32(21,13) );
        msg.polygons.back().points.push_back( getPoint32(22.5,12) );
        msg.polygons.back().points.push_back( getPoint32(23,11) );
        msg.polygons.back().points.push_back( getPoint32(23,10) );
        msg.polygons.back().points.push_back( getPoint32(22.5,9) );
        msg.polygons.back().points.push_back( getPoint32(21,8) );
        msg.polygons.back().points.push_back( getPoint32(20,8) );
        msg.polygons.back().points.push_back( getPoint32(18.5,9) );
    } else if(testnum == 4) {
        // Obstacle 1
        msg.polygons.push_back(geometry_msgs::Polygon());
        msg.polygons.back().points.push_back( getPoint32(4,2) );
        msg.polygons.back().points.push_back( getPoint32(6,4) );
        msg.polygons.back().points.push_back( getPoint32(7,3) );
        msg.polygons.back().points.push_back( getPoint32(6,1) );

        // Obstacle 2
        msg.polygons.push_back(geometry_msgs::Polygon());
        msg.polygons.back().points.push_back( getPoint32(10,2) );
        msg.polygons.back().points.push_back( getPoint32(11,5) );
        msg.polygons.back().points.push_back( getPoint32(13.5,4) );
        msg.polygons.back().points.push_back( getPoint32(12,3) );
        msg.polygons.back().points.push_back( getPoint32(13,1.5) );
        msg.polygons.back().points.push_back( getPoint32(11,1) );
    } else if(testnum == 5) {
        // closed in
        // Obstacle 1
        msg.polygons.push_back(geometry_msgs::Polygon());
        msg.polygons.back().points.push_back( getPoint32(5,6) );
        msg.polygons.back().points.push_back( getPoint32(5,10) );
        msg.polygons.back().points.push_back( getPoint32(10,10) );
        msg.polygons.back().points.push_back( getPoint32(10,5) );
        msg.polygons.back().points.push_back( getPoint32(5,5) );
        msg.polygons.back().points.push_back( getPoint32(5,6) );
        msg.polygons.back().points.push_back( getPoint32(9,6) );
        msg.polygons.back().points.push_back( getPoint32(9,9) );
        msg.polygons.back().points.push_back( getPoint32(6,9) );
        msg.polygons.back().points.push_back( getPoint32(6,6) );

        // self intersecting
        msg.polygons.push_back(geometry_msgs::Polygon());
        msg.polygons.back().points.push_back( getPoint32(20,20) );
        msg.polygons.back().points.push_back( getPoint32(20,30) );
        msg.polygons.back().points.push_back( getPoint32(30,20) );
        msg.polygons.back().points.push_back( getPoint32(30,30) );
    } else if(testnum == 6) {
        // testing iteration of visibility
        // Obstacle 1
        msg.polygons.push_back(geometry_msgs::Polygon());
        msg.polygons.back().points.push_back( getPoint32(5,1) );
        msg.polygons.back().points.push_back( getPoint32(5,5) );
        msg.polygons.back().points.push_back( getPoint32(10,5) );
        msg.polygons.back().points.push_back( getPoint32(10,1) );

        // Obstacle 2
        msg.polygons.push_back(geometry_msgs::Polygon());
        msg.polygons.back().points.push_back( getPoint32(15,1) );
        msg.polygons.back().points.push_back( getPoint32(15,5) );
        msg.polygons.back().points.push_back( getPoint32(20,5) );
        msg.polygons.back().points.push_back( getPoint32(20,1) );

        // Obstacle 3
        msg.polygons.push_back(geometry_msgs::Polygon());
        msg.polygons.back().points.push_back( getPoint32(11,3) );
        msg.polygons.back().points.push_back( getPoint32(11,6) );
        msg.polygons.back().points.push_back( getPoint32(12,3) );

        // Obstacle 2
        msg.polygons.push_back(geometry_msgs::Polygon());
        msg.polygons.back().points.push_back( getPoint32(10.5,0.5) );
        msg.polygons.back().points.push_back( getPoint32(10.5,1.75) );
        msg.polygons.back().points.push_back( getPoint32(12,1.5) );
    }
    
    return msg;
}
