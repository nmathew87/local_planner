
/*************************************************************************//**
 *****************************************************************************
 * @file        local_path.cpp
 * @brief       Create Local Plan
 * @author      Talha Jawaid
 * @date        2013-02-12
 *****************************************************************************
 ****************************************************************************/

// http://www.ros.org/wiki/CppStyleGuide


//#define DEBUG
#define DEBUG
//#define USE_DUMMY_START
// #define USE_SIM_POSE

#include <ros/ros.h>
#include <math.h>
#include <time.h>
#include <sstream>
#include <map>
#include <Eigen/Core>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Polygon.h>
// #include "nasa_msgs/PolyArray.h"
#include "nasa_msgs/MapMonitor.h"
#include <local_planner/RobotStates.h>

#include <actionlib/server/simple_action_server.h>
//#include <move_base_msgs/MoveBaseAction.h>	//deprecated
#include <nasa_msgs/PlanAction.h>

// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/core/core.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include "opencv2/highgui/highgui.hpp"

// using namespace cv;
using namespace std;
using namespace Eigen;

#include "local_planner/localplan.hpp"
#include "local_planner/localctrlr.hpp"

/*****************************************************************************
* Node Variables
*****************************************************************************/
ros::Publisher vis_pub, ctrl_pub, pose_pub;
Localplan plan;
Localctrlr ctrlr;

#ifdef USE_DUMMY_START
Vector2f curPos = Vector2f(800,800);
#endif
robot_state rs;
Vector2f goalPos = Vector2f(INFINITY,INFINITY);

// Synchronization status
bool needsReplan = true;
bool gotObstacleData = false;
bool gotGoalData = false; // alternatively can just check  goalPos ...

// vel cmd publish timer
ros::Timer publishTimer;
double publishRate;
bool dataReady;
geometry_msgs::TwistStamped outputCmd;
//ros::Time accept_time;
	
//typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> action_server;
typedef actionlib::SimpleActionServer<nasa_msgs::PlanAction> action_server;
action_server *as;
bool preempt;
double nominal_stop_tol=0.5;


/*****************************************************************************
 * Function Prototypes
 ****************************************************************************/
bool isObsValid();
bool isGoalValid();
bool isStateValid();
void planPath();
void obsCallback(const nasa_msgs::MapMonitor::ConstPtr& msg);
// void obsCallback(const nasa_msgs::PolyArray::ConstPtr& msg);
// void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void goalCallback();
void goalPreemptCallback();
void timerCallback(const ros::TimerEvent&);
void speedCallback(const std_msgs::Float32::ConstPtr& maxSpeed);

#ifdef USE_SIM_POSE
void odomCallback(const simBotCommon::RobotStates msg);
#else
void odomCallback(const geometry_msgs::PoseStamped msg);;
#endif

void publishRobotState(const geometry_msgs::TwistStamped & heading);

/*****************************************************************************
 * Main
 ****************************************************************************/
int main(int argc, char **argv)
{
	ROS_INFO("Initializing local_path_node");
	ros::init(argc, argv, "local_path_node");
	ros::NodeHandle node_handle("");

    //////////////////////// Parameters
    // velocity command publish rate
    if ( node_handle.getParam("/local/publishrate", publishRate) ) {}
    else {
        publishRate = 0;
        node_handle.setParam("/local/publishrate", publishRate);
    }

    // maximum robot linear speed
	if ( node_handle.getParam("/local/maxlnrspeed", Localctrlr::MAX_LNR_SPEED) ) {}
	else {
        // SET DEFAULT IN localctrlr.cpp
		node_handle.setParam("/local/maxlnrspeed", Localctrlr::MAX_LNR_SPEED);
	}
    // maximum robot angular speed (radians/time)
    if ( node_handle.getParam("/local/maxangspeed", Localctrlr::MAX_ANG_SPEED) ) {}
    else {
        // SET DEFAULT IN localctrlr.cpp
        node_handle.setParam("/local/maxangspeed", Localctrlr::MAX_ANG_SPEED);
    }
    // goal location tolerance
    if ( node_handle.getParam("/local/stoptol", ctrlr.STOP_TOL) ) {
        nominal_stop_tol = ctrlr.STOP_TOL;
    }
    else {
        // SET DEFAULT IN localctrlr.cpp
         node_handle.setParam("/local/stoptol", ctrlr.STOP_TOL);
        nominal_stop_tol = ctrlr.STOP_TOL;
    }
    // path tolerance
    if ( node_handle.getParam("/local/pathtol", Localctrlr::PATH_TOL) ) {}
    else {
        // SET DEFAULT IN localctrlr.cpp
        node_handle.setParam("/local/pathtol", Localctrlr::PATH_TOL);
    }
    // desired yaw (radians): 99 to use initial yaw
    if ( node_handle.getParam("/local/desyaw", Localctrlr::DES_YAW) ) {}
    else {
        // SET DEFAULT IN localctrlr.cpp
        node_handle.setParam("/local/desyaw", Localctrlr::DES_YAW);
    }
    // yaw tolerance (radians)
    if ( node_handle.getParam("/local/yawtol", Localctrlr::YAW_TOL) ) {}
    else {
        // SET DEFAULT IN localctrlr.cpp
        node_handle.setParam("/local/yawtol", Localctrlr::YAW_TOL);
    }
    // yaw tolerance (radians)
    if ( node_handle.getParam("/local/yawmovecone", Localctrlr::YAW_MOVECONE) ) {}
    else {
        // SET DEFAULT IN localctrlr.cpp
        node_handle.setParam("/local/yawmovecone", Localctrlr::YAW_MOVECONE);
    }
    // distance beyond which robot "returns to path" instead of "tracks path"
    if ( node_handle.getParam("/local/perpreturn", Localctrlr::PERP_RETURN) ) {}
    else {
        // SET DEFAULT IN localctrlr.cpp
        node_handle.setParam("/local/perpreturn", Localctrlr::PERP_RETURN);
    }
    // gain for yaw velocity
    if ( node_handle.getParam("/local/Ksteer", Localctrlr::K_STEER) ) {}
    else {
        // SET DEFAULT IN localctrlr.cpp
        node_handle.setParam("/local/Ksteer", Localctrlr::K_STEER);
    }
    // gain to weight crosstrack error
    if ( node_handle.getParam("/local/Kxtrack", Localctrlr::K_CROSSTRACK) ) {}
    else {
        // SET DEFAULT IN localctrlr.cpp
        node_handle.setParam("/local/Kxtrack", Localctrlr::K_CROSSTRACK);
    }

    // algorithm type: bug or visibility
    if ( node_handle.getParam("/local/algotype", Localplan::ALGO_TYPE) ) {}
    else {
        // SET DEFAULT IN localplan.cpp
        node_handle.setParam("/local/algotype", Localplan::ALGO_TYPE);
    }
    
    // delta control, linear
    if ( node_handle.getParam("/local/d_linear", Localctrlr::D_LINEAR) ) {}
    else {
        // SET DEFAULT IN localctrlr.cpp
        node_handle.setParam("/local/d_linear", Localctrlr::D_LINEAR);
    }
    
    // delta control, rotational
    if ( node_handle.getParam("/local/d_rot", Localctrlr::D_ROT) ) {}
    else {
        // SET DEFAULT IN localctrlr.cpp
        node_handle.setParam("/local/d_rot", Localctrlr::D_ROT);
    }
    
    
    
    
    //////////////////////// Action Server stuff
    ROS_DEBUG ("Initializing Action Server");
    action_server localPlanAS(node_handle, "/planning/move", false);
    as = & localPlanAS;
    ROS_DEBUG ("Registering goal callback");
    localPlanAS.registerGoalCallback(&goalCallback);
    ROS_DEBUG ("Registering pre-empt callback");
    localPlanAS.registerPreemptCallback(&goalPreemptCallback);         
    ROS_DEBUG ("Starting Action Server...");
    localPlanAS.start();
    ROS_INFO ("Successfully Initialized Action Server!");

	//////////////////////// Publishers
	ROS_DEBUG("Creating Publishers");
    //publisher01 = node_handle.advertise<nav_msgs::Path>("/local/path", 1, true);
    vis_pub = node_handle.advertise<visualization_msgs::MarkerArray>("/local/test/visGraph", 1, true);
    pose_pub = node_handle.advertise<visualization_msgs::MarkerArray>("/local/test/robotPose", 1);
    ctrl_pub = node_handle.advertise<geometry_msgs::TwistStamped>("/cmd_vel_t", 1);

	//////////////////////// Subscribers
	ROS_DEBUG("Creating Subscribers");
	// ros::Subscriber subcriber02 = node_handle.subscribe("/local/test/goal", 1, goalCallback); // only for testing
	// ros::Subscriber subcriber01 = node_handle.subscribe("/global/obstacles", 1, obsCallback); // list of obstacles
	
    ros::Subscriber subcriber01 = node_handle.subscribe("/global/local_monitor", 1, obsCallback); // list of obstacles
    #ifdef USE_SIM_POSE
    ros::Subscriber subcriber02 = node_handle.subscribe("/robot_states", 1, odomCallback); // pose estimate
    #else
    ros::Subscriber subcriber02 = node_handle.subscribe("/mapping/ekf/pose", 1, odomCallback); // pose estimate
    #endif
    ros::Subscriber subcriber03 = node_handle.subscribe("/nasa/max_speed", 1, speedCallback); // list of obstacles

    //////////////////////// Timer
    if(publishRate > 0)
        publishTimer = node_handle.createTimer(ros::Duration(1/publishRate), timerCallback);

	/*****************************************************************************
	 * Main Loop
	 ****************************************************************************/
	ROS_DEBUG("Done initializaiton. Going into spin ...");
	ros::spin();
	ROS_INFO("Exit!");
	return 0;
}


/*****************************************************************************
 * Functions
 ****************************************************************************/
/****************************************************************************/
bool isStateValid()
{
    #ifdef USE_DUMMY_START
	return true;
    #else
	return !( (rs.x==INFINITY) || (rs.y==INFINITY) || (rs.yaw==INFINITY) );
    #endif
}

/****************************************************************************/
void planPath()
{
	if(!preempt && needsReplan)	
	{
		ROS_INFO("planPath: needsReplan");
		
		ROS_INFO("goal state %d", int(gotGoalData));
		ROS_INFO("obstacle state %d", int(gotObstacleData));
		
		
		if(gotObstacleData && gotGoalData && isStateValid())
		{
			ROS_INFO("planPath: replanning");
		    #ifdef USE_DUMMY_START
			plan.setGoalInfo(curPos, goalPos);
		    #else
			plan.setGoalInfo(Vector2f(rs.x,rs.y), goalPos);
		    #endif
			// plan.updateGraphGoalAndPlan();
			// plan.asRunAStarSearch(0,1);

			#ifdef DEBUG
			ROS_INFO("planPath: Publishing visual message");
			vis_pub.publish(plan.getVisualMsg());
			#endif

			ROS_INFO("planPath: sending plan to ctrlr");
			ctrlr.setPath(plan.getPath());
			ctrlr.ignoreOrientation = plan.ignoreOrientation;
            ctrlr.goalPose =plan.goalPose; 
			
			needsReplan = false;
		}
	}
}

/****************************************************************************/
void publishRobotState(const geometry_msgs::Twist & heading)
{
    String frameId = "/nasa";
    double robotLength = 0.5;

    // colours  
    std_msgs::ColorRGBA red, green, blue, white, black, yellow;
    red.r = 1;      red.g = 0;      red.b = 0;      red.a = 1;
    green.r = 0;    green.g = 1;    green.b = 0;    green.a = 1;
    blue.r = 0;     blue.g = 0;     blue.b = 1;     blue.a = 1;
    white.r = 1;    white.g = 1;    white.b = 1;    white.a = 1;
    black.r = 0;    black.g = 0;    black.b = 0;    black.a = 0;
    yellow.r = 1;    yellow.g = 1;    yellow.b = 0;    yellow.a = 1;

    // >>>>> message headers
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker msg_line; // edges
    visualization_msgs::Marker msg_arrow; // arrow
    geometry_msgs::Point msg_point, robCtr;
    stringstream text;
    int id = 0;

    // >>>>> general init
    msg_line.header.frame_id = frameId;
    msg_line.type = visualization_msgs::Marker::LINE_LIST;
    msg_line.action = visualization_msgs::Marker::ADD;
    msg_line.lifetime = ros::Duration(0);

    msg_arrow.header.frame_id = frameId;
    msg_arrow.type = visualization_msgs::Marker::ARROW;
    msg_arrow.action = visualization_msgs::Marker::ADD;
    msg_arrow.lifetime = ros::Duration(0);

    // >>>>> Construct Markers
    // robot - Lines
    msg_line.id = id;
    msg_line.ns = "robot";
    msg_line.scale.x = 0.5;
    msg_line.color = yellow;
    // lines are drawn b/w every pair of points (0-1, 2-3, ...)
    // robot center point
    robCtr.x = rs.x;
    robCtr.y = rs.y;
    robCtr.z = 1;
    // front
    msg_line.points.push_back(robCtr);
    msg_point.x = robCtr.x + robotLength*cos(rs.yaw);
    msg_point.y = robCtr.y + robotLength*sin(rs.yaw);
    msg_point.z = robCtr.z;
    msg_line.points.push_back(msg_point);
    // side 1
    msg_line.points.push_back(robCtr);
    msg_point.x = robCtr.x + robotLength*sin(rs.yaw);
    msg_point.y = robCtr.y - robotLength*cos(rs.yaw);
    msg_point.z = robCtr.z;
    msg_line.points.push_back(msg_point);
    // side 2
    msg_line.points.push_back(robCtr);
    msg_point.x = robCtr.x - robotLength*sin(rs.yaw);
    msg_point.y = robCtr.y + robotLength*cos(rs.yaw);
    msg_point.z = robCtr.z;
    msg_line.points.push_back(msg_point);
    marker_array.markers.push_back(msg_line);
    ++id;

    // heading
    msg_arrow.id = id++;
    msg_arrow.ns = "heading";
    msg_arrow.scale.x = 0.25;
    msg_arrow.scale.y = 0.25;
    msg_arrow.scale.z = 0.5;
    msg_arrow.color = blue;
    msg_arrow.points.push_back(robCtr);
    msg_point.x = robCtr.x + 2*robotLength*cos(rs.yaw+heading.angular.z);
    msg_point.y = robCtr.y + 2*robotLength*sin(rs.yaw+heading.angular.z);
    msg_point.z = robCtr.z;
    msg_arrow.points.push_back(msg_point);
    // msg_arrow.pose.position.x = robCtr.x;
    // msg_arrow.pose.position.y = robCtr.y;
    // msg_arrow.pose.position.z = robCtr.z;
    // msg_arrow.pose.orientation.z = heading.angular.z - rs.yaw;
    marker_array.markers.push_back(msg_arrow);

    // PUBLISH
    pose_pub.publish(marker_array);
}

/*****************************************************************************
 * Callback Functions
 ****************************************************************************/
/*****************************************************************************
 * timerCallback
 * Publish a velocity command if a new command is ready
 ****************************************************************************/
void timerCallback(const ros::TimerEvent&)
{
    //ROS_DEBUG("timerCallback: Enter");
    if(dataReady && !preempt && as->isActive()) ctrl_pub.publish(outputCmd);
    dataReady = false;
}

/*****************************************************************************
 * speedCallback
 * set maximum speed
 ****************************************************************************/
void speedCallback(const std_msgs::Float32::ConstPtr& maxSpeed)
{
    Localctrlr::MAX_LNR_SPEED = maxSpeed->data;
}

/*****************************************************************************
 * obsCallback
 * Update plan with obstacle data and create the graph using obstacle vertices
 * If goal data is present, then add the start/goal vertices to the graph
 ****************************************************************************/
// void obsCallback(const nasa_msgs::PolyArray::ConstPtr& msg)
void obsCallback(const nasa_msgs::MapMonitor::ConstPtr& msg)
{
	ROS_INFO("obsCallback: Entered");

	vector< vector<Vector2f> > obstacles;

    // dont do anything if preempted
    if(preempt)
    {
        ROS_INFO("obsCallback: PREEMPTED");
        return;
    }
	
	ROS_DEBUG("obsCallback: Obstacle Data Received, %lu objects.", msg->obstacles.size() );
    // Create list of obstacles
    // NOTE: First polygon is the environement boundary
    obstacles.push_back(vector<Vector2f>());
    for (unsigned int j=0; j < msg->boundaries[0].points.size(); j++) {
        obstacles.back().push_back( Vector2f(msg->boundaries[0].points[j].x, msg->boundaries[0].points[j].y) );
    }
    for (unsigned int i=0; i<msg->obstacles.size(); i++) {
    	obstacles.push_back(vector<Vector2f>());
        for (unsigned int j=0; j < msg->obstacles[i].points.size(); j++) {
            obstacles.back().push_back( Vector2f(msg->obstacles[i].points[j].x, msg->obstacles[i].points[j].y) );
        }
    }
    //if(!plan.pathExists() || plan.checkPathCollision(obstacles))
    {
    	ROS_DEBUG("obsCallback: path collision ... updating obstacles");
	    plan.setObstacleInfo(obstacles);
	    gotObstacleData = true;

	    plan.updateGraphObstacles();
	    needsReplan = true;
	}
	
	// just incase got 2 things at once ... avoid replanning repeatedly
	//ROS_DEBUG("obsCallback: spinOnce");
	//ros::spinOnce();
	
	ROS_DEBUG("obsCallback: Calling planPath");
	planPath();

	ROS_DEBUG("Exiting obsCallback");
}

/*****************************************************************************
 * goalCallback
 * Update start/goal information in plan
 * If obstacle data has already arrived, then add the start/goal vertices
 * otherwise cannot do anything since visibility is unknown
 ****************************************************************************/
// void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
void goalCallback()
{
	//accept_time = ros::Time::now();
	nasa_msgs::PlanGoal buffer;
    //move_base_msgs::MoveBaseGoal buffer;

	buffer = *as->acceptNewGoal();
	
    Vector2f newGoal = Vector2f(buffer.target_pose.pose.position.x, buffer.target_pose.pose.position.y);
	
	ROS_INFO("Entered goalCallback: Goal: %f, %f", newGoal.x(), newGoal.y() );
	
	//2 new modes. ignore obstacles or not.
	//buffer.ignore_obs = true   	:ignore all obstacles 
	//bugger.ignore_obs = false 	:normal mode
    
    plan.ignoreObs = buffer.ignore_obstacles;
    plan.ignoreOrientation = buffer.ignore_orientation;
    plan.goalPose = buffer.target_pose;
    if( (!gotGoalData) || ((newGoal-goalPos).norm() > 1e-8) ) {
        goalPos = newGoal;
        needsReplan = true;
    }
	gotGoalData = true;
    preempt = false;
	ROS_INFO("goalCallback: Got goal (%f,%f)", goalPos.x(),goalPos.y());
	
	// just incase got 2 things at once ... avoid replanning repeatedly
	//ROS_DEBUG("goalCallback: spinOnce");
	//ros::spinOnce();
	
	//setting stop tolerance from the goalcallback
		
	if(buffer.acurate) //do we want accurate positioning? 
		ctrlr.STOP_TOL = 0.15;
	else
		ctrlr.STOP_TOL = nominal_stop_tol;
		
	
	ROS_DEBUG("goalCallback: Calling planPath");
	planPath();

	ROS_DEBUG("Exiting goalCallback");
}

/*****************************************************************************
 * odomCallback
 * Get a new pose estimate
 ****************************************************************************/
#ifdef USE_SIM_POSE
void odomCallback(const simBotCommon::RobotStates msg)
#else
void odomCallback(const geometry_msgs::PoseStamped msg)
#endif
{	
    vel_cmd ctrlCmd;

	// ROS_INFO("In odomCallBack");
    
    #ifdef USE_SIM_POSE
    
    // Y forard X right, CCW pos. 
    rs.x = msg.x;
    rs.y = msg.y ;
    rs.yaw = msg.yaw_psi;
    ROS_DEBUG("odomCallback: Current Simulation Yaw: %f", rs.yaw);

    #else
    
    rs.x = msg.pose.position.x;
    rs.y = msg.pose.position.y;

    float q0, q1,q2,q3;
    q0 = msg.pose.orientation.w;
    q1 = msg.pose.orientation.x;
    q2 = msg.pose.orientation.y;
    q3 = msg.pose.orientation.z;

    // Converting to Yaw
    rs.yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
    ROS_DEBUG("odomCallback: Current Mapping Yaw: %f", rs.yaw);

    #endif
    ROS_DEBUG("odomCallback: Pose Updated: (X,Y,YAW)=( %f,%f,%f)",rs.x,rs.y,rs.yaw );
    
    // dont do anything if preempted
    if(preempt || !(as->isActive()))
    {
        if(preempt) ROS_DEBUG("odomCallback: PREEMPTED");
        else ROS_DEBUG("odomCallback: INACTIVE");

        #ifdef DEBUG
        publishRobotState(ctrlCmd.cmd.twist);
        #endif
        return;
    }

    // call plan path incase a new plan is needed and no valid current pose was available
    // ... really only need this right at the start
    // ROS_INFO("odomCallback: Calling planPath");
	planPath();
	
	// call controller to get new velocity commands
    if(ctrlr.hasPath())
    {
        ctrlCmd = ctrlr.getVelCmd(rs);
        if(ctrlCmd.finished) {
            ROS_INFO("odomCallback: SUCCESS ... Arrived at goal");
            //if ((ros::Time::now().toSec() - accept_time.toSec()) < 0.5){
				//ros::Duration(0.5).sleep(); //make sure we give goal accept state at least half a second so FSM picks up transition
			//}
            as->setSucceeded();
            ctrlr.clearPath();
            gotGoalData = false;
            // ctrl_pub.publish(geometry_msgs::TwistStamped()); // publish v=0
            outputCmd = geometry_msgs::TwistStamped();
            dataReady = true;
        } else {
            ROS_DEBUG("odomCallback: vel_cmd lnr(%f,%f), ang(%f)", ctrlCmd.cmd.twist.linear.x, ctrlCmd.cmd.twist.linear.y, ctrlCmd.cmd.twist.angular.z);
            if( isnan(ctrlCmd.cmd.twist.linear.x) || isnan(ctrlCmd.cmd.twist.linear.y) || isnan(ctrlCmd.cmd.twist.angular.z) ) {
                ROS_INFO("odomCallback: ctrlCmd.cmd.twist is NaN!!");
                // ctrl_pub.publish(geometry_msgs::TwistStamped()); // publish v=0
                outputCmd = geometry_msgs::TwistStamped();
                dataReady = true;
            } else {
                // ctrl_pub.publish(ctrlCmd.cmd);
                outputCmd = ctrlCmd.cmd;
                dataReady = true;
            }
        }
    }
    else
    {
        // publish 0 command
        // ctrl_pub.publish(geometry_msgs::TwistStamped());
        outputCmd = geometry_msgs::TwistStamped();
        dataReady = true;
    }
    if(publishRate == 0)
    {
        // timer is inactive
        timerCallback(ros::TimerEvent());
        // ROS_INFO("odomCallback: Publishing vel_cmd");
        // ctrl_pub.publish(outputCmd);
        // dataReady = false;
    }
    #ifdef DEBUG
    publishRobotState(ctrlCmd.cmd.twist);
    #endif
}

/*****************************************************************************
 * goalPreemptCallback
 * Cancel a goal
 ****************************************************************************/
void goalPreemptCallback()
{
    ROS_INFO ("------------->PRE-EMPT request received");
    ctrl_pub.publish( geometry_msgs::TwistStamped());
    as->setPreempted();
    preempt = true;
}
