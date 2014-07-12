/// Includes
#include "local_planner/localctrlr.hpp"


double Localctrlr::MAX_LNR_SPEED = 2;
double Localctrlr::MAX_ANG_SPEED = 0.3;
double Localctrlr::DES_YAW = 99;
double Localctrlr::YAW_TOL = 0.16;
double Localctrlr::YAW_MOVECONE = M_PI/10;
//double Localctrlr::STOP_TOL = 0.5;
double Localctrlr::PATH_TOL = 0.1; //Localctrlr::STOP_TOL;
double Localctrlr::PERP_RETURN = 3;
double Localctrlr::K_STEER = 0.5; 
double Localctrlr::K_CROSSTRACK = 0.2;
double Localctrlr::D_LINEAR = 0.05;
double Localctrlr::D_ROT = 0.05;
geometry_msgs::TwistStamped Localctrlr::OLD_TWIST;

/*****************************************************************************
 * Functions
 ****************************************************************************/

/*****************************************************************************
 * getDesiredYaw
 * @brief   gets the desired yaw angle
 ****************************************************************************/
double Localctrlr::getDesiredYaw()
{
	if(DES_YAW == 99) return initialYaw;

	if(DES_YAW > M_PI) DES_YAW -= 2*M_PI;
	else if (DES_YAW <= -M_PI) DES_YAW += 2*M_PI;

	return DES_YAW;
}

/*****************************************************************************
 * clearPath
 * @brief   clears the current path
 ****************************************************************************/
void Localctrlr::clearPath()
{
    waypoints.clear();
    curSegment = 0;
}

/*****************************************************************************
 * clearPath
 * @brief   clears the current path
 ****************************************************************************/
void Localctrlr::setPath(const vector<Vector2f> &newPath)
{
    waypoints = newPath;
    curSegment = 0;
}

/*****************************************************************************
 * hasPath
 * @brief   is path valid
 ****************************************************************************/
bool Localctrlr::hasPath()
{
	return (waypoints.size() > 0);
}

/*****************************************************************************
 * getVelCmd
 * @brief   returns the velocity to move at
 ****************************************************************************/
vel_cmd Localctrlr::getVelCmd(robot_state rs)
{
    vel_cmd velCmd;
    Vector2f robPos, startPt, endPt, relRobPos, projPos, pathVector, newHeading, xerrVector;
    double projRelNorm, segLength;

    // just incase:
    if( isnan(rs.x) || isnan(rs.y) || isnan(rs.yaw) ) {
    	ROS_ERROR("Localctrlr::getVelCmd: rs is NaN!!!");
    	return velCmd;
    }

    // set initial yaw if not already set
    if(initialYaw == INFINITY) initialYaw = rs.yaw;

    // check if curSegment is valid
    if( !hasPath() || ((int)curSegment >= (int)waypoints.size()-1) ) {
    	ROS_WARN("Localctrlr::getVelCmd: no path OR curSegment=%u >= waypoints.size()=%lu", curSegment, waypoints.size());
    	return velCmd;
    }

    robPos = Vector2f(rs.x,rs.y);
    startPt = waypoints[curSegment];
    endPt = waypoints[curSegment+1];

	// Check if reached end of segment
	if( ((Vector2f(rs.x,rs.y) - endPt).norm() <= STOP_TOL) || ((endPt - startPt).norm() < 1e-8) )
	{
		ROS_INFO("Localctrlr::getVelCmd: Finished segment %u", curSegment);
		++curSegment;
		if((int)curSegment >= (int)waypoints.size()-1) {
			velCmd.finished = true;
			return velCmd;
		}
	}

	// Get closest point on line segment
	// (pathVector.relRobPos) = ||pathVector|| ||relRobPos|| cos(theta) = ||pathVector|| ||projection||
	pathVector = endPt - startPt;
	relRobPos = robPos - startPt; // position relative to segment start vertex
	segLength = pathVector.norm();
	projRelNorm = relRobPos.dot(pathVector)/segLength; // +/- distance from start pt to projected point
	projPos = startPt + projRelNorm*(pathVector/segLength); // abs pos of projected point

	// ROS_INFO("Localctrlr::getVelCmd: pathVector%s, relRobPos%s, segLength(%f), projRelNorm(%f), projPos%s", vector2Str(pathVector).c_str(), vector2Str(relRobPos).c_str(), segLength, projRelNorm, vector2Str(projPos).c_str());

	if(projRelNorm - segLength > PATH_TOL)
	{
		ROS_WARN("Localctrlr::getVelCmd: projection beyond segment %u endpoint", curSegment);
		newHeading = endPt - robPos;
	}
	else if(projRelNorm < (0-PATH_TOL))
	{
		ROS_WARN("Localctrlr::getVelCmd: projection beyond segment %u startpoint", curSegment);
		newHeading = startPt - robPos;
	}
	else
	{
		// ROS_INFO("Localctrlr::getVelCmd: projection on segment %u (%f)", curSegment, projRelNorm);
		xerrVector = (projPos - robPos);

		// ROS_INFO("Localctrlr::getVelCmd: xerrVector%s, norm=%f", vector2Str(xerrVector).c_str(), xerrVector.norm());
		if(xerrVector.norm() > PERP_RETURN) { // if too far, return to path first
			newHeading = xerrVector;
		}
        else if(xerrVector.norm() > PATH_TOL) {
		    newHeading = ((endPt-projPos)/abs(segLength-projRelNorm))*MAX_LNR_SPEED + K_CROSSTRACK*xerrVector;
	    }
	    else { // close enough ... position is inaccurate
            newHeading = ((endPt-projPos)/abs(segLength-projRelNorm))*MAX_LNR_SPEED;
        }
    }

    // NOTE: newHeading is in the absolute frame of reference ... should convert to robot frame
	double headAngle = atan2(newHeading.y(), newHeading.x());
	double relAngle = headAngle - rs.yaw;
	if(relAngle <= -M_PI) relAngle += 2.0*M_PI;
	else if(relAngle > M_PI) relAngle -= 2.0*M_PI;

	#ifdef ACKERMANN_DRIVE

	velCmd.cmd.twist.angular.z = K_STEER*relAngle;
	// velCmd.cmd.twist.yaw = atan2(newHeading.y(), newHeading.x());
	velCmd.cmd.twist.angular.z = min( max(velCmd.cmd.twist.angular.z, -MAX_ANG_SPEED), MAX_ANG_SPEED ); // saturate at MAX_ANG_SPEED
	
	if(abs(relAngle) > M_PI/8) velCmd.cmd.twist.linear.x = 0;
	else velCmd.cmd.twist.linear.x = MAX_LNR_SPEED;

	#else


	double yawError;
	#ifdef USE_PRESET_YAW
	yawError = getDesiredYaw()-rs.yaw;
	if(yawError <= -M_PI) yawError += 2.0*M_PI;
	else if(yawError > M_PI) yawError -= 2.0*M_PI;
	#else
    if(ignoreOrientation) {
	    yawError = relAngle;
    } else {
        double desired_heading = tf::getYaw(goalPose.pose.orientation);
	if(isnan(desired_heading)) desired_heading = 0;
        yawError = desired_heading - rs.yaw;
        if(yawError <= -M_PI) yawError += 2.0*M_PI;
        else if(yawError > M_PI) yawError -= 2.0*M_PI;
    }
    #endif

	if(abs(yawError) >= M_PI/4) {
		 // maximum yaw speed if error is too big
		velCmd.cmd.twist.angular.z = ((yawError < 0)?(-1):1)*MAX_ANG_SPEED;
		velCmd.cmd.twist.linear.x = 0;
		velCmd.cmd.twist.linear.y = 0;
	} else {
		// gradually adjust yaw and move fwd
		// cone of ignorance
		if(abs(yawError) >= YAW_TOL) {
			velCmd.cmd.twist.angular.z = min( max(K_STEER*yawError, -MAX_ANG_SPEED), MAX_ANG_SPEED ); // saturate at MAX_ANG_SPEED
		}
		else {
			velCmd.cmd.twist.angular.z = 0;
		}
		
		// cone in which to move fwd and turn
		if(abs(yawError) < YAW_MOVECONE) {
			double cur_speed = min(MAX_LNR_SPEED, max(MAX_LNR_SPEED/2,MAX_LNR_SPEED/4*(endPt-projPos).norm()));
			velCmd.cmd.twist.linear.x = cur_speed*cos(relAngle);
			velCmd.cmd.twist.linear.y = cur_speed*sin(relAngle);
		}
	}
	//rate control to smooth acceleration
	double delta_x = velCmd.cmd.twist.linear.x - OLD_TWIST.twist.linear.x;
	double delta_y = velCmd.cmd.twist.linear.y - OLD_TWIST.twist.linear.y;
	double delta_r = velCmd.cmd.twist.angular.z - OLD_TWIST.twist.angular.z;
	
	if ( fabs(delta_x) > D_LINEAR)
		velCmd.cmd.twist.linear.x = OLD_TWIST.twist.linear.x + D_LINEAR*sign(delta_x);

	if ( fabs(delta_y) > D_LINEAR)
		velCmd.cmd.twist.linear.y = OLD_TWIST.twist.linear.y + D_LINEAR*sign(delta_y);

	if ( fabs(delta_r) > D_LINEAR)
		velCmd.cmd.twist.angular.z = OLD_TWIST.twist.angular.z + D_ROT*sign(delta_r);

	#endif
	OLD_TWIST = velCmd.cmd;

//SCREW YOU PLANNING!
/*
	headAngle = atan2(pathVector.y(), pathVector.x());
	relAngle = headAngle - rs.yaw;
	if(relAngle <= -M_PI) relAngle += 2.0*M_PI;
	else if(relAngle > M_PI) relAngle -= 2.0*M_PI;


	if(abs(relAngle) < M_PI/6) {
		velCmd.cmd.twist.angular.z = ((relAngle < 0)?(-1):1)*MAX_ANG_SPEED;
		velCmd.cmd.twist.linear.x = max(min(K_CROSSTRACK*pathVector.norm(), MAX_LNR_SPEED), 0.2);
		velCmd.cmd.twist.linear.y = 0;
	} else {
		velCmd.cmd.twist.angular.z = K_STEER*relAngle;
		velCmd.cmd.twist.linear.x = 0;
		velCmd.cmd.twist.linear.y = 0;
	}
*/
	return velCmd;
}

int sign(double num){
	if (num < 0){
		return -1;
	}
	else if (num > 0){
		return 1;
	}
	else{
		return 0;
	}
}
