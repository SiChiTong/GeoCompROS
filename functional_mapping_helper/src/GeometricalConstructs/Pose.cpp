/*
 * Pose.cpp
 *
 *  Created on: May 15, 2012
 *      Author: shanker
 */

#ifndef FUNCTIONALMAPPING_POSE_CPP_
#define FUNCTIONALMAPPING_POSE_CPP_

#include <functional_mapping_helper/GeometricalConstructs/Pose.hpp>

geometry_msgs::Pose constructPose(float px, float py, float pz, float ox, float oy, float oz, float ow)
{
	geometry_msgs::Pose pose;
	pose.position = constructPoint(px,py,pz);
	pose.orientation = constructQuaternion(ox,oy,oz,ow);
	return(pose);
}

bool checkPoseEquality(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
	return(checkPointEquality(pose1.position,pose2.position)&&checkQuaternionEquality(pose1.orientation,pose2.orientation));
}

void printPose(geometry_msgs::Pose pose, const char *msg)
{
	printPoint(pose.position,msg);
	printQuaternion(pose.orientation,msg);
}


#endif /* FUNCTIONALMAPPING_POSE_CPP_ */


