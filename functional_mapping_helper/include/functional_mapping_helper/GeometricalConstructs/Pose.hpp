/*
 * Pose.hpp
 *
 *  Created on: May 15, 2012
 *      Author: shanker
 */

#ifndef FUNCTIONALMAPPING_POSE_HPP_
#define FUNCTIONALMAPPING_POSE_HPP_

#include <functional_mapping_helper/GeometricalConstructs/Point.hpp>
#include <functional_mapping_helper/GeometricalConstructs/Quaternion.hpp>
#include <geometry_msgs/Pose.h>

geometry_msgs::Pose constructPose(float px, float py, float pz, float ox, float oy, float oz, float ow);

bool checkPoseEquality(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);

void printPose(geometry_msgs::Pose, const char *);

#endif /* FUNCTIONALMAPPING_POSE_HPP_ */
