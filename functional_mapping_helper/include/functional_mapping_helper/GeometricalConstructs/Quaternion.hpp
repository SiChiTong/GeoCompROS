/*
 * Quaternion.hpp
 *
 *  Created on: May 15, 2012
 *      Author: shanker
 */

#ifndef FUNCTIONALMAPPING_QUATERNION_HPP_
#define FUNCTIONALMAPPING_QUATERNION_HPP_

#include <geometry_msgs/Quaternion.h>
#include <functional_mapping_helper/GeometricalConstructs/BasicDefinitions.hpp>
#include <LinearMath/btQuaternion.h>
#include <functional_mapping_helper/GeometricalConstructs/Vector.hpp>

void printQuaternion(geometry_msgs::Quaternion, const char *);

geometry_msgs::Quaternion normalizeQuaternion(geometry_msgs::Quaternion);

float getAngleOfQuaternion(geometry_msgs::Quaternion);

geometry_msgs::Quaternion constructQuaternion(float qx, float qy, float qz, float qw);

geometry_msgs::Quaternion constructROSQuaternionFromAxisAngle(geometry_msgs::Vector3, float);

geometry_msgs::Quaternion convertBTToGeoQuat(btQuaternion);

btQuaternion convertGeoToBTQuat(geometry_msgs::Quaternion);

geometry_msgs::Quaternion rotateQuaternionByAxisAngle(geometry_msgs::Quaternion , geometry_msgs::Vector3, float);

bool checkNullQuaternion(geometry_msgs::Quaternion q);

bool checkQuaternionEquality(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2);

geometry_msgs::Quaternion computeQuaternionOfXYPlaneVector(geometry_msgs::Vector3);

geometry_msgs::Quaternion multiplyQuaternions(geometry_msgs::Quaternion, geometry_msgs::Quaternion);

#endif /* FUNCTIONALMAPPING_QUATERNION_HPP_ */
