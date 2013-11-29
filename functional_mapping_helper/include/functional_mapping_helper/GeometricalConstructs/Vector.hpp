#ifndef FUNCTIONALMAPPING_VECTOR_HPP
#define FUNCTIONALMAPPING_VECTOR_HPP

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>
#include <functional_mapping_helper/GeometricalConstructs/Point.hpp>
#include <LinearMath/btQuaternion.h>
#include <functional_mapping_helper/GeometricalConstructs/BasicDefinitions.hpp>

void printVector3(geometry_msgs::Vector3 thisVec);

void printVector3(geometry_msgs::Vector3 thisVec, const char *message);

geometry_msgs::Point32 convertVector3ToPoint32(geometry_msgs::Vector3 vec);

geometry_msgs::Vector3 convertPoint32ToVector3(geometry_msgs::Point32 pnt);

geometry_msgs::Point convertVector3ToPoint(geometry_msgs::Vector3 vec);

geometry_msgs::Vector3 convertPointToVector3(geometry_msgs::Point pnt);

geometry_msgs::Vector3 constructVector3(float x, float y, float z);

geometry_msgs::Vector3 constructVector3(geometry_msgs::Point fromPoint, geometry_msgs::Point toPoint);

float getLengthOfVector(geometry_msgs::Vector3 thisVec);

geometry_msgs::Vector3 negateVector(geometry_msgs::Vector3 thisVec);

bool checkVectorEquality(geometry_msgs::Vector3 vecA, geometry_msgs::Vector3 vecB);

bool checkVectorInverse(geometry_msgs::Vector3 vecA, geometry_msgs::Vector3 vecB);

geometry_msgs::Vector3 reduceToUnitVector(geometry_msgs::Vector3 vec);

geometry_msgs::Vector3 rescaleVector(geometry_msgs::Vector3 fromVector, geometry_msgs::Vector3 toVector);

float getDotProductOfVectors(geometry_msgs::Vector3 vec1, geometry_msgs::Vector3 vec2);

geometry_msgs::Vector3 getCrossProductOfVectors(geometry_msgs::Vector3 vec1, geometry_msgs::Vector3 vec2);

bool checkVectorPerpendicularity(geometry_msgs::Vector3 vec1, geometry_msgs::Vector3 vec2);

btVector3 convertROSVectorToBulletVector(geometry_msgs::Vector3 rosVec);

geometry_msgs::Vector3 convertBulletVectorToROSVector(btVector3 bulletVec);

geometry_msgs::Vector3 rotateVectorByBTQuaternion(geometry_msgs::Vector3, btQuaternion);

geometry_msgs::Vector3 rotateVectorByROSQuaternion(geometry_msgs::Vector3, geometry_msgs::Quaternion);

geometry_msgs::Vector3 rotateVectorByQuaternionUsingAxisAngle(geometry_msgs::Vector3 origVec, geometry_msgs::Vector3 axis, double angle);

geometry_msgs::Vector3 getVectorFromQuaternion(geometry_msgs::Quaternion q);

#endif
