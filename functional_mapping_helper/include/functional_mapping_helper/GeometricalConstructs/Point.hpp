
#ifndef FUNCTIONALMAPPING_POINT_HPP
#define FUNCTIONALMAPPING_POINT_HPP

#include <geometry_msgs/Point.h>
#include <functional_mapping_helper/GeometricalConstructs/RotationMatrix.hpp>
#include <functional_mapping_helper/GeometricalConstructs/BasicDefinitions.hpp>
#include <geometry_msgs/Point32.h>

class RotationMatrix;

void printPoint(geometry_msgs::Point pnt);

void printPoint(geometry_msgs::Point pnt, const char *message);

geometry_msgs::Point constructPoint(float x, float y, float z);

geometry_msgs::Point32 constructPoint32(float x, float y, float z);

geometry_msgs::Point addPoints(geometry_msgs::Point, geometry_msgs::Point);

bool checkPointEquality(geometry_msgs::Point pointA, geometry_msgs::Point pointB);

geometry_msgs::Point rotatePointAboutPoint(geometry_msgs::Point oldPoint, geometry_msgs::Point pointOfRotation, RotationMatrix *rotMat);

geometry_msgs::Point32 convertPointToPoint32(geometry_msgs::Point pnt);

geometry_msgs::Point convertPoint32ToPoint(geometry_msgs::Point32 pnt32);

geometry_msgs::Point rotateAndTranslatePointAboutOrigin(geometry_msgs::Point, geometry_msgs::Quaternion, geometry_msgs::Point);

float distanceBetweenPoints(geometry_msgs::Point pntA, geometry_msgs::Point pntB);

bool minAbsDiffInPoint(geometry_msgs::Point, geometry_msgs::Point, geometry_msgs::Point);

#endif
