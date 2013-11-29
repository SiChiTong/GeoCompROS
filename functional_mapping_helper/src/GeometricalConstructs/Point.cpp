/*
 * Point.cpp
 *
 *  Created on: May 15, 2012
 *      Author: shanker
 */

#ifndef FUNCTIONALMAPPING_POINT_CPP
#define FUNCTIONALMAPPING_POINT_CPP

#include <functional_mapping_helper/GeometricalConstructs/Point.hpp>

void printPoint(geometry_msgs::Point pnt)
{
	ROS_INFO("Point: X: %6.4f Y: %6.4f Z: %6.4f", pnt.x , pnt.y , pnt.z);
	return;
}

void printPoint(geometry_msgs::Point pnt, const char *message)
{
	ROS_INFO("Printing Point: %s",message);
	printPoint(pnt);
	return;
}
geometry_msgs::Point constructPoint(float x, float y, float z)
{
	geometry_msgs::Point newPoint;
	newPoint.x = x;
	newPoint.y = y;
	newPoint.z = z;
	return newPoint;
}

geometry_msgs::Point32 constructPoint32(float x, float y, float z)
{
	geometry_msgs::Point32 newPoint;
	newPoint.x = x;
	newPoint.y = y;
	newPoint.z = z;
	return newPoint;
}

geometry_msgs::Point addPoints(geometry_msgs::Point pntA, geometry_msgs::Point pntB)
{
	return(constructPoint(pntA.x+pntB.x,pntA.y+pntB.y,pntA.z+pntB.z));
}

bool checkPointEquality(geometry_msgs::Point pointA, geometry_msgs::Point pointB)
{
	return( fabs(pointA.x-pointB.x)<MIN_ERROR_GEOMETRY_POINT
			&& fabs(pointA.y-pointB.y)<MIN_ERROR_GEOMETRY_POINT
			&& fabs(pointA.z-pointB.z)<MIN_ERROR_GEOMETRY_POINT
			);
}

geometry_msgs::Point rotatePointAboutPoint(geometry_msgs::Point oldPoint, geometry_msgs::Point pointOfRotation, RotationMatrix *rotMat)
{
	geometry_msgs::Point toPoint;
	geometry_msgs::Vector3 oldVec = constructVector3(pointOfRotation,oldPoint);
	geometry_msgs::Vector3 newVec = (*rotMat)*oldVec;
	toPoint = constructPoint(pointOfRotation.x+newVec.x,pointOfRotation.y+newVec.y,pointOfRotation.z+newVec.z);
	return(toPoint);
}

geometry_msgs::Point32 convertPointToPoint32(geometry_msgs::Point pnt)
{
    geometry_msgs::Point32 pnt32;
    pnt32.x = pnt.x;
    pnt32.y = pnt.y;
    pnt32.z = pnt.z;
    return(pnt32);
}

geometry_msgs::Point convertPoint32ToPoint(geometry_msgs::Point32 pnt32)
{
    geometry_msgs::Point pnt;
    pnt.x = pnt32.x;
    pnt.y = pnt32.y;
    pnt.z = pnt32.z;
    return(pnt);
}

geometry_msgs::Point rotateAndTranslatePointAboutOrigin(geometry_msgs::Point pnt, geometry_msgs::Quaternion rot, geometry_msgs::Point trans)
{
	geometry_msgs::Vector3 origVec = convertPointToVector3(pnt); // Convert each point to a vector starting from the origin
	geometry_msgs::Vector3 rotVec = rotateVectorByROSQuaternion(origVec,rot); // Rotate this vector by the quaternion
	geometry_msgs::Point rotPnt = convertVector3ToPoint(rotVec); // Convert the vector back into a point (Get the endpoint of the vector that starts from the origin)
	pnt = constructPoint(rotPnt.x+trans.x,rotPnt.y+trans.y,rotPnt.z+trans.z); // Translate the points
	return pnt;
}

float distanceBetweenPoints(geometry_msgs::Point pntA, geometry_msgs::Point pntB)
{
	float dist;
	float sqr = pow(pntA.x-pntB.x,2.0) + pow(pntA.y-pntB.y,2.0) + pow(pntA.z-pntB.z,2.0);
	dist = pow(sqr,0.5);
	return(dist);
}

bool minAbsDiffInPoint(geometry_msgs::Point pntA, geometry_msgs::Point pntB, geometry_msgs::Point diffPoint) // Is abs(pntA - pntB) < abs(diffPoint) ?
{
	bool minAbsDiff = false;
	geometry_msgs::Point lhPoint = constructPoint(fabs(pntA.x - pntB.x),fabs(pntA.y - pntB.y),fabs(pntA.z - pntB.z));
	geometry_msgs::Point rhPoint = constructPoint(fabs(diffPoint.x),fabs(diffPoint.y),fabs(diffPoint.z));
	minAbsDiff = lhPoint.x <= rhPoint.x && lhPoint.y <= rhPoint.y && lhPoint.z <= rhPoint.z;
	return minAbsDiff;
}

#endif

