/*
 * Ray.cpp
 *
 *  Created on: May 15, 2012
 *      Author: shanker
 */

#ifndef FUNCTIONALMAPPING_RAY_CPP
#define FUNCTIONALMAPPING_RAY_CPP

#include <functional_mapping_helper/GeometricalConstructs/Ray.hpp>

Ray::Ray(geometry_msgs::Point fromPoint, geometry_msgs::Point toPoint)
{
	startPoint = fromPoint;
	float distBetweenPoints = distanceBetweenPoints(fromPoint,toPoint);
	if(checkPointEquality(fromPoint,toPoint))
	{
		throw "Cannot construct ray from a single point";
		return;
	}
	angleWithPositiveXAxis = acos( (toPoint.x-fromPoint.x) / distBetweenPoints );
	angleWithPositiveYAxis = acos( (toPoint.y-fromPoint.y) / distBetweenPoints );
	angleWithPositiveZAxis = acos( (toPoint.z-fromPoint.z) / distBetweenPoints );
	//Calculate the Unit Vector corresponding to this Ray
	unitRay = constructVector3((toPoint.x-fromPoint.x)/distBetweenPoints,(toPoint.y-fromPoint.y)/distBetweenPoints,(toPoint.z-fromPoint.z)/distBetweenPoints);
}

Ray::Ray(geometry_msgs::Point fromPoint, geometry_msgs::Vector3 direction)
	{
		if(checkVectorEquality(direction,constructVector3(0,0,0))==true)
		{
			throw "Cannot construct ray from a null vector";
			return;
		}
		initializeRay(fromPoint, direction);
	}

void Ray::initializeRay(geometry_msgs::Point fromPoint, geometry_msgs::Vector3 direction)
{
		startPoint = fromPoint;
		unitRay = reduceToUnitVector(direction);
		angleWithPositiveXAxis = acos(unitRay.x);
		angleWithPositiveYAxis = acos(unitRay.y);
		angleWithPositiveZAxis = acos(unitRay.z);
}

geometry_msgs::Point Ray::getOrigin(void)
{
	return(startPoint);
}

geometry_msgs::Vector3 Ray::getUnitRay()
{
	geometry_msgs::Vector3 unitRayVec = unitRay;
	return(unitRayVec);
}

bool Ray::isPerpendicularTo(Ray *otherRay)
{
	bool areVectorsPerpendicular = checkVectorPerpendicularity(unitRay,otherRay->getUnitRay());
	bool sameStartPoint = checkPointEquality(startPoint,otherRay->getOrigin());
	return areVectorsPerpendicular && sameStartPoint;
}

void Ray::rotateRayByQuaternionUsingAxisAngle(Ray *axisRay, double angle)
{
	// This function gets the unit vectors corresponding to the rays and uses the ray function for rotation.
	unitRay = rotateVectorByQuaternionUsingAxisAngle(unitRay,axisRay->getUnitRay(),angle);
}

geometry_msgs::Point Ray::getPointAtLength(float l)
{
	// Return point on the ray at distance l from the ray origin in the direction of ray
	geometry_msgs::Point lPoint;
	lPoint = constructPoint(startPoint.x+(l*unitRay.x),startPoint.y+(l*unitRay.y),startPoint.z+(l*unitRay.z));
	return(lPoint);
}


bool operator ==(const Ray& rayA, const Ray& rayB)
{
   return checkPointEquality(rayA.startPoint,rayB.startPoint) &&
			  checkVectorEquality(rayA.unitRay,rayB.unitRay);
}

#endif


