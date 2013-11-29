/*
 * GeometricalComputations.cpp
 *
 *  Created on: May 15, 2012
 *      Author: shanker
 */

#ifndef FUNCTIONALMAPPING_GEOMETRICAL_COMPUTATIONS_CPP
#define FUNCTIONALMAPPING_GEOMETRICAL_COMPUTATIONS_CPP

#include<functional_mapping_helper/GeometricalConstructs/GeometricalComputations.hpp>

double convertDegreesToRadians(double degrees)
{
	double factor = M_PI/180;
	return (degrees * factor);
}

double convertRadiansToDegrees(double radians)
{
	double factor = 180/M_PI;
	return (radians * factor);
}

IntersectionOfRayAndPlane findIntersectionOfRayAndPlane(Ray ray, Plane plane)
{
	IntersectionOfRayAndPlane intersection;
	// Ray is R0 + Rd * t, where R0 is initial point and Rd is the unit vector
	geometry_msgs::Point R0 = ray.startPoint;
	geometry_msgs::Vector3 Rd = ray.unitRay;
	// Plane is Ax + By + Cz + D = 0
	// Then, t = -( ((A * R0.x) + (B * R0.y) + (C * R0.z) + D) / ((A * Rd.x) + (B * Rd.y) + (C * Rd.z)) )
	if(((plane.A * Rd.x) + (plane.B * Rd.y) + (plane.C * Rd.z))==0)
	{
	  // This is a special case where the Ray is in the plane, we return the starting point of the ray
	  intersection.pointOfXn = R0;
	  intersection.doesIntersect = 1;
	  return(intersection);
	}

	float t = -( ((plane.A * R0.x) + (plane.B * R0.y) + (plane.C * R0.z) + plane.D) / ((plane.A * Rd.x) + (plane.B * Rd.y) + (plane.C * Rd.z)) );
	if(t<0)
	intersection.doesIntersect = 0;
	else
	intersection.doesIntersect = 1;

	intersection.pointOfXn = constructPoint(R0.x+(Rd.x*t),R0.y+(Rd.y*t),R0.z+(Rd.z*t));

	return intersection;
}

#endif
