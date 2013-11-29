#ifndef FUNCTIONALMAPPING_GEOMETRICAL_COMPUTATIONS_HPP
#define FUNCTIONALMAPPING_GEOMETRICAL_COMPUTATIONS_HPP

#include <ros/ros.h>
#include <functional_mapping_helper/GeometricalConstructs/Ray.hpp>
#include <functional_mapping_helper/GeometricalConstructs/Plane.hpp>

double convertDegreesToRadians(double degrees);
double convertRadiansToDegrees(double radians);
struct IntersectionOfRayAndPlane
{
	// When a ray intersects a plane, we return the point of intersection and a bool that verifies intersection, we combine them in a single structure
	geometry_msgs::Point pointOfXn;
	bool doesIntersect;
};

IntersectionOfRayAndPlane findIntersectionOfRayAndPlane(Ray ray, Plane plane);

#endif 
