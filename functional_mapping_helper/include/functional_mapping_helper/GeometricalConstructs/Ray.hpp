
#ifndef FUNCTIONALMAPPING_RAY_HPP
#define FUNCTIONALMAPPING_RAY_HPP

#include <ros/ros.h>
#include <functional_mapping_helper/GeometricalConstructs/Point.hpp>
#include <sstream>
#include <functional_mapping_helper/GeometricalConstructs/Vector.hpp>

class Plane; // For the friend function defined below
struct IntersectionOfRayAndPlane; // Also for friend function

class Ray
{
	geometry_msgs::Point startPoint;
	float angleWithPositiveXAxis, angleWithPositiveYAxis, angleWithPositiveZAxis;
	geometry_msgs::Vector3 unitRay;

	friend bool operator ==(const Ray& rayA, const Ray& rayB);

	public:
	Ray(geometry_msgs::Point fromPoint, geometry_msgs::Point toPoint);

	Ray(geometry_msgs::Point fromPoint, geometry_msgs::Vector3 direction);

	void initializeRay(geometry_msgs::Point fromPoint, geometry_msgs::Vector3 direction);

	geometry_msgs::Point getOrigin(void);

	geometry_msgs::Vector3 getUnitRay();

	bool isPerpendicularTo(Ray *otherRay);

	void rotateRayByQuaternionUsingAxisAngle(Ray *axisRay, double angle);

	geometry_msgs::Point getPointAtLength(float l);

	friend IntersectionOfRayAndPlane findIntersectionOfRayAndPlane(Ray ray, Plane plane);
	
};

#endif
