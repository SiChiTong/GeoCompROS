
#ifndef FUNCTIONALMAPPING_PLANE_HPP
#define FUNCTIONALMAPPING_PLANE_HPP

#define MINIMUM_ABSOLUTE_NON_ZERO_PLANE_CONSTANT 1.0 // This value is to make sure that while comparing planes , the smallest absolute non - zero plane constant is always bigger than MIN_ERROR_GEOMETRY defined elsewhere
#define NON_ZERO_THRESHOLD_PLANE_CONSTANTS 0.001

#include <ros/ros.h>
#include <functional_mapping_helper/GeometricalConstructs/Point.hpp>
#include <functional_mapping_helper/GeometricalConstructs/Vector.hpp>
#include <functional_mapping_helper/GeometricalConstructs/RotationMatrix.hpp>
#include <functional_mapping_helper/GeometricalConstructs/Line.hpp>
#include <functional_mapping_helper/GeometricalConstructs/GeometricalComputations.hpp>

class Line;

class Plane
{
	typedef float planeConstants;
	planeConstants A, B, C, D; // In the equation Ax + By + Cz + D = 0
	geometry_msgs::Vector3 unitNormal; // The Unit Normal to the Plane, this is not unique, the inverse will also be a normal
	friend bool operator==(const Plane &A, const Plane &B);
    // Convert plane constants so that the minimum modulo non zero plane constant is greater than MIN_ERROR_GEOMETRY
	void maintainPlaneConstantsHigherThanThreshold();
	friend geometry_msgs::Point getPointLyingOnPlanes(Plane* plane1, Plane* plane2);
        
	public:
	Plane (geometry_msgs::Point point1, geometry_msgs::Point point2, geometry_msgs::Point point3); // Construct a plane from 3 points on the plane
    Plane (geometry_msgs::Vector3 normalToPlane, geometry_msgs::Point pointOnPlane); // Construct a plane given the normal to the plane, and a point on the Plane
    //Constructor of plane from plane constants
	Plane(float plConstA, float plConstB, float plConstC, float plConstD) ;
	float getA();
    float getB();
    float getC();
    float getD();
	// Find intersection of ray and plane
	friend IntersectionOfRayAndPlane findIntersectionOfRayAndPlane(Ray ray, Plane plane);
	// Substitute point in plane equation and return result
	float substitutePointInPlaneEquation(geometry_msgs::Point);
	// Check if a point lies on this plane
	bool checkPointInclusion(geometry_msgs::Point checkPoint);
	// Get Unit Normal of this plane
	geometry_msgs::Vector3 getUnitNormalVector();
	// Comparing the Unit Normal of this plane to another "otherPlane"
	bool checkUnitNormalEquality(Plane otherPlane);
	// Rotate this plane around a point (lying on it) to a new normal vector. If the point does not lie on the plane, the original plane is returned.
	Plane* rotatePlaneAroundPointToNormalVector(geometry_msgs::Vector3 newNormalVector,geometry_msgs::Point RotationPoint);
	geometry_msgs::Point getAnyPointOnPlane();
};

Line* getLineOfIntersectionOfTwoPlanes(Plane *planeA, Plane *planeB);
geometry_msgs::Point closestPointOnLineToGivenPoint(geometry_msgs::Point outsidePoint, Line *line); // Defined elsewhere
geometry_msgs::Point getPointOnPlaneAfterRotation(Plane* oldPlane, Plane* newPlane, geometry_msgs::Point oldPoint);

#endif
