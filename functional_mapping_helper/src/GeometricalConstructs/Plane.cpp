/*
 * Plane.cpp
 *
 *  Created on: May 15, 2012
 *      Author: shanker
 */

#ifndef FUNCTIONALMAPPING_PLANE_CPP
#define FUNCTIONALMAPPING_PLANE_CPP

#include <functional_mapping_helper/GeometricalConstructs/Plane.hpp>

Plane::Plane (geometry_msgs::Point point1, geometry_msgs::Point point2, geometry_msgs::Point point3) // Construct a plane from 3 points on the plane
{
	//Calculating Plane Constants
	//If the three points are x1,y1,z1 etc
	A = point1.y * (point2.z - point3.z) + point2.y * (point3.z - point1.z) + point3.y * (point1.z - point2.z) ;// y1 (z2 - z3) + y2 (z3 - z1) + y3 (z1 - z2)
	B = point1.z * (point2.x - point3.x) + point2.z * (point3.x - point1.x) + point3.z * (point1.x - point2.x) ;// z1 (x2 - x3) + z2 (x3 - x1) + z3 (x1 - x2)
	C = point1.x * (point2.y - point3.y) + point2.x * (point3.y - point1.y) + point3.x * (point1.y - point2.y) ;// x1 (y2 - y3) + x2 (y3 - y1) + x3 (y1 - y2)
	D = - ( point1.x * ( point2.y * point3.z - point3.y * point2.z) + point2.x * (point3.y * point1.z - point1.y * point3.z) + point3.x * ( point1.y * point2.z - point2.y * point1.z ) ) ;// -( x1 (y2 z3 - y3 z2) + x2 (y3 z1 - y1 z3) + x3 (y1 z2 - y2 z1) )

   //Calculating Unit Normal to Plane
   unitNormal = reduceToUnitVector(constructVector3(A,B,C));
   maintainPlaneConstantsHigherThanThreshold();
}

Plane::Plane (geometry_msgs::Vector3 normalToPlane, geometry_msgs::Point pointOnPlane) // Construct a plane given the normal to the plane, and a point on the Plane
{
	normalToPlane = reduceToUnitVector(normalToPlane);
	// Given a point r0 and normal vector n, the equation of a plane can be written as n.(r-r0) = 0, where r = (x,y,z)
	// Thus using the plane Constants constructor
	A = normalToPlane.x;
	B = normalToPlane.y;
	C = normalToPlane.z;
	D = -(normalToPlane.x*pointOnPlane.x)-(normalToPlane.y*pointOnPlane.y)-(normalToPlane.z*pointOnPlane.z);
	unitNormal = normalToPlane;
	maintainPlaneConstantsHigherThanThreshold();
}

//Constructor of plane from plane constants
Plane::Plane(float plConstA, float plConstB, float plConstC, float plConstD)
{
	A = plConstA;
	B = plConstB;
	C = plConstC;
	D = plConstD;
	unitNormal = reduceToUnitVector(constructVector3(A,B,C));
	maintainPlaneConstantsHigherThanThreshold();
}

void Plane::maintainPlaneConstantsHigherThanThreshold()
{
	planeConstants absA = fabs(A);
	planeConstants absB = fabs(B);
	planeConstants absC = fabs(C);
	planeConstants absD = fabs(D);

	// Make all the constants above the non zero threshold
	A = absA < NON_ZERO_THRESHOLD_PLANE_CONSTANTS ? 0.000000 : A;
	B = absB < NON_ZERO_THRESHOLD_PLANE_CONSTANTS ? 0.000000 : B;
	C = absC < NON_ZERO_THRESHOLD_PLANE_CONSTANTS ? 0.000000 : C;
	D = absD < NON_ZERO_THRESHOLD_PLANE_CONSTANTS ? 0.000000 : D;

	std::vector<double> nonZeroAbsPlaneConstants;
	nonZeroAbsPlaneConstants.clear();
	if(absA>NON_ZERO_THRESHOLD_PLANE_CONSTANTS) {nonZeroAbsPlaneConstants.push_back(absA);}
	if(absB>NON_ZERO_THRESHOLD_PLANE_CONSTANTS) {nonZeroAbsPlaneConstants.push_back(absB);}
	if(absC>NON_ZERO_THRESHOLD_PLANE_CONSTANTS) {nonZeroAbsPlaneConstants.push_back(absC);}
	if(absD>NON_ZERO_THRESHOLD_PLANE_CONSTANTS) {nonZeroAbsPlaneConstants.push_back(absD);}

	if(nonZeroAbsPlaneConstants.size()==0)
	{
		throw "Plane::maintainPlaneConstantsHigherThanThreshold : All plane constants are zero, this is a fundamental geometric problem.";
	}
	planeConstants smallestNonZeroPlaneConstant = nonZeroAbsPlaneConstants[0];
	for(unsigned int ctr = 0; ctr < nonZeroAbsPlaneConstants.size(); ctr++)
	{
		smallestNonZeroPlaneConstant = (smallestNonZeroPlaneConstant <= nonZeroAbsPlaneConstants[ctr]) ? smallestNonZeroPlaneConstant : nonZeroAbsPlaneConstants[ctr];
	}

	if(smallestNonZeroPlaneConstant < MINIMUM_ABSOLUTE_NON_ZERO_PLANE_CONSTANT)
	{
		double ratio = MINIMUM_ABSOLUTE_NON_ZERO_PLANE_CONSTANT / smallestNonZeroPlaneConstant;
		A = A * ratio;
		B = B * ratio;
		C = C * ratio;
		D = D * ratio;
	}
}

float Plane::getA()
{
	return(A);
}

float Plane::getB()
{
	return(B);
}

float Plane::getC()
{
	return(C);
}

float Plane::getD()
{
	return(D);
}

// Substitute point in plane equation and return result
float Plane::substitutePointInPlaneEquation(geometry_msgs::Point pnt)
{
	float eqn = A*pnt.x + B*pnt.y + C*pnt.z + D;
	return(eqn);
}

// Check if a point lies on this plane
bool Plane::checkPointInclusion(geometry_msgs::Point checkPoint)
{
	float eqn = substitutePointInPlaneEquation(checkPoint);
	return fabs(eqn)<MIN_ERROR_GEOMETRY;
}

// Get Unit Normal of this plane
geometry_msgs::Vector3 Plane::getUnitNormalVector()
{
	return(unitNormal);
}

// Comparing the Unit Normal of this plane to another "otherPlane"
bool Plane::checkUnitNormalEquality(Plane otherPlane)
{
	// A unit normal to a plane is considered equal to proportional vectors irrespective of direction. This is in concurrence with the geometrical view of a plane being unidirectional
			bool equals = checkVectorEquality(unitNormal,otherPlane.unitNormal) || checkVectorEquality( constructVector3(-unitNormal.x,-unitNormal.y,-unitNormal.z) , otherPlane.unitNormal) ;
	return(equals);
}

// Rotate this plane around a point (lying on it) to a new normal vector. If the point does not lie on the plane, the original plane is returned.
Plane* Plane::rotatePlaneAroundPointToNormalVector(geometry_msgs::Vector3 newNormalVector,geometry_msgs::Point RotationPoint)
{
	// Check Point Inclusion
	bool doesPointLieOnPlane = checkPointInclusion(RotationPoint);
	if(doesPointLieOnPlane==0)
	{
		return this;
	}
	// Given a point r0 and normal vector n, the equation of a plane can be written as n.(r-r0) = 0, where r = (x,y,z)
	// Separating them into the plane constants and constructing the plane
	return (new Plane(newNormalVector,RotationPoint));
}

geometry_msgs::Point Plane::getAnyPointOnPlane()
{
	//Get the point of intersection of any of the axes and the plane
	if (C!=0) //Z Axis
	{ return(constructPoint(0,0,-D/C)); }
	else if (B!=0) //Y Axis
	{ return(constructPoint(0,-D/B,0)); }
	else // X Axis
	{ return(constructPoint(-D/A,0,0)); }
}

geometry_msgs::Point getPointOnPlaneAfterRotation(Plane* oldPlane, Plane* newPlane, geometry_msgs::Point oldPoint)
{
	geometry_msgs::Point newPoint;
	geometry_msgs::Vector3 normalOldPlane = (*oldPlane).getUnitNormalVector();
	geometry_msgs::Vector3 normalNewPlane = (*newPlane).getUnitNormalVector();
	RotationMatrix* rotPlanes = new RotationMatrix(normalOldPlane,normalNewPlane);
    Line* lineOfIntersection = getLineOfIntersectionOfTwoPlanes(oldPlane,newPlane);
	geometry_msgs::Point rotationPoint = closestPointOnLineToGivenPoint(oldPoint,lineOfIntersection);
	newPoint = rotatePointAboutPoint(oldPoint,rotationPoint,rotPlanes);
	delete lineOfIntersection;
	delete rotPlanes;
	return newPoint;
}

bool operator ==(const Plane &plane1, const Plane &plane2)
{
    //If the plane constants are in ratio, it is the same plane
	float ratio;

	if(plane1.A!=0&&plane2.A!=0)
	ratio = plane1.A/plane2.A;
	else if(plane1.B!=0&&plane2.B!=0)
	ratio = plane1.B/plane2.B;
	else if(plane1.C!=0&&plane2.C!=0)
	ratio = plane1.C/plane2.C;
	else if(plane1.D!=0&&plane2.D!=0)
	ratio = plane1.D/plane2.D;
	else return(0);
	return fabs(plane1.A/ratio - plane2.A)<MIN_ERROR_GEOMETRY &&
			fabs(plane1.B/ratio - plane2.B)<MIN_ERROR_GEOMETRY &&
			  fabs(plane1.C/ratio - plane2.C)<MIN_ERROR_GEOMETRY &&
			    fabs(plane1.D/ratio - plane2.D)<MIN_ERROR_GEOMETRY;
}


#endif
