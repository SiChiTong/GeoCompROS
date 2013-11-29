/*
 * LineSegment.cpp
 *
 *  Created on: Jul 6, 2012
 *      Author: shanker
 */

#ifndef FUNCTIONALMAPPING_LINESEGMENT_CPP_
#define FUNCTIONALMAPPING_LINESEGMENT_CPP_

#include <functional_mapping_helper/GeometricalConstructs/LineSegment.hpp>

LineSegment::LineSegment(geometry_msgs::Point pntA, geometry_msgs::Point pntB) : Line(pntA, constructVector3(pntA,pntB))
{
	p1 = pntA;
	p2 = pntB;
}

geometry_msgs::Point LineSegment::getPoint1()
{
	return p1;
}

geometry_msgs::Point LineSegment::getPoint2()
{
	return p2;
}

// This function gets the distance from a point to the line segment and is not concerned if the projection is on the line segment or not (it is really a distance to the line)
float LineSegment::getDistanceToPoint(geometry_msgs::Point p)
{
	// If the line segment is AB and the point is P, the distance should be
	// mod( AP CROSS BP) / mod (AB)
	geometry_msgs::Vector3 ap = constructVector3(p1,p);
	geometry_msgs::Vector3 bp = constructVector3(p2,p);
	geometry_msgs::Vector3 ab = constructVector3(p1,p2);
	geometry_msgs::Vector3 apcrossbp = getCrossProductOfVectors(ap,bp);
	float distance = getLengthOfVector(apcrossbp) / getLengthOfVector(ab);
	return (distance);
}

// Given a line segment and a point, this evaluates whether the closest point on the line (of the line segment) to the given point, is on the line segment
bool LineSegment::isClosestPointOnLineSegment(geometry_msgs::Point p)
{
	// If the line segment is AB and the point is P, then the projection of AP on AB is AP DOT AB (with sign). This divided by AB DOT AB should be between 0 and 1.
	bool onLineSeg = false;
	geometry_msgs::Vector3 ap = constructVector3(p1,p);
	geometry_msgs::Vector3 ab = constructVector3(p1,p2);
	float ratio = getDotProductOfVectors(ap,ab)/getDotProductOfVectors(ab,ab);
	if(ratio>0 && ratio<=1)
	{
		onLineSeg = true;
	}
	return onLineSeg;
}

// Same as above, but also gets the point
LineSegment::pointOnLineSegment LineSegment::getClosestPointOnLineSegment(geometry_msgs::Point p)
{
	LineSegment::pointOnLineSegment pntOnLineSeg;
	// If the line segment is AB and the point is P, then the projection of AP on AB is AP DOT AB (with sign). This divided by AB DOT AB should be between 0 and 1.
	geometry_msgs::Vector3 ap = constructVector3(p1,p);
	geometry_msgs::Vector3 ab = constructVector3(p1,p2);
	float ratio = getDotProductOfVectors(ap,ab)/getDotProductOfVectors(ab,ab);
	if(ratio>0 && ratio<=1)
	{
		pntOnLineSeg.isPointOnLineSegment = true;
	}
	else
	{
		pntOnLineSeg.isPointOnLineSegment = false;
	}
	geometry_msgs::Vector3 vecToPoint = constructVector3(ab.x * ratio, ab.y * ratio, ab.z * ratio);
	pntOnLineSeg.pnt = constructPoint(p1.x + vecToPoint.x, p1.y + vecToPoint.y, p1.z + vecToPoint.z);
	return pntOnLineSeg;
}

// Find the point of intersection with ray. If the point of intersection does not lie on the line segment, intersection is false
pointOfIntersection LineSegment::getIntersectionWithRay(Ray *ray)
{
	pointOfIntersection xn;
	// Any point on the line segment AB can be represented as A + t*(B-A). Any point on the ray (vector part V, starting point U) can be represented as U + s*(V)
	// If the 2 points are equal, we should be able to equate the X,Y and Z components. Then we get 3 equations in 2 variables (t and s)
	// The approach we take is to solve sets of 2 linear equations and substitute in the other. If the solution applies to the third equation, we accept the solution.
	// If the 2 linear equations are not consistent we proceed to the next set of equations.
	// (A set of 2 linear equations might not be consistent, for e.g. if the ray and line segment lie in the same plane. However, this does not mean that they will not intersect!)
	bool solutionExists = false;
	float t = -1;  // A false default value to suggest that the solution does not lie on the line segment
	float s = -1;  // A false default value to suggest that the solution does not lie on the positive dir. of ray
	float LHS = 0;
	float RHS = 0;
	geometry_msgs::Point diffBA = constructPoint(p2.x-p1.x,p2.y-p1.y,p2.z-p1.z);
	// Case 1: Equate X and Y and substitute in Z
	// The equations are Ax + t*(BAx) = Ux + s*(Vx) & Ay + t*(BAy) = Uy + s*(Vy) or t*(BAx) + s*(-Vx) = Ux - Ax & t*(BAy) + s*(-Vy) = Uy - Ay
	LinearEquationSystem2D* linsys = new LinearEquationSystem2D(diffBA.x , -(ray->getUnitRay().x) , (ray->getOrigin().x)- (p1.x) , diffBA.y , -(ray->getUnitRay().y) , (ray->getOrigin().y)- (p1.y));
	if(linsys->getConsistency()==true)
	{
		t = linsys->getX();
		s = linsys->getY();
		// Substitute this in the third equation Az + t*(BAz) = Uz + s*(Vz)
		LHS = p1.z + t*(diffBA.z);
		RHS = (ray->getOrigin().z) + s * (ray->getUnitRay().z);
		solutionExists = compareFloats(LHS,RHS);
	}
	else
	{
		delete linsys;
		// Case 2: Equate Y and Z and substitute in X
		// The equations are Ay + t*(BAy) = Uy + s*(Vy) & Az + t*(BAz) = Uz + s*(Vz) or t*(BAy) + s*(-Vy) = Uy - Ay & t*(BAz) + s*(-Vz) = Uz - Az
		linsys = new LinearEquationSystem2D(diffBA.y , -(ray->getUnitRay().y) , (ray->getOrigin().y)- (p1.y) , diffBA.z , -(ray->getUnitRay().z) , (ray->getOrigin().z)- (p1.z));
		if(linsys->getConsistency()==true)
		{
			t = linsys->getX();
			s = linsys->getY();
			// Substitute this in the third equation Ax + t*(BAx) = Ux + s*(Vx)
			LHS = p1.x + t*(diffBA.x);
			RHS = (ray->getOrigin().x) + s * (ray->getUnitRay().x);
			solutionExists = compareFloats(LHS,RHS);
		}
		else
		{
			delete linsys;
			// Case 3: Equate Z and X and substitute in Y
			// The equations are Az + t*(BAz) = Uz + s*(Vz) & Ax + t*(BAx) = Ux + s*(Vx) or t*(BAz) + s*(-Vz) = Uz - Az & t*(BAx) + s*(-Vx) = Ux - Ax
			linsys = new LinearEquationSystem2D(diffBA.z , -(ray->getUnitRay().z) , (ray->getOrigin().z)- (p1.z) , diffBA.x , -(ray->getUnitRay().x) , (ray->getOrigin().x)- (p1.x));
			if(linsys->getConsistency()==true)
			{
				t = linsys->getX();
				s = linsys->getY();
				// Substitute this in the third equation Ay+ t*(BAy) = Uy + s*(Vy)
				LHS = p1.y + t*(diffBA.y);
				RHS = (ray->getOrigin().y) + s * (ray->getUnitRay().y);
				solutionExists = compareFloats(LHS,RHS);
			}
			else
			{
				solutionExists = false;
			}
		}
	}
	delete linsys;

	// At the end of this exercise, if solutionExists is false (the equations do not have consistency or a solution), or t does not lie between 0 and 1 (ray intersects point on line but not line segment)
	// or s is not greater than 0 (ray in the positive direction does not intersect line segment) , we conclude that there is no point of intersection
	if((solutionExists==false)||(t<0.00)||(t>1.00)||(s<0.00))
	{
		xn.doesIntersect = false;
		xn.point = constructPoint(0,0,0);
	}
	else
	{
		xn.doesIntersect = true;
		xn.point = constructPoint(p1.x + t*(diffBA.x) , p1.y + t*(diffBA.y) , p1.z + t*(diffBA.z));
	}

	return xn;
}

bool operator ==(const LineSegment &lsegA, const LineSegment &lsegB)
{
    // For 2 line segments to be equal, the endpoints have to be equal in either order
    return(
    		(checkPointEquality(lsegA.p1,lsegB.p1)&&checkPointEquality(lsegA.p2,lsegB.p2))
    		||(checkPointEquality(lsegA.p2,lsegB.p1)&&checkPointEquality(lsegA.p1,lsegB.p2))
    		);
}

#endif /* FUNCTIONALMAPPING_LINESEGMENT_CPP_ */


