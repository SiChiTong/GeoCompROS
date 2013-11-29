/*
 * Line.cpp
 *
 *  Created on: May 15, 2012
 *      Author: shanker
 */

#ifndef FUNCTIONALMAPPING_LINE_CPP
#define	FUNCTIONALMAPPING_LINE_CPP

#include <functional_mapping_helper/GeometricalConstructs/Line.hpp>


Line::Line(geometry_msgs::Point point, geometry_msgs::Vector3 vec)
{
	if(compareDoubles(getLengthOfVector(vec),0)==true)
	{
		throw "Trying to construct a line with a null vector";
	}
	vecLine = reduceToUnitVector(vec);
	pointOnLine = point;
	return;
}

geometry_msgs::Vector3 Line::getVectorOfLine(void)
{
	return(vecLine);
}

geometry_msgs::Point Line::getPointOnLine(void)
{
	return(pointOnLine);
}

Line* getLineOfIntersectionOfTwoPlanes(Plane *planeA, Plane *planeB)
{
    // If the 2 planes are A1x + B1y + C1z + D1 = 0 and A2x + B2y + C2z + D2 = 0
    // Let us find the cross product of the two normals to the plane, this is a vector of the line we need
    geometry_msgs::Vector3 vecLine = getCrossProductOfVectors(planeA->getUnitNormalVector(), planeB->getUnitNormalVector());
    geometry_msgs::Point pointOnLine = constructPoint(0,0,0);
    if(vecLine.z!=0) // If line is not parrallel to the XY plane
    {
        //Set Z to 0 in the plane equations, solving this set of relationships
        LinearEquationSystem2D* eq = new LinearEquationSystem2D(planeA->getA(),planeA->getB(),-planeA->getD(),planeB->getA(),planeB->getB(),-planeB->getD());
        if((eq)->getConsistency()==false)
        {
            ROS_ERROR("Cannot find line of intersection of 2 planes, returning random line");
            delete eq;
            return(new Line(constructPoint(rand(),rand(),rand()),constructVector3(rand(),rand(),rand())));
        }
        pointOnLine = constructPoint(eq->getX(),eq->getY(),0);
        delete eq;
    }
    else if(vecLine.y!=0) // If line is not parrallel to the XZ plane
    {
        //Set Y to 0 in the plane equations, solving this set of relationships
        LinearEquationSystem2D* eq = new LinearEquationSystem2D(planeA->getA(),planeA->getC(),-planeA->getD(),planeB->getA(),planeB->getC(),-planeB->getD());
        if(eq->getConsistency()==false)
        {
            ROS_ERROR("Cannot find line of intersection of 2 planes, returning random line");
            delete eq;
            return(new Line(constructPoint(rand(),rand(),rand()),constructVector3(rand(),rand(),rand())));
        }
        pointOnLine = constructPoint(eq->getX(),0,eq->getY());
        delete eq;
    }
    else if(vecLine.x!=0) // If line is not parrallel to the YZ plane
    {
        //Set X to 0 in the plane equations, solving this set of relationships
        LinearEquationSystem2D* eq = new LinearEquationSystem2D(planeA->getB(),planeA->getC(),-planeA->getD(),planeB->getB(),planeB->getC(),-planeB->getD());
        if(eq->getConsistency()==false)
        {
            ROS_ERROR("Cannot find line of intersection of 2 planes, returning random line");
            delete eq;
            return(new Line(constructPoint(rand(),rand(),rand()),constructVector3(rand(),rand(),rand())));
        }
        pointOnLine = constructPoint(0,eq->getX(),eq->getY());
        delete eq;
    }
    else
    {
        ROS_ERROR("Cannot find line of intersection of 2 parrallel/null planes, returning random line");
        return(new Line(constructPoint(rand(),rand(),rand()),constructVector3(rand(),rand(),rand())));
    }
    return(new Line(pointOnLine,vecLine));
}

geometry_msgs::Point closestPointOnLineToGivenPoint(geometry_msgs::Point outsidePoint, Line *line)
{
    geometry_msgs::Point closestPoint;
    // (i) Equating the dot product of the vector formed by closestPoint to outsidePoint and line->getVectorOfLine() to 0, gives a plane that has both points and has line->getVectorOfLine() as the normal
    Plane *planeWithClosestPoint = new Plane( (line->getVectorOfLine()).x, (line->getVectorOfLine()).y, (line->getVectorOfLine()).z, -( ((line->getVectorOfLine()).x*outsidePoint.x) + ((line->getVectorOfLine()).y*outsidePoint.y) + ((line->getVectorOfLine()).z*outsidePoint.z) ) );
    // (ii) Now if we find the intersection of the ray from line->getPointOnLine() with direction line->getVectorOfLine(), with this plane the point of intersection should be this point
    Ray* interRay = new Ray(line->getPointOnLine(),line->getVectorOfLine());
    IntersectionOfRayAndPlane closestPointIntersection = findIntersectionOfRayAndPlane(*interRay, *planeWithClosestPoint);
    closestPoint = closestPointIntersection.pointOfXn;
    delete interRay;
    delete planeWithClosestPoint;
    return closestPoint;
}

float Line::getDistanceBetweenPointAndLine(geometry_msgs::Point p0)
{
	// Calculate first the closest point on the line to this point
	geometry_msgs::Point clPoint = closestPointOnLineToGivenPoint(p0, this);
	// Then calculate the distance between the 2 points
	float distance = distanceBetweenPoints(clPoint, p0);
	return distance;
}

bool operator ==(const Line &lineA, const Line &lineB)
{
    // For 2 lines to be equal, (the vectors have to be equal or inverse equal) AND (the point of one should lie on the other line)
    // In addition the point on line B - point on line A should yield the vector of (sufficiently) one of the vectors of the line OR the points can be equal
    geometry_msgs::Vector3 diffVec = constructVector3(lineA.pointOnLine,lineB.pointOnLine);
    return(
            ( (checkVectorEquality(lineA.vecLine,lineB.vecLine)) || (checkVectorInverse(lineA.vecLine,lineB.vecLine)) )
             && ( (checkVectorEquality(lineA.vecLine,diffVec)) || (checkVectorInverse(lineA.vecLine,diffVec)) || (checkPointEquality(lineA.pointOnLine,lineB.pointOnLine)) )
            );
}

#endif


