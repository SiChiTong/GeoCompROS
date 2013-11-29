/* 
 * File:   Line.hpp
 * Author: shanker
 *
 * Created on December 12, 2011, 11:13 AM
 */


#ifndef FUNCTIONALMAPPING_LINE_HPP
#define	FUNCTIONALMAPPING_LINE_HPP

#include "functional_mapping_helper/GeometricalConstructs/Vector.hpp"
#include "functional_mapping_helper/GeometricalConstructs/Point.hpp"
#include "functional_mapping_helper/GeometricalConstructs/Plane.hpp"
#include "functional_mapping_helper/MathematicalConstructs/LinearEquationSystem2D.hpp"
#include "functional_mapping_helper/GeometricalConstructs/GeometricalComputations.hpp"

class Line
{
    // A line can be defined in parametric terms as X = X0 + v*t
    // Here X0 is a euclidean point and v is a 3D vector
    // So (x,y,z) = (x0,y0,z0) + t* (vecX,vecY,vecZ)
    geometry_msgs::Vector3 vecLine; // This is not unique to the line
    geometry_msgs::Point pointOnLine; // This is not unique to the line , the combination of these 2 parameters is not unique to the line
    friend bool operator ==(const Line &lineA, const Line &lineB);

    public:
    Line(geometry_msgs::Point point, geometry_msgs::Vector3 vec);

    geometry_msgs::Vector3 getVectorOfLine(void);

    geometry_msgs::Point getPointOnLine(void);

    float getDistanceBetweenPointAndLine(geometry_msgs::Point);
};

Line* getLineOfIntersectionOfTwoPlanes(Plane *planeA, Plane *planeB);

geometry_msgs::Point closestPointOnLineToGivenPoint(geometry_msgs::Point outsidePoint, Line *line);

#endif	/* FUNCTIONALMAPPING_LINE_HPP */

