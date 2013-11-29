/*
 * LineSegment.hpp
 *
 *  Created on: Jul 6, 2012
 *      Author: shanker
 */

#ifndef FUNCTIONALMAPPING_LINESEGMENT_HPP_
#define FUNCTIONALMAPPING_LINESEGMENT_HPP_

#include <functional_mapping_helper/GeometricalConstructs/Line.hpp>
#include <functional_mapping_helper/GeometricalConstructs/PointOfIntersection.hpp>
#include <functional_mapping_helper/MathematicalConstructs/LinearEquationSystem2D.hpp>

class LineSegment : public Line
{
	geometry_msgs::Point p1;
	geometry_msgs::Point p2;
	friend bool operator ==(const LineSegment &lsegA, const LineSegment &lsegB);

public:
struct pointOnLineSegment
{
	geometry_msgs::Point pnt;
	bool isPointOnLineSegment;
};
geometry_msgs::Point getPoint1();
geometry_msgs::Point getPoint2();
LineSegment(geometry_msgs::Point, geometry_msgs::Point);
float getDistanceToPoint(geometry_msgs::Point);
bool isClosestPointOnLineSegment(geometry_msgs::Point pnt);
pointOnLineSegment getClosestPointOnLineSegment(geometry_msgs::Point pnt);
pointOfIntersection getIntersectionWithRay(Ray *ray);

};


#endif /* FUNCTIONALMAPPING_LINESEGMENT_HPP_ */
