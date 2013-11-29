/* 
 * File:   BasicDefinitions.hpp
 * Author: shanker
 *
 * Created on December 14, 2011, 5:13 PM
 */

#ifndef FUNCTIONALMAPPING_BASICDEFINITIONS_HPP
#define	FUNCTIONALMAPPING_BASICDEFINITIONS_HPP

#define MIN_ERROR_GEOMETRY 0.1
#define MIN_ERROR_DOUBLES 0.00000001
#define MIN_ERROR_GEOMETRY_POINT 0.05
#define MIN_ERROR_GEOMETRY_ANGLE 0.01

#include <ros/ros.h>

bool compareFloats(float a, float b);
bool compareDoubles(double a, double b);

#endif	/* BASICDEFINITIONS_HPP */

