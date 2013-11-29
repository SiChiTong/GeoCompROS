/* 
 * File:   Axis-Angle.hpp
 * Author: shanker
 *
 * Created on December 22, 2011, 12:05 PM
 */

#ifndef FUNCTIONALMAPPING_AXIS_ANGLE_HPP
#define	FUNCTIONALMAPPING_AXIS_ANGLE_HPP

#include <functional_mapping_helper/GeometricalConstructs/Vector.hpp>

struct Axis_Angle
{
    geometry_msgs::Vector3 axis; // Will be a unit vector
    float angle; // In radians

public:
    Axis_Angle(geometry_msgs::Vector3 fromVector, geometry_msgs::Vector3 toVector);

};


#endif	/* AXIS_ANGLE_HPP */

