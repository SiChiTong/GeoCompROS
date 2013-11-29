/*
 * Axis_Angle.cpp
 *
 *  Created on: May 15, 2012
 *      Author: shanker
 */

#ifndef FUNCTIONALMAPPING_AXIS_ANGLE_CPP
#define	FUNCTIONALMAPPING_AXIS_ANGLE_CPP

#include <functional_mapping_helper/GeometricalConstructs/Axis_Angle.hpp>

Axis_Angle::Axis_Angle(geometry_msgs::Vector3 fromVector, geometry_msgs::Vector3 toVector)
{
	// Constructs Axis_Angle for the rotation from fromVector to toVector
	fromVector = reduceToUnitVector(fromVector);
	toVector = reduceToUnitVector(toVector);
	angle = acos(getDotProductOfVectors(fromVector,toVector));
	axis = reduceToUnitVector(getCrossProductOfVectors(fromVector,toVector));
}

#endif


