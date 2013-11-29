/*
 * BasicDefinitions.cpp
 *
 *  Created on: May 15, 2012
 *      Author: shanker
 */

#ifndef FUNCTIONALMAPPING_BASICDEFINITIONS_CPP
#define	FUNCTIONALMAPPING_BASICDEFINITIONS_CPP

#include <functional_mapping_helper/GeometricalConstructs/BasicDefinitions.hpp>

bool compareFloats(float a, float b)
{
	return(fabs(a-b)<MIN_ERROR_GEOMETRY);
}

bool compareDoubles(double a, double b)
{
	return(fabs(a-b)<MIN_ERROR_DOUBLES);
}


#endif	/* BASICDEFINITIONS_CPP */


