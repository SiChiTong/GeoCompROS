/*
 * Matrix2X2.cpp
 *
 *  Created on: May 23, 2012
 *      Author: shanker
 */

#ifndef FUNCTIONALMAPPING_MATRIX2X2_CPP
#define	FUNCTIONALMAPPING_MATRIX2X2_CPP

#include <functional_mapping_helper/MathematicalConstructs/Matrix2X2.h>

Matrix2X2::Matrix2X2(float a, float b,float c,float d)
{
	a11 = a;
	a12 = b;
	a21 = c;
	a22 = d;
	computeDeterminant();
}

void Matrix2X2::computeDeterminant(void)
{
	determinant = (a11*a22) - (a12*a21);
}

float Matrix2X2::getDeterminant(void)
{
	return(determinant);
}

#endif
