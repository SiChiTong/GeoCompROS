/*
 * RotationMatrix.cpp
 *
 *  Created on: May 15, 2012
 *      Author: shanker
 */

#ifndef FUNCTIONALMAPPING_ROTATION_MATRIX_CPP
#define FUNCTIONALMAPPING_ROTATION_MATRIX_CPP

#include <functional_mapping_helper/GeometricalConstructs/RotationMatrix.hpp>

inline void RotationMatrix::initRotationMatrix(float el11, float el12, float el13, float el21, float el22, float el23, float el31, float el32, float el33)
{
	e11 = el11; e12 = el12; e13 = el13;
	e21 = el21; e22 = el22; e23 = el23;
	e31 = el31; e32 = el32; e33 = el33;
}

RotationMatrix::RotationMatrix(geometry_msgs::Vector3 fromVector, geometry_msgs::Vector3 toVector)
{
	//Computes the Rotation Matrix as a transformation from fromVector to toVector
	toVector = reduceToUnitVector(toVector);
		fromVector = reduceToUnitVector(fromVector);
	if(checkVectorEquality(fromVector,toVector)==true)
	{
		// This is the case that the vectors are equal (0 degree angle between them)
		initRotationMatrix(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0);
		return;
	}
	else if(checkVectorInverse(fromVector,toVector)==true)
	{
		// This is the case that the vectors are in opposite directions (180 degree angle between them)
		initRotationMatrix(-1.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,-1.0);
		return;
	}
	else
	{   // In other cases
		geometry_msgs::Vector3 axis = getCrossProductOfVectors(fromVector,toVector); // Axis Of Rotation
		axis = reduceToUnitVector(axis);
		float theta = acos(getDotProductOfVectors(fromVector,toVector)); // Angle of Rotation
		// Rotation Matrix from Axis-Angle formula www.euclideanspace.com
		initRotationMatrix( cos(theta) + ((axis.x*axis.x)*(1-cos(theta))) , ((axis.x)*(axis.y)*(1-cos(theta))) - ((axis.z)*(sin(theta))),
							((axis.x)*(axis.z)*(1-cos(theta))) + ((axis.y)*(sin(theta))), ((axis.y)*(axis.x)*(1-cos(theta))) + ((axis.z)*(sin(theta))), (cos(theta)) + ((axis.y)*(axis.y)*(1-cos(theta))) , ((axis.y)*(axis.z)*(1-cos(theta))) - ((axis.x)*(sin(theta))) , ((axis.z)*(axis.x)*(1-cos(theta))) - ((axis.y)*(sin(theta))) , ((axis.z)*(axis.y)*(1-cos(theta))) + ((axis.x)*(sin(theta))) , (cos(theta)) + ((axis.z*axis.z)*(1-cos(theta)))  );
	}
	return;
}

geometry_msgs::Vector3 RotationMatrix::operator*(geometry_msgs::Vector3 multVec)
{
	// Multiply this Matrix with a Vector
	return( constructVector3( (e11 * multVec.x) + (e12 * multVec.y) + (e13 * multVec.z) ,
							  (e21 * multVec.x) + (e22 * multVec.y) + (e23 * multVec.z) ,
							  (e31 * multVec.x) + (e32 * multVec.y) + (e33 * multVec.z) ) );
}

bool operator ==(const RotationMatrix &r1, const RotationMatrix &r2)
{
	return (
			((r1.e11)-(r2.e11)<MIN_ERROR_GEOMETRY)&&
			((r1.e12)-(r2.e12)<MIN_ERROR_GEOMETRY)&&
			((r1.e13)-(r2.e13)<MIN_ERROR_GEOMETRY)&&
			((r1.e21)-(r2.e21)<MIN_ERROR_GEOMETRY)&&
			((r1.e22)-(r2.e22)<MIN_ERROR_GEOMETRY)&&
			((r1.e23)-(r2.e23)<MIN_ERROR_GEOMETRY)&&
			((r1.e31)-(r2.e31)<MIN_ERROR_GEOMETRY)&&
			((r1.e32)-(r2.e32)<MIN_ERROR_GEOMETRY)&&
			((r1.e33)-(r2.e33)<MIN_ERROR_GEOMETRY)
			);
}

#endif


