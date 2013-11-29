/*
 * Quaternion.cpp
 *
 *  Created on: May 15, 2012
 *      Author: shanker
 */

#ifndef FUNCTIONALMAPPING_QUATERNION_CPP_
#define FUNCTIONALMAPPING_QUATERNION_CPP_

#include <functional_mapping_helper/GeometricalConstructs/Quaternion.hpp>

void printQuaternion(geometry_msgs::Quaternion q, const char *msg)
{
	ROS_INFO("%s x:%6.4f y:%6.4f z:%6.4f w:%6.4f",msg,q.x,q.y,q.z,q.w);
}

geometry_msgs::Quaternion normalizeQuaternion(geometry_msgs::Quaternion q)
{
	btQuaternion bq = convertGeoToBTQuat(q);
	bq = bq.normalize();
	return(convertBTToGeoQuat(bq));
}

float getAngleOfQuaternion(geometry_msgs::Quaternion q)
{
	btQuaternion bq = convertGeoToBTQuat(q);
	bq = bq.normalize();
	return(bq.getAngle());
}

geometry_msgs::Quaternion convertBTToGeoQuat(btQuaternion q)
{
	geometry_msgs::Quaternion quat;
	quat.x = q.x();
	quat.y = q.y();
	quat.z = q.z();
	quat.w = q.w();
	return quat;
}

btQuaternion convertGeoToBTQuat(geometry_msgs::Quaternion q)
{
	return(btQuaternion(q.x,q.y,q.z,q.w));
}

geometry_msgs::Quaternion constructQuaternion(float qx, float qy, float qz, float qw)
{
	geometry_msgs::Quaternion quat;
	quat.x = qx;
	quat.y = qy;
	quat.z = qz;
	quat.w = qw;
	return(quat);
}

geometry_msgs::Quaternion constructROSQuaternionFromAxisAngle(geometry_msgs::Vector3 axis, float angle)
{
	btVector3 btaxis(axis.x,axis.y,axis.z);
	btQuaternion btquat(btaxis,angle);
	return(convertBTToGeoQuat(btquat));
}
geometry_msgs::Quaternion rotateQuaternionByAxisAngle(geometry_msgs::Quaternion q, geometry_msgs::Vector3 axis, float angle)
{
	btQuaternion rot = btQuaternion(btVector3(axis.x,axis.y,axis.z),angle);
	btQuaternion rotated = convertGeoToBTQuat(q)*rot;
	return(convertBTToGeoQuat(rotated));
}

bool checkNullQuaternion(geometry_msgs::Quaternion q)
{
	q = normalizeQuaternion(q);
	return(checkQuaternionEquality(q,constructQuaternion(0,0,0,0)));
}


bool checkQuaternionEquality(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2)
{
	q1 = normalizeQuaternion(q1);
	q2 = normalizeQuaternion(q2);
	return( fabs(q1.x-q2.x)<MIN_ERROR_GEOMETRY_ANGLE
				&& fabs(q1.y-q2.y)<MIN_ERROR_GEOMETRY_ANGLE
				&& fabs(q1.z-q2.z)<MIN_ERROR_GEOMETRY_ANGLE
				&& fabs(q1.w-q2.w)<MIN_ERROR_GEOMETRY_ANGLE
				);
}

geometry_msgs::Quaternion computeQuaternionOfXYPlaneVector(geometry_msgs::Vector3 vec)
{
	geometry_msgs::Quaternion quat = constructQuaternion(0,0,0,1);
	if(compareDoubles(vec.z,0.00)==false)
	{
		throw "computeQuaternionOfXYPlaneVector : Given vector does not lie on XY plane";
	}
	else if(compareDoubles(vec.x,0.00)==true && compareDoubles(vec.y,0.00)==true)
	{
		throw "computeQuaternionOfXYPlaneVector : Given vector is null";
	}
	else
	{
		float angle = atan2(vec.y,vec.x);
		geometry_msgs::Vector3 unitZAxis = constructVector3(0,0,1);
		quat = constructROSQuaternionFromAxisAngle(unitZAxis,angle);
	}
	return quat;
}

geometry_msgs::Quaternion multiplyQuaternions(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2)
{
	btQuaternion bq = convertGeoToBTQuat(q1) * convertGeoToBTQuat(q2);
	return( convertBTToGeoQuat( bq ));
}


#endif /* FUNCTIONALMAPPING_QUATERNION_CPP_ */


