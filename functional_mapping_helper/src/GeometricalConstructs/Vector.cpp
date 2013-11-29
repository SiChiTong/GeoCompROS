/*
 * Vector.cpp
 *
 *  Created on: May 15, 2012
 *      Author: shanker
 */

#ifndef FUNCTIONALMAPPING_VECTOR_CPP
#define FUNCTIONALMAPPING_VECTOR_CPP

#include <functional_mapping_helper/GeometricalConstructs/Vector.hpp>

void printVector3(geometry_msgs::Vector3 thisVec)
{
	ROS_INFO("Vector: X: %6.4f Y: %6.4f Z: %6.4f", thisVec.x , thisVec.y , thisVec.z);
	return;
}

void printVector3(geometry_msgs::Vector3 thisVec, const char *message)
{
	ROS_INFO("Printing Vector: %s",message);
	printVector3(thisVec);
	return;
}

geometry_msgs::Point32 convertVector3ToPoint32(geometry_msgs::Vector3 vec)
{
	geometry_msgs::Point32 pnt32 = constructPoint32(vec.x,vec.y,vec.z);
	return (pnt32);
}

geometry_msgs::Vector3 convertPoint32ToVector3(geometry_msgs::Point32 pnt32)
{
	geometry_msgs::Vector3 vec3 = constructVector3(pnt32.x, pnt32.y, pnt32.z);
	return (vec3);
}

geometry_msgs::Point convertVector3ToPoint(geometry_msgs::Vector3 vec)
{
	geometry_msgs::Point pnt = constructPoint(vec.x,vec.y,vec.z);
	return (pnt);
}

geometry_msgs::Vector3 convertPointToVector3(geometry_msgs::Point pnt)
{
	geometry_msgs::Vector3 vec3 = constructVector3(pnt.x, pnt.y, pnt.z);
	return (vec3);
}


geometry_msgs::Vector3 constructVector3(float x, float y, float z)
{
	//Constructs geometry_msgs::Vector3 message from individual components
	geometry_msgs::Vector3 thisVec;
	thisVec.x = x;
	thisVec.y = y;
	thisVec.z = z;
	return(thisVec);
}

geometry_msgs::Vector3 constructVector3(geometry_msgs::Point fromPoint, geometry_msgs::Point toPoint)
{
	//Construct a vector from two end points
	geometry_msgs::Vector3 thisVec;
	thisVec.x = toPoint.x - fromPoint.x;
	thisVec.y = toPoint.y - fromPoint.y;
	thisVec.z = toPoint.z - fromPoint.z;
	return(thisVec);
}

float getLengthOfVector(geometry_msgs::Vector3 thisVec)
{
	//Get Length Of The Input Vector
	return( pow ( pow(thisVec.x,2.0) + pow(thisVec.y,2.0) + pow(thisVec.z,2.0), 0.5) ) ;
}

geometry_msgs::Vector3 negateVector(geometry_msgs::Vector3 thisVec)
{
	//Negates the input vector
	return(constructVector3(-thisVec.x,-thisVec.y,-thisVec.z));
}

bool checkVectorEquality(geometry_msgs::Vector3 vecA, geometry_msgs::Vector3 vecB)
{
	//Check if the input vectors are identical
	if(vecA.x==vecB.x && vecA.y==vecB.y && vecA.z==vecB.z)
	return 1;
	//Check if there is a positive ratio connecting them, then it is a Vector
	float ratio;
	if(vecA.x!=0&&vecB.x!=0)
	ratio = vecA.x/vecB.x;
	else if(vecA.y!=0&&vecB.y!=0)
	ratio = vecA.y/vecB.y;
	else if(vecA.z!=0&&vecB.z!=0)
	ratio = vecA.z/vecB.z;
	else return 0; // The vectors are unequal
	if(ratio<0)
	return 0; // The vectors are in opposite directions
	return fabs(vecA.x/ratio - vecB.x)<MIN_ERROR_GEOMETRY &&
	              fabs(vecA.y/ratio - vecB.y)<MIN_ERROR_GEOMETRY &&
	                 fabs(vecA.z/ratio - vecB.z)<MIN_ERROR_GEOMETRY;
}

bool checkVectorInverse(geometry_msgs::Vector3 vecA, geometry_msgs::Vector3 vecB)
{
	//Check if the input vectors are inverse of each other
	return(checkVectorEquality(vecA,negateVector(vecB)));
}

geometry_msgs::Vector3 reduceToUnitVector(geometry_msgs::Vector3 vec)
{
	//Reduce input vector to corresponding unit vector
	float dividend = pow(pow(vec.x,2.0) + pow(vec.y,2.0) + pow(vec.z,2.0),0.5);
	vec.x /= dividend;
	vec.y /= dividend;
	vec.z /= dividend;
	return vec;
}

geometry_msgs::Vector3 rescaleVector(geometry_msgs::Vector3 fromVector, geometry_msgs::Vector3 toVector)
{
	// Rescales the fromVector to the length of the toVector
	float fromLength = getLengthOfVector(fromVector);
	float toLength = getLengthOfVector(toVector);
	if( fromLength == 0)
	{
		// Incase, the fromVector is a null vector , we return it
		return fromVector;
	}
	float ratio = toLength / fromLength;
	return( constructVector3(  ratio*fromVector.x , ratio*fromVector.y , ratio*fromVector.z ) );
}

float getDotProductOfVectors(geometry_msgs::Vector3 vec1, geometry_msgs::Vector3 vec2)
{
	//Return scalar dot product of input vectors
	return( (vec1.x*vec2.x) + (vec1.y*vec2.y) + (vec1.z*vec2.z) );
}

geometry_msgs::Vector3 getCrossProductOfVectors(geometry_msgs::Vector3 vec1, geometry_msgs::Vector3 vec2)
{
	//Return cross product of input vectors i.e. (vec1 X vec2)
	return( constructVector3( ((vec1.y*vec2.z) - (vec1.z*vec2.y)) , ((vec1.z*vec2.x) - (vec1.x*vec2.z)) , ((vec1.x*vec2.y) - (vec1.y*vec2.x)) ) );
}

bool checkVectorPerpendicularity(geometry_msgs::Vector3 vec1, geometry_msgs::Vector3 vec2)
{
	if(getDotProductOfVectors(vec1,vec2)<MIN_ERROR_GEOMETRY)
	{
		return true;
	}
	else
	{
		return false;
	}
}


btVector3 convertROSVectorToBulletVector(geometry_msgs::Vector3 rosVec)
{
	return( btVector3(rosVec.x,rosVec.y,rosVec.z) );
}

geometry_msgs::Vector3 convertBulletVectorToROSVector(btVector3 bulletVec)
{
	return( constructVector3( bulletVec.getX(), bulletVec.getY(), bulletVec.getZ() ) );
}

geometry_msgs::Vector3 rotateVectorByBTQuaternion(geometry_msgs::Vector3 vec, btQuaternion btq)
{
	geometry_msgs::Vector3 resVec = convertBulletVectorToROSVector( quatRotate(btq,convertROSVectorToBulletVector(vec)) );
	return(resVec);
}

geometry_msgs::Vector3 rotateVectorByROSQuaternion(geometry_msgs::Vector3 vec, geometry_msgs::Quaternion q)
{
	btQuaternion quat(q.x,q.y,q.z,q.w);
	geometry_msgs::Vector3 resVec = rotateVectorByBTQuaternion(vec,quat);
	return(resVec);
}

geometry_msgs::Vector3 rotateVectorByQuaternionUsingAxisAngle(geometry_msgs::Vector3 origVec, geometry_msgs::Vector3 axis, double angle)
{
	geometry_msgs::Vector3 rotatedVec;
	axis = reduceToUnitVector(axis);
	btQuaternion quatAxisAngle(convertROSVectorToBulletVector(axis),btScalar(angle));
	quatAxisAngle.normalize();
	rotatedVec = rotateVectorByBTQuaternion(origVec,quatAxisAngle);
	return rotatedVec;
}

geometry_msgs::Vector3 getVectorFromQuaternion(geometry_msgs::Quaternion q)
{
	geometry_msgs::Vector3 xAxisVec = constructVector3(1,0,0);
	geometry_msgs::Vector3 rotatVec = rotateVectorByROSQuaternion(xAxisVec, q);
	return rotatVec;
}



#endif
