#ifndef FUNCTIONALMAPPING_ROTATION_MATRIX_HPP
#define FUNCTIONALMAPPING_ROTATION_MATRIX_HPP

#include <ros/ros.h>
#include <functional_mapping_helper/GeometricalConstructs/Vector.hpp>
#include <LinearMath/btQuaternion.h>

class RotationMatrix
{
	// | e11 e12 e13 |
	// | e21 e22 e23 |
	// | e31 e32 e33 |
	float e11, e12, e13, e21, e22, e23, e31, e32, e33;
	friend bool operator==(const RotationMatrix &r1, const RotationMatrix &r2);

	public:
	void initRotationMatrix(float el11, float el12, float el13, float el21, float el22, float el23, float el31, float el32, float el33);
	RotationMatrix(geometry_msgs::Vector3 fromVector, geometry_msgs::Vector3 toVector);
	geometry_msgs::Vector3 operator*(geometry_msgs::Vector3 multVec);
		
};

#endif
