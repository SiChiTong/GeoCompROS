/*
 * Std2DFloatVector.hpp
 *
 *  Created on: Jul 25, 2013
 *      Author: shanker
 */

#ifndef STD2DFLOATVECTOR_HPP_
#define STD2DFLOATVECTOR_HPP_

#include <ros/ros.h>
#include <functional_mapping_helper/Exceptions/FMGeneralException.hpp>

class Std2DFloatVector
{
protected:
	std::vector<std::vector<float> > data;
public:
	struct addressElement
	{
		unsigned int row, col;
	};
	Std2DFloatVector();
	Std2DFloatVector(std::vector<std::vector<float> >);
	void clear();
	void printData();
	void pushRow(std::vector<float>);
	float getElement(unsigned int,unsigned int);
	unsigned int noOfRows();
	void addToEachElement(float);
	addressElement getMinimumValue();
	addressElement getMinimumValueForColumn(unsigned int);
	addressElement getMinimumValueForRow(unsigned int);
	addressElement getMaximumValue();
	void unitNormalizationPositive();
	void unitExponentialNormalizationPositive(float factor = 1.0);
	Std2DFloatVector unitExponentialNormalizationPositiveReturnNew(float factor = 1.0);
	void unitInverseExponentialNormalizationPositive(float factor = 1.0);
	Std2DFloatVector unitInverseExponentialNormalizationPositiveReturnNew(float factor = 1.0);
	void unitNormalizationNegative();
	std::vector<std::vector<float> > getData();
};



#endif /* STD2DFLOATVECTOR_HPP_ */
