/*
 * Std2DFloatVector.cpp
 *
 *  Created on: Jul 25, 2013
 *      Author: shanker
 */

#include <functional_mapping_helper/HelperClasses/Std2DFloatVector.hpp>

Std2DFloatVector::Std2DFloatVector()
{

}

Std2DFloatVector::Std2DFloatVector(std::vector<std::vector<float> > inData)
{
	data = inData;
}

void Std2DFloatVector::clear()
{
	data.clear();
}

void Std2DFloatVector::printData()
{
	std::cout<<"\n";
	for(unsigned int i = 0; i < data.size(); i++)
	{
		for(unsigned int j = 0; j < data[i].size(); j++)
		{
			std::cout<<data[i][j]<<" ";
		}
		std::cout<<"\n";
	}
}

void Std2DFloatVector::pushRow(std::vector<float> row)
{
	data.push_back(row);
}

float Std2DFloatVector::getElement(unsigned int row, unsigned int col)
{
	float el;
	if((data.size()<=row)||(data[row].size()<=col))
	{
		throw((FMGeneralException("Std2DFloatVector::getElement : Tried to access an invalid element")));
	}
	else
	{
		el = data[row][col];
	}
	return el;
}

unsigned int Std2DFloatVector::noOfRows()
{
	return(data.size());
}

/*
 * Add a float to each element
 */
void Std2DFloatVector::addToEachElement(float n)
{
	for(unsigned int i = 0; i < data.size(); i++)
	{
		for(unsigned int j = 0; j < data[i].size(); j++)
		{
			data[i][j] += n;
		}
	}
	return;
}

/*
 * Find the minimum value element of data
 */
Std2DFloatVector::addressElement Std2DFloatVector::getMinimumValue()
{
	addressElement minElement;
	if(!((data.size()>0)&&(data[0].size()>0)))
	{
		throw(FMGeneralException("Std2DFloatVector::getMinimumValue : Cannot find minimum value of an empty matrix"));
	}
	else
	{
		try{
			minElement.row = 0;
			minElement.col = 0;
			float minElementData = data[0][0];
			for(unsigned int i = 0; i < data.size(); i++)
			{
				std::vector<float>::iterator minIt;
				if(data[i].size()==0)
				{
					continue;
				}
				else
				{
					minIt = std::min_element(data[i].begin(),data[i].end());
				}
				if(*minIt < minElementData)
				{
					minElementData = *minIt;
					minElement.row = i;
					minElement.col = minIt - data[i].begin();
				}
			}
		}
		catch(std::exception &e)
		{
			throw(FMGeneralException(std::string("Std2DFloatVector::getMinimumValue : Caught a standard exception ").append(e.what()) ));
		}
	}
	return minElement;
}

/*
 * Get minimum value element for given column
 */
Std2DFloatVector::addressElement Std2DFloatVector::getMinimumValueForColumn(unsigned int c)
{
	addressElement minElement;
	if(!((data.size()>0)&&(data[0].size()>0)))
	{
		throw(FMGeneralException("Std2DFloatVector::getMinimumValueForColumn : Cannot find minimum value of an empty matrix"));
	}
	try{
		float minValue = data[0][0];
		minElement.row = 0;
		minElement.col = 0;
		for(unsigned int i = 0; i < data.size(); i++)
		{
			if(data[i].size() <= c)
			{
				throw(FMGeneralException("Std2DFloatVector::getMinimumValueForColumn : The size of the row was smaller than the column element requested"));
			}
			else if(data[i][c] < minValue)
			{
				minValue = data[i][c];
				minElement.row = i;
				minElement.col = c;
			}
		}
	}
	catch(std::exception &e)
	{
		throw(FMGeneralException(std::string("Std2DFloatVector::getMinimumValueForColumn : Caught a standard exception ").append(e.what()) ));
	}
	return minElement;
}

/*
 * Get minimum value for given row
 */
Std2DFloatVector::addressElement Std2DFloatVector::getMinimumValueForRow(unsigned int r)
{
	addressElement minElement;
	if(data.size()<=r || data[r].empty())
	{
		throw(FMGeneralException("Std2DFloatVector::getMinimumValueForRow : Row doesn't exist or empty row"));
	}
	try{
		minElement.row = r;
		minElement.col = 0;
		std::vector<float>::iterator minIt = std::min_element(data[r].begin(),data[r].end());
		minElement.col = minIt - data[r].begin();
	}
	catch(std::exception &e)
	{
		throw(FMGeneralException(std::string("Std2DFloatVector::getMinimumValueForColumn : Caught a standard exception ").append(e.what()) ));
	}
	return minElement;
}

/*
 * Find the maximum value element of data
 */
Std2DFloatVector::addressElement Std2DFloatVector::getMaximumValue()
{
	addressElement maxElement;
	if(!((data.size()>0)&&(data[0].size()>0)))
	{
		throw(FMGeneralException("Std2DFloatVector::getMaximumValue : Cannot find minimum value of an empty matrix"));
	}
	else
	{
		try{
			maxElement.row = 0;
			maxElement.col = 0;
			float maxElementData = data[0][0];
			for(unsigned int i = 0; i < data.size(); i++)
			{
				std::vector<float>::iterator maxIt;
				if(data[i].size()==0)
				{
					continue;
				}
				else
				{
					maxIt = std::max_element(data[i].begin(),data[i].end());
				}
				if(*maxIt > maxElementData)
				{
					maxElementData = *maxIt;
					maxElement.row = i;
					maxElement.col = maxIt - data[i].begin();
				}
			}
		}
		catch(std::exception &e)
		{
			throw(FMGeneralException(std::string("Std2DFloatVector::getMaximumValue : Caught a standard exception").append(e.what())));
		}
	}
	return maxElement;
}

/*
 * Find the maximum value element of data, divide all other elements by it, in order to yield 0 to 1. Only to be used for non-negative fields.
 */
void Std2DFloatVector::unitNormalizationPositive()
{
	addressElement maxE = getMaximumValue();
	float maxVal = data[maxE.row][maxE.col];
	if(maxVal==0.0)
	{
		return;
	}
	for(unsigned int i = 0; i < data.size(); i++)
	{
		for(unsigned int j = 0; j< data[i].size(); j++)
		{
			data[i][j] /= maxVal;
		}
	}
}

/*
 * Find the minimum value element of data, divide all other elements by it's absolute value to yield 0 to -1. Only to be used for non-positive fields.
 */
void Std2DFloatVector::unitNormalizationNegative()
{
	addressElement minE = getMinimumValue();
	float minVal = data[minE.row][minE.col];
	minVal = fabs(minVal);
	if(minVal==0.0)
	{
		return;
	}
	for(unsigned int i = 0; i < data.size(); i++)
	{
		for(unsigned int j = 0; j< data[i].size(); j++)
		{
			data[i][j] /= minVal;
		}
	}
}

/*
 * Similar to unit normalization positive, except it follows an exponential curve.
 */
void Std2DFloatVector::unitExponentialNormalizationPositive(float factor)
{
	// If for all elements d, the maximum is D; the formula used is e^(d * factor /D)-1. This is followed by unit normalization.
	// First find the largest element
	addressElement maxE = getMaximumValue();
	float D = data[maxE.row][maxE.col];
	if(D==0)
	{
		return;
	}
	for(unsigned int i = 0; i < data.size(); i++)
	{
		for(unsigned int j = 0; j< data[i].size(); j++)
		{
			data[i][j] = exp( data[i][j] * factor / D ) - 1;
		}
	}

	// Now unit normalization
	unitNormalizationPositive();
}

/*
 * similar function and returns a new vector
 */
Std2DFloatVector Std2DFloatVector::unitExponentialNormalizationPositiveReturnNew(float factor)
{
	// If for all elements d, the maximum is D; the formula used is e^(d * factor /D)-1. This is followed by unit normalization.
	// First find the largest element
	addressElement maxE = getMaximumValue();
	float D = data[maxE.row][maxE.col];
	if(D==0)
	{
		return (*this);
	}
	Std2DFloatVector outVec;
	std::vector<float> outRow;
	for(unsigned int i = 0; i < data.size(); i++)
	{
		outRow.clear();
		for(unsigned int j = 0; j< data[i].size(); j++)
		{
			float outData = exp( data[i][j] * factor / D ) - 1;
			outRow.push_back(outData);
		}
		outVec.pushRow(outRow);
	}

	// Now unit normalization
	outVec.unitNormalizationPositive();

	return outVec;
}

/*
 * Similar to unit exponential normalization positive, except it follows the curve 1 to 0
 */
void Std2DFloatVector::unitInverseExponentialNormalizationPositive(float factor)
{
	// Follows the formula 1 / e^(x)
	for(unsigned int i = 0; i < data.size(); i++)
	{
		for(unsigned int j = 0; j< data[i].size(); j++)
		{
			data[i][j] = 1 / exp(factor * data[i][j]);
		}
	}

	// Now unit normalization
	unitNormalizationPositive();
}

/*
 * similar function and returns a new vector
 */
Std2DFloatVector Std2DFloatVector::unitInverseExponentialNormalizationPositiveReturnNew(float factor)
{
	Std2DFloatVector outVec;
	std::vector<float> outRow;

	// Follows the formula 1 / e^(x)
	for(unsigned int i = 0; i < data.size(); i++)
	{
		outRow.clear();
		for(unsigned int j = 0; j< data[i].size(); j++)
		{
			float outData = 1 / exp(factor * data[i][j]);
			outRow.push_back(outData);
		}
		outVec.pushRow(outRow);
	}

	// Now unit normalization
	outVec.unitNormalizationPositive();
	return outVec;
}



std::vector<std::vector<float> > Std2DFloatVector::getData()
{
	return data;
}


