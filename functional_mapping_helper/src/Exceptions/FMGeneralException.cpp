/*
 * FMGeneralException.cpp
 *
 *  Created on: Jan 22, 2013
 *      Author: shanker
 */

#ifndef FMGENERALEXCEPTION_CPP_
#define FMGENERALEXCEPTION_CPP_

#include <functional_mapping_helper/Exceptions/FMGeneralException.hpp>

FMGeneralException::FMGeneralException(const char* str)
{
	exception.assign(str);
}

FMGeneralException::FMGeneralException(std::string str)
{
	exception = str;
}

FMGeneralException::~FMGeneralException()
{

}

const char* FMGeneralException::getMessage()
{
	return(exception.c_str());
}

std::string FMGeneralException::getMessageStr()
{
	return(exception);
}

#endif /* FMGENERALEXCEPTION_CPP_ */
