/*
 * FMGeneralException.hpp
 *
 *  Created on: Jan 22, 2013
 *      Author: shanker
 */

#ifndef FMGENERALEXCEPTION_HPP_
#define FMGENERALEXCEPTION_HPP_

#include <ros/ros.h>

class FMGeneralException
{
	std::string exception;
public:
	FMGeneralException(std::string);
	FMGeneralException(const char *);
	~FMGeneralException();
	std::string getMessageStr();
	const char* getMessage();
};


#endif /* FMGENERALEXCEPTION_HPP_ */
