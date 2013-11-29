/*
 * StringHelper.hpp
 *
 *  Created on: Jul 25, 2013
 *      Author: shanker
 */

#ifndef STRINGHELPER_HPP_
#define STRINGHELPER_HPP_

#include <ros/ros.h>
#include <functional_mapping_helper/Exceptions/FMGeneralException.hpp>

class StringHelper
{
public:
	std::string concatenateFilePathAndName(const char*, const char*);
};


#endif /* STRINGHELPER_HPP_ */
