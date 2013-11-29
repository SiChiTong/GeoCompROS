#ifndef STRINGHELPER_CPP_
#define STRINGHELPER_CPP_

#include <functional_mapping_helper/HelperClasses/StringHelper.hpp>

std::string StringHelper::concatenateFilePathAndName(const char* pathToFile, const char* fileName)
{
	std::string concatPath;
	std::string pathStr(pathToFile);
	try
	{
		if(pathStr.size()!=0)
		{
			if( *(pathStr.end()-1) != '/')
			{
				pathStr.append("/");
			}
		}
		concatPath = pathStr;
		concatPath.append(fileName);
	}
	catch(std::exception &e)
	{
		ROS_ERROR("PointCloudReader::readFromPCDFile : Got std exception : %s",e.what());
		throw(FMGeneralException(e.what()));
	}
	return concatPath;
}

#endif
