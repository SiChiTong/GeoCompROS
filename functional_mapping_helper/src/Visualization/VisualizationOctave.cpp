/*
 * VisualizationOctave.cpp
 *
 *  Created on: May 23, 2012
 *      Author: shanker
 */

#ifndef FUNCTIONALMAPPING_VISUALIZATIONOCTAVE_CPP_
#define FUNCTIONALMAPPING_VISUALIZATIONOCTAVE_CPP_

#include <functional_mapping_helper/Visualization/VisualizationOctave.hpp>

VisualizationOctave::VisualizationOctave(const char *file) : filename(file)
{
	surfaceColorMapValue = 1;
}

VisualizationOctave::~VisualizationOctave()
{
}

bool VisualizationOctave::openfile() // Opens the file using append option
{
	try{fwrite.open(filename.c_str(),std::ios::out | std::ios::app);}
	catch (std::ofstream::failure &e) {
		ROS_ERROR("Could not open %s for writing to",filename.c_str());
		return(false);
	}
	return(true);
}

bool VisualizationOctave::closefile()
{
	try{fwrite.close();}
	catch (std::ofstream::failure &e) {
		ROS_ERROR("Could not close file %s",filename.c_str());
		return(false);
	}
	return(true);
}

bool VisualizationOctave::clearfile() // Clears the file using trunc option
{
	try{fwrite.open(filename.c_str());}
	catch (std::ofstream::failure &e) {
		ROS_ERROR("Could not open %s for writing to",filename.c_str());
		return(false);
	  }
	init();
	if(closefile()==false)
	{
		return(false);
	}
	return(true);
}

void VisualizationOctave::init() // Write initial statements for octave plotting
{
	fwrite<<"clear all\nfigure\naxis square\nxlabel('X')\nylabel('Y')\nzlabel('Z')\nhold on\n";
}

// Write command to print line from point A to point B to file
bool VisualizationOctave::addLine(geometry_msgs::Point A, geometry_msgs::Point B, char color)
{
	if(openfile()==false)
		{return(false);}
	fwrite<<"x=["<<A.x<<","<<B.x<<"];\n";
	fwrite<<"y=["<<A.y<<","<<B.y<<"];\n";
	fwrite<<"z=["<<A.z<<","<<B.z<<"];\n";
	fwrite<<"plot3(x,y,z,'color','"<<color<<"');\n";
	if(closefile()==false)
		{return(false);}
	return(true);
}

//Write command to print polygon poly to file
bool VisualizationOctave::addPolygon(geometry_msgs::Polygon poly, char color)
{
	if(openfile()==false)
		{return(false);}
	// X Array
	fwrite<<"x=[";
	for(unsigned int ctr=0;ctr<poly.points.size();ctr++)
	{
		fwrite<<poly.points[ctr].x;
		fwrite<<",";
		if(ctr==poly.points.size()-1)
		{
			fwrite<<poly.points[0].x;
		}
	}
	fwrite<<"];\n";
	// Y Array
	fwrite<<"y=[";
	for(unsigned int ctr=0;ctr<poly.points.size();ctr++)
	{
		fwrite<<poly.points[ctr].y;
		fwrite<<",";
		if(ctr==poly.points.size()-1)
		{
			fwrite<<poly.points[0].y;
		}
	}
	fwrite<<"];\n";
	// Z Array
	fwrite<<"z=[";
	for(unsigned int ctr=0;ctr<poly.points.size();ctr++)
	{
		fwrite<<poly.points[ctr].z;
		fwrite<<",";
		if(ctr==poly.points.size()-1)
		{
			fwrite<<poly.points[0].z;
		}
	}
	fwrite<<"];\n";
	fwrite<<"plot3(x,y,z,'color','"<<color<<"');\n";
	if(closefile()==false)
		{return(false);}
	return(true);
}

bool VisualizationOctave::addArrow(float x1, float y1, float x2, float y2, char color)
{
	if(openfile()==false)
		{return(false);}
	fwrite<<"x1=["<<x1<<"];\n";
	fwrite<<"y1=["<<y1<<"];\n";
	fwrite<<"x2=["<<x2<<"];\n";
	fwrite<<"y2=["<<y2<<"];\n";
	fwrite<<"quiver(x1,y1,x2,y2,'color','"<<color<<"');\n";
	if(closefile()==false)
		{return(false);}
	return true;
}

bool VisualizationOctave::addArrow(float x1, float y1, float z1, float x2, float y2, float z2, char color)
{
	if(openfile()==false)
		{return(false);}
	fwrite<<"x1=["<<x1<<"];\n";
	fwrite<<"y1=["<<y1<<"];\n";
	fwrite<<"z1=["<<z1<<"];\n";
	fwrite<<"x2=["<<x2<<"];\n";
	fwrite<<"y2=["<<y2<<"];\n";
	fwrite<<"z2=["<<z2<<"];\n";
	fwrite<<"quiver(x1,y1,z1,x2,y2,z2,'color','"<<color<<"');\n";
	if(closefile()==false)
		{return(false);}
	return true;
}

bool VisualizationOctave::addArrow(geometry_msgs::Pose* pose, char color)
{
	bool success = true;
	geometry_msgs::Vector3 unitX = constructVector3(1,0,0);
	geometry_msgs::Vector3 dirVec = rotateVectorByROSQuaternion(unitX,pose->orientation);
	dirVec = reduceToUnitVector(dirVec);
	success = success && addArrow(pose->position.x,pose->position.y,pose->position.z,dirVec.x,dirVec.y,dirVec.z,color);
	return success;
}



bool VisualizationOctave::addArrowCluster(std::vector<geometry_msgs::Pose*> poseClust)
{
	if(openfile()==false)
		{return(false);}
	bool success = true;
	for(unsigned int ctr = 0; ctr < poseClust.size(); ctr++ )
	{
		success = success & addArrow(poseClust[ctr]);
	}
 	if(closefile()==false)
		{return(false);}
	return success;
}

bool VisualizationOctave::addPointScatter(std::list<geometry_msgs::Point> pntClust, int size)
{
	if(openfile()==false)
		{return(false);}

	fwrite<<"x = [";

	std::list<geometry_msgs::Point>::iterator it;
	std::list<geometry_msgs::Point>::iterator lastbutone;
	lastbutone = pntClust.end(); lastbutone--;
	for(it=pntClust.begin(); it!=pntClust.end(); it++)
	{
		fwrite<<(*it).x;
		if(it!=lastbutone)
		{
			fwrite<<",";
		}
	}
	fwrite<<"];\n";

	fwrite<<"y = [";
	for(it=pntClust.begin(); it!=pntClust.end(); it++)
	{
		fwrite<<(*it).y;
		if(it!=lastbutone)
		{
			fwrite<<",";
		}
	}
	fwrite<<"];\n";

	fwrite<<"z = [";
	for(it=pntClust.begin(); it!=pntClust.end(); it++)
	{
		fwrite<<(*it).z;
		if(it!=lastbutone)
		{
			fwrite<<",";
		}
	}
	fwrite<<"];\n";

	fwrite<<"scatter3(x(:),y(:),z(:),"<<size<<",z(:));\n";
	if(closefile()==false)
		{return(false);}
	return true;
}

/*
 * Adds the surface defined by this 3D vector. This function will return false and throw an error if the no. of rows and cols in the matrix do not correspond to the individual arrays.
 */
bool VisualizationOctave::addSurface(std::vector<float> rows, std::vector<float> cols, std::vector< std::vector < float > > surf, bool differentColour)
{
	unsigned int colsize;
	if(surf.size()==0)
	{
		ROS_ERROR("VisualizationOctave::addSurface : The input surface is empty, cannot print to file");
		return false;
	}
	else if( (surf.size()!=rows.size()) || (surf[0].size()!=cols.size()) )
	{
		ROS_ERROR("VisualizationOctave::addSurface : The no. of rows %d and columns %d of the surface are not equal to those of the individual arrays (%d,%d). Cannot print to file",rows.size(),cols.size(),surf.size(),surf[0].size());
		return false;
	}
	else
	{
		colsize = cols.size();
	}
	if(openfile()==false)
		{return(false);}

	fwrite<<"\n rows = [";

	std::vector<float>::iterator it;
	std::vector<float>::iterator lastbutone;
	lastbutone = rows.end(); lastbutone--;
	for(it=rows.begin(); it!=rows.end(); it++)
	{
		fwrite<<(*it);
		if(it!=lastbutone)
		{
			fwrite<<",";
		}
	}
	fwrite<<"];\n";

	fwrite<<"\n cols = [";
	lastbutone = cols.end(); lastbutone--;
	for(it=cols.begin(); it!=cols.end(); it++)
	{
		fwrite<<(*it);
		if(it!=lastbutone)
		{
			fwrite<<",";
		}
	}
	fwrite<<"];\n";

	fwrite<<"\n surf = [";

	std::vector<std::vector<float> >::iterator rowIt;
	std::vector<float>::iterator colIt;
	for(rowIt=surf.begin(); rowIt!=surf.end(); rowIt++)
	{
		if((*rowIt).size()!=colsize)
		{
			ROS_ERROR("VisualizationOctave::addSurface : The no. of rows and columns of the surface are not equal to those of the individual arrays. Cannot print to file");
			return false;
		}
		for(colIt = (*rowIt).begin(); colIt != (*rowIt).end(); colIt++)
		{
			fwrite<<*colIt;
			if(colIt!=(*rowIt).end()-1)
			{
				fwrite<<",";
			}
		}
		fwrite<<";\n";
	}
	fwrite<<"];\n";

	if(differentColour)
	{
		fwrite<<"\n map = jet()\n colormap(map)";
		fwrite<<"\n c = ones(size(surf))";
		fwrite<<"\n c = "<<surfaceColorMapValue<<"* c";
		fwrite<<"\n mesh(cols,rows,surf,c)";
		surfaceColorMapValue++;
	}
	else
	{
		fwrite<<"mesh(cols,rows,surf);\n";
	}

	if(closefile()==false)
		{return(false);}
	return true;
}

#endif
