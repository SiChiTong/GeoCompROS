/*
 * VisualizationOctave.hpp
 *
 *  Created on: May 4, 2012
 *      Author: shanker
 */

// This class is used to write lines, polygons to a specified octave file

#ifndef FUNCTIONALMAPPING_VISUALIZATIONOCTAVE_HPP_
#define FUNCTIONALMAPPING_VISUALIZATIONOCTAVE_HPP_

#include <ros/ros.h>
#include <functional_mapping_helper/GeometricalConstructs/Pose.hpp>
#include <functional_mapping_helper/GeometricalConstructs/Polygon.hpp>
#include <functional_mapping_helper/GeometricalConstructs/Quaternion.hpp>
#include <fstream>

#define AXISLENGTH 5

class VisualizationOctave
{

std::ofstream fwrite;
std::string filename;
bool openfile();
bool closefile();

int surfaceColorMapValue;

public:
	VisualizationOctave(const char *file);

	~VisualizationOctave();

	bool clearfile();

	void init();

	// Write command to print line from point A to point B to file
	bool addLine(geometry_msgs::Point A, geometry_msgs::Point B, char color = 'b');

	//Write command to print polygon poly to file
	bool addPolygon(geometry_msgs::Polygon poly, char color = 'g');

	// Write command to print arrow (quiver) to file
	bool addArrow(float x1, float y1, float x2, float y2, char color = 'b');
	bool addArrow(float x1, float y1, float z1, float x2, float y2, float z2, char color = 'b');
	bool addArrow(geometry_msgs::Pose* pose, char color = 'b');

	// Write command to print a cluster of arrows to file
	bool addArrowCluster(std::vector<geometry_msgs::Pose*>);

	// Write command to print a scatter of points to file
	bool addPointScatter(std::list<geometry_msgs::Point>, int size = 5);

	// Write command to print a surface to file
	bool addSurface(std::vector<float>, std::vector<float>, std::vector<std::vector<float> >, bool differentColour = false);

};





#endif /* VISUALIZATIONOCTAVE_HPP_ */
