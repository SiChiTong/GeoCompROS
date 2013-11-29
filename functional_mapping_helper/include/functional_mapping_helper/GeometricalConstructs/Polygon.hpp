/* 
 * File:   Polygon.hpp
 * Author: shanker
 *
 * Created on December 21, 2011, 11:06 AM
 */

#ifndef FUNCTIONALMAPPING_POLYGON_HPP
#define	FUNCTIONALMAPPING_POLYGON_HPP

#include <geometry_msgs/Polygon.h>
#include <eu_nifti_env/Polygon.h>
#include <functional_mapping_helper/GeometricalConstructs/PointOfIntersection.hpp>
#include <functional_mapping_helper/GeometricalConstructs/pnpoly.hpp>
#include <functional_mapping_helper/GeometricalConstructs/Axis_Angle.hpp>
#include <functional_mapping_helper/GeometricalConstructs/GeometricalComputations.hpp>

void printPolygon(geometry_msgs::Polygon poly);
void pushPoint(geometry_msgs::Polygon& poly,geometry_msgs::Point pnt);
Plane* getPlaneOfPolygon(geometry_msgs::Polygon poly);

geometry_msgs::Polygon convertPolygonToXYPlane(geometry_msgs::Polygon poly);
bool isPolygonOnXYPlane(geometry_msgs::Polygon poly);

geometry_msgs::Point32 findCenterOfPolygon(geometry_msgs::Polygon poly);

geometry_msgs::Polygon orderXYPolygonPointsClockwise(geometry_msgs::Polygon unordPoly);

bool checkPointInclusionInXYPolygon(geometry_msgs::Point pnt, geometry_msgs::Polygon poly);

bool checkPolygonEqualityWithOrder(geometry_msgs::Polygon poly1, geometry_msgs::Polygon poly2 );

bool check_eu_nifti_env_PolygonEqualityWithOrder(eu_nifti_env::Polygon poly1, eu_nifti_env::Polygon poly2 );

geometry_msgs::Polygon rotatePolygonToNewPlane(Plane* oldPlane, Plane* newPlane, geometry_msgs::Polygon poly);

bool checkPointInclusionInPolygon(geometry_msgs::Point pnt, geometry_msgs::Polygon poly);

pointOfIntersection intersectionOfRayAndPolygon(Ray *ray, geometry_msgs::Polygon poly);

eu_nifti_env::Polygon convertROSPolygonToeu_nifti_envPolygon(geometry_msgs::Polygon rospoly);

geometry_msgs::Polygon converteu_nifti_envPolygonToROSPolygon(eu_nifti_env::Polygon niftirospoly);

eu_nifti_env::Polygon rotateAboutOriginAndTranslateeu_nifti_envPolygon(eu_nifti_env::Polygon niftirospoly, geometry_msgs::Quaternion, geometry_msgs::Point);

geometry_msgs::Polygon rotateAboutOriginAndTranslateROSPolygon(geometry_msgs::Polygon rospoly, geometry_msgs::Quaternion, geometry_msgs::Point);

#endif	/* POLYGON_HPP */

