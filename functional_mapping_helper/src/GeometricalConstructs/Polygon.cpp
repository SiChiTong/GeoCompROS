/*
 * Polygon.cpp
 *
 *  Created on: May 15, 2012
 *      Author: shanker
 */

#ifndef FUNCTIONALMAPPING_POLYGON_CPP
#define	FUNCTIONALMAPPING_POLYGON_CPP

#include <functional_mapping_helper/GeometricalConstructs/Polygon.hpp>

void printPolygon(geometry_msgs::Polygon poly)
{
	for(unsigned int ctr=0;ctr<poly.points.size();ctr++)
	{
		geometry_msgs::Point pnt = convertPoint32ToPoint(poly.points[ctr]);
		printPoint(pnt);
	}
}
void pushPoint(geometry_msgs::Polygon& poly,geometry_msgs::Point pnt)
{
	geometry_msgs::Point32 pnt32 = convertPointToPoint32(pnt);
	poly.points.push_back(pnt32);
	return;
}

Plane* getPlaneOfPolygon(geometry_msgs::Polygon poly)
{
	if(poly.points.size()<3)
	{
		throw "Cannot construct polygon with less than 3 points";
		return(new Plane(constructPoint(-12,41,0),constructPoint(11,22,0),constructPoint(32,13,0)));
	}
	else
	{
		return(new Plane(convertPoint32ToPoint(poly.points[0]),convertPoint32ToPoint(poly.points[1]),convertPoint32ToPoint(poly.points[2])));
	}
}

geometry_msgs::Polygon convertPolygonToXYPlane(geometry_msgs::Polygon poly)
{
	// This function is simply to correct small errors in XY plane orientation, it is NOT to rotate to the XY plane
	for(unsigned int ctr = 0;ctr<poly.points.size();ctr++)
	{
		poly.points[ctr].z = 0;
	}
	return(poly);
}

bool isPolygonOnXYPlane(geometry_msgs::Polygon poly)
{
	bool polyOnXYPlane = true;
	Plane* xyPlane = new Plane(constructPoint(-12,41,0),constructPoint(11,22,0),constructPoint(32,13,0));
	try
	{
		Plane* polyPlane = getPlaneOfPolygon(poly);
		polyOnXYPlane = (*(xyPlane)==*(polyPlane));
		delete polyPlane;
	}
	catch (const char *e){
		ROS_ERROR("isPolygonOnXYPlane : Caught %s",e);
		throw e;}
	delete xyPlane;
	return polyOnXYPlane;
}


geometry_msgs::Point32 findCenterOfPolygon(geometry_msgs::Polygon poly)
{
	geometry_msgs::Point32 center = constructPoint32(0,0,0);
	if(poly.points.size()==0)
	{
		throw "Attempted to find center of Polygon with no points, returning (0,0,0)";
		return(center);
	}
	for(unsigned int ctr = 0;ctr<poly.points.size();ctr++)
	{
		center.x = poly.points[ctr].x + center.x;
		center.y = poly.points[ctr].y + center.y;
		center.z = poly.points[ctr].z + center.z;
	}
	center.x = center.x / poly.points.size();
	center.y = center.y / poly.points.size();
	center.z = center.z / poly.points.size();
	return(center);
}

geometry_msgs::Polygon orderXYPolygonPointsClockwise(geometry_msgs::Polygon unordPoly)
{
	// This function orders the Polygon on the XY plane with its points clockwise, using the angle the points subtend with the center
	// The first point of the polygon is kept in it's position, and the other points are ordered with respect to it
	geometry_msgs::Polygon orderedPoly;
	try
	{
		if(unordPoly.points.size()<3)
		{
			throw "Attempted to order points on XY polygon clockwise, with a polygon of less than 3 points, returning original polygon";
			return unordPoly;
		}
		bool polyOnXYPlane = isPolygonOnXYPlane(unordPoly);
		if(polyOnXYPlane==false)
		{
			throw "Attempted to order points on XY polygon clockwise, with a polygon not on the XY Plane, returning original polygon";
			return unordPoly;
		}
		geometry_msgs::Point center = convertPoint32ToPoint(findCenterOfPolygon(unordPoly));
		geometry_msgs::Vector3 refVec = constructVector3(center,convertPoint32ToPoint(unordPoly.points[0])); // The reference vector will be the vector connecting the center to the first point of the polygon
		std::vector<float> angles; // Keeps track of angles in the sorting algorithm
		std::vector<int> iters;  // Keeps track of the original iterator positions in the sorting
		angles.push_back(0.0); // This is to indicate the refVec which has a zero angle with itself
		iters.push_back(0); // This is to indicate that the refVec corresponds to the iterator 0 of the polygons, i.e. it corresponds to the first point
		for(unsigned int ctr=1;ctr<unordPoly.points.size();ctr++)
		{
			geometry_msgs::Vector3 currentVec = constructVector3(center,convertPoint32ToPoint(unordPoly.points[ctr]));
			Axis_Angle currentAxisAngle = Axis_Angle(refVec,currentVec);
			if(checkVectorEquality(currentAxisAngle.axis,constructVector3(0,0,1))) // Rotation around the Positive Z Axis is anti-clockwise
			{
				currentAxisAngle.angle = (2*M_PI) - currentAxisAngle.angle;
				currentAxisAngle.axis = constructVector3(0,0,-1);
			}
			angles.push_back(currentAxisAngle.angle);
			iters.push_back(ctr);
		}
		//Sorting algorithm (min to max)
		bool swappable = true; float tempangle; float tempiter;
		while(swappable==true)
		{
			swappable = false;
			for(unsigned int ctr=1;ctr<angles.size();ctr++)
			{
				if(angles[ctr-1]>angles[ctr])
				{
					swappable = true;
					tempangle = angles[ctr-1]; angles[ctr-1] = angles[ctr]; angles[ctr] = tempangle;
					tempiter = iters[ctr-1]; iters[ctr-1] = iters[ctr]; iters[ctr] = tempiter;
				}
			}
		}
		for(unsigned int ctr=0;ctr<angles.size();ctr++)
		{
			orderedPoly.points.push_back(unordPoly.points[iters[ctr]]);
		}
	}
	catch (char *e) {
		ROS_ERROR("orderXYPolygonPointsClockwise : Caught %s ", e);
		throw e;}
	return orderedPoly;
}

bool checkPointInclusionInXYPolygon(geometry_msgs::Point pnt, geometry_msgs::Polygon poly)
{
	int inclusion;
	try
	{
		unsigned int polySize = poly.points.size();
		if(polySize<4)
		{
			throw "Attempted to check point inclusion in XY polygon, with a polygon of less than 4 points, returning FALSE";
			return false;
		}
		bool polyOnXYPlane = isPolygonOnXYPlane(poly);
		if( fabs(pnt.z)>1 || polyOnXYPlane==false)
		{
			throw "Attempted to check point inclusion in XY polygon, with a polygon not on the XY Plane, returning FALSE";
			return false;
		}
		poly = orderXYPolygonPointsClockwise(poly);
		float *polygonPointX = new float[polySize];
		float *polygonPointY = new float[polySize];
		for(unsigned int ctr=0;ctr<polySize;ctr++)
		{
			polygonPointX[ctr] = poly.points[ctr].x;
			polygonPointY[ctr] = poly.points[ctr].y;
		};
		inclusion = pnpoly(polySize,polygonPointX,polygonPointY,pnt.x,pnt.y);
		delete []polygonPointX;
		delete []polygonPointY;
	}
	catch (char *e) {
		ROS_ERROR("checkPointInclusionInXYPolygon : Caught %s",e);
		throw e;}
	return(inclusion);

}

bool checkPolygonEqualityWithOrder(geometry_msgs::Polygon poly1, geometry_msgs::Polygon poly2 )
{
	// Checks polygon equality with order taken into account
	if(poly1.points.size()!=poly2.points.size())
	{
		return false;
	}
	else
	{
		bool polygonEquality = true;
		for(unsigned int ctr=0;ctr<poly1.points.size();ctr++)
		{
			if(fabs(poly1.points[ctr].x-poly2.points[ctr].x)>MIN_ERROR_GEOMETRY
					|| fabs(poly1.points[ctr].y-poly2.points[ctr].y)>MIN_ERROR_GEOMETRY
					|| fabs(poly1.points[ctr].z-poly2.points[ctr].z)>MIN_ERROR_GEOMETRY)
			{
				polygonEquality = false;
			}
		}
		return polygonEquality;
	}
}

bool check_eu_nifti_env_PolygonEqualityWithOrder(eu_nifti_env::Polygon poly1, eu_nifti_env::Polygon poly2 )
{
	// Checks polygon equality with order taken into account
	if(poly1.points.size()!=poly2.points.size())
	{
		return false;
	}
	else
	{
		bool polygonEquality = false;
		for(unsigned int ctr=0;ctr<poly1.points.size();ctr++)
		{
			if(checkPointEquality(poly1.points[ctr],poly2.points[ctr])==false)
			{
				polygonEquality = false;
			}
		}
		return polygonEquality;
	}
}

geometry_msgs::Polygon rotatePolygonToNewPlane(Plane* oldPlane, Plane* newPlane, geometry_msgs::Polygon poly)
{
	geometry_msgs::Polygon rotatedPoly;
	geometry_msgs::Point tempPnt;
	for(unsigned int ctr = 0; ctr < poly.points.size(); ctr++)
	{
		tempPnt = getPointOnPlaneAfterRotation(oldPlane, newPlane, convertPoint32ToPoint(poly.points[ctr]));
		rotatedPoly.points.push_back(convertPointToPoint32(tempPnt));
	}
	return(rotatedPoly);
}

bool checkPointInclusionInPolygon(geometry_msgs::Point pnt, geometry_msgs::Polygon poly)
{
	bool inclusion = false;
	try
	{
		Plane* polyPlane = getPlaneOfPolygon(poly);
		if(!(polyPlane->checkPointInclusion(pnt)))
		{
			delete polyPlane;
			return false;
		}
		else if(isPolygonOnXYPlane(poly)==true) // If polygon is on XY plane (then pnt is also on XY plane) , we do not rotate to XY plane
		{
			ROS_ERROR("Polygon is on xy plane");
			printPolygon(poly);
		}
		else
		{
			Plane* xyPlane = new Plane(constructPoint(-12,41,0),constructPoint(11,22,0),constructPoint(32,13,0));
			poly = rotatePolygonToNewPlane(polyPlane, xyPlane, poly);
			pnt = getPointOnPlaneAfterRotation(polyPlane, xyPlane, pnt);
			delete xyPlane;
		}
		delete polyPlane;
		inclusion = checkPointInclusionInXYPolygon(pnt,poly);
	}
	catch (char *e)
	{
		ROS_ERROR("checkPointInclusionInPolygon : Caught %s", e);
		throw e;
	}
	return(inclusion);
}

pointOfIntersection intersectionOfRayAndPolygon(Ray *ray, geometry_msgs::Polygon poly)
{
	pointOfIntersection intersectionPoint;
	try{
		Plane* plane = getPlaneOfPolygon(poly); // Find which plane the polygon belongs to
		IntersectionOfRayAndPlane xnrayplane = findIntersectionOfRayAndPlane(*ray, *plane); // Find intersection of ray and plane
		if(xnrayplane.doesIntersect==false)
		{
			intersectionPoint.doesIntersect = false;
			intersectionPoint.point = constructPoint(0,0,0);
		}
		else
		{
			bool pointInPoly = checkPointInclusionInPolygon(xnrayplane.pointOfXn,poly); // Is the intersection point in the polygon
			if(pointInPoly == false)
			{
				intersectionPoint.doesIntersect = false;
				intersectionPoint.point = constructPoint(0,0,0);
			}
			else
			{
				intersectionPoint.doesIntersect = true;
				intersectionPoint.point = xnrayplane.pointOfXn;
			}
		}
		delete plane;
	}
	catch (char *e) {
		ROS_ERROR("intersectionOfRayAndPolygon : Caught %s ",e);
		throw e;}
	return intersectionPoint;
}

eu_nifti_env::Polygon convertROSPolygonToeu_nifti_envPolygon(geometry_msgs::Polygon rospoly)
{
	eu_nifti_env::Polygon niftirospoly;
	for(unsigned int i=0; i<rospoly.points.size(); i++)
	{
		niftirospoly.points.push_back(convertPoint32ToPoint(rospoly.points[i]));
	}
	return(niftirospoly);
}

geometry_msgs::Polygon converteu_nifti_envPolygonToROSPolygon(eu_nifti_env::Polygon niftiROSpolygon)
{
	geometry_msgs::Polygon rospoly;
	for(unsigned int i=0; i<niftiROSpolygon.points.size(); i++)
	{
		rospoly.points.push_back(convertPointToPoint32(niftiROSpolygon.points[i]));
	}
	return rospoly;
}

eu_nifti_env::Polygon rotateAboutOriginAndTranslateeu_nifti_envPolygon(eu_nifti_env::Polygon niftirospoly, geometry_msgs::Quaternion rot, geometry_msgs::Point trans)
{
	for(unsigned int i = 0; i<niftirospoly.points.size(); i++)
	{
		niftirospoly.points[i] = rotateAndTranslatePointAboutOrigin(niftirospoly.points[i],rot,trans);
	}
	return(niftirospoly);
}

geometry_msgs::Polygon rotateAboutOriginAndTranslateROSPolygon(geometry_msgs::Polygon rospoly, geometry_msgs::Quaternion rot, geometry_msgs::Point trans)
{
	eu_nifti_env::Polygon niftirospoly = convertROSPolygonToeu_nifti_envPolygon(rospoly);
	eu_nifti_env::Polygon transPoly = rotateAboutOriginAndTranslateeu_nifti_envPolygon(niftirospoly,rot,trans);
	geometry_msgs::Polygon transROSPoly = converteu_nifti_envPolygonToROSPolygon(transPoly);
	return(transROSPoly);
}

#endif

