/* 
 * File:   LinearEquation2D.hpp
 * Author: shanker
 *
 * Created on December 12, 2011, 2:18 PM
 */



#ifndef FUNCTIONALMAPPING_LINEAREQUATIONSYSTEM2D_HPP
#define	FUNCTIONALMAPPING_LINEAREQUATIONSYSTEM2D_HPP

#include <ros/ros.h>
#include <functional_mapping_helper/MathematicalConstructs/Matrix2X2.h>


class LinearEquationSystem2D
{
    // Equations are
    // A1x + B1y = C1
    // A2x + B2y = C2
    float x, y; // Solutions to the equation
    float A1, A2, B1, B2, C1, C2; // Parameters
    bool consistency;

public:
    LinearEquationSystem2D(float a1, float b1, float c1, float a2, float b2, float c2);

    void solveLinearEquationSystem2DUsingKramersRule(void);

    float getX();

    float getY();

    bool getConsistency();
};


#endif	/* LINEAREQUATIONSYSTEM2D_HPP */

