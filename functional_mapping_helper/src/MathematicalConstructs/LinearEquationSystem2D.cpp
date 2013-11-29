/*
 * LinearEquationSystem2D.cpp
 *
 *  Created on: May 23, 2012
 *      Author: shanker
 */


#ifndef FUNCTIONALMAPPING_LINEAREQUATIONSYSTEM2D_CPP
#define	FUNCTIONALMAPPING_LINEAREQUATIONSYSTEM2D_CPP

#include <functional_mapping_helper/MathematicalConstructs/LinearEquationSystem2D.hpp>

LinearEquationSystem2D::LinearEquationSystem2D(float a1, float b1, float c1, float a2, float b2, float c2)
{
	A1 = a1;
	B1 = b1;
	C1 = c1;
	A2 = a2;
	B2 = b2;
	C2 = c2;
	consistency = true;
	solveLinearEquationSystem2DUsingKramersRule();
}

void LinearEquationSystem2D::solveLinearEquationSystem2DUsingKramersRule(void)
{
	Matrix2X2* M0 = new Matrix2X2(A1,B1,A2,B2);
	if(M0->getDeterminant()==0)
	{
		x = 0;
		y = 0;
		consistency = false;
		delete M0;
		return;
	}
	// Subcase 1: A1==0 and B1!=0 and A2!=0 and B2==0
	else if((A1==0)&&(B1!=0)&&(A2!=0)&&(B2==0))
	{
		 y = C1/B1;
		 x = C2/A2;
		 delete M0;
	}
	// Subcase 2: A1!=0 and B1==0 and A2==0 and B2!=0
	else if((A1!=0)&&(B1==0)&&(A2==0)&&(B2!=0))
	{
		 x = C1/A1;
		 y = C2/B2;
		 delete M0;
	}
	else
	{
		Matrix2X2* Mx = new Matrix2X2(C1,B1,C2,B2);
		Matrix2X2* My = new Matrix2X2(A1,C1,A2,C2);
		x = Mx->getDeterminant()/M0->getDeterminant();
		y = My->getDeterminant()/M0->getDeterminant();
		delete M0;
		delete Mx;
		delete My;
		return;
	}
}

float LinearEquationSystem2D::getX()
{
	return(x);
}

float LinearEquationSystem2D::getY()
{
	return(y);
}

bool LinearEquationSystem2D::getConsistency()
{
	return(consistency);
}

#endif
