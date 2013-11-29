/* 
 * File:   Matrix2X2.h
 * Author: shanker
 *
 * Created on December 12, 2011, 2:09 PM
 */

#ifndef FUNCTIONALMAPPING_MATRIX2X2_H
#define	FUNCTIONALMAPPING_MATRIX2X2_H

class Matrix2X2
{
    // We define this matrix as
    // | A11 A12 |
    // | A21 A22 |

    float a11, a12, a21, a22;
    float determinant;

public:
    Matrix2X2(float a, float b,float c,float d);

    void computeDeterminant(void);
    
    float getDeterminant(void);

};

#endif	/* MATRIX2X2_H */

