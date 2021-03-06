/*
 *  MatrixMath.h Library for Matrix Math
 * 
 *  Modified by Nate Costello on 4/1/15 to remove printing functions and
 *  to rename the instance created to MatrixInst to eliminate compilation
 *  conflicts when using the Arduino Due and compiling in Sublime Text + Stino
 *  Created by Charlie Matlack on 12/18/10.
 *  Modified from code by RobH45345 on Arduino Forums, algorithm from
 *  NUMERICAL RECIPES: The Art of Scientific Computing.
 *  Modified to work with Arduino 1.0/1.5 by randomvibe & robtillaart
 *  Made into a real library on GitHub by Vasilis Georgitzikis (tzikis)
 *  so that it's easy to use and install (March 2015)
 */

#ifndef MatrixMath_h
#define MatrixMath_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class MatrixMath
{
public:
	//MatrixMath();
	void Print(float* A, int m, int n);
	void Copy(float* A, int n, int m, float* B);
	void Multiply(float* A, float* B, int m, int p, int n, float* C);
	void Add(float* A, float* B, int m, int n, float* C);
	void Subtract(float* A, float* B, int m, int n, float* C);
	void Transpose(float* A, int m, int n, float* C);
	void Scale(float* A, int m, int n, float k);
	int Invert(float* A, int n);
};

extern MatrixMath MatrixInst;
#endif
