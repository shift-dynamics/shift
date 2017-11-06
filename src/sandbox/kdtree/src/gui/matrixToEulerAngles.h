#ifndef MATRIXTOEULERANGLES_H
#define MATRIXTOEULERANGLES_H

#define PI 3.141592653589793
#define TWOPI 2.0*PI
#define QUARTER_PI 0.25*PI

#include <cmath>
#include <QMatrix4x4>
#include <QVector3D>
#include <QtGlobal>

///////////////////////////////////////////////////////
///
///	This file is part of my collection of functions for kinematics and dynamics
/// I have modified the files to accept QMatrix4x4 input
///
/// The algorithm for extracting proper Euler/Cardan angles
/// was developed by Roger A. Wehage.
///
///////////////////////////////////////////////////////////////////////////

double catan2pi(double y, double x, double theta)
{
	///////////////////////////////////////////////////////////////////////
	//
	// Caution: atan2(y,x) will return an angle with +-2m*pi accuracy only if the
	// sign on x and y are both correct. If x and y are both off by (-1) then
	// atan2 will return an angle with accuracy of only +-m*pi. If the sign of
	// x and y are uncertain, use the alternate function, catan1pi.
	// This function returns a continuous angle obtained from the standard math
	// atan2(y,x) function. On input, the argument theta contains the 
	// previous value associated with the angle that is being computed.
	// catan2() adjusts the value returned from atan2(), so it will be as close to
	// theta as possible. The assumption is that the current value to be returned
	// by catan2() is not more than (+-)pi/4 from the previous value. This should
	// be a valid assumption, because angles generally don't change very fast over
	// the course of a simulation. Both input arguments x and y must have the
	// correct sign for catan2pi to work correctly.
	//
	//////////////////////////////////////////////////////////////////////////

	double temp;
	double a;

	// temp holds the exact current angle, but it may be off by multiples of 2*pi.
	// So it is brought closer to theta by adding multiples of 2*pi obtained from theta.
	temp = atan2(y, x) + TWOPI*floor(theta/TWOPI);

	// temp might still be a 2*pi increment above the desired angle.
	// a is the upper limit for the current angle. Add QUARTER_PI to bracket
	// the current angle.

	a = theta + QUARTER_PI;

	// Decrement the angle until it is below this upper limit if necessary.
	while (temp > a)
	temp -= TWOPI;

	// Or temp may be a 2*pi increment below the desired angle.
	// Now a is the lower limit for the current angle. Subtract QUARTER_PI
	// to bracket the current angle.

	a = theta - QUARTER_PI;

	// Increment the angle until it is above this lower limit if necessary.

	while (temp < a)
	temp += TWOPI;

	// Now the true angle is bracketed by theta - pi/4 and theta + pi/4.
	return temp;
}

double catan1pi(double y, double x, double theta)
{
	/////////////////////////////////////////////////////////////////////
	//
	// Caution: atan2(y,x) will return an angle with +-2m*pi accuracy only if the
	// sign on x and y are both correct. If x and y are both off by (-1) then
	// atan2 will return an angle with accuracy of only +-m*pi. If the sign of
	// x and y are certain, this or the alternate function, catan2pi will work.
	// If the sign of x and y are uncertain, do not use catan2pi.
	// This function returns a continuous angle from the standard 
	// math atan2(y,x) function. On input, the argument theta contains 
	// the previous value associated with the angle that is being computed.
	// catan2() adjusts the value returned from atan2(), so it will be as close to
	// theta as possible. The assumption is that the current value to be returned
	// by catan2() is not more than (+-)pi/4 from the previous value. This should
	// be a valid assumption, because angles generally don't change very fast over
	// the course of a simulation. Both input arguments x and y must either
	// have the correct sign or be off by the same sign for catan1pia() to work
	// correctly.
	//
	///////////////////////////////////////////////////////////////////////////////

	double temp;
	double a;

	// temp holds the exact current angle, but it may be off by multiples of pi.
	// So it is brought closer to theta by adding multiples of pi obtained from theta.

	temp = atan2(y, x) + PI*floor(theta/PI);

	// temp might still be one or more pi increments above the desired angle.
	// a is the upper limit for the current angle. Add QUARTER_PI to bracket
	// the current angle.

	a = theta + QUARTER_PI;

	// Decrement the angle until it is below this upper limit if necessary.

	while (temp > a)
	temp -= PI;

	// Or temp may be one or more pi increments below the desired angle.
	// Now a is the lower limit for the current angle. Subtract QUARTER_PI
	// to bracket the current angle.

	a = theta - QUARTER_PI;

	// Increment the angle until it is above this lower limit if necessary.

	while (temp < a)
	temp += PI;

	// Now the true angle is bracketed by theta - pi/4 and theta + pi/4.
	return temp;
}


void ExtractEulerAngles( QMatrix4x4 Rin, QVector3D* angles )
{
	/////////////////////////////////////////////////////////////////////////////////
	//
	// This function decomposes a general direction cosine matrix into three primitive
	// direction cosine matrices, whose axes are determined by the integers passed
	// in through axis (see below). The direction cosine matrix is passed in through Ri, and
	// the three computed angles are updated and returned in angle. The previous
	// values assigned to angle are passed in to RtoA in angle. RtoA calls catan2,
	// which uses the values stored in angle to maintain continuity if at all possible.
	// This function does not check for bad axis indices. These checks could easily
	// be added. For actual reporting purposes, especially when a large
	// number of user output requests are to be processed, it would be more efficient
	// to program four or twelve different functions and have a preprocessor
	// generate pointers to the appropriate functions as needed. That way, no logical 
	// tests would have to be performed at runtime, except in the function catan2().
	// If four functions were coded, each would handle one of the four Cases below.
	// In this case, the if statements would be eliminated, and only the axis indices
	// would be needed. If twelve functions were coded, each would handle one of the
	// twelve Cases. Again, the if statements would be eliminated, and the axis indices
	// would be hard coded, eliminating the need to pass them in or to perform any
	// other tests.
	//
	// Used to determine if the next axis is forward '+' or reverse '-' sequence.
	// It also identifies index locations in the direction cosine matrix when
	// axes are repeated, such as in the sequence 1 --> 2 --> 1.
	//
	// The function can be modified to decompose a matrix in any sequence of angles by
	// by changing "axis", but presently the sequence [rx=0, ry=1, rz=2] is hard coded 
	// in to the function.
	// 
	//////////////////////////////////////////////////////////////////

	const int axis[3] = {0, 1, 2};

	const int f[3] = {1, 2, 0};

	// Short notation for the axis identifiers.

	int i = axis[0];
	int j = axis[1];
	int k = axis[2]; 

	// The value of k will be changed in Cases 3 and 4.
	// Temp variables to hold true sin and cos values of third angle.
	double c;
	double s;
	double angle[3] = {0,0,0};

	// Get a pointer to the data in QMatrix4x4 Rin
	const float* Rpointer = Rin.constData();
	double Ri[3][3] = {{Rpointer[0], Rpointer[4], Rpointer[8]},
	                   {Rpointer[1], Rpointer[5], Rpointer[9]},
	                   {Rpointer[2], Rpointer[6], Rpointer[10]}};

	if (j == f[i]) // Identifies Cases 1 and 3: forward cycle i --> j
	{
		if (k != i) //forward cycle j --> k
		{
			// Case 1: 0 --> 1 --> 2; 1 --> 2 --> 0; 2 --> 0 --> 1
			angle[2] = catan1pi(-Ri[i][j], Ri[i][i], angle[2]);
			c = cos(angle[2]);
			s = sin(angle[2]);
			angle[0] = catan2pi(Ri[k][j]*c + Ri[k][i]*s, Ri[j][j]*c + Ri[j][i]*s, angle[0]);
			angle[1] = catan2pi(Ri[i][k], Ri[i][i]*c - Ri[i][j]*s, angle[1]);
		}
		else // k == i; reverse cycle j --> i
		{
			// Case 3: 0 --> 1 --> 0; 1 --> 2 --> 1; 2 --> 0 --> 2
			// Reset k for correct index into direction cosine matrix.
			k = f[j];
			angle[2] = catan1pi(Ri[i][j], Ri[i][k], angle[2]);
			c = cos(angle[2]);
			s = sin(angle[2]);
			angle[0] = catan2pi(Ri[k][j]*c - Ri[k][k]*s, Ri[j][j]*c - Ri[j][k]*s, angle[0]);
			angle[1] = catan2pi(Ri[i][k]*c + Ri[i][j]*s, Ri[i][i], angle[1]);
		}
	}
	else // Cases 2 and 4: reverse cycle i --> j
	{
		if (k != i) //reverse cycle j --> k
		{
			// Case 2: 0 --> 2 --> 1; 1 --> 0 --> 2; 2 --> 1 --> 0
			angle[2] = catan1pi(Ri[i][j], Ri[i][i], angle[2]);
			c = cos(angle[2]);
			s = sin(angle[2]);
			angle[0] = catan2pi(-(Ri[k][j]*c - Ri[k][i]*s), Ri[j][j]*c - Ri[j][i]*s, angle[0]);
			angle[1] = catan2pi(-Ri[i][k], Ri[i][i]*c + Ri[i][j]*s, angle[1]);
		}
		else // k == i; forward cycle j --> i
		{
			// Case 4: 0 --> 2 --> 0; 1 --> 0 --> 1; 2 --> 1 --> 2
			// Reset k for correct index into direction cosine matrix.
			k = f[i];
			angle[2] = catan1pi(-Ri[i][j], Ri[i][k], angle[2]);
			c = cos(angle[2]);
			s = sin(angle[2]);
			angle[0] = catan2pi(-(Ri[k][j]*c + Ri[k][k]*s), Ri[j][j]*c + Ri[j][k]*s, angle[0]);
			angle[1] = catan2pi(-(Ri[i][k]*c - Ri[i][j]*s), Ri[i][i], angle[1]);
		}
	}
	angles->setX(angle[0]);
	angles->setY(angle[1]);
	angles->setZ(angle[2]);
}


float ExtractScalar( QMatrix4x4 Rin )
{
	/////////////////////////////////////////
	// 
	// The norm of each column or row of an orthonormal rotation matrix is one.
	// Therefore, to find the scale factor for a rotation matrix that has been 
	// <uniformly> scaled, simply find the norm of any row or column of the rotation
	// matrix. Warning, this approach will not work for nonuniform scaling. 
	// For nonuniform scaling, one needs to find the eigenvectors using SVD or other means.
	//
	// QMatrix data is stored in column major order
	//
	///////////////////////////////////////////////////////

	const float *Ri = Rin.constData();
	// qDebug() << Ri[0] << Ri[1] << Ri[2];
	// qDebug() << sqrt(Ri[0]*Ri[0] + Ri[1]*Ri[1] + Ri[2]*Ri[2]);
	// qDebug() << sqrt(Ri[4]*Ri[4] + Ri[5]*Ri[5] + Ri[6]*Ri[6]);
	// qDebug() << sqrt(Ri[8]*Ri[8] + Ri[9]*Ri[9] + Ri[10]*Ri[10]);
	return sqrt( Ri[0]*Ri[0] + Ri[1]*Ri[1] + Ri[2]*Ri[2] );
}


QVector3D ExtractTranslation( QMatrix4x4 Rin )
{
	const float *Ri = Rin.constData();
	QVector3D translation;
	//qDebug() << Ri[3] << Ri[7] << Ri[11];
	translation.setX(Ri[3]);
	translation.setY(Ri[7]);
	translation.setZ(Ri[11]);
	return translation;
}



#endif
