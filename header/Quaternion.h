#ifndef DEF_QUATERNION
#define DEF_QUATERNION

//#define _USE_MATH_DEFINES

#include "boost\math\quaternion.hpp"
#include "boost\numeric\ublas\matrix.hpp"
#include <math.h>
#include <iostream>
#include "SDL2\SDL_stdinc.h"
#include "Structure.h"

using namespace boost::math;
using namespace boost::numeric::ublas;
/**
* 1-Creer 2 quaternions : pour la mesure et pour l'estimation
* 2-Convertir les angles d'Euler en quaternions
* 3-Appliquer Kalman
* 4-Réaliser la rotation (cette étape peut peut-être se faire avec la matrice de rotation,
* 							ce qui nous éviterais les singularités des angles d'euler à +90 et -90)
*/
quaternion<double> danglesToQuat(double phi, double teta, double rho);
void quatComp(quaternion<double> q, vect3D &axe, double &angle);
vect3D quatToAngles_deg(quaternion<double> quaternion);
void afficherQuat(quaternion<double>);
vect3D rotateVector(quaternion<double> q, vect3D v);
vect3D changeRepere (quaternion<double> q, vect3D v);
void normalizeQuat(quaternion<double> &q, double tolerance=0);
void centrerAngle(double &angle);	
quaternion<double> hamiltonProduct(quaternion<double> q1, quaternion<double> q2);
matrix<double> quatToMat(quaternion<double> q);
matrix<double> transposerMat(matrix<double> mat);
quaternion<double> operator*(quaternion<double> q1, quaternion<double> q2);
quaternion<double> operator*(quaternion<double> q1, quaternion<double> q2);
quaternion<double> operator/(quaternion<double> q1, double b);
vect3D operator*(matrix<double> m, vect3D v);

void test_eulerQuat(void);
void test_changeRepere(void);

#endif