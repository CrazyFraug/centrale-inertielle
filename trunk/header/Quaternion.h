#ifndef DEF_QUATERNION
#define DEF_QUATERNION

#include "boost\math\quaternion.hpp"
#include "boost\numeric\ublas\matrix.hpp"
#include <math.h>
#include <iostream>
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
quaternion<double> anglesToQuat(double phi, double teta, double rho);
vect3D quatToAngles(quaternion<double> quaternion);
void afficherQuat(quaternion<double>);
vect3D rotateVector(quaternion<double> q, vect3D v);
double calculateNorm(quaternion<double> q);
quaternion<double> hamiltonProduct(quaternion<double> q1, quaternion<double> q2);
matrix<double> quatToMat(quaternion<double> q);

#endif