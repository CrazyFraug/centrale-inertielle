#ifndef DEF_QUATERNION
#define DEF_QUATERNION

#include "boost\math\quaternion.hpp"
#include <math.h>
#include <iostream>

using namespace boost::math;
/** 
* 1-Creer 2 quaternions : pour la mesure et pour l'estimation
* 2-Convertir les angles d'Euler en quaternions
* 3-Appliquer Kalman
* 4-Réaliser la rotation (cette étape peut peut-être se faire avec la matrice de rotation, 
* 							ce qui nous éviterais les singularités des angles d'euler à +90 et -90)
*/
quaternion<double> anglesToQuat(double phi, double teta, double rho);
//double* quatToAngles(quaternion<double> q);
void afficherQuat(quaternion<double>);

#endif