#ifndef DEF_QUATERNION
#define DEF_QUATERNION

#include "boost\math\quaternion.hpp"
#include "boost\numeric\ublas\matrix.hpp"
#include <math.h>
#include <iostream>
#include "Structure.h"
#include "SDL2\SDL_stdinc.h"



using namespace boost::math;
using namespace boost::numeric::ublas;
/**
* 1-Creer 2 quaternions : pour la mesure et pour l'estimation
* 2-Convertir les angles d'Euler en quaternions
* 3-Appliquer Kalman
* 4-Réaliser la rotation (cette étape peut peut-être se faire avec la matrice de rotation,
* 							ce qui nous éviterais les singularités des angles d'euler à +90 et -90)
*/

/**
* \brief convertie des angles d'Euler (en degrés) en un quaternion unitaire
* body 3-2-1 sequence (yaw, pitch, roll)
* with euler angles : psi = yaw (body-Z), teta = pitch (body-Y), phi = roll (body-X)
* l'axe z est dirigé vers le haut
* http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
* \param phi,teta,psi : angles en degrés autour des axes de rotation x,y,z (respectivement)
*/
quaternion<double> anglesToQuat(double phi, double teta, double rho);


/**
* angles_result.x = phi (roll)
* angles_result.y = teta (pitch)
* angles_result.z = psi (yaw)
* singularity if 2*(qw*qy - qz*qx) = +/- 1
*/
vect3D quatToAngles_deg(quaternion<double> quaternion);


/**
*	\brief donne la valeur de l'axe et de l'angle de rotation à partir d'un quaternion
*/
void quatComp(quaternion<double> q, vect3D &axe, double &angle);



void afficherQuat(quaternion<double>);

/**
* \brief realise la rotation decrite par le quaternion sur le vecteur v
* \param q quaternion representatn la rotation
* \param v vecteur sur lequel on effectue la rotation
* \return un vecteur 3D qui correspond a v apres rotation
*/
vect3D rotateVector(quaternion<double> q, vect3D v);


/**
* \brief normalise le quaternion
* permet de mettre le quaternion sous forme unitaire quand sa norme est au dela de (1 + tolerance)
* \param tolerance : (facultatif, défaut = 0)
* \return norme du quaternion
*/
void normalizeQuat(quaternion<double> &q, double tolerance);


/**
* \brief produit hamiltonien de 2 quaternions
*/
quaternion<double> hamiltonProduct(quaternion<double> q1, quaternion<double> q2);


matrix<double> quatToMat(quaternion<double> q);

#endif