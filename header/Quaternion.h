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


/*	\brief	Convertie des angles d'Euler (en degrés) en un quaternion unitaire
			body 3-2-1 sequence (yaw, pitch, roll)
			with euler angles : psi = yaw (body-Z), teta = pitch (body-Y), phi = roll (body-X)
			l'axe z est dirigé vers le haut
			http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	\param	double	phi,teta,psi	angles en degrés autour des axes de rotation x,y,z (respectivement)
*/
quaternion<double> anglesToQuat(double phi, double teta, double rho);


/*	\brief	Convertie le quaternion en angles 
			Convention : 3-2-1
	\param	quatertnion<double>		quaternion		Quaternion à convertir
	\return	vect3D					angles_result	angles_result.x = phi (roll)
													angles_result.y = teta (pitch)
													angles_result.z = psi (yaw)
													singularity if 2*(qw*qy - qz*qx) = +/- 1
*/
vect3D quatToAngles_deg(quaternion<double> quaternion);


/*	\brief	Donne la valeur de l'axe et de l'angle de rotation à partir d'un quaternion
	\param	quaternion<double>	q		Quaternion de rotation
	\param	vect3D				&axe	Référence de mémoire du vecteur de l'axe de rotation
	\param	double				&angle	Référence de mémoire de l'angle de rotation
*/
void quatComp(quaternion<double> q, vect3D &axe, double &angle);


/*	\brief	Affichage d'un quaternion
	\param	quaternion<double>	q		Quaternion à afficher
*/
void afficherQuat(quaternion<double> q);

/*	\brief	Realise la rotation decrite par le quaternion sur le vecteur v
	\param	quaternion<double>	q	Quaternion representatn la rotation
	\param	vect3D				v	Vecteur sur lequel on effectue la rotation
	\return vect3D					un vecteur qui correspond a v apres rotation
*/
vect3D rotateVector(quaternion<double> q, vect3D v);

/*
	\brief renvoie les vitesses angulaires dans le repère inertiel à partir de mesures satellitaires
	\param q : quaternion définissant l'orientation du mobile
	\param v : vitesses angulaires (repère satellitaire)
	\return vitesse angulaire dans repère inertiel
*/
vect3D changeRepere(quaternion<double> q, vect3D v);

/*	
	\brief	Normalise le quaternion
			permet de mettre le quaternion sous forme unitaire quand sa norme est au dela de (1 + tolerance)
	\param	quaternion<double>	&q			Référence de mémoire du quaternion à normaliser
	\param	double				tolerance	(facultatif, défaut = 0)
*/
void normalizeQuat(quaternion<double> &q, double tolerance);


/*	
	\brief	Produit hamiltonien de 2 quaternions
	\param	quaternion<double>	q1, q2	Les quaternions pour le produit q1*q2
	\return	quaternion<double>			Résultat du produit hamiltonien q1*q2
*/
quaternion<double> hamiltonProduct(quaternion<double> q1, quaternion<double> q2);

/*	
	\brief	Convertie un quaternion en matrice de rotation
	\param	quaternion<double>	q	Quaternion à convertir
	\return	matrix<double>			Résultat de la conversion
*/
matrix<double> quatToMat(quaternion<double> q);

#endif