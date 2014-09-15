#include "Quaternion.h"

#define _USE_MATH_DEFINES


/** 
* \brief converti des angles d'Euler (en degrés) en un quaternion unitaire
* body 3-2-1 sequence (yaw, pitch, roll)
* angles d'euler: psi = yaw (body-Z), teta = pitch (body-Y), phi = roll (body-X)
* l'axe z est dirigé vers le haut
* le sens diirect(positif) est le sens inverse des aiguilles d'une montre
* basé sur : http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
* \param phi,teta,psi : angles en degrés autour des axes de rotation x,y,z (respectivement)
*/
quaternion<double> danglesToQuat(double phi, double teta, double psi)
{
	centrerAngle(phi);
	centrerAngle(teta);
	centrerAngle(psi);
	quaternion<double> q1(cos(phi*M_PI / 360), sin(phi*M_PI / 360), 0, 0);
	quaternion<double> q2(cos(teta*M_PI / 360), 0, sin(teta*M_PI / 360), 0);
	quaternion<double> q3(cos(psi*M_PI / 360), 0, 0, sin(psi*M_PI / 360));
	quaternion<double> result(1,0,0,0);

	result = result*q3*q2*q1;
	normalizeQuat(result);

	return result;
}


/**
* \brief donne les coordonées de l'axe de rotation ainsi que son angle à partir d'un quaternion
* \param [in]q		: quaternion représentatn la rotation
* \param [out]axe	: coordonées de l'axe données dans le repère inertiel
* \param [out]angle	: angle de la rotation en radians
*/
void quatComp(quaternion<double> q, vect3D &axe, double &angle)
{
	angle = 2*acos(q.R_component_1());
	//_RPT1(0,"acos(0) = %f\n", acos(0));
	axe.x = q.R_component_2()/sin(angle/2);
	axe.y = q.R_component_3()/sin(angle/2);
	axe.z = q.R_component_4()/sin(angle/2);

}


/**
* \brief donne, à partir d'un quaternion, les angles d'euler (en degrés)
* angles_result.x = phi (roll)
* angles_result.y = teta (pitch)
* angles_result.z = psi (yaw)
* singularité quand 2*(qw*qy - qz*qx) = +/- 1
*/
vect3D quatToAngles_deg(quaternion<double> a_quaternion)
{
	vect3D angles_result;
	double qw, qx, qy, qz;
	double qw2, qx2, qy2, qz2;
	qw = a_quaternion.R_component_1();
	qx = a_quaternion.R_component_2();
	qy = a_quaternion.R_component_3();
	qz = a_quaternion.R_component_4();
	qw2 = qw*qw;
	qx2 = qx*qx;
	qy2 = qy*qy;
	qz2 = qz*qz;
	double test = qx* qy - qz* qw;

	if (test > 0.499999999)
	{
		// Singularity at +1
		angles_result.x = 0;									// Roll
		angles_result.y = M_PI/2;								// Pitch
		angles_result.z = 2 * (float)atan2(qx, qw);				// Yaw
		
	}
	else if (test < -0.499999999)
	{
		// Singularity at -1
		angles_result.x = 0;									// Roll
		angles_result.y = -M_PI/2;								// Pitch
		angles_result.z = -2 * (float)atan2(qx, qw);			// Yaw
		
	}
	else{
		angles_result.x = (float)atan2(2*(qx*qw + qy*qz), 1 - 2*(qx2 + qy2));			// Roll
		angles_result.y = (float)asin(2*(qw*qy - qz*qx));								// Pitch 
		angles_result.z = (float)atan2(2*(qw*qz + qx*qy), 1 - 2*(qy2 + qz2));			// Yaw
	}

	//Conversion en degrés
	angles_result.x *= (double)(180/M_PI);
	angles_result.y *= (double)(180/M_PI);
	angles_result.z *= (double)(180/M_PI);
	return angles_result;
}


/**
* \brief donne la matrice de rotation à partir d'un quaternion
*/
matrix<double> quatToMat(quaternion<double> q)
{
	matrix<double> rotation(3, 3);
	rotation(0, 0) = pow(q.R_component_1(), 2) + pow(q.R_component_2(), 2) - pow(q.R_component_3(), 2) - pow(q.R_component_4(), 2);
	rotation(1, 1) = pow(q.R_component_1(), 2) - pow(q.R_component_2(), 2) + pow(q.R_component_3(), 2) - pow(q.R_component_4(), 2);
	rotation(2, 2) = pow(q.R_component_1(), 2) - pow(q.R_component_2(), 2) - pow(q.R_component_3(), 2) + pow(q.R_component_4(), 2);

	rotation(0, 1) = 2 * (q.R_component_2()*q.R_component_3() - q.R_component_1()*q.R_component_4());
	rotation(0, 2) = 2 * (q.R_component_1()*q.R_component_3() + q.R_component_2()*q.R_component_4());
	rotation(1, 0) = 2 * (q.R_component_1()*q.R_component_4() + q.R_component_2()*q.R_component_3());
	rotation(1, 2) = 2 * (q.R_component_3()*q.R_component_4() - q.R_component_1()*q.R_component_2());
	rotation(2, 0) = 2 * (q.R_component_2()*q.R_component_4() - q.R_component_1()*q.R_component_3());
	rotation(2, 1) = 2 * (q.R_component_1()*q.R_component_2() + q.R_component_3()*q.R_component_4());

	return rotation;
}


/**
* \brief realise la rotation decrite par le quaternion sur le vecteur v
* \param q quaternion representatn la rotation
* \param v vecteur sur lequel on effectue la rotation
* \return un vecteur 3D qui correspond au param v apres rotation
*/
vect3D rotateVector(quaternion<double> q, vect3D v)
{
	quaternion<double> qb(q.R_component_1(), -q.R_component_2(), -q.R_component_3(), -q.R_component_4());
	quaternion<double> vquat(0, v.x, v.y, v.z);
	quaternion<double> result = hamiltonProduct(hamiltonProduct(q, vquat), qb);
	v.x = result.R_component_2();
	v.y = result.R_component_3();
	v.z = result.R_component_4();
	return v;
}


/**
* \brief renvoie les vitesses angulaires dans le repère inertiel à partir de mesures satellitaires
* \param q : quaternion définissant l'orientation du mobile
* \param v : vitesses angulaires (repère satellitaire)
* \return vitesse angulaire dans repère inertiel
*/
vect3D changeRepere (quaternion<double> q, vect3D v)
{
	matrix<double> rotation = quatToMat(q); //obtention de la matrice de rotation 
	rotation = transposerMat(rotation); //inversion de la matrice, revient à effectuer sa transposée dans ce cas précis (rotation autour de x, y, z)
	
	//_RPT3(0, "Matrice transposée = \n\t%f\t%f\t%f\n", rotation(0,0), rotation(0,1), rotation(0,2)); 
	//_RPT3(0, "\t%f\t%f\t%f\n", rotation(1,0), rotation(1,1), rotation(1,2)); 
	//_RPT3(0, "\t%f\t%f\t%f\n", rotation(2,0), rotation(2,1), rotation(2,2)); 
	
	v = rotation*v; // V = PV' avec P la matrice de passage
	return v;
}


/**
* \brief normalise le quaternion
* permet de mettre le quaternion sous forme unitaire quand sa norme est au dela de (1 + tolerance)
* \param tolerance : (facultatif, défaut = 0)
*/
void normalizeQuat(quaternion<double> &q, double tolerance)
{
	double norm2 = pow(q.R_component_1(), 2) + pow(q.R_component_2(), 2) + pow(q.R_component_3(), 2) + pow(q.R_component_4(), 2);
	if ( norm2 > 1+tolerance)
		q = q/sqrt(norm2);
}


/**
* \brief renvoi la transposée d'une matrice 3x3
* aussi utilisé pour effectuer l'inverse d'une matrice dans le cas de rotations selon x,y,z
*/
matrix<double> transposerMat(matrix<double> mat)
{
	matrix<double> transpose(3,3);
	transpose(0,0) = mat(0,0);
	transpose(1,1) = mat(1,1);
	transpose(2,2) = mat(2,2);
	transpose(0,1) = mat(1,0);
	transpose(0,2) = mat(2,0);
	transpose(1,0) = mat(0,1);
	transpose(1,2) = mat(2,1);
	transpose(2,0) = mat(0,2);
	transpose(2,1) = mat(1,2);

	return transpose;
}


/**
* \brief centre les angles sur 0 dans un intervalle de [-180;+180]
* \param [in/out]angle : angle en degré à recentrer
*/
void centrerAngle(double &angle)
{
	while (angle > 180 || angle <= -180)
	{
		if (angle > 180)
			angle -= 180;
		if (angle <= -180)
			angle += 180;
	}
}


/**
* \brief produit hamiltonien de 2 quaternions
* operateur* surchargé pour prendre en compte cette multiplication ed quaternions
*/
quaternion<double> hamiltonProduct(quaternion<double> q1, quaternion<double> q2)
{
	double a, b, c, d;
	a = (q1.R_component_1()*q2.R_component_1() - q1.R_component_2()*q2.R_component_2() - q1.R_component_3()*q2.R_component_3() - q1.R_component_4()*q2.R_component_4());
	b = (q1.R_component_1()*q2.R_component_2() + q1.R_component_2()*q2.R_component_1() + q1.R_component_3()*q2.R_component_4() - q1.R_component_4()*q2.R_component_3());
	c = (q1.R_component_1()*q2.R_component_3() - q1.R_component_2()*q2.R_component_4() + q1.R_component_3()*q2.R_component_1() + q1.R_component_4()*q2.R_component_2());
	d = (q1.R_component_1()*q2.R_component_4() + q1.R_component_2()*q2.R_component_3() - q1.R_component_3()*q2.R_component_2() + q1.R_component_4()*q2.R_component_1());
	quaternion<double> result(a, b, c, d);

	return result;
}


/** Affiche le quaternion sur la console */
void afficherQuat(quaternion<double> q)
{
	using namespace std;
	cout << "Quaternion = " << endl;
	cout << q.R_component_1() << endl;
	cout << q.R_component_2() << endl;
	cout << q.R_component_3() << endl;
	cout << q.R_component_4() << endl;
}


	/**OPERATEURS**/


/**
* \brief opérateur de multiplication entre une matrice 3x3 et un vecteur 3x1
* \return vecteur 3x1
*/
vect3D operator*(matrix<double> m, vect3D v)
{
	vect3D vRes = {0,0,0};

	if (m.size1() == 3 && m.size2() == 3)
	{
			vRes.x = v.x*m(0,0) + v.y*m(0,1) + v.z*m(0,2);
			vRes.y = v.x*m(1,0) + v.y*m(1,1) + v.z*m(1,2);
			vRes.z = v.x*m(2,0) + v.y*m(2,1) + v.z*m(2,2);
	}
	else
		std::cout << "erreur : matrix dimensions" << std::endl;

	return vRes;
}


/**
* \brief operateur multiplication entre quaternions
*/
quaternion<double> operator*(quaternion<double> q1, quaternion<double> q2)
{
	return hamiltonProduct(q1,q2);
}


/**
* \brief operateur division entre un quaternion et un double 
*/
quaternion<double> operator/(quaternion<double> q1, double b)
{
	quaternion<double> result(q1.R_component_1()/b, q1.R_component_2()/b, q1.R_component_3()/b, q1.R_component_4()/b);
	return result;
}