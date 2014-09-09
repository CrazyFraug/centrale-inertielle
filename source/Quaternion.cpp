#include "Quaternion.h"

#define _USE_MATH_DEFINES

/**Fonction non verifiée, utiliser danglesToQuat() de préférence */
quaternion<double> anglesToQuat(double phi, double teta, double rho)
{

	double c1, c2, c3;
	double s1, s2, s3;
	double q1, q2, q3, q0;
	double a, b1, b2, b3;
	c1 = cos(phi / 2);
	c2 = cos(teta / 2);
	c3 = cos(rho / 2);
	s1 = sin(phi / 2);
	s2 = sin(teta / 2);
	s3 = sin(rho / 2);
	q0 = c1*s2*c3 - s1*c2*s3;
	q1 = c1*c2*c3 + s1*s2*s3;
	q2 = c1*c2*s3 - s1*s2*c3;
	q3 = s1*c2*c3 + c1*s2*s3;
	a = 2 * acos(q0);
	b1 = acos(q1/sin(a/2));
	b2 = acos(q2/sin(a/2));
	b3 = acos(q3/sin(a/2));
	quaternion<double> quat_resultat(q0, q1 , q2, q3);


	return quat_resultat;
}

/** 
* \brief convertie des angles d'Euler (en degrés) en un quaternion unitaire
* body 3-2-1 sequence (yaw, pitch, roll)
* with euler angles : psi = yaw (body-Z), teta = pitch (body-Y), phi = roll (body-X)
* l'axe z est dirigé vers le haut
* http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
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

	_RPT4(0, "q = %f  %f  %f  %f \n", result.R_component_1(), result.R_component_2(), result.R_component_3(), result.R_component_4());

	return result;
}


/**
* \brief donne la valeur de l'axe et de l'angle de rotation à partir d'un quaternion
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
* angles_result.x = phi (roll)
* angles_result.y = teta (pitch)
* angles_result.z = psi (yaw)
* singularity if 2*(qw*qy - qz*qx) = +/- 1
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
		// Singularity at north pole
		angles_result.x = 0;									// Roll
		angles_result.y = M_PI/2;								// Pitch
		angles_result.z = 2 * (float)atan2(qx, qw);				// Yaw
		
	}
	else if (test < -0.499999999)
	{
		// Singularity at south pole
		angles_result.x = 0;									// Roll
		angles_result.y = -M_PI/2;								// Pitch
		angles_result.z = -2 * (float)atan2(qx, qw);			// Yaw
		
	}
	else{
		angles_result.x = (float)atan2(2*(qx*qw + qy*qz), 1 - 2*(qx2 + qy2));			// Roll
		angles_result.y = (float)asin(2*(qw*qy - qz*qx));								// Pitch 
		angles_result.z = (float)atan2(2*(qw*qz + qx*qy), 1 - 2*(qy2 + qz2));			// Yaw
	}

	angles_result.x *= (double)(180/M_PI);
	angles_result.y *= (double)(180/M_PI);
	angles_result.z *= (double)(180/M_PI);
	return angles_result;
}


void afficherQuat(quaternion<double> q)
{
	using namespace std;
	cout << q.R_component_1() << endl;
	cout << q.R_component_2() << endl;
	cout << q.R_component_3() << endl;
	cout << q.R_component_4() << endl;
}

/**
* \brief realise la rotation decrite par le quaternion sur le vecteur v
* \param q quaternion representatn la rotation
* \param v vecteur sur lequel on effectue la rotation
* \return un vecteur 3D qui correspond a v apres rotation
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
* \brief normalise le quaternion
* permet de mettre le quaternion sous forme unitaire quand sa norme est au dela de (1 + tolerance)
* \param tolerance : (facultatif, défaut = 0)
* \return norme du quaternion
*/
void normalizeQuat(quaternion<double> &q, double tolerance)
{
	double norm2 = pow(q.R_component_1(), 2) + pow(q.R_component_2(), 2) + pow(q.R_component_3(), 2) + pow(q.R_component_4(), 2);
	if ( norm2 > 1+tolerance)
		q = q/sqrt(norm2);

}

/**
* \brief centre les angles sur 0 dans un intervalle de [-180;+180]
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

matrix<double> quatToMat(quaternion<double> q)
{
	matrix<double> rotation(3, 3);
	rotation(0, 0) = (pow(q.R_component_2(), 2) + pow(q.R_component_1(), 2)) - (pow(q.R_component_3(), 2) + pow(q.R_component_4(), 2));
	rotation(1, 1) = (pow(q.R_component_3(), 2) + pow(q.R_component_1(), 2)) - (pow(q.R_component_4(), 2) + pow(q.R_component_2(), 2));
	rotation(2, 2) = (pow(q.R_component_4(), 2) + pow(q.R_component_1(), 2)) - (pow(q.R_component_3(), 2) + pow(q.R_component_2(), 2));
	rotation(0, 1) = 2 * (q.R_component_2()*q.R_component_3() - q.R_component_1()*q.R_component_4());
	rotation(0, 2) = 2 * (q.R_component_2()*q.R_component_4() + q.R_component_1()*q.R_component_3());
	rotation(1, 0) = 2 * (q.R_component_2()*q.R_component_3() + q.R_component_1()*q.R_component_4());
	rotation(1, 2) = 2 * (q.R_component_3()*q.R_component_4() - q.R_component_1()*q.R_component_2());
	rotation(2, 0) = 2 * (q.R_component_2()*q.R_component_4() - q.R_component_1()*q.R_component_3());
	rotation(2, 1) = 2 * (q.R_component_3()*q.R_component_4() - q.R_component_1()*q.R_component_2());

	return rotation;
}


quaternion<double> operator*(quaternion<double> q1, quaternion<double> q2)
{
	return hamiltonProduct(q1,q2);
}

quaternion<double> operator/(quaternion<double> q1, double b)
{
	quaternion<double> result(q1.R_component_1()/b, q1.R_component_2()/b, q1.R_component_3()/b, q1.R_component_4()/b);
	return result;
}
