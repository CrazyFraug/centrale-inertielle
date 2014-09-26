#include "Quaternion.h"
#include "Tools.h"
#include <math.h>

/************************************************************************************************************/

/* Surcharge operateur * et / pour les quaternions */

quaternion<double> operator*(quaternion<double> q1, quaternion<double> q2)
{
	return hamiltonProduct(q1, q2);
}

quaternion<double> operator/(quaternion<double> q1, double b)
{
	quaternion<double> result(q1.R_component_1() / b, q1.R_component_2() / b, q1.R_component_3() / b, q1.R_component_4() / b);
	return result;
}

/*
\brief opérateur de multiplication entre une matrice 3x3 et un vecteur 3x1
\return vecteur 3x1
*/
vect3D operator*(matrix<double> m, vect3D v)
{
	vect3D vRes = { 0, 0, 0 };

	if (m.size1() == 3 && m.size2() == 3)
	{
		vRes.x = v.x*m(0, 0) + v.y*m(0, 1) + v.z*m(0, 2);
		vRes.y = v.x*m(1, 0) + v.y*m(1, 1) + v.z*m(1, 2);
		vRes.z = v.x*m(2, 0) + v.y*m(2, 1) + v.z*m(2, 2);
	}
	else
		std::cout << "erreur : matrix dimensions" << std::endl;

	return vRes;
}

/************************************************************************************************************/

quaternion<double> anglesToQuat(double phi_deg, double teta_deg, double rho_deg)
{
	normAngle(phi_deg);
	normAngle(teta_deg);
	normAngle(rho_deg);
	quaternion<double> q1(cos(phi_deg*M_PI / 360), sin(phi_deg*M_PI / 360), 0, 0);
	quaternion<double> q2(cos(teta_deg*M_PI / 360), 0, sin(teta_deg*M_PI / 360), 0);
	quaternion<double> q3(cos(rho_deg*M_PI / 360), 0, 0, sin(rho_deg*M_PI / 360));
	quaternion<double> result(1, 0, 0, 0);

	result = result*q3*q2*q1;
	normalizeQuat(result, 0);

	_RPT4(0, "q = %f  %f  %f  %f \n", result.R_component_1(), result.R_component_2(), result.R_component_3(), result.R_component_4());

	return result;
}

/************************************************************************************************************/

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

	if (test > 0.49999999999)
	{
		// Singularity at north pole
		angles_result.x = 0;									// Roll
		angles_result.y = M_PI / 2;								// Pitch
		angles_result.z = 2 * (float)atan2(qx, qw);				// Yaw

	}
	else if (test < -0.49999999999)
	{
		// Singularity at south pole
		angles_result.x = 0;									// Roll
		angles_result.y = -M_PI / 2;							// Pitch
		angles_result.z = -2 * (float)atan2(qx, qw);			// Yaw

	}
	else
	{
		angles_result.x = (float)atan2(2 * (qx*qw + qy*qz), 1 - 2 * (qx2 + qy2));		// Roll
		angles_result.y = (float)asin(2 * (qw*qy - qz*qx));								// Pitch 
		angles_result.z = (float)atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy2 + qz2));		// Yaw
	}

	angles_result.x *= (double)(180 / M_PI);
	angles_result.y *= (double)(180 / M_PI);
	angles_result.z *= (double)(180 / M_PI);
	return angles_result;
}

/************************************************************************************************************/

void quatComp(quaternion<double> q, vect3D &axe, double &angle)
{
	angle = 2 * acos(q.R_component_1());
	//_RPT1(0,"acos(0) = %f\n", acos(0));
	axe.x = q.R_component_2() / sin(angle / 2);
	axe.y = q.R_component_3() / sin(angle / 2);
	axe.z = q.R_component_4() / sin(angle / 2);
}

/************************************************************************************************************/

void afficherQuat(quaternion<double> q)
{
	using namespace std;
	cout << q.R_component_1() << endl;
	cout << q.R_component_2() << endl;
	cout << q.R_component_3() << endl;
	cout << q.R_component_4() << endl;
}

/************************************************************************************************************/

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

/************************************************************************************************************/

void normalizeQuat(quaternion<double> &q, double tolerance)
{
	double norm2 = pow(q.R_component_1(), 2) + pow(q.R_component_2(), 2) + pow(q.R_component_3(), 2) + pow(q.R_component_4(), 2);
	if (norm2 > 1 + tolerance)
		q = q / sqrt(norm2);

}



/************************************************************************************************************/

vect3D changeRepere(quaternion<double> q, vect3D v)
{
	matrix<double> rotation = quatToMat(q); //obtention de la matrice de rotation 
	rotation = trans(rotation); //inversion de la matrice, revient à effectuer sa transposée dans ce cas précis (rotation autour de x, y, z)

	//_RPT3(0, "Matrice transposée = \n\t%f\t%f\t%f\n", rotation(0,0), rotation(0,1), rotation(0,2)); 
	//_RPT3(0, "\t%f\t%f\t%f\n", rotation(1,0), rotation(1,1), rotation(1,2)); 
	//_RPT3(0, "\t%f\t%f\t%f\n", rotation(2,0), rotation(2,1), rotation(2,2)); 

	v = rotation*v; // V = PV' avec P la matrice de passage
	return v;
}



/************************************************************************************************************/

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

/************************************************************************************************************/

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


/************************************************************************************************************/
