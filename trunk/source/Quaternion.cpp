#include "Quaternion.h"
#include <math.h>
quaternion<double> anglesToQuat(double phi, double teta, double rho)
{

	double c1, c2, c3;
	double s1, s2, s3;
	double q1, q2, q3, q4;
	double norm;
	c1 = cos(phi / 2);
	c2 = cos(teta / 2);
	c3 = cos(rho / 2);
	s1 = sin(phi / 2);
	s2 = sin(teta / 2);
	s3 = sin(rho / 2);
	q1 = c1*s2*c3 - s1*c2*s3;
	q2 = c1*c2*c3 + s1*s2*s3;
	q3 = c1*c2*s3 - s1*s2*c3;
	q4 = s1*c2*c3 + c1*s2*s3;
	norm = sqrt(pow(q1, 2) + pow(q2, 2) + pow(q3, 2) + pow(q4, 2));
	quaternion<double> quat_resultat(q1 / norm, q2 / norm, q3 / norm, q4 / norm);
	/*quaternion<double> q1(cos(phi / 2), sin(phi / 2), 0, 0);
	quaternion<double> q2(cos(teta / 2), 0, sin(teta / 2), 0);
	quaternion<double> q3(cos(rho / 2), 0, 0, sin(rho / 2));*/

	return quat_resultat;
}

/*heading = atan2(2*qy*qw+2*qx*qz , 1 - 2*qy2 - 2*qz2)
attitude = asin(2*qx*qy - 2*qz*qw)
bank = atan2(2*qx*qw+2*qy*qz , 1 - 2*qx2 - 2*qy2)

except when qx*qy + qz*qw = 0.5 (north pole)
which gives:
heading = 2 * atan2(x,w)
bank = 0
and when qx*qy + qz*qw = -0.5 (south pole)
which gives:
heading = -2 * atan2(x,w)
bank = 0*/
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
	double unit = qx2 + qy2 + qz2 + qw2;
	double test = qx* qy + qz* qw;

	if (test > 0.4999 * unit)                              // 0.4999f OR 0.5f - EPSILON
	{
		// Singularity at north pole
		angles_result.y = 2 * (float)atan2(qx, qw) * 180 / (atan(1) * 4);  // Yaw
		angles_result.x = 90;                         // Pitch
		angles_result.z = 0;                                // Roll
	}
	else if (test < -0.4999 * unit)                        // -0.4999f OR -0.5f + EPSILON
	{
		// Singularity at south pole
		angles_result.y = -2 * (float)atan2(qx, qw) * 180 / (atan(1) * 4); // Yaw
		angles_result.x = -90;                        // Pitch
		angles_result.z = 0;                                // Roll
	}
	else{
		angles_result.y = (float)atan2(2 * qx * qw + 2 * qy * qz, 1 - 2 * (qz2 + qw2)) * 180 / (atan(1) * 4);     // Yaw 
		angles_result.x = (float)asin(2 * (qx * qz - qw * qy)) * 180 / (atan(1) * 4);                             // Pitch 
		angles_result.z = (float)atan2(2 * qx * qy + 2 * qz * qw, 1 - 2 * (qy2 + qz2)) * 180 / (atan(1) * 4);      // Roll 
	}
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
* \brief calcul la norme du quaternion q
* permet de mettre le quaternion sous forme unitaire
* \return norme du quaternion
*/
double calculateNorm(quaternion<double> q)
{
	double norm = sqrt(pow(q.R_component_1(), 2) + pow(q.R_component_2(), 2) + pow(q.R_component_3(), 2) + pow(q.R_component_4(), 2));
	return norm;
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