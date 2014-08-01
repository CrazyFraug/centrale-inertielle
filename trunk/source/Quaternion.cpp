#include "Quaternion.h"
#include <math.h>
quaternion<double> anglesToQuat(double phi, double teta, double rho)
{
	quaternion<double> q1(cos(phi/2),sin(phi/2),0,0);
	quaternion<double> q2(cos(teta/2),0,sin(teta/2),0);
	quaternion<double> q3(cos(rho/2),0,0,sin(rho/2));

	q1 = q3*q2*q1;
	return q1;
}

/*heading = atan2(2*qy*qw-2*qx*qz , 1 - 2*qy2 - 2*qz2)
attitude = asin(2*qx*qy + 2*qz*qw) 
bank = atan2(2*qx*qw-2*qy*qz , 1 - 2*qx2 - 2*qz2)

except when qx*qy + qz*qw = 0.5 (north pole)
which gives:
heading = 2 * atan2(x,w)
bank = 0
and when qx*qy + qz*qw = -0.5 (south pole)
which gives:
heading = -2 * atan2(x,w)
bank = 0*/
vect3D quatToAngles(quaternion<double> a_quaternion)
{
	vect3D angles_result;
	double qw, qx, qy, qz;
	double qw2, qx2, qy2, qz2;
	qw = a_quaternion.R_component_1();
	qx = a_quaternion.R_component_2();
	qy = a_quaternion.R_component_3();
	qz = a_quaternion.R_component_4();
	qw2 = pow(qw, 2);
	qx2 = pow(qx, 2);
	qy2 = pow(qy, 2);
	qz2 = pow(qz, 2);
	if (qx*qy + qz*qw == 0.5){
		angles_result.x = 2 * atan2(qx, qw);
		angles_result.y = 0;
		angles_result.z = atan(1)*4 / 2;
	}
	else if (qx*qy+ qz*qw== -0.5){
		angles_result.x = - 2 * atan2(qx, qw);
		angles_result.y = 0;
		angles_result.z = -atan(1) * 4 / 2;
	}
	else{
		angles_result.x = atan2(2 * qy*qw - 2 * qx*qz, 1 - 2 * qy2 - 2 * qz2);
		angles_result.y = asin(2 * qx*qy + 2 * qz*qw);
		angles_result.z = atan2(2 * qx*qw - 2 * qy*qz, 1 - 2 * qx2 - 2 * qz2);
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
	quaternion<double> qb(q.R_component_1(), -q.R_component_2(), - q.R_component_3(), -q.R_component_4());
	quaternion<double> vquat(0, v.x, v.y, v.z);
	quaternion<double> result = hamiltonProduct(hamiltonProduct(q,vquat),qb);
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
	double norm = sqrt(pow(q.R_component_1(),2) + pow (q.R_component_2(),2) + pow(q.R_component_3(),2) + pow(q.R_component_4(),2)); 
	return norm;
}

/**
* \brief produit hamiltonien de 2 quaternions
*/
quaternion<double> hamiltonProduct(quaternion<double> q1, quaternion<double> q2)
{
	double a,b,c,d;
	a = (q1.R_component_1()*q2.R_component_1() - q1.R_component_2()*q2.R_component_2() - q1.R_component_3()*q2.R_component_3() - q1.R_component_4()*q2.R_component_4());
	b = (q1.R_component_1()*q2.R_component_2() + q1.R_component_2()*q2.R_component_1() + q1.R_component_3()*q2.R_component_4() - q1.R_component_4()*q2.R_component_3());
	c = (q1.R_component_1()*q2.R_component_3() - q1.R_component_2()*q2.R_component_4() + q1.R_component_3()*q2.R_component_1() + q1.R_component_4()*q2.R_component_2());
	d = (q1.R_component_1()*q2.R_component_4() + q1.R_component_2()*q2.R_component_3() - q1.R_component_3()*q2.R_component_2() + q1.R_component_4()*q2.R_component_1());
	quaternion<double> result(a,b,c,d);	
	
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