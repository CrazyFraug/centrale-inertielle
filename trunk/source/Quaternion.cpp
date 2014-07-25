#include "Quaternion.h"

quaternion<double> anglesToQuat(double phi, double teta, double rho)
{
	quaternion<double> q1(cos(phi/2),sin(phi/2),0,0);
	quaternion<double> q2(cos(teta/2),0,sin(teta/2),0);
	quaternion<double> q3(cos(rho/2),0,0,sin(rho/2));

	q1 = q3*q2*q1;
	return q1;
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
