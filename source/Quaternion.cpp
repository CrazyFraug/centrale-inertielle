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