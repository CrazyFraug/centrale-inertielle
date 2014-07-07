#ifndef DEF_QUATERNION
#define DEF_QUATERNION

#include "Mobile.h";


struct struct_eulerAngle {
	double phi,teta,psi;
};

class Quaternion {

public:
	Quaternion();
	Quaternion(double, double, double, double);
	~Quaternion();
	double* get_q();
	double* projection();

private:
	double m_q0, m_q1, m_q2, m_q3;
};

double* eulerToQuat(double phi, double teta, double psi);
Quaternion quatMultiply(Quaternion Q1, Quaternion Q2);


#endif