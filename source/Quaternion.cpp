#include "Quaternion.h"

Quaternion::Quaternion()
{
	m_q0=0;
	m_q1=0;
	m_q2=0;
	m_q3=0;
}

Quaternion::Quaternion(double q0, double q1, double q2, double q3)
{
	m_q0=q0;
	m_q1=0;
	m_q2=0;
	m_q3=0;
}

Quaternion::~Quaternion()
{
}

double* Quaternion::get_q()
{
	double q[4] = {m_q0,m_q1,m_q2,m_q3};
	return q;
}

double* eulerToQuat(double phi, double teta, double psi)
{
	Quaternion Q1(cos(phi/2),sin(phi/2),0,0), Q2(cos(teta/2),0,sin(teta/2),0), Q3(cos(psi/2),0,0,sin(psi/2));
	Q1 = quatMultiply(Q1,Q2);
	Q1 = quatMultiply(Q1,Q3);
	return Q1.get_q();
}