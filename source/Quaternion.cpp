#include "Quaternion.h"

using namespace boost::numeric::ublas;

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
	m_q1=q1;
	m_q2=q2;
	m_q3=q3;
}

Quaternion::~Quaternion()
{
}

double* Quaternion::get_q()
{
	double* q;
	q = new double[4];
	q[0] = m_q0;
	q[1] = m_q1;
	q[2] = m_q2;
	q[3] = m_q3;
	return q;
}

double* eulerToQuat(double phi, double teta, double psi)
{
	Quaternion Q1(cos(phi/2),sin(phi/2),0,0), Q2(cos(teta/2),0,sin(teta/2),0), Q3(cos(psi/2),0,0,sin(psi/2));
	Q1 = quatMultiply(Q1,Q2);
	Q1 = quatMultiply(Q1,Q3);
	return Q1.get_q();
}

matrix<double> Quaternion::matricePassage() 
{
	matrix<double> p(3,3);
	p(0,0) = 2*(pow(m_q0,2) + pow(m_q1,2))-1;	p(0,1) = 2*(m_q1*m_q2-m_q0*m_q3);			p(0,2) = 2*(m_q1*m_q3 + m_q0*m_q2);
	p(1,0) = 2*(m_q1*m_q2 + m_q0*m_q3);			p(1,1) = 2*(pow(m_q0,2) + pow(m_q2,2))-1	;	p(1,2) = 2*(m_q2*m_q3-m_q0*m_q1);
	p(2,0) = 2*(m_q1*m_q3 - m_q0*m_q2);			//p(2,1) = 2*(m_q2*m_q3 + m_q0*m_q1);			
	p(2,2) = 2*(pow(m_q0,2)+pow(m_q3,2))-1;
	p(2,1) = 12;
	return p;
}

Quaternion quatMultiply(Quaternion Q1, Quaternion Q2)
{
	return Q1;
}