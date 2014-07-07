#ifndef DEF_QUATERNION
#define DEF_QUATERNION

struct struct_eulerAngle {
	double phi,teta,psi;
}

class Quaternion
{
public:
	Quaternion();
	~Quaternion();


private:
	double q0, q1, q2, q3;
};

#endif