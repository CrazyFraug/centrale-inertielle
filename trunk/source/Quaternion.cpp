#include "Quaternion.h"

struct struct_eulerAngle {
	double teta;

}
class Quaternion
{
public:
	Quaternion();
	~Quaternion();


private:
	double q0, q1, q2, q3;
};

Quaternion::Quaternion()
{
}

Quaternion::~Quaternion()
{
}
