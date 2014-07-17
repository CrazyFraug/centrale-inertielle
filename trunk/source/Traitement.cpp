#include "Traitement.h"


Traitement::Traitement(double sigma1, double sigma2, double sigma3): A(4,4), B(0,0,0), Q(4,4), R(4,4,0), C(4), filtre(0,4,4,100)
{
	double sx2, sy2, sz2;
	sx2 = pow(sx,2);
	sy2 = pow(sy,2);
	sz2 = pow(sz,2);

	double wx,wy,wz,dt, Ax, Ay, Az;

	Ax = 0.5*wx*dt;
	Ay = 0.5*wy*dt;
	Az = 0.5*wz*dt;

	A(0,0) = 1; A(1,1) = 1; A(2,2) = 1; A(3,3) = 1; //diagonale
	A(0,1) = -Ax; A(0,2) = -Ay; A(0,3) = -Az;
	A(1,0) = Ax; A(1,2) = Az; A(1,3) = -Ay;
	A(2,0) = Ay; A(2,1) = -Az; A(2,3) = Ax;
	A(3,0) = Az; A(3,1) = Ay; A(3,2) = -Ax;

	Q(0,0) = sx2 + sy2 + sz2; Q(1,1) = sx2 + sy2 + sz2; Q(2,2) = sx2 + sy2 + sz2; Q(3,3) = sx2 + sy2 + sz2; //diagonale
	Q(0,1) = -sx2 + sy2 - sz2; Q(0,2) = -sx2 -sy2 + sz2; Q(0,3) = sx2 - sy2 - sz2;
	Q(1,0) = -sx2 + sy2 - sz2; Q(1,2) = sx2 - sy2 - sz2; Q(1,3) = -sx2 - sy2 + sz2;
	Q(2,0) = -sx2 -sy2 + sz2; Q(2,1) = sx2 - sy2 - sz2; Q(2,3) = -sx2 + sy2 - sz2;
	Q(3,0) = sx2 - sy2 - sz2; Q(3,1) = -sx2 - sy2 + sz2; Q(3,2) = -sx2 + sy2 - sz2;

	R(0,0) = 0.05;
	R(1,1) = 0.05;
	R(2,2) = 0.05;
	R(3,3) = 0.05;
	
}
int Traitement::initSystem(double* mesures[3]) {

	filtre.declare_system(A, B, C);
	filtre.declare_noise(Q, R);

	filtre.predict_step(B);

	matrix_type mesuresQuat;
	filtre.update_step(mesuresQuat);

	return 0;
}
