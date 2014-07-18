#ifndef DEF_TRAITEMENT
#define DEF_TRAITEMENT

#include "kalman.h"

class Traitement
{
public:
	Traitement(double sigma1, double sigma2, double sigma3);
	int initSystem(double* mesures[3]);

private:
	matrix_type A, B, Q, R, C;
	double* mesures[3];
	kalman filtre;
	double sx, sy, sz;

};

#endif
