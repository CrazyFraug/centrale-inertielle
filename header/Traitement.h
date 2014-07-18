#ifndef DEF_TRAITEMENT
#define DEF_TRAITEMENT

#include <iostream>
#include "Structure.h"
#include "cqrlib.h"
#include "kalman.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#define nbValeurs 10

typedef boost::numeric::ublas::matrix<double> matrix_double;

class Traitement
{
public:
	Traitement(matrix_double init_cov_estimate);
	~Traitement();

	void testd(void);
	void stockerValeurs(vect3D val);
	double moyenner(int axe);

	/*quaternion*/
	CQRQuaternionHandle* calculerOrientation(double teta_pitch, double teta_roll, double teta_yaw, double *matrice[3][3]);
	int rotate_vector(CQRQuaternionHandle, double* out, double* in);

	/*kalman filter*/
	void initMatrices(double sigma1, double sigma2, double sigma3);
	double* initSystem(double mesures[3], double dt);

private:
	int test;
	int compteur;
	double* valeurs[3]; //matrice � 3 lignes pour contenir les mesures selon les 3 axes
	matrix<double> A, B, Q, R, C;
	double* mesures[3];
	kalman filtre;
	double sx, sy, sz;

};

#endif