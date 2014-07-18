#ifndef DEF_TRAITEMENT
#define DEF_TRAITEMENT

#include <iostream>
#include "Structure.h"
#include "cqrlib.h"

#define nbValeurs 10

class Traitement
{
public:
	Traitement();
	~Traitement();
	void testd(void);
	void stockerValeurs(vect3D val);
	double moyenner(int axe);

	CQRQuaternionHandle* calculerOrientation(double teta_pitch, double teta_roll, double teta_yaw, double *matrice[3][3]);
	int rotate_vector(CQRQuaternionHandle, double* out, double* in);


private:
	int test;
	int compteur;
	double* valeurs[3]; //matrice à 3 lignes pour contenir les mesures selon les 3 axes

};

#endif