#ifndef DEF_TRAITEMENT
#define DEF_TRAITEMENT

#include <iostream>
#include "Structure.h"
#include "kalman.h"
#include "source/Instrument.cpp"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#define NB_VALEURS 5

typedef boost::numeric::ublas::matrix<double> matrix_double;

class Traitement
{
public:
        Traitement(Instrument* inst);
        ~Traitement();

        void testd(void);
        void stockerValeurs();
        double moyenner(int axe);
		vect3D calculerAngle();
		void afficherValeurs(void);
		void calculer_dt();
		bool tabFull(void);

        /*quaternion*/
        //CQRQuaternionHandle* calculerOrientation(double teta_pitch, double teta_roll, double teta_yaw, double *matrice[3][3]);
        //int rotate_vector(CQRQuaternionHandle, double* out, double* in);

        /*kalman filter*/
        void initMatrices(double sigma1, double sigma2, double sigma3);
        double* initSystem(double mesures[3], double dt);

private:
        int _test;
        int _compteur;
        double* _valeurs[3]; //matrice à 3 lignes pour contenir les mesures selon les 3 axes
		double _t[3];


        matrix<double> _A, _B, _Q, _R, _C;
        //kalman filtre;
        double _sx, _sy, _sz;        
		double _mesures[3]; //tableau de mesures destiné au filtre kalman
		Instrument* _capteur;
};

#endif