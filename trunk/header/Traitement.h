#ifndef DEF_TRAITEMENT
#define DEF_TRAITEMENT

#include <iostream>
#include "Instrument.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#define NB_VALEURS 2

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

private:
        int _test;
        int _compteur;
        double* _valeurs[3]; //matrice à 3 lignes pour contenir les mesures selon les 3 axes
		double _t[3];
		Instrument* _capteur;
		double _dt, _tempsPrec, _tempsAct; /*variables comprenant le l'heure à laquelle la mesure a ete effectuee (selon l'arduino)
		* _dt = _tempsAct - _tempsPrec ce qui correspond à la différence de temps entre les deux mesures effectuées.
		*/
};

#endif