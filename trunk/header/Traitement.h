#ifndef DEF_TRAITEMENT
#define DEF_TRAITEMENT

#include <iostream>
#include <fstream>
#include <hash_map>
#include "Instrument.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#define NB_VALEURS 2
#define G 9.81
#define _DEFINE_DEPRECATED_HASH_CLASSES 0
class Traitement
{
public:
	Traitement(Instrument* inst);
	Traitement(std::string nom_capteur);
	~Traitement();

	Instrument *getInstrument(void);
	double get_dt(void);
	void setInstrument(Instrument *un_Instrument);
	void testd(void);
	void stockerValeurs();
	void stockerValeurs(vect4D val);
	double moyenner(int axe);
	vect3D calculerAngle_deg();
	void afficherValeurs(void);
	void calculer_dt();
	bool tabFull(void);
	void getTraitement(Traitement *un_Traitement);

private:
	int _test;
	int _compteur;
	double* _valeurs[3]; //matrice à 3 lignes pour contenir les mesures selon les 3 axes
	Instrument* _capteur;
	double _dt, _tempsPrec, _tempsAct; /*variables comprenant le l'heure à laquelle la mesure a ete effectuee (selon l'arduino)
									   * _dt = _tempsAct - _tempsPrec ce qui correspond à la différence de temps entre les deux mesures effectuées.*/
};
#endif