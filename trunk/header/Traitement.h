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
	Traitement();
	~Traitement();

	Instrument *getInstrument(void);
	double get_dt(void);
	void setInstrument(Instrument_serie *un_Instrument);
	void testd(void);
	void stockerValeurs();
	void stockerValeurs(vect4D val);
	double moyenner(int axe);
	vect3D calculerAngle_deg();
	void afficherValeurs(void);
	void calculer_dt();
	bool tabFull(void);
	void filefromSensor(std::string filename, std::fstream &myfile, Instrument* inst);
	//void filefromSensor2(std::string filename, std::fstream myfile, Instrument* inst);
	vect4D readDatafromFile(std::string filename, std::fstream &myfile, int turns);
	void openfile_readwrite(std::fstream& myfile, std::string filename);
	void getTraitement(Traitement *un_Traitement);

	void afficherTraitement(void);
private:
	int _test;
	int _compteur;
	double* _valeurs[3]; //matrice à 3 lignes pour contenir les mesures selon les 3 axes
	Instrument* _capteur;
	stdext::hash_map<int, std::string> dataFromFile;
	double _dt, _tempsPrec, _tempsAct; /*variables comprenant le l'heure à laquelle la mesure a ete effectuee (selon l'arduino)
									   * _dt = _tempsAct - _tempsPrec ce qui correspond à la différence de temps entre les deux mesures effectuées.*/
};
void writeHeading(std::string filename, std::fstream &myfile);
int choiceMode();
#endif