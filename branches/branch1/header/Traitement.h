#ifndef DEF_TRAITEMENT
#define DEF_TRAITEMENT

#include <iostream>
#include <fstream>
#include <cstdio>
#include "Instrument.h"
#include "Kalman.h"
//#include <boost/numeric/ublas/matrix.hpp>
//#include <boost/numeric/ublas/io.hpp>

#define NB_VALEURS 2
#define G 9.81
#define FILENAME "simMeasures_gyro"
#define DIRECTIONS "directions.txt"

class Traitement
{
public:
	Traitement(std::string id, std::string filename=FILENAME);
	virtual ~Traitement();
	//Setter//
	void set_dt(double t);
	void set_tempsPrec(double t);
	//Getter//
	double get_dt(void);
	double get_tempsPrec(void);
	int get_compteur(void);
	static bool get_finFichier(void);
	//Gestion des valeurs//
	virtual vect4D acquisition(void);
	void stockerValeurs(void);
	double moyennerAxe(int axe);
	vect3D moyenner(int nb);
	vect3D renvoyerVal(int nb);
	void renvoyerQuat(int nb, quaternion<double> &q_base, vect3D &axe, double &angle);
	void calculer_dt();
	bool tabFull(void);
	vect4D lastVal(void);
	vect4D returnData(void);
	//Affichage//
	void afficherValeurs(void);
	//Gestion du fichier de valeur//
	void filefromSensor(std::string filename, Instrument* inst);
	vect4D readDatafromFile();
	void openfile_readwrite(std::fstream& myfile, std::string filename);
	void resetCompteur(void);
	static void resetCursor(void);

protected:
	std::string _filename, _id;
	static int _cursor; //cursor qui détermine la ligne du fichier à lire
	int _compteur; //compte le nombre de valeurs stockés. _compteur max = nombre de colonnes max de la matrice _valeurs = constante NB_MAX
	double* _valeurs[3]; //matrice à 3 lignes pour contenir les mesures selon les 3 axes
	double _dt, _tempsPrec, _tempsAct; /*variables comprenant l'heure à laquelle la mesure à été effectuée (selon l'arduino)
									   * _dt = _tempsAct - _tempsPrec ce qui correspond à la différence de temps entre les deux mesures effectuées.*/
	static bool _finFichier;
};


class Traitement_serie : public Traitement
{
private:
	Instrument_serie* _capteur;

public:
	Traitement_serie(std::string id, std::string filename, Serial* link);
	~Traitement_serie();
	//Getter//
	Instrument_serie *getInstrument(void);
	//Affichage//
	void afficherTraitSerie(void);
	vect4D acquisition(void);
	void stockerValeurs();

};

// Fonctions globales
void writeHeading(std::string filename);
int choiceMode();
quaternion<double> orienterVect(quaternion<double> q);

#endif