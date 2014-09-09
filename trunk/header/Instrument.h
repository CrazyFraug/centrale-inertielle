#ifndef DEF_INSTRUMENT
#define DEF_INSTRUMENT

#include "Serial.h"
#include <time.h>
#include <iostream>
#include "Structure.h"

vect3D operator+(vect3D v1, vect3D v2);

vect3D operator*(vect3D v2, double* v1);
vect3D operator*(vect3D v2, double v1);
vect4D operator+(vect4D v1, vect3D v2);

class Instrument {

private:
	char* ID;
	Serial *_serialLink;
	vect4D _mesures;
	vect3D _valeursInitiales;
	clock_t _t_acq[3];
	std::string fileName;

	//retire les valeurs initiales des mesures
	void soustraireVI(void);

public:
	//constructor
	Instrument(char* nom, int mode);

	//destructor//
	~Instrument();

	//getter//
	vect4D getMesures(void);
	double getMesure(int axe);
	clock_t* getTemps(void);
	std::string getnomfichier(void);
	char* getID();

	/** \brief return the time of the last measure according to the parameter axe
	*/
	clock_t getTemps(int axe);

	/**Setter**/
	void setTemps(clock_t* nvTemps);
	void setVI(vect3D valeurs);
	void setMesuresX(double);
	void setMesuresY(double);
	void setMesuresZ(double);
	void setMesuresT(double);
	void setMesures(vect4D nvMesures);

	/**Mise a jour des mesures stock�es */
	void majMesures();

	/**calibrer l'instrument en initialisant les valeurs initiales*/
	void calibrer(void);
	void majSerial(/*char* IDSensor*/);
	/*Fonctions d'affichage*/
	void afficherCapteur(void);
	void afficherMesures() const;
	void afficherTemps() const;
	void afficherVI() const;
};


#endif