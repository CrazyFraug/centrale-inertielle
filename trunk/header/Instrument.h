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
	vect4D _mesures;
	vect3D _valeursInitiales;
	clock_t _t_acq[3];

	//retire les valeurs initiales des mesures
	void soustraireVI(void);

public:
	//constructor
	Instrument(char* nom);
	Instrument();
	//destructor//
	~Instrument();

	//getter//
	vect4D getMesures(void);
	double getMesure(int axe);
	clock_t* getTemps(void);

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

	/**Mise a jour des mesures stockées */
	virtual void majMesures();
	virtual std::string getnomfichier(void);
	/**calibrer l'instrument en initialisant les valeurs initiales*/
	void calibrer(void);

	/*Fonctions d'affichage*/
	void afficherCapteur(void);
	void afficherMesures() const;
	void afficherTemps() const;
	void afficherVI() const;
};


class Instrument_serie : public Instrument
{
private:
	Serial* _serialLink;
	std::string nom_fichier;
public:
	Instrument_serie(char* nom, Serial* link);
	~Instrument_serie();
	std::string getnomfichier(void);
	/** \brief mise a jour des valeurs de l'instrument avec les données envoyées via le port série	*/
	void majMesures(void);
	void majSerial(/*char* IDSensor*/);
};

#endif