#ifndef DEF_INSTRUMENT
#define DEF_INSTRUMENT

#include "Serial.h"
#include <time.h>
#include <iostream>
#include "Structure.h"

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

	/*	Constructeur
		\param	char*	nom		Nom de l'instrument
		\param	int		mode	Mode de simulation avec l'instrument (1->capteur, 2->fichier)
	*/
	Instrument(char* nom, int mode);

	/*	Destructeur	*/
	~Instrument();

	/*	Getter	*/

	/*	\brief	Return the last measure of the Instrument
		\return	vect4D	Measures of the Instrument
	*/
	vect4D getMesures(void);

	/*	\brief	Return the last measure of the Instrument relative to an axe
		\param	int		Axe is the axe you want the measure from (axe = 1 -> X axis | ... | axe = 4 -> time axis
		\return	double	Measure of the Instrument relative to axe
	*/
	double getMesure(int axe);

	/*	\brief	Return the time of the last measure
		\return	clock_t*	Pointeur au tableau de sauvegarde le temps (en ms)
	*/
	clock_t* getTemps(void);

	/*	\brief	Return the time of the last measure according to the parameter axe
		\param	int		axe		Axe qu'on veut prendre le temps
		\return	clock_t			Temps de la dernière mesure (en ms)
	*/
	clock_t getTemps(int axe);

	/*	\brief	Return le nom du fichier de données de l'Instrument
		\return	std::string		Nom du fichier
	*/
	std::string getnomfichier(void);

	/*	\brief	Return l'ID de l'Instrument
		\return	char*	ID de l'Instrument
	*/
	char* getID();


	/*	Setter	*/
	/*	\brief	Fills the _t_acq table with the new values
		\param	clock_t*	Tableau de nouvelles valeurs de temps
	*/
	void setTemps(clock_t* nvTemps);

	/*	\brief	Fills the _valeursInitiales table with the new values
		\param	vect3D		Nouvelles Valeurs Initiales
	*/
	void setVI(vect3D valeurs);

	/*	\brief	Affecter les valeurs selon les 4 axes
	*/
	void setMesuresX(double);
	void setMesuresY(double);
	void setMesuresZ(double);
	void setMesuresT(double);

	/*	\brief	Affecter les valeurs de l'Instrument
		\param	vect4D	nvMesures	Nouvelles valeurs pour l'Instrument
	*/
	void setMesures(vect4D nvMesures);


	/*	\brief	Mise a jour des mesures
				En cas pas de Serial, la fonction ne fait rien
	*/
	void majMesures();

	/*	\brief	Calibrer l'instrument en initialisant les valeurs initiales
	*/
	void calibrer(void);

	/*	Fonctions d'affichage	*/
	void afficherCapteur(void);
	void afficherMesures() const;
	void afficherTemps() const;
	void afficherVI() const;
};


#endif