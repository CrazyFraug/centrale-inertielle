#include "Traitement.h"
#include "Tools.h"
#include <iostream>
#include <hash_map>
#define _DEFINE_DEPRECATED_HASH_CLASSES 0
/** Constructeur **/
Traitement::Traitement(Instrument* inst) :_compteur(0), _dt(0)
{
	_capteur = inst;
	/** Allocation mémoire de la matrice de valeurs **/
	for (int i = 0; i < 3; i++)
	{
		_valeurs[i] = new double[NB_VALEURS];
	}
}
Traitement::Traitement(std::string nom_capteur) :_compteur(0), _dt(0)
{
	/** Allocation mémoire de la matrice de valeurs **/
	for (int i = 0; i < 3; i++)
	{
		_valeurs[i] = new double[NB_VALEURS];
	}
}


/** Destructeur **/
Traitement::~Traitement()
{
	for (int i = 0; i < 3; i++)
	{
		delete _valeurs[i];
	}
}

Instrument* Traitement::getInstrument(void){
	return (_capteur);
}

double	Traitement::get_dt(){
	return _dt;
}

void Traitement::setInstrument(Instrument *un_Instrument){
	_capteur = un_Instrument;
}

void Traitement::stockerValeurs(vect4D val)
{
	_tempsAct = val.temps;
	_dt = (_tempsAct - _tempsPrec) / 1000.0;
	_tempsPrec = _tempsAct;

#ifdef TEST
	_RPT1(0, "nouvelles valeurs à stocker : \r\nval.x = %f\n", val.x);
	_RPT1(0, "val.y = %f\n", val.y);
	_RPT1(0, "val.x = %f\n", val.z);
#endif

	if (_compteur < NB_VALEURS) // cas ou le tableau n'est pas plein
	{
		_valeurs[0][_compteur] = val.x;
		_valeurs[1][_compteur] = val.y;
		_valeurs[2][_compteur] = val.z;
		_compteur++;
	}

	else // cas ou le tableau est deja rempli
	{
		//_RPT0(0, "tableau rempli !\n");
		for (int i = 0; i < NB_VALEURS - 1; i++)
		{
			_valeurs[0][i] = _valeurs[0][i + 1];
			_valeurs[1][i] = _valeurs[1][i + 1];
			_valeurs[2][i] = _valeurs[2][i + 1];
		}

		if (_compteur < 0)
			_compteur = 0;
		_valeurs[0][_compteur - 1] = val.x;
		_valeurs[1][_compteur - 1] = val.y;
		_valeurs[2][_compteur - 1] = val.z;

	}
	//_RPT1(0, "compteur = %d\n", _compteur);
}

/**
*      \brief stocke des valeurs passées en argument dans la matrice de valeur attribut
*      la variable compteur permet de savoir si la matrice est remplie ou pas
*      si la matrice est deja remplie, elle se contente de remplacer la premiere valeur de chaque ligne avec les valeurs en argument
*      \param val vect3D défini dans structure.h contenant les mesures a stocker
*/
void Traitement::stockerValeurs()
{
	_capteur->majMesures(/*_capteur->getID()*/);
	_tempsAct = _capteur->getMesure(4); /* 4 correspond à l'axe temporel (mesures.temps) */
	_dt = (_tempsAct - _tempsPrec) / 1000.0;
	_tempsPrec = _tempsAct;

	if (_compteur < NB_VALEURS) // cas ou le tableau n'est pas plein
	{
		_valeurs[0][_compteur] = _capteur->getMesure(1);
		_valeurs[1][_compteur] = _capteur->getMesure(2);
		_valeurs[2][_compteur] = _capteur->getMesure(3);
		_compteur++;
	}

	else // cas ou le tableau est deja rempli
	{
		//_RPT0(0, "tableau rempli !\n");
		for (int i = 0; i < NB_VALEURS - 1; i++)
		{
			_valeurs[0][i] = _valeurs[0][i + 1];
			_valeurs[1][i] = _valeurs[1][i + 1];
			_valeurs[2][i] = _valeurs[2][i + 1];
		}

		_valeurs[0][_compteur - 1] = _capteur->getMesure(1);
		_valeurs[1][_compteur - 1] = _capteur->getMesure(2);
		_valeurs[2][_compteur - 1] = _capteur->getMesure(3);

	}
	//_RPT1(0, "compteur = %d\n", _compteur);
}

/**
*  \brief realise la moyenne des valeurs d'une ligne de la matrice
*  \param axe numero de la ligne que l'on veut moyenner
*  \return la moyenne d'une ligne
*/
double Traitement::moyenner(int axe)
{
	double moyenne = 0;
	for (int i = 0; i < _compteur; i++)
	{
		moyenne += _valeurs[axe - 1][i];
	}

	return (moyenne / NB_VALEURS);
}


/**
* \brief calcul la variation d'angle (en degré)
* la fonction utilise les attributs privés _dt et appelle la fonction Traitement::moyenner pour calculer l'angle
* \return angles d'euler sous forme d'un vect3D
*/
vect3D Traitement::calculerAngle_deg()
{
	vect3D angles;
	angles.x = moyenner(1)*_dt * 180 / (atan(1) * 4);
	angles.y = moyenner(2)*_dt * 180 / (atan(1) * 4);
	angles.z = moyenner(3)*_dt * 180 / (atan(1) * 4);
	_RPT1(0, " _dt = %f ms\n", _dt);
	std::cout << " _dt = " << _dt << " ms " << std::endl;
	return angles;
}

/** A CHANGER
* \brief Soustraction de l'influence de l'accélération g sur chacun des axes de l'accéléromètre
*/
void substractG(double matrice[3][3], double* accel_x, double* accel_y, double* accel_z)
{
	(*accel_x) = G*matrice[0][2];
	(*accel_y) = G*matrice[1][2];
	(*accel_z) = G*matrice[2][2];
}

/**
* \brief affiche les moyennes de chaque axe sur la sortie et sur la console
*/
void Traitement::afficherValeurs()
{
	_RPT1(0, "CAPTEUR %s : \n", _capteur->getID());
	std::cout << "CAPTEUR " << _capteur->getID() << ": " << std::endl;
	if (_compteur == NB_VALEURS) {
		_RPT1(0, "moyenne X = %f\n", moyenner(1));
		_RPT1(0, "moyenne Y = %f\n", moyenner(2));
		_RPT1(0, "moyenne Z = %f\n", moyenner(3));

		std::cout << "moyenne X = " << moyenner(1) << "                      " << std::endl;
		std::cout << "moyenne Y = " << moyenner(2) << "                      " << std::endl;
		std::cout << "moyenne Z = " << moyenner(3) << "                      " << std::endl;
	}
}


/**
* \brief indique si le tableau _valeurs est rempli
* i.e. si le nombres de mesures relevées suffit à remplir le tableau au moins une fois
* \return boolean
*/
bool Traitement::tabFull()
{
	if (_compteur == NB_VALEURS)
		return true;
	else
		return false;
}


