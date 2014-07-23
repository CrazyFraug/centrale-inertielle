#include "Traitement.h"


/**Constructeur**/
Traitement::Traitement(Instrument* inst):_compteur(0), _dt(0)
{
		_capteur = inst;
        /** Allocation mémoire de la matrice de valeurs **/
        for (int i = 0; i<3; i++)
        {
                _valeurs[i] = new double[NB_VALEURS];
        }
}


/** Destructeur **/
Traitement::~Traitement()
{
        for (int i = 0; i<3; i++)
        {
                delete _valeurs[i];
        }
}


/** 
 *      \brief stocke des valeurs passées en argument dans la matrice de valeur attribut
 *      la variable compteur permet de savoir si la matrice est remplie ou pas
 *      si la matrice est deja remplie, elle se contente de remplacer la premiere valeur de chaque ligne avec les valeurs en argument
 *      \param val vect3D défini dans structure.h contenant les mesures a stocker
*/
void Traitement::stockerValeurs() 
{
		_capteur->majSerial();
		_tempsAct = _capteur->getMesure(4); /* 4 correspond à l'axe temporel (mesures.temps) */
		_dt = (_tempsAct - _tempsPrec)/1000.0;
		_tempsPrec = _tempsAct;

        if (_compteur < NB_VALEURS)
        {
                _valeurs[0][_compteur] = _capteur->getMesure(1);
                _valeurs[1][_compteur] = _capteur->getMesure(2);
                _valeurs[2][_compteur] = _capteur->getMesure(3);
                _compteur++;
        }

        else
        {
                for(int i = NB_VALEURS-1; i>0; i--)
                {
                        _valeurs[0][i] = _valeurs[0][i-1];
                        _valeurs[1][i] = _valeurs[1][i-1];
                        _valeurs[2][i] = _valeurs[2][i-1];
                }

                _valeurs[0][0] = _capteur->getMesure(1);
                _valeurs[1][0] = _capteur->getMesure(2);
                _valeurs[2][0] = _capteur->getMesure(3);

				_t[0] = _capteur->getTemps(1);
				_t[1] = _capteur->getTemps(2);
				_t[2] = _capteur->getTemps(3);
        }
}

/**
 *  \brief realise la moyenne des valeurs d'une ligne de la matrice
 *  \param axe numero de la ligne que l'on veut moyenner
 *  \return la moyenne d'une ligne
*/
double Traitement::moyenner(int axe)
{
        double moyenne = 0;
        for (int i =0; i < _compteur; i++)
        {
                moyenne += _valeurs[axe-1][i];
        }

        return (moyenne/NB_VALEURS);
}

vect3D Traitement::calculerAngle()
{
	vect3D angles;
	angles.x = moyenner(1)*_dt;
	angles.y = moyenner(2)*_dt;
	angles.z = moyenner(3)*_dt;
	std::cout << " _dt = " << _dt << " ms " << std::endl;
	return angles;
}


void Traitement::afficherValeurs()
{
	if (_compteur == NB_VALEURS) {
		std::cout << "moyenne X = " << moyenner(1) << "                      " << std::endl;
		std::cout << "moyenne Y = " << moyenner(2) << "                      " << std::endl;
		std::cout << "moyenne Z = " << moyenner(3) << "                      " << std::endl;
	}
}

bool Traitement::tabFull()
{
	if (_compteur == NB_VALEURS)
		return true;
	else
		return false;
}


/**
\brief fonction inutile
*/
void Traitement::testd (void){
        std::cout << "test" << std::endl;
}