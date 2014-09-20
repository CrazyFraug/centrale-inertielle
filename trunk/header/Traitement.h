#ifndef DEF_TRAITEMENT
#define DEF_TRAITEMENT

#include <iostream>
#include <fstream>
#include <hash_map>
#include "Instrument.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#define _DEFINE_DEPRECATED_HASH_CLASSES 0

#define NB_VALEURS 2
#define G 9.81

class Traitement
{
public:
	/*	\brief	Constructeur
	\param	Instrument*	inst	L'instrument pour le traitement
	*/
	Traitement(Instrument* inst);

	/*	\brief	Destructeur
	*/
	~Traitement();

	/*	Getter	*/
	double get_dt(void);

	/*	\brief	Stocker les valeurs dans la tableau du Traitement
	\param	void			cas utilisant le capteur
	\param	vect4D	val		les valeurs lisent du fichier
	*/
	void stockerValeurs();
	void stockerValeurs(vect4D val);

	/*	\brief	Realise la moyenne des valeurs d'une ligne de la matrice
	\param	int		axe		numero de la ligne que l'on veut moyenner
	\return	double			la moyenne d'une ligne
	*/
	double moyennerAxe(int axe);

	/*	\brief	Renvoie la moyenne des valeurs de chaque axe sous forme de vect3D
	\param	int		nb	nombre de valeurs à moyenner. La fonction réalisera la moyenne des "nb" dernières valeurs mesurées
	\return	vect4D		vecteur contenant la moyenne
	*/
	vect4D moyenner(int nb);

	/*	\brief	Renvoie la moyenne de <nb> valeurs à partir du moment <temps>
	\param	double	temps	Le moment du premier acquisition à moyenner
	Si le <temps> est plus grand que le moment indiqué, le programme va prendre le moment suivant
	\param	int		nb		Nombre de valeurs à moyenner
	\return	vect4D			Vecteur contenant la moyenne
	*/
	vect4D moyenner(double temps, int nb);

	/*	\brief	Calcul la variation d'angle (en degré)
	la fonction utilise les attributs privés _dt et appelle la fonction Traitement::moyenner pour calculer l'angle
	\return vect3D	Angles selon 3 axe x, y, z
	*/
	vect3D calculerAngle_deg();

	/*	\brief	Affiche les moyennes de chaque axe sur la sortie et sur la console
	*/
	void afficherValeurs(void);

	/*	\brief	Indique si le tableau _valeurs est rempli
	i.e. si le nombres de mesures relevées suffit à remplir le tableau au moins une fois
	\return bool	true->oui et false->non
	*/
	bool tabFull(void);

private:
	int _test;
	int _compteur;
	double* _valeurs[3]; //matrice à 3 lignes pour contenir les mesures selon les 3 axes
	stdext::hash_map<double, vect3D> tabData;	//	Hash Map contenant les valeurs bruts, avec clé le temps d'acquisition
	Instrument* _capteur;
	double _dt, _tempsPrec, _tempsAct; /*variables comprenant le l'heure à laquelle la mesure a ete effectuee (selon l'arduino)
									   * _dt = _tempsAct - _tempsPrec ce qui correspond à la différence de temps entre les deux mesures effectuées.*/
};
#endif