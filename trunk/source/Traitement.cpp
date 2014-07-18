#include "Traitement.h"


/**Constructeur**/
Traitement::Traitement():compteur(0)
{
	/** Allocation mémoire de la matrice de valeurs **/
	for (int i = 0; i<3; i++)
	{
		valeurs[i] = new double[nbValeurs];
	}

}


/** Destructeur **/
Traitement::Traitement()
{
	for (int i = 0; i<3; i++)
	{
		delete valeurs[i];
	}
}

/** 
\brief stocke des valeurs passées en argument dans la matrice de valeur attribut
 la variable compteur permet de savoir si la matrice est remplie ou pas
 si la matrice est deja remplie, elle se contente de remplacer la premiere valeur de chaque ligne avec les valeurs en argument
\param val vect3D défini dans structure.h contenant les mesures a stocker
*/
void Traitement::stockerValeurs(vect3D val) 
{
	if (compteur < nbValeurs)
	{
		valeurs[0][compteur] = val.x;
		valeurs[1][compteur] = val.y;
		valeurs[2][compteur] = val.z;
		compteur++;
	}

	else
	{
		for(int i = nbValeurs-1; i>0; i--)
		{
			valeurs[0][i] = valeurs[0][i-1];
			valeurs[1][i] = valeurs[1][i-1];
			valeurs[2][i] = valeurs[2][i-1];
		}

		valeurs[0][0] = val.x;
		valeurs[1][0] = val.y;
		valeurs[2][0] = val.z;
	}
}

/**
\brief realise la moyenne des valeurs d'une ligne de la matrice
\param axe numero de la ligne que l'on veut moyenner
\return la moyenne d'une ligne
*/
double Traitement::moyenner(int axe)
{
	double moyenne;
	for (int i =0; i < nbValeurs; i++)
	{
		moyenne += valeurs[axe][i];
	}

	return (moyenne/nbValeurs);
}


/**
\brief fonction inutile
*/
void Traitement::testd (void){
	std::cout << "test" << std::endl;
}
