#ifndef KALMAN_H
#define KALMAN_H
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "Quaternion.h"


using namespace boost::numeric::ublas;

/*
	struct représentant l'etat du systeme 
	composé de 3 matrice ainsi que des dimensions de la représentation d'état et des entrée sorties
*/
typedef struct _systemstate{
	int sizeState;
	int sizeOut;
	int sizeIn;
	matrix<double> matTransition;  //matrice de transition (état k+1 - état k)            (A/F)
	matrix<double> matCommande;   //matrice de commande (état - commade)                      (B)
	matrix<double> matSortie; //matrice de sortie  (sortie - état)                        (C/H)
} system_state;

typedef struct _kalmanstate{

	matrix<double> matIdent;
	int    nbstep;           // nombre d'échantillon d'estimation
	matrix<double> noiseCommande;  //bruit de la commande                                    (v)
	matrix<double> covCommande;        // covariance de la commande                          (Q)
	matrix<double> noiseMesure;  // bruit de la mesure                                   (w)
	matrix<double> covMesure;      //covariance de la mesure                             (R)
	matrix<double> covEstimate;    // prédiction de la covariance						  (P)
	matrix<double> predictVector; //état prédit                                          (X^)
	matrix<double> kalmanGain;     // gain de Kalman                                     (K)

} kalmanstate;


class Kalman
{
public:
	/*	\brief Constructor d'un objet Kalman
		\param	int				nbIn				--	Nombre d'entrées du système
		\param	int				nbOut				--	Nombre de sorties du système
		\param	int				nbState				--	Nombre de variables d'état du système
		\param	int				nbstep				--	Nombre d'échantillon d'initialisation du système
	*/
	Kalman(int nbIn, int nbOut, int nbState, int step);

	/* Destructeur */
	~Kalman();

	/* Getter */
	int getSizeIn();
	int getSizeOut();
	int getSizeState();
	matrix<double> getKalmanGain();
	matrix<double> getCovEstimate();

	/*	\brief	Declaration des matrices du système
		\param	matrix<double>	A	Matrice de transtion
		\param	matrix<double>	B	Matrice de commande
		\param	matrix<double>	C	Matrice de sortie
	*/
	void declare_system(matrix<double> &A, matrix<double> &B, matrix<double> &C);
	
	/*	\brief	Declaration des matrices de bruit
		\param	matrix<double>	Q	Matrice de bruit de commande
		\param	matrix<double>	R	Matrice de bruit de mesure
	*/
	void declare_noise(matrix<double> &Q, matrix<double> &R);

	/*	\brief	Initialisation du système
		\param	matrix<double>	A		Matrice de transtion
		\param	matrix<double>	B		Matrice de commande
		\param	matrix<double>	C		Matrice de sortie
		\param	matrix<double>	Q		Matrice de bruit de commande
		\param	matrix<double>	R		Matrice de bruit de mesure
		\param	matrix<double>	X_init	Vecteur d'initialisation de l'état
		\param	matrix<double>	P_init	Vecteur d'initialisation de la covariance
	*/
	void initsystem(matrix<double> &A, matrix<double> &B, matrix<double> &C, matrix<double> &Q, matrix<double> &R, matrix<double> &X_init, matrix<double> &P_init);
	
	/*	\brief	Mettre à jour les matrices (A, B, C) du système
		\param	bool			maj[3]	Tableau contenant l'indication de la mise à jour des matrices (true->oui, false->non)
		\param	matrix<double>	A		Matrice de transition (l'ancien valeur au cas où il n'y a pas de modification)
		\param	matrix<double>	B		Matrice de commande	  (							"					  		)
		\param	matrix<double>	C		Matrice de sortie	  (							"							)
	*/
	void majsystem(bool maj[3], matrix<double> &A, matrix<double> &B, matrix<double> &C);
	
	
	/** \brief Etape prédiction du filtre de Kalman à partir des données de la commande
		\param	matrix<double>	value_cmd	--	matrice des valeurs de la commande, matrice de taille : nombre d'états du système x nombre d'entrées
		\test	test_Kalman		VALIDE!
	*/
	void predictStep(matrix<double> &value_cmd);


	/** \brief Etape mettre à jour les prédictions d'état et de covariance du filtre de Kalman grâce aux valeurs mesurées
		\param	matrix<double>	value_measure			--	 matrice des valeurs mesurées, matrice de taille : nombre de sortie x nombre d'états du système
		\return	matrix<double>	_kalmsys.predictVector	--	matrice des valeurs estimées
		\test	test_Kalman		VALIDE!
	*/
	matrix<double> updateStep(matrix<double> &value_mesure);

private:
	system_state _sys;		//	système à appliquer le filtre Kalman
	kalmanstate _kalmsys;	//	Le filtre de Kalman
};
#endif // KALMAN_H