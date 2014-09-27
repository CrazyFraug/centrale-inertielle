#ifndef KALMAN_H
#define KALMAN_H
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "Quaternion.h"


using namespace boost::numeric::ublas;

/*
	struct repr�sentant l'etat du systeme 
	compos� de 3 matrice ainsi que des dimensions de la repr�sentation d'�tat et des entr�e sorties
*/
typedef struct _systemstate{
	int sizeState;
	int sizeOut;
	int sizeIn;
	matrix<double> matTransition;  //matrice de transition (�tat k+1 - �tat k)            (A/F)
	matrix<double> matCommande;   //matrice de commande (�tat - commade)                      (B)
	matrix<double> matSortie; //matrice de sortie  (sortie - �tat)                        (C/H)
} system_state;

typedef struct _kalmanstate{

	matrix<double> matIdent;
	int    nbstep;           // nombre d'�chantillon d'estimation
	matrix<double> noiseCommande;  //bruit de la commande                                    (v)
	matrix<double> covCommande;        // covariance de la commande                          (Q)
	matrix<double> noiseMesure;  // bruit de la mesure                                   (w)
	matrix<double> covMesure;      //covariance de la mesure                             (R)
	matrix<double> covEstimate;    // pr�diction de la covariance						  (P)
	matrix<double> predictVector; //�tat pr�dit                                          (X^)
	matrix<double> kalmanGain;     // gain de Kalman                                     (K)

} kalmanstate;


class Kalman
{
public:
	/*	\brief Constructor d'un objet Kalman
		\param	int				nbIn				--	Nombre d'entr�es du syst�me
		\param	int				nbOut				--	Nombre de sorties du syst�me
		\param	int				nbState				--	Nombre de variables d'�tat du syst�me
		\param	int				nbstep				--	Nombre d'�chantillon d'initialisation du syst�me
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

	/*	\brief	Declaration des matrices du syst�me
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

	/*	\brief	Initialisation du syst�me
		\param	matrix<double>	A		Matrice de transtion
		\param	matrix<double>	B		Matrice de commande
		\param	matrix<double>	C		Matrice de sortie
		\param	matrix<double>	Q		Matrice de bruit de commande
		\param	matrix<double>	R		Matrice de bruit de mesure
		\param	matrix<double>	X_init	Vecteur d'initialisation de l'�tat
		\param	matrix<double>	P_init	Vecteur d'initialisation de la covariance
	*/
	void initsystem(matrix<double> &A, matrix<double> &B, matrix<double> &C, matrix<double> &Q, matrix<double> &R, matrix<double> &X_init, matrix<double> &P_init);
	
	/*	\brief	Mettre � jour les matrices (A, B, C) du syst�me
		\param	bool			maj[3]	Tableau contenant l'indication de la mise � jour des matrices (true->oui, false->non)
		\param	matrix<double>	A		Matrice de transition (l'ancien valeur au cas o� il n'y a pas de modification)
		\param	matrix<double>	B		Matrice de commande	  (							"					  		)
		\param	matrix<double>	C		Matrice de sortie	  (							"							)
	*/
	void majsystem(bool maj[3], matrix<double> &A, matrix<double> &B, matrix<double> &C);
	
	
	/** \brief Etape pr�diction du filtre de Kalman � partir des donn�es de la commande
		\param	matrix<double>	value_cmd	--	matrice des valeurs de la commande, matrice de taille : nombre d'�tats du syst�me x nombre d'entr�es
		\test	test_Kalman		VALIDE!
	*/
	void predictStep(matrix<double> &value_cmd);


	/** \brief Etape mettre � jour les pr�dictions d'�tat et de covariance du filtre de Kalman gr�ce aux valeurs mesur�es
		\param	matrix<double>	value_measure			--	 matrice des valeurs mesur�es, matrice de taille : nombre de sortie x nombre d'�tats du syst�me
		\return	matrix<double>	_kalmsys.predictVector	--	matrice des valeurs estim�es
		\test	test_Kalman		VALIDE!
	*/
	matrix<double> updateStep(matrix<double> &value_mesure);

private:
	system_state _sys;		//	syst�me � appliquer le filtre Kalman
	kalmanstate _kalmsys;	//	Le filtre de Kalman
};
#endif // KALMAN_H