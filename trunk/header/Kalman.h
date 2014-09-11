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
	int size_state;
	int size_out;
	int size_in;
	matrix<double> mat_transition;  //matrice de transition (�tat k+1 - �tat k)            (A/F)
	matrix<double> mat_cmde;   //matrice de commande (�tat - commade)                      (B)
	matrix<double> mat_sortie; //matrice de sortie  (sortie - �tat)                        (C/H)
} system_state;

typedef struct _kalmanstate{

	matrix<double> matrix_ident;
	int    nb_step;           // nombre d'�chantillon d'estimation
	matrix<double> noise_cmde;  //bruit de la commande                                    (v)
	matrix<double> cov_cmde;        // covariance de la commande                          (Q)
	matrix<double> noise_mesure;  // bruit de la mesure                                   (w)
	matrix<double> cov_mesure;      //covariance de la mesure                             (R)
	matrix<double> cov_estimate;    // pr�diction de la covariance						  (P)
	matrix<double> predict_vector; //�tat pr�dit                                          (X^)
	matrix<double> kalman_gain;     // gain de Kalman                                     (K)

} kalmanstate;


class Kalman
{
public:
	/*	\brief Constructor d'un objet Kalman
		\param	int				nb_in				--	Nombre d'entr�es du syst�me
		\param	int				nb_out				--	Nombre de sorties du syst�me
		\param	int				nb_state			--	Nombre de variables d'�tat du syst�me
		\param	int				nb_step				--	Nombre d'�chantillon d'initialisation du syst�me
		\param	matrix<double>	init_predict		--	Vector d'initialisation du vecteur d'�tat
		\param	matrix<double>	init_cov_estimate	--	Matrice d'initialisation de la matrice de covariance
	*/
	Kalman(int nb_in, int nb_out, int nb_state, int step);

	/* Destructeur */
	~Kalman();

	/* Getter */
	int getSizeIn();
	int getSizeOut();
	int getSizeState();

	/*	\brief	Declaration des matrices du syst�me
		\param	matrix<double>	A	Matrice de transtion
		\param	matrix<double>	B	Matrice de commande
		\param	matrix<double>	C	Matrice de sortie
	*/
	void declare_system(matrix<double> A, matrix<double> B, matrix<double> C);

	/*	\brief	Declaration des matrices de bruit
		\param	matrix<double>	Q	Matrice de bruit de commande
		\param	matrix<double>	R	Matrice de bruit de mesure
	*/
	void declare_noise(matrix<double> Q, matrix<double> R);

	/*	\brief	Initialisation du syst�me
		\param	matrix<double>	A		Matrice de transtion
		\param	matrix<double>	B		Matrice de commande
		\param	matrix<double>	C		Matrice de sortie
		\param	matrix<double>	Q		Matrice de bruit de commande
		\param	matrix<double>	R		Matrice de bruit de mesure
		\param	matrix<double>	X_init	Vecteur d'initialisation de l'�tat
		\param	matrix<double>	P_init	Vecteur d'initialisation de la covariance
	*/
	void initSystem(matrix<double> A, matrix<double> B, matrix<double> C, matrix<double> Q, matrix<double> R, matrix<double> X_init, matrix<double> P_init);

	/*	\brief	Mettre � jour les matrices (A, B, C) du syst�me
		\param	bool			maj[3]	Tableau contenant l'indication de la mise � jour des matrices (true->oui, false->non)
		\param	matrix<double>	A		Matrice de transition (l'ancien valeur au cas o� il n'y a pas de modification)
		\param	matrix<double>	B		Matrice de commande	  (							"					  		)
		\param	matrix<double>	C		Matrice de sortie	  (							"							)
	*/
	void majSystem(bool maj[3], matrix<double> A, matrix<double> B, matrix<double> C);


	/** \brief Etape pr�diction du filtre de Kalman � partir des donn�es de la commande
		\param	matrix<double>	value_cmd	--	matrice des valeurs de la commande, matrice de taille : nombre d'�tats du syst�me x nombre d'entr�es
		\test	test_Kalman		VALIDE!
	*/
	void predict_step(matrix<double> value_cmd);


	/** \brief Etape mettre � jour les pr�dictions d'�tat et de covariance du filtre de Kalman gr�ce aux valeurs mesur�es
		\param	matrix<double>	value_measure			--	 matrice des valeurs mesur�es, matrice de taille : nombre de sortie x nombre d'�tats du syst�me
		\return	matrix<double>	kalm_sys.predict_vector	--	matrice des valeurs estim�es
		\test	test_Kalman		VALIDE!
	*/
	matrix<double> update_step(matrix<double> value_mesure);

private:
	system_state sys;		//	Syst�me � appliquer le filtre Kalman
	kalmanstate kalm_sys;	//	Le filtre de Kalman
};
#endif // KALMAN_H