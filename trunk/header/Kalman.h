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
	int size_state;
	int size_out;
	int size_in;
	matrix<double> mat_transition;  //matrice de transition (état k+1 - état k)            (A/F)
	matrix<double> mat_cmde;   //matrice de commande (état - commade)                      (B)
	matrix<double> mat_sortie; //matrice de sortie  (sortie - état)                        (C/H)
} system_state;

typedef struct _kalmanstate{

	matrix<double> matrix_ident;
	int    nb_step;           // nombre d'échantillon d'estimation
	matrix<double> noise_cmde;  //bruit de la commande                                    (v)
	matrix<double> cov_cmde;        // covariance de la commande                          (Q)
	matrix<double> noise_mesure;  // bruit de la mesure                                   (w)
	matrix<double> cov_mesure;      //covariance de la mesure                             (R)
	matrix<double> cov_estimate;    // prédiction de la covariance						  (P)
	matrix<double> predict_vector; //état prédit                                          (X^)
	matrix<double> kalman_gain;     // gain de Kalman                                     (K)

} kalmanstate;


class Kalman
{
public:
	/*	\brief Constructor d'un objet Kalman
		\param	int				nb_in				--	Nombre d'entrées du système
		\param	int				nb_out				--	Nombre de sorties du système
		\param	int				nb_state			--	Nombre de variables d'état du système
		\param	int				nb_step				--	Nombre d'échantillon d'initialisation du système
		\param	matrix<double>	init_predict		--	Vector d'initialisation du vecteur d'état
		\param	matrix<double>	init_cov_estimate	--	Matrice d'initialisation de la matrice de covariance
	*/
	Kalman(int nb_in, int nb_out, int nb_state, int step);

	/* Destructeur */
	~Kalman();

	/* Getter */
	int getSizeIn();
	int getSizeOut();
	int getSizeState();

	/*	\brief	Declaration des matrices du système
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

	/*	\brief	Initialisation du système
		\param	matrix<double>	A		Matrice de transtion
		\param	matrix<double>	B		Matrice de commande
		\param	matrix<double>	C		Matrice de sortie
		\param	matrix<double>	Q		Matrice de bruit de commande
		\param	matrix<double>	R		Matrice de bruit de mesure
		\param	matrix<double>	X_init	Vecteur d'initialisation de l'état
		\param	matrix<double>	P_init	Vecteur d'initialisation de la covariance
	*/
	void initSystem(matrix<double> A, matrix<double> B, matrix<double> C, matrix<double> Q, matrix<double> R, matrix<double> X_init, matrix<double> P_init);

	/*	\brief	Mettre à jour les matrices (A, B, C) du système
		\param	bool			maj[3]	Tableau contenant l'indication de la mise à jour des matrices (true->oui, false->non)
		\param	matrix<double>	A		Matrice de transition (l'ancien valeur au cas où il n'y a pas de modification)
		\param	matrix<double>	B		Matrice de commande	  (							"					  		)
		\param	matrix<double>	C		Matrice de sortie	  (							"							)
	*/
	void majSystem(bool maj[3], matrix<double> A, matrix<double> B, matrix<double> C);


	/** \brief Etape prédiction du filtre de Kalman à partir des données de la commande
		\param	matrix<double>	value_cmd	--	matrice des valeurs de la commande, matrice de taille : nombre d'états du système x nombre d'entrées
		\test	test_Kalman		VALIDE!
	*/
	void predict_step(matrix<double> value_cmd);


	/** \brief Etape mettre à jour les prédictions d'état et de covariance du filtre de Kalman grâce aux valeurs mesurées
		\param	matrix<double>	value_measure			--	 matrice des valeurs mesurées, matrice de taille : nombre de sortie x nombre d'états du système
		\return	matrix<double>	kalm_sys.predict_vector	--	matrice des valeurs estimées
		\test	test_Kalman		VALIDE!
	*/
	matrix<double> update_step(matrix<double> value_mesure);

private:
	system_state sys;		//	Système à appliquer le filtre Kalman
	kalmanstate kalm_sys;	//	Le filtre de Kalman
};
#endif // KALMAN_H