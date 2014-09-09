
#include <boost/numeric/ublas/operation.hpp>
#include "Kalman.h"
#include "Tools.h"


/** \brief Constructor d'un objet Kalman

*	\param	int				nb_in				--	Nombre d'entrées du système
\param	int				nb_out				--	Nombre de sorties du système
\param	int				nb_state			--	Nombre de variables d'état du système
\param	int				nb_step				--	Nombre d'échantillon d'initialisation du système
\param	matrix<double>	init_predict		--	Vector d'initialisation du vecteur d'état
\param	matrix<double>	init_cov_estimate	--	Matrice d'initialisation de la matrice de covariance
*/
Kalman::Kalman(int nb_in, int nb_out, int nb_state, int step) : test_value(0)
{
	/********************************************************************/
	/* Affectation les nombres d'entrées, sorties et d'état du système	*
	*  Déclaration le nombre d'échantillon d'initialisation				*/
	/********************************************************************/
	sys.size_in = nb_in;
	sys.size_out = nb_out;
	sys.size_state = nb_state;
	kalm_sys.nb_step = step;

	/********************************************************************/
	/* Affectation les matrices aux tailles du système					*/
	/********************************************************************/
	sys.mat_transition.resize(sys.size_state, sys.size_state);
#pragma warning(suppress: 6282)
	if (sys.size_in != 0){
		sys.mat_cmde.resize(sys.size_in, sys.size_state);
		kalm_sys.noise_cmde.resize(1, sys.size_in);
	}
	sys.mat_sortie.resize(sys.size_out, sys.size_state);

	kalm_sys.matrix_ident.resize(sys.size_state, sys.size_state);
	for (int i = 0; i < kalm_sys.matrix_ident.size1(); i++){
		for (int j = 0; j < kalm_sys.matrix_ident.size2(); j++){
			if (i == j){
				kalm_sys.matrix_ident(i, j) = 1;
			}
			else{
				kalm_sys.matrix_ident(i, j) = 0;
			}
		}
	}

	kalm_sys.cov_cmde.resize(sys.size_state, sys.size_state);
	kalm_sys.noise_mesure.resize(1, sys.size_out);
	kalm_sys.cov_mesure.resize(sys.size_out, sys.size_state);
	kalm_sys.cov_estimate.resize(sys.size_state, sys.size_state);


	kalm_sys.predict_vector.resize(sys.size_state, 1);
	kalm_sys.kalman_gain.resize(sys.size_state, sys.size_out);

}

/** \brief	Destructor
*/
Kalman::~Kalman()
{
}

int Kalman::getSizeIn(){
	return sys.size_in;
}

int Kalman::getSizeOut(){
	return sys.size_out;
}

int Kalman::getSizeState(){
	return sys.size_state;
}

/** \brief	Fonction déclare le système sur lequel on veut travailler/filtrer

*	\param	matrix<double>	A	--	matrice de transition, matrice carrée de taille = nombre d'états du système
\param	matrix<double>	B	--	matrice de commande, matrice de taille : nombre d'états du système x nombre d'entrées
\param	matrix<double>	C	--	matrice de sortie, matrice de taille : nombre de sortie x nombre d'états du système

*	\return void
*/
void Kalman::declare_system(matrix<double> A, matrix<double> B, matrix<double> C){
	sys.mat_transition = A;
	sys.mat_cmde = B;
	sys.mat_sortie = C;
}

/** \brief	Fonction déclare les matrices de covariance des bruits de commande et de mesure du système

*	\param	matrix<double>	Q	-- matrice de covariance de commande, matrice carrée de taille = nombre d'états du système
\param	matrix<double>	R	-- matrice de covariance de mesure, matrice de taille : nombre de sortie x nombre d'états du système

*	\return void
*/
void Kalman::declare_noise(matrix<double> Q, matrix<double> R){
	kalm_sys.cov_cmde = Q;

	kalm_sys.cov_mesure = R;
}

/** \brief Etape prédiction du filtre de Kalman à partir des données de la commande

*	\param	matrix<double>	value_cmd	--	matrice des valeurs de la commande, matrice de taille : nombre d'états du système x nombre d'entrées

*	\return void

*	\test	test_Kalman	inclus dans le test_Kalman_rotation
*/
void Kalman::predict_step(matrix<double> value_cmd)
{
	/****************************************/
	/* ^Xk = A*Xk-1 + B*u	(cas normal)	*
	* ^Xk = A*Xk-1	(cas non entrée)		*/
	/****************************************/
	if (sys.size_in == 0){
		_RPT0(0, "MAT TRANSITION : \n");
		printMatrix(sys.mat_transition);

		_RPT0(0, "PREDICT VECTOR : \n");
		printMatrix(kalm_sys.predict_vector);

		kalm_sys.predict_vector = product_matrix(sys.mat_transition, kalm_sys.predict_vector);

	}
	else{
		kalm_sys.predict_vector = product_matrix(sys.mat_transition, kalm_sys.predict_vector) + product_matrix(sys.mat_cmde, value_cmd);
	}

	/****************************************/
	/* Pk = F*Pk-1*trans(F) + Q				*/
	/****************************************/
	matrix<double> aux_2 = product_matrix(sys.mat_transition, kalm_sys.cov_estimate);
	aux_2 = product_matrix(aux_2, trans(sys.mat_transition));
	kalm_sys.cov_estimate = aux_2 + kalm_sys.cov_cmde;

	/*_RPT0(0, "MATRICE DE COVARIANCE : \n");
	printMatrix(kalm_sys.cov_estimate);*/
}

/** \brief Etape mettre à jour les prédictions d'état et de covariance du filtre de Kalman grâce aux valeurs mesurées

*	\param	matrix<double>	value_measure			--	 matrice des valeurs mesurées, matrice de taille : nombre de sortie x nombre d'états du système

*	\return	matrix<double>	kalm_sys.predict_vector	--	matrice des valeurs estimées

*	\test	test_Kalman		VALIDE!
*/
matrix<double> Kalman::update_step(matrix<double> value_measure){
	/****************************************/
	/* Hk*Xk-1								*/
	/****************************************/
	matrix<double> predict_measure = product_matrix(sys.mat_sortie, kalm_sys.predict_vector);
	_RPT0(0, "PREDICT MEASURE : \n");
	printMatrix(predict_measure);
	/****************************************/
	/* Yk = Zk - Hk*Xk-1					*/
	/****************************************/
	matrix<double> innov_measure = value_measure - predict_measure;
	_RPT0(0, "VALUE MEASURE : \n");
	printMatrix(value_measure);
	_RPT0(0, "INNOVATION MEASURE : \n");
	printMatrix(innov_measure);
	/****************************************/
	/* Hk*Pk-1*trans(Hk)					*
	* Sk = Hk*Pk-1*trans(Hk) + Rk			*/
	/****************************************/
	matrix<double> aux;
	matrix<double> innov_covariance;
	aux = product_matrix(sys.mat_sortie, kalm_sys.cov_estimate);
	aux = product_matrix(aux, trans(sys.mat_sortie));
	innov_covariance = aux + kalm_sys.cov_mesure;

	_RPT0(0, "INNOVATION COVARIANCE : \n");
	printMatrix(innov_covariance);
	/****************************************/
	/* Kk = Pk-1*trans(Hk)*Sk^-1			*/
	/****************************************/
	matrix<double> aux_2 = product_matrix(kalm_sys.cov_estimate, trans(sys.mat_sortie));
	_RPT0(0, "AUX_2 : \n");
	printMatrix(aux_2);
	kalm_sys.kalman_gain = product_matrix(aux_2, conj(innov_covariance));
	_RPT0(0, "KALMAN GAIN : \n");
	printMatrix(kalm_sys.kalman_gain);
	/****************************************/
	/* ^Xk = ^Xk-1 +  Kk*Yk					*/
	/****************************************/
	matrix<double> aux_3 = product_matrix(kalm_sys.kalman_gain, innov_measure);

	kalm_sys.predict_vector = kalm_sys.predict_vector + aux_3;
	_RPT0(0, "PREDICT VECTOR : \n");
	printMatrix(kalm_sys.predict_vector);
	/****************************************/
	/* Pk = (I - Kk*Hk)*Pk-1				*/
	/****************************************/
	matrix<double> aux_4 = product_matrix(kalm_sys.kalman_gain, sys.mat_sortie);
	aux_4 = kalm_sys.matrix_ident - aux_4;

	kalm_sys.cov_estimate = product_matrix(aux_4, kalm_sys.cov_estimate);
	/*_RPT0(0, "COVARIANCE ESTIMATE : \n");
	printMatrix(kalm_sys.cov_estimate);*/
	return kalm_sys.predict_vector;
}

void Kalman::initSystem(matrix<double> A, matrix<double> B, matrix<double> C, matrix<double> Q, matrix<double> R, matrix<double> init_predict, matrix<double> init_cov_estimate){
	kalm_sys.cov_estimate = init_cov_estimate;
	kalm_sys.predict_vector = init_predict;
	declare_system(A, B, C);
	declare_noise(Q, R);
}

void Kalman::majSystem(bool maj, char mat, matrix<double> A, matrix<double> B, matrix<double> C){
	if (maj == true){
		if (mat = 'A'){
			sys.mat_transition = A;
		}
		else if (mat = 'B'){
			sys.mat_cmde = B;
		}
		else if (mat = 'C'){
			sys.mat_sortie = C;
		}
	}
}
