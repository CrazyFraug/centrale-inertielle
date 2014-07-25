#include "Kalman.h"

/** \brief Fonction déclare un objet type kalman avec le nombre d'entrées, sorties, états du système
 *
 * \param nb_in int
 * \param nb_out int
 * \param nb_state int
 * \param nb_step int
 *
 */
Kalman::Kalman(int nb_in, int nb_out, int nb_state, int step, matrix<double> init_cov_estimate)
{
    sys.size_in = nb_in;
    sys.size_out = nb_out;
    sys.size_state = nb_state;    
	kalm_sys.nb_step = step;

    sys.mat_transition(sys.size_state,sys.size_state);
#pragma warning(suppress: 6282)
	if (sys.size_in == 0){
		sys.mat_cmde(0,0);
	}
	else{
		sys.mat_cmde(sys.size_in, sys.size_state);
	}
    sys.mat_sortie(sys.size_out,sys.size_state);

    kalm_sys.matrix_ident(sys.size_state,sys.size_state);
    kalm_sys.noise_cmde(1,sys.size_in);
    kalm_sys.cov_cmde(sys.size_in,sys.size_state);
    kalm_sys.noise_mesure(1, sys.size_out);
    kalm_sys.cov_mesure(sys.size_out, sys.size_state);

	kalm_sys.cov_estimate(sys.size_state,sys.size_state);
	kalm_sys.cov_estimate=init_cov_estimate;

    kalm_sys.predict_vector(1,sys.size_state);
    kalm_sys.kalman_gain(sys.size_state,sys.size_state);

}

Kalman::~Kalman()
{
}

/** \brief Fonction déclare le système sur lequel on veut travailler/filtrer
 *
 * \param A matrix_type -- matrice de transition, matrice carrée de taille = nombre d'états du système
 * \param B matrix_type -- matrice de commande, matrice de taille : nombre d'états du système x nombre d'entrées
 * \param C matrix_type -- matrice de sortie, matrice de taille : nombre de sortie x nombre d'états du système
 * \return void
 *
 */
void Kalman::declare_system(matrix<double> A, matrix<double> B, matrix<double> C){
    sys.mat_transition =A ;
    sys.mat_cmde = B;
    sys.mat_sortie = C;
}

/** \brief Fonction déclare les matrices de covariance des bruits de commande et de mesure du système
 *
 * \param Q matrix_type -- matrice de covariance de commande, matrice carrée de taille = nombre d'états du système
 * \param R matrix_type -- matrice de covariance de mesure, matrice de taille : nombre de sortie x nombre d'états du système
 * \return void
 *
 */
void Kalman::declare_noise(matrix<double> Q, matrix<double> R){
	if (sys.size_in == 0){
		kalm_sys.cov_cmde(0, 0);
	}
	else{
		kalm_sys.cov_cmde = Q;
	}
		
    kalm_sys.cov_mesure = R;
}

/** \brief Etape prédiction du filtre de Kalman à partir des données de la commande
 *
 * \param value_cmd matrix_type -- matrice des valeurs de la commande, matrice de taille : nombre d'états du système x nombre d'entrées
 * \return void
 *
 */
void Kalman::predict_step(matrix<double> value_cmd)
{
    matrix<double> aux = prod(sys.mat_transition,kalm_sys.predict_vector);

	if (sys.size_in == 0){
		kalm_sys.predict_vector = aux;
	}
	else{
		matrix<double> aux_1 = prod(sys.mat_cmde, value_cmd);
		kalm_sys.predict_vector = aux + aux_1;
	}

	matrix<double> aux_2 = prod(sys.mat_transition,kalm_sys.cov_estimate);
    aux_2 = prod(aux,trans(sys.mat_transition));
    kalm_sys.cov_estimate = aux_2 + kalm_sys.cov_cmde;
}

/** \brief Etape mettre à jour les prédictions d'état et de covariance du filtre de Kalman grâce aux valeurs mesurées
 *
 * \param value_measure matrix_type -- matrice des valeurs mesurées, matrice de taille : nombre de sortie x nombre d'états du système
 * \return void
 *
 */
matrix<double> Kalman::update_step(matrix<double> value_measure){
    /* Hk*Xk-1*/
    matrix<double> predict_measure = prod(sys.mat_sortie,kalm_sys.predict_vector);

    /* Yk = Zk - Hk*Xk-1*/
    matrix<double> innov_measure = value_measure - predict_measure;

    /* Hk*Pk-1*trans(Hk) */
    matrix<double> aux = prod(kalm_sys.cov_mesure,kalm_sys.cov_estimate);
    aux = prod(aux,trans(kalm_sys.cov_mesure));

    /* Sk = Hk*Pk-1*trans(Hk) + Rk */
    matrix<double> innov_covariance = aux + kalm_sys.cov_mesure;

    /* Pk-1*trans(Hk)  */
    matrix<double> aux_2 = prod(kalm_sys.cov_estimate,trans(sys.mat_sortie));

    /* Kk = Pk-1*trans(Hk)*Sk^-1 */
    kalm_sys.kalman_gain = prod(aux_2,conj(innov_covariance));

    /* Kk*Yk */
    matrix<double> aux_3 = prod(kalm_sys.kalman_gain,innov_measure);

    /* ^Xk = ^Xk-1 +  Kk*Yk */
    kalm_sys.predict_vector += aux_3;

    /* I - Kk*Hk */
    matrix<double> aux_4 = prod(kalm_sys.kalman_gain,sys.mat_sortie);
    aux_4 = kalm_sys.matrix_ident - aux_4;

    /*Pk = (I - Kk*Hk)*Pk-1 */
    kalm_sys.cov_estimate= prod(aux_4,kalm_sys.cov_estimate);

	return kalm_sys.predict_vector;
}


