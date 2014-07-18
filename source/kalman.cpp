#include "kalman.h"

/** \brief Fonction d�clare un objet type kalman avec le nombre d'entr�es, sorties, �tats du syst�me
 *
 * \param nb_in int
 * \param nb_out int
 * \param nb_state int
 * \param nb_step int
 *
 */
kalman::kalman(int nb_in, int nb_out, int nb_state, int step, matrix_double init_cov_estimate)
{
    sys.size_in = nb_in;
    sys.size_out = nb_out;
    sys.size_state = nb_state;
    sys.mat_transition.get_matrix(sys.size_state,sys.size_state);
    sys.mat_cmde.get_matrix(sys.size_in,sys.size_state);
    sys.mat_sortie.get_matrix(sys.size_out,sys.size_state);
    kalm_sys.filtre_sys = sys;
    kalm_sys.nb_step = step;
    kalm_sys.matrix_ident.get_ident(sys.size_state,1);
    kalm_sys.noise_cmde.get_matrix(1,sys.size_in);
    kalm_sys.cov_cmde.get_matrix(sys.size_in,sys.size_state);
    kalm_sys.noise_mesure.get_matrix(1, sys.size_out);
    kalm_sys.cov_mesure.get_matrix(sys.size_out, sys.size_state);
    kalm_sys.cov_estimate.copy_matrix(init_cov_estimate) ;
    kalm_sys.predict_vector.get_matrix(1,sys.size_state);
    kalm_sys.kalman_gain.get_matrix(sys.size_state,sys.size_state);

}


/** \brief Fonction d�clare le syst�me dont on veut travailler/filtrer
 *
 * \param A matrix_type -- matrice de transition, matrice carr�e de taille = nombre d'�tats du syst�me
 * \param B matrix_type -- matrice de commande, matrice de taille : nombre d'�tats du syst�me x nombre d'entr�es
 * \param C matrix_type -- matrice de sortie, matrice de taille : nombre de sortie x nombre d'�tats du syst�me
 * \return void
 *
 */
void kalman::declare_system(matrix_double A, matrix_double B, matrix_double C){
    sys.mat_transition.copy_matrix (A);
    sys.mat_cmde.copy_matrix (B);
    sys.mat_sortie.copy_matrix (C);
}

/** \brief Fonction d�clare les matrices de covariance des bruits de commande et de mesure du syst�me
 *
 * \param Q matrix_type -- matrice de covariance de commande, matrice carr�e de taille = nombre d'�tats du syst�me
 * \param R matrix_type -- matrice de covariance de mesure, matrice de taille : nombre de sortie x nombre d'�tats du syst�me
 * \return void
 *
 */
void kalman::declare_noise(matrix_double Q, matrix_double R){
    kalm_sys.cov_cmde.copy_matrix(Q);
    kalm_sys.cov_mesure.copy_matrix(R);
}

/** \brief Etape pr�diction du filtre de Kalman � partir des donn�es de la commande
 *
 * \param value_cmd matrix_type -- matrice des valeurs de la commande, matrice de taille : nombre d'�tats du syst�me x nombre d'entr�es
 * \return void
 *
 */
void kalman::predict_step(matrix_double value_cmd){
    matrix_double aux = prod(kalm_sys.filtre_sys.mat_transition.mat,kalm_sys.predict_vector.mat);
    matrix_double aux_1 = prod(kalm_sys.filtre_sys.mat_cmde.mat,value_cmd);
    kalm_sys.predict_vector.mat = aux + aux_1;
    matrix_double matrix_doublex_2 = prod(kalm_sys.filtre_sys.mat_transition.mat,kalm_sys.cov_estimate.mat);
    aux = prod(aux,trans(kalm_sys.filtre_sys.mat_transition.mat));
    kalm_sys.cov_estimate.mat = aux + kalm_sys.cov_cmde.mat;
}

/** \brief Etape mettre � jour les pr�dictions d'�tat et de covariance du filtre de Kalman gr�ce aux valeurs mesur�es
 *
 * \param value_measure matrix_type -- matrice des valeurs mesur�es, matrice de taille : nombre de sortie x nombre d'�tats du syst�me
 * \return void
 *
 */
matrix_type kalman::update_step(matrix_double value_measure){
    /* Hk*Xk-1*/
    matrix_double predict_measure = prod(kalm_sys.filtre_sys.mat_sortie.mat,kalm_sys.predict_vector.mat);

    /* Yk = Zk - Hk*Xk-1*/
    matrix_double innov_measure = value_measure - predict_measure;

    /* Hk*Pk-1*trans(Hk) */
    matrix_double aux = prod(kalm_sys.cov_mesure.mat,kalm_sys.cov_estimate.mat);
    aux = prod(aux,trans(kalm_sys.cov_mesure.mat));

    /* Sk = Hk*Pk-1*trans(Hk) + Rk */
    matrix_double innov_covariance = aux + kalm_sys.cov_mesure.mat;

    /* Pk-1*trans(Hk)  */
    matrix_double aux_2 = prod(kalm_sys.cov_estimate.mat,trans(kalm_sys.filtre_sys.mat_sortie.mat));

    /* Kk = Pk-1*trans(Hk)*Sk^-1 */
    kalm_sys.kalman_gain.mat = prod(aux_2,conj(innov_covariance));

    /* Kk*Yk */
    matrix_double aux_3 = prod(kalm_sys.kalman_gain.mat,innov_measure);

    /* ^Xk = ^Xk-1 +  Kk*Yk */
    kalm_sys.predict_vector.mat += aux_3;

    /* I - Kk*Hk */
    matrix_double aux_4 = prod(kalm_sys.kalman_gain.mat,kalm_sys.filtre_sys.mat_sortie.mat);
    aux_4 = kalm_sys.matrix_ident.mat - aux_4;

    /*Pk = (I - Kk*Hk)*Pk-1 */
    kalm_sys.cov_estimate.mat = prod(aux_4,kalm_sys.cov_estimate.mat);

	return kalm_sys.predict_vector;
}
