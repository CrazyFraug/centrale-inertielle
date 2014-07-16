#include "kalman.h"

/** \brief Fonction d�clare un objet type kalman avec le nombre d'entr�es, sorties, �tats du syst�me
 *
 * \param nb_in int
 * \param nb_out int
 * \param nb_state int
 *
 */
kalman::kalman(int nb_in, int nb_out, int nb_state)
{
    sys.size_in = nb_in;
    sys.size_out = nb_out;
    sys.size_state = nb_state;

    kalm_sys.filtre_sys = sys;
}

/** \brief Fonction d�clare le syst�me dont on veut travailler/filtrer
 *
 * \param A matrix_type -- matrice de transition, matrice carr�e de taille = nombre d'�tats du syst�me
 * \param B matrix_type -- matrice de commande, matrice de taille : nombre d'�tats du syst�me x nombre d'entr�es
 * \param C matrix_type -- matrice de sortie, matrice de taille : nombre de sortie x nombre d'�tats du syst�me
 * \return void
 *
 */
void kalman::declare_system(matrix_type A, matrix_type B, matrix_type C){
    sys.mat_transition = copy_matrix (A);
    sys.mat_cmde = copy_matrix (B);
    sys.mat_sortie = copy_matrix (C);
}

/** \brief Fonction d�clare les matrices de covariance des bruits de commande et de mesure du syst�me
 *
 * \param Q matrix_type -- matrice de covariance de commande, matrice carr�e de taille = nombre d'�tats du syst�me
 * \param R matrix_type -- matrice de covariance de mesure, matrice de taille : nombre de sortie x nombre d'�tats du syst�me
 * \return void
 *
 */
void kalman::declare_noise(matrix_type Q, matrix_type R){
    kalm_sys.cov_cmde = copy_matrix(Q);
    kalm_sys.cov_mesure = copy_matrix(R);
}

/** \brief Etape pr�diction du filtre de Kalman � partir des donn�es de la commande
 *
 * \param value_cmd matrix_type -- matrice des valeurs de la commande, matrice de taille : nombre d'�tats du syst�me x nombre d'entr�es
 * \return void
 *
 */
void kalman::predict_step(matrix_type value_cmd){
    matrix_type aux = prod(kalm_sys.filtre_sys.mat_transition,kalm_sys.predict_vector);
    matrix_type aux_1 = prod(kalm_sys.filtre_sys.mat_cmde,value_cmd);
    kalm_sys.predict_vector = aux + aux_1;
    matrix_type aux_2 = prod(kalm_sys.filtre_sys.mat_transition,kalm_sys.cov_estimate);
    aux = prod(aux,trans(kalm_sys.filtre_sys.mat_transition));
    kalm_sys.cov_estimate = aux + kalm_sys.cov_cmde;
}

/** \brief Etape mettre � jour les pr�dictions d'�tat et de covariance du filtre de Kalman gr�ce aux valeurs mesur�es
 *
 * \param value_measure matrix_type -- matrice des valeurs mesur�es, matrice de taille : nombre de sortie x nombre d'�tats du syst�me
 * \return void
 *
 */
void kalman::update_step(matrix_type value_measure){
    /* Hk*Xk-1*/
    matrix_type predict_measure = prod(kalm_sys.filtre_sys.mat_sortie,kalm_sys.predict_vector);
    /* Yk = Zk - Hk*Xk-1*/
    matrix_type innov_measure = value_measure - predict_measure;
    /* Hk*Pk-1*trans(Hk) */
    matrix_type aux = prod(kalm_sys.cov_mesure,kalm_sys.cov_estimate);
    aux = prod(aux,trans(kalm_sys.cov_mesure));
    /* Sk = Hk*Pk-1*trans(Hk) + Rk */
    matrix_type innov_covariance = aux + kalm_sys.cov_mesure;
    /* Pk-1*trans(Hk)  */
    matrix_type aux_2 = prod(kalm_sys.cov_estimate,trans(kalm_sys.filtre_sys.mat_sortie));
    /* Kk = Pk-1*trans(Hk)*Sk^-1 */
    kalm_sys.kalman_gain = prod(aux_2,conj(innov_covariance));
    /* Kk*Yk */
    matrix_type aux_3 = prod(kalm_sys.kalman_gain,innov_measure);
    /* ^Xk = ^Xk-1 +  Kk*Yk */
    kalm_sys.predict_vector += aux_3;
    /* I - Kk*Hk */
    matrix_type aux_4 = prod(kalm_sys.kalman_gain,kalm_sys.filtre_sys.mat_sortie);
    aux_4 = kalm_sys.matrix_ident - aux_4;
    /*Pk = (I - Kk*Hk)*Pk-1 */
    kalm_sys.cov_estimate = prod(aux_4,kalm_sys.cov_estimate);
}
