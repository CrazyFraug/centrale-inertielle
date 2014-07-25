#include "kalman.h"

/** \brief Fonction déclare un objet type kalman avec le nombre d'entrées, sorties, états du système
 *
 * \param nb_in int
 * \param nb_out int
 * \param nb_state int
 * \param nb_step int
 *
 */
kalman::kalman(int nb_in, int nb_out, int nb_state, int step, matrix<double> init_cov_estimate)
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

kalman::~kalman()
{
}


/**
 *  Initialisation des matrices pour le filtre kalman
 *  \param      sx double   vitesse angulaire selon l'axe Ax
 *  \param      sy double   vitesse angulaire selon l'axe Ay
 *  \param      sz double   vitesse angulaire selon l'axe Az
 *
*/
void kalman::initMatrices(double sx, double sy, double sz)
{
        double sx2, sy2, sz2;
        sx2 = pow(sx,2);
        sy2 = pow(sy,2);
        sz2 = pow(sz,2);

        kalm_sys.cov_cmde(0,0) = sx2 + sy2 + sz2; kalm_sys.cov_cmde(1,1) = sx2 + sy2 + sz2; kalm_sys.cov_cmde(2,2) = sx2 + sy2 + sz2; kalm_sys.cov_cmde(3,3) = sx2 + sy2 + sz2; //diagonale
        kalm_sys.cov_cmde(0,1) = -sx2 + sy2 - sz2; kalm_sys.cov_cmde(0,2) = -sx2 -sy2 + sz2; kalm_sys.cov_cmde(0,3) = sx2 - sy2 - sz2;
        kalm_sys.cov_cmde(1,0) = -sx2 + sy2 - sz2; kalm_sys.cov_cmde(1,2) = sx2 - sy2 - sz2; kalm_sys.cov_cmde(1,3) = -sx2 - sy2 + sz2;
        kalm_sys.cov_cmde(2,0) = -sx2 -sy2 + sz2; kalm_sys.cov_cmde(2,1) = sx2 - sy2 - sz2; kalm_sys.cov_cmde(2,3) = -sx2 + sy2 - sz2;
        kalm_sys.cov_cmde(3,0) = sx2 - sy2 - sz2; kalm_sys.cov_cmde(3,1) = -sx2 - sy2 + sz2; kalm_sys.cov_cmde(3,2) = -sx2 + sy2 - sz2;

        kalm_sys.cov_mesure(0,0) = 0.05;
        kalm_sys.cov_mesure(1,1) = 0.05;
        kalm_sys.cov_mesure(2,2) = 0.05;
        kalm_sys.cov_mesure(3,3) = 0.05;


        sys.mat_sortie(0,0) = 1;
        sys.mat_sortie(1,1) = 1;
        sys.mat_sortie(2,2) = 1;
        sys.mat_sortie(3,3) = 1;     

}


/**
 *  Traitement avec le filtre Kalman
 *  \param      mesures[3]   vecteur de 3 mesures de vitesse angulaire selon l'axe Ax, Ay, Az
 *  \return measure_estimate vecteur de 3 angles de rotation selon l'axe Ax, Ay, Az estimés
 *
*/

double* kalman::initSystem(double mesures[3], double dt) {

        double* estimation;
        double anglex, angley, anglez, wx,wy,wz, Ax, Ay, Az;
        //CQRQuaternionHandle quat_meas, quat_estimate;

        estimation = new double[3];

        //CQRCreateEmptyQuaternion(&quat_meas);
        //CQRCreateEmptyQuaternion(&quat_estimate);

        wx = mesures[0];
        wy = mesures[1];
        wz = mesures[2];
        /* Mettre à jour la matrice de transition A */
        Ax = 0.5*wx*dt;
        Ay = 0.5*wy*dt;
        Az = 0.5*wz*dt;

        A(0,0)=1;
        sys.mat_transition(0,0) = 1; sys.mat_transition(1,1) = 1; sys.mat_transition(2,2) = 1; sys.mat_transition(3,3) = 1; //diagonale
        sys.mat_transition(0,1) = -Ax; sys.mat_transition(0,2) = -Ay; sys.mat_transition(0,3) = -Az;
        sys.mat_transition(1,0) = Ax; sys.mat_transition(1,2) = Az; sys.mat_transition(1,3) = -Ay;
        sys.mat_transition(2,0) = Ay; sys.mat_transition(2,1) = -Az; sys.mat_transition(2,3) = Ax;
        sys.mat_transition(3,0) = Az; sys.mat_transition(3,1) = Ay; sys.mat_transition(3,2) = -Ax;

        /* Mettre à jour le système à filtrer */
        //filtre.declare_system(A, B, C);
        //filtre.declare_noise(Q, R);

        /* Etape prédiction du filtre Kalman */
        predict_step(B);


        /* Convertir les données mesurées(angles) en quaternion (type matrix) */
        anglex = wx*dt;
        angley = wy*dt;
        anglez = wz*dt;

        CQRAngles2Quaternion (quat_meas, anglex, angley, anglez );

        matrix<double> mesuresQuat(4,1);

        mesuresQuat(0,0) = quat_meas->w;
        mesuresQuat(1,0) = quat_meas->x;
        mesuresQuat(2,0) = quat_meas->y;
        mesuresQuat(3,0) = quat_meas->z;

        matrix<double> estimate_result(4,1,0);

        /* Etape update l'estimation */
		estimate_result = filtre.update_step(mesuresQuat);

        quat_estimate->w = estimate_result(0,0);
        quat_estimate->x = estimate_result(1,0);
        quat_estimate->y = estimate_result(2,0);
        quat_estimate->z = estimate_result(3,0);

        /* Convertir les données estimées (quaternion) en angles pour l'affichage */
        matrix<double> measure_estimate(3,1);
        CQRQuaternion2Angles(&measure_estimate(0,1), &measure_estimate(1,1), &measure_estimate(2,1), quat_estimate);

        return estimation;
}

/** \brief Fonction déclare le système sur lequel on veut travailler/filtrer
 *
 * \param A matrix_type -- matrice de transition, matrice carrée de taille = nombre d'états du système
 * \param B matrix_type -- matrice de commande, matrice de taille : nombre d'états du système x nombre d'entrées
 * \param C matrix_type -- matrice de sortie, matrice de taille : nombre de sortie x nombre d'états du système
 * \return void
 *
 */
void kalman::declare_system(matrix<double> A, matrix<double> B, matrix<double> C){
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
void kalman::declare_noise(matrix<double> Q, matrix<double> R){
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
void kalman::predict_step(matrix<double> value_cmd)
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
matrix<double> kalman::update_step(matrix<double> value_measure){
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


