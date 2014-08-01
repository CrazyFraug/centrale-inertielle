#include "Kalman.h"
#include <boost/numeric/ublas/operation.hpp>

matrix<double> product_matrix(matrix<double> M1, matrix<double> M2){
	if (M1.size2() != M2.size1()){
		_RPT0(_CRT_ERROR, "ERROR SIZE MATRIX \n");
	}
	else{
		matrix<double> result(M1.size1(),M2.size2(),0);
		for (int i = 0; i < M1.size1(); i++){
			for (int j = 0; j < M2.size2(); j++){
				for (int k = 0; k < M1.size1(); k++){
					result(i, j) += M1(i, k)*M2(k, j);
				}
			}
		}
		return result;
	}
}


/** \brief Fonction déclare un objet type kalman avec le nombre d'entrées, sorties, états du système
 *
 * \param nb_in int
 * \param nb_out int
 * \param nb_state int
 * \param nb_step int
 *
 */
Kalman::Kalman(int nb_in, int nb_out, int nb_state, int step, matrix<double> init_predict, matrix<double> init_cov_estimate)
{
    sys.size_in = nb_in;
    sys.size_out = nb_out;
    sys.size_state = nb_state;    
	kalm_sys.nb_step = step;
    sys.mat_transition.resize(sys.size_state,sys.size_state);
#pragma warning(suppress: 6282)
	if (sys.size_in != 0){
		sys.mat_cmde.resize(sys.size_in, sys.size_state);
		kalm_sys.noise_cmde.resize(1, sys.size_in);
		kalm_sys.cov_cmde.resize(sys.size_in, sys.size_state);
	}
    sys.mat_sortie.resize(sys.size_out,sys.size_state);

    kalm_sys.matrix_ident.resize(sys.size_state,sys.size_state);
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
		
    kalm_sys.noise_mesure.resize(1, sys.size_out);
    kalm_sys.cov_mesure.resize(sys.size_out, sys.size_state);
	kalm_sys.cov_estimate.resize(sys.size_state,sys.size_state);
	kalm_sys.cov_estimate=init_cov_estimate;

	kalm_sys.predict_vector.resize(sys.size_state,1);
	kalm_sys.predict_vector = init_predict;
    kalm_sys.kalman_gain.resize(sys.size_state,sys.size_state);

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
	if (sys.size_in != 0){
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
	if (sys.size_in == 0){
		kalm_sys.predict_vector = product_matrix(sys.mat_transition, kalm_sys.predict_vector);
	}
	else{
		kalm_sys.predict_vector = product_matrix(sys.mat_transition, kalm_sys.predict_vector) + product_matrix(sys.mat_cmde, value_cmd);
	}

	matrix<double> aux_2 = product_matrix(sys.mat_transition,kalm_sys.cov_estimate);
    aux_2 = product_matrix(aux_2,trans(sys.mat_transition));
	if (sys.size_in != 0){
		kalm_sys.cov_estimate = aux_2 + kalm_sys.cov_cmde;
	}
	else{
		kalm_sys.cov_estimate = aux_2;
	}

	
}

/** \brief Etape mettre à jour les prédictions d'état et de covariance du filtre de Kalman grâce aux valeurs mesurées
 *
 * \param value_measure matrix_type -- matrice des valeurs mesurées, matrice de taille : nombre de sortie x nombre d'états du système
 * \return void
 *
 */
matrix<double> Kalman::update_step(matrix<double> value_measure){
    /* Hk*Xk-1*/
    matrix<double> predict_measure = product_matrix(sys.mat_sortie,kalm_sys.predict_vector);
	
    /* Yk = Zk - Hk*Xk-1*/
    matrix<double> innov_measure = value_measure - predict_measure;
	
    /* Hk*Pk-1*trans(Hk) */
	/* Sk = Hk*Pk-1*trans(Hk) + Rk */
	matrix<double> aux;
	matrix<double> innov_covariance;
	aux = product_matrix(sys.mat_sortie, kalm_sys.cov_estimate);
	aux = product_matrix(aux, trans(sys.mat_sortie));

	innov_covariance = aux + kalm_sys.cov_mesure;

    /* Pk-1*trans(Hk)  */
    matrix<double> aux_2 = product_matrix(kalm_sys.cov_estimate,trans(sys.mat_sortie));

    /* Kk = Pk-1*trans(Hk)*Sk^-1 */
    kalm_sys.kalman_gain = product_matrix(aux_2,conj(innov_covariance));

    /* Kk*Yk */
    matrix<double> aux_3 = product_matrix(kalm_sys.kalman_gain,innov_measure);

    /* ^Xk = ^Xk-1 +  Kk*Yk */
    kalm_sys.predict_vector += aux_3;
	
    /* I - Kk*Hk */
    matrix<double> aux_4 = product_matrix(kalm_sys.kalman_gain,sys.mat_sortie);
    aux_4 = kalm_sys.matrix_ident - aux_4;
	
    /*Pk = (I - Kk*Hk)*Pk-1 */
    kalm_sys.cov_estimate= product_matrix(aux_4,kalm_sys.cov_estimate);
	return kalm_sys.predict_vector;
}


quaternion<double> Kalman::kalman_rotation(vect4D v_angulaire, vect4D acceleration, double dt){
	
	/* Affectation les données pour le filtre de Kalman + Système */
	double sx, sy, sz;
	sx = sy = sz = 0.5;
	matrix<double> A(4, 4, 0), B(0, 0, 0), C(4, 4, 0), Q(4, 4, 0), R(4, 4, 0);
	C(0, 0) = C(1, 1) = C(2, 2) = C(3, 3) = 1;
	R(0, 0) = R(1, 1) = R(2, 2) = R(3, 3) = 0.5;

	double sx2, sy2, sz2;
	sx2 = pow(sx, 2);
	sy2 = pow(sy, 2);
	sz2 = pow(sz, 2);

	Q(0, 0) = sx2 + sy2 + sz2; Q(1, 1) = sx2 + sy2 + sz2; Q(2, 2) = sx2 + sy2 + sz2; Q(3, 3) = sx2 + sy2 + sz2; //diagonale
	Q(0, 1) = -sx2 + sy2 - sz2; Q(0, 2) = -sx2 - sy2 + sz2; Q(0, 3) = sx2 - sy2 - sz2;
	Q(1, 0) = -sx2 + sy2 - sz2; Q(1, 2) = sx2 - sy2 - sz2; Q(1, 3) = -sx2 - sy2 + sz2;
	Q(2, 0) = -sx2 - sy2 + sz2; Q(2, 1) = sx2 - sy2 - sz2; Q(2, 3) = -sx2 + sy2 - sz2;
	Q(3, 0) = sx2 - sy2 - sz2; Q(3, 1) = -sx2 - sy2 + sz2; Q(3, 2) = -sx2 + sy2 - sz2;

	double wx, wy, wz, Ax, Ay, Az;
	wx = v_angulaire.x;
	wy = v_angulaire.y;
	wz = v_angulaire.z;

	/* Mettre à jour la matrice de transition A */
	Ax = 0.5*wx*dt;
	Ay = 0.5*wy*dt;
	Az = 0.5*wz*dt;

	A(0, 0) = 1;
	A(0, 0) = 1; A(1, 1) = 1; A(2, 2) = 1; A(3, 3) = 1; //diagonale
	A(0, 1) = -Ax; A(0, 2) = -Ay; A(0, 3) = -Az;
	A(1, 0) = Ax; A(1, 2) = Az; A(1, 3) = -Ay;
	A(2, 0) = Ay; A(2, 1) = -Az; A(2, 3) = Ax;
	A(3, 0) = Az; A(3, 1) = Ay; A(3, 2) = -Ax;

	declare_system(A, B, C);
	declare_noise(Q, R);

	/* Etape de prédiction du système */
	predict_step(zero_matrix<double>(4, 4));

	/*
	*	Récupération des accélérations pour l'observation du système
	*	Passage de l'accélération en angles d'Euler
	*/
	vect3D angle_meas;
	angle_meas.x = atan(pow(acceleration.x / (pow(acceleration.x, 2) + pow(acceleration.y, 2)), 2)) * 180 / (atan(1) * 4);
	angle_meas.y = 0;
	angle_meas.z = atan(pow(acceleration.z / (pow(acceleration.z, 2) + pow(acceleration.y, 2)), 2)) * 180 / (atan(1) * 4);

	quaternion<double> quat_meas;

	quat_meas = anglesToQuat(angle_meas.x, angle_meas.y, angle_meas.z);

	matrix<double> mesuresQuat(4, 1, 0), estimate_result(4, 1, 0);

	vect3D measure_estimate;
	mesuresQuat(0, 0) = quat_meas.R_component_1();
	mesuresQuat(1, 0) = quat_meas.R_component_2();
	mesuresQuat(2, 0) = quat_meas.R_component_3();
	mesuresQuat(3, 0) = quat_meas.R_component_4();

	/* Etape update les données du filtre de Kalman */
	estimate_result = update_step(mesuresQuat);

	quaternion<double> quat_estimate(estimate_result(0, 0), estimate_result(1, 0), estimate_result(2, 0), estimate_result(3, 0));
	return quat_estimate;
}