#include "Kalman.h"
#include <boost/numeric/ublas/operation.hpp>

/** \brief	Produit de deux matrices

*	\param	matrix<double>	M1		--	Matrice gauche de taille m x n
\param	matrix<double>	M2		--	Matrice gauche de taille n x o

*	\return	matrix<double>	result	-- Résultat du produit de taille m x o (cas normal) || Message d'erreur si la taille des matrices ne convient pas
*/
matrix<double> product_matrix(matrix<double> M1, matrix<double> M2){
	if (M1.size2() != M2.size1()){
		_RPT0(_CRT_ERROR, "ERROR SIZE MATRIX \n");
	}
	else{
		matrix<double> result(M1.size1(), M2.size2(), 0);
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

void printMatrix(matrix<double> A){
	for (int i = 0; i < A.size1(); i++){
		_RPT0(0, "| ");
		for (int j = 0; j < A.size2(); j++){
			_RPT1(0, " %f ", A(i, j));
		}
		_RPT0(0, " |\n");
	}
}


/** \brief Constructor d'un objet Kalman

*	\param	int				nb_in				--	Nombre d'entrées du système
\param	int				nb_out				--	Nombre de sorties du système
\param	int				nb_state			--	Nombre de variables d'état du système
\param	int				nb_step				--	Nombre d'échantillon d'initialisation du système
\param	matrix<double>	init_predict		--	Vector d'initialisation du vecteur d'état
\param	matrix<double>	init_cov_estimate	--	Matrice d'initialisation de la matrice de covariance
*/
Kalman::Kalman(int nb_in, int nb_out, int nb_state, int step, matrix<double> init_predict, matrix<double> init_cov_estimate) : test_value(0)
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
	kalm_sys.cov_estimate = init_cov_estimate;

	kalm_sys.predict_vector.resize(sys.size_state, 1);
	kalm_sys.predict_vector = init_predict;
	kalm_sys.kalman_gain.resize(sys.size_state, sys.size_state);

}

/** \brief	Destructor
*/
Kalman::~Kalman()
{
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
	if (sys.size_in != 0){
		kalm_sys.cov_estimate = aux_2 + kalm_sys.cov_cmde;
	}
	else{
		kalm_sys.cov_estimate = aux_2;
	}
	/*_RPT0(0, "MATRICE DE COVARIANCE : \n");
	printMatrix(kalm_sys.cov_estimate);*/
}

/** \brief Etape mettre à jour les prédictions d'état et de covariance du filtre de Kalman grâce aux valeurs mesurées

*	\param	matrix<double>	value_measure			--	 matrice des valeurs mesurées, matrice de taille : nombre de sortie x nombre d'états du système

*	\return	matrix<double>	kalm_sys.predict_vector	--	matrice des valeurs estimées

*	\test	test_Kalman		inclus dans le test_Kalman_rotation
*/
matrix<double> Kalman::update_step(matrix<double> value_measure){
	/****************************************/
	/* Hk*Xk-1								*/
	/****************************************/
	matrix<double> predict_measure = product_matrix(sys.mat_sortie, kalm_sys.predict_vector);
	/*_RPT0(0, "PREDICT MEASURE : \n");
	printMatrix(predict_measure);*/
	/****************************************/
	/* Yk = Zk - Hk*Xk-1					*/
	/****************************************/
	matrix<double> innov_measure = value_measure - predict_measure;
	/*_RPT0(0, "INNOVATION MEASURE : \n");
	printMatrix(innov_measure);*/
	/****************************************/
	/* Hk*Pk-1*trans(Hk)					*
	* Sk = Hk*Pk-1*trans(Hk) + Rk			*/
	/****************************************/
	matrix<double> aux;
	matrix<double> innov_covariance;
	aux = product_matrix(sys.mat_sortie, kalm_sys.cov_estimate);
	aux = product_matrix(aux, trans(sys.mat_sortie));
	innov_covariance = aux + kalm_sys.cov_mesure;

	/*_RPT0(0, "INNOVATION COVARIANCE : \n");
	printMatrix(innov_covariance);*/
	/****************************************/
	/* Kk = Pk-1*trans(Hk)*Sk^-1			*/
	/****************************************/
	matrix<double> aux_2 = product_matrix(kalm_sys.cov_estimate, trans(sys.mat_sortie));

	kalm_sys.kalman_gain = product_matrix(aux_2, conj(innov_covariance));
	/*_RPT0(0, "KALMAN GAIN : \n");
	printMatrix(kalm_sys.kalman_gain);*/
	/****************************************/
	/* ^Xk = ^Xk-1 +  Kk*Yk					*/
	/****************************************/
	matrix<double> aux_3 = product_matrix(kalm_sys.kalman_gain, innov_measure);

	kalm_sys.predict_vector = kalm_sys.predict_vector + aux_3;
	/*_RPT0(0, "PREDICT VECTOR : \n");
	printMatrix(kalm_sys.predict_vector);*/
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

/** \brief	Filtre de Kalman cas de la rotation - algorithme de D. Comotti , M. Ermidoro
Les états sont les paramètres d'un quaternion
Les valeurs à comparer viennent de l'accéléromètre
Le résultat du filtre est sortie sous forme quarternion<double>

*	\param	vect4D				v_angulaire		--	vitesse angulaire vient du gyroscope
\param	vect4D				acceleration	--	accélération selon 3 axes vient de l'accéléromètre
\param	double				dt				--	durée entre deux échantillons
\param	Kalman				rotation		--	système à appliquer le filtre

*	\return quaternion<double>	result_estimate	--	un quaternion après avoir estimé

*	\test	test_Kalman_rotation	à tester les constants pour valider l'algorithme!

*/

quaternion<double> kalman_rotation(vect4D v_angulaire, vect4D acceleration, vect4D magnetic, vect4D orientation, double dt, Kalman &rotation){

	/********************************************************************/
	/*	Déclaration des matrices du Kalman : A, B, C, Q, R				*
	*	Mise à jour de la matrice de transition A à chaque nouvel appel	*/
	/********************************************************************/
	matrix<double> A(4, 4, 0), B(0, 0, 0), C(4, 4, 0), Q(4, 4, 0), R(4, 4, 0);
	C(0, 0) = C(1, 1) = C(2, 2) = C(3, 3) = 1;
	R(0, 0) = 0.99;
	R(1, 1) = 1.52;
	R(2, 2) = 0.05;
	R(3, 3) = 0.05;
	double wx, wy, wz, Ax, Ay, Az;
	wx = v_angulaire.x;
	wy = v_angulaire.y;
	wz = v_angulaire.z;

	Ax = 0.5*wx*dt;
	Ay = 0.5*wy*dt;
	Az = 0.5*wz*dt;
	A(0, 0) = 1;
	A(0, 0) = 1; A(1, 1) = 1; A(2, 2) = 1; A(3, 3) = 1; //diagonale
	A(0, 1) = -Ax; A(0, 2) = -Ay; A(0, 3) = -Az;
	A(1, 0) = Ax; A(1, 2) = Az; A(1, 3) = -Ay;
	A(2, 0) = Ay; A(2, 1) = -Az; A(2, 3) = Ax;
	A(3, 0) = Az; A(3, 1) = Ay; A(3, 2) = -Ax;

	rotation.declare_system(A, B, C);
	rotation.declare_noise(Q, R);


	/********************************************************************/
	/*	Etape update les données du filtre de Kalman					*
	/********************************************************************/

	rotation.predict_step(zero_matrix<double>(4, 4));
	/********************************************************************/
	/*	Récupération des accélérations pour l'observation du système	*
	*	Passage de l'accélération en angles d'Euler						*/
	/********************************************************************/
	vect3D angle_meas{ 0, 0, 0 };

	quaternion<double> quat_meas;

	angle_meas.x = orientation.x;
	angle_meas.y = orientation.y;
	angle_meas.z = orientation.z;
	quat_meas = anglesToQuat(angle_meas.x, angle_meas.y, angle_meas.z);
	/********************************************************************/

	/********************************************************************/
	/*	Passage du quaternion en vecteur								*
	*	Etape update les données du filtre de Kalman					*
	*	Passage du vecteur en quaternion retourner le resultat			*/
	/********************************************************************/
	matrix<double> mesuresQuat(4, 1, 0), estimate_result(4, 1, 0);

	vect3D measure_estimate;
	mesuresQuat(0, 0) = quat_meas.R_component_1();
	mesuresQuat(1, 0) = quat_meas.R_component_2();
	mesuresQuat(2, 0) = quat_meas.R_component_3();
	mesuresQuat(3, 0) = quat_meas.R_component_4();

	estimate_result = rotation.update_step(mesuresQuat);
	double e1, e2, e3, e4;
	double norm;
	e1 = estimate_result(0, 0);
	e2 = estimate_result(1, 0);
	e3 = estimate_result(2, 0);
	e4 = estimate_result(3, 0);
	norm = sqrt(pow(e1, 2) + pow(e2, 2) + pow(e3, 2) + pow(e4, 2));
	quaternion<double> quat_estimate(e1 / norm, e2 / norm, e3 / norm, e4 / norm);
	return quat_estimate;
	/********************************************************************/
}