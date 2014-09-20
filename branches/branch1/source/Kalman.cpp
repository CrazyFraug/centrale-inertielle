#include "Kalman.h"
#include <boost/numeric/ublas/operation.hpp>

/** \brief	Produit de deux matrices

*	\param	matrix<double>	M1		--	Matrice gauche de taille m x n
\param	matrix<double>	M2		--	Matrice gauche de taille n x o

*	\return	matrix<double>	result	-- R�sultat du produit de taille m x o (cas normal) || Message d'erreur si la taille des matrices ne convient pas
*/
matrix<double> product_matrix(matrix<double> M1, matrix<double> M2){
	if (M1.size2() != M2.size1()){
		matrix<double> result(M1.size1(), M2.size2(), 0);
		_RPT0(_CRT_ERROR, "ERROR SIZE MATRIX \n");
		return result;
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


/** \brief Constructor d'un objet Kalman

\param	int				nb_in				--	Nombre d'entr�es du syst�me
\param	int				nb_out				--	Nombre de sorties du syst�me
\param	int				nb_state			--	Nombre de variables d'�tat du syst�me
\param	int				nb_step				--	Nombre d'�chantillon d'initialisation du syst�me
\param	matrix<double>	init_predict		--	Vector d'initialisation du vecteur d'�tat
\param	matrix<double>	init_cov_estimate	--	Matrice d'initialisation de la matrice de covariance
*/
Kalman::Kalman(int nb_in, int nb_out, int nb_state, int step, matrix<double> init_predict, matrix<double> init_cov_estimate)
{
	/********************************************************************/
	/* Initialisation des matrices de covariance et de prediction       */
	/********************************************************************/
	//kalm_sys.predict_vector = matrix<double> (nb_state,1,0);
	//kalm_sys.cov_estimate = matrix<double> (nb_state, nb_state,0);
	//for (int i = 0; i < nb_state; i++){
	//	(kalm_sys.predict_vector)(i, i) = 1;
	//	(kalm_sys.cov_estimate)(i, 0) = 1;
	//}
	kalm_sys.predict_vector = init_predict;
	kalm_sys.cov_estimate = init_cov_estimate;

	/********************************************************************/
	/* Affectation les nombres d'entr�es, sorties et d'�tat du syst�me	*
	*  D�claration le nombre d'�chantillon d'initialisation				*/
	/********************************************************************/
	sys.size_in = nb_in;
	sys.size_out = nb_out;
	sys.size_state = nb_state;
	kalm_sys.nb_step = step;

	/********************************************************************/
	/* Affectation les matrices aux tailles du syst�me					*/
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
	//kalm_sys.cov_estimate = init_cov_estimate;

	kalm_sys.predict_vector.resize(sys.size_state, 1);
	//kalm_sys.predict_vector = init_predict;
	kalm_sys.kalman_gain.resize(sys.size_state, sys.size_state);

}

/** \brief	Destructor
*/
Kalman::~Kalman()
{
}

/** \brief	Fonction d�clare le syst�me sur lequel on veut travailler/filtrer

*	\param	matrix<double>	A	--	matrice de transition, matrice carr�e de taille = nombre d'�tats du syst�me
*	\param	matrix<double>	B	--	matrice de commande, matrice de taille : nombre d'�tats du syst�me x nombre d'entr�es
*	\param	matrix<double>	C	--	matrice de sortie, matrice de taille : nombre de sortie x nombre d'�tats du syst�me

*	\return void
*/
void Kalman::declare_system(matrix<double> A, matrix<double> B, matrix<double> C){
	sys.mat_transition = A;
	sys.mat_cmde = B;
	sys.mat_sortie = C;
}

/** \brief	Fonction d�clare les matrices de covariance des bruits de commande et de mesure du syst�me

*	\param	matrix<double>	Q	-- matrice de covariance de commande, matrice carr�e de taille = nombre d'�tats du syst�me
\param	matrix<double>	R	-- matrice de covariance de mesure, matrice de taille : nombre de sortie x nombre d'�tats du syst�me

*	\return void
*/
void Kalman::declare_noise(matrix<double> Q, matrix<double> R){
	kalm_sys.cov_cmde = Q;

	kalm_sys.cov_mesure = R;
}

/** \brief Etape pr�diction du filtre de Kalman � partir des donn�es de la commande

*	\param	matrix<double>	value_cmd	--	matrice des valeurs de la commande, matrice de taille : nombre d'�tats du syst�me x nombre d'entr�es
*	\return void

*	\test	test_Kalman	inclus dans le test_Kalman_rotation
*/
void Kalman::predict_step(matrix<double> value_cmd)
{
	/****************************************/
	/* ^Xk = A*Xk-1 + B*u	(cas normal)	*
	* ^Xk = A*Xk-1	(cas non entr�e)		*/
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
	if (sys.size_in != 0)
		kalm_sys.cov_estimate = aux_2 + kalm_sys.cov_cmde;
}

/** \brief Etape mettre � jour les pr�dictions d'�tat et de covariance du filtre de Kalman gr�ce aux valeurs mesur�es

*	\param	matrix<double>	value_measure			--	 matrice des valeurs mesur�es, matrice de taille : nombre de sortie x nombre d'�tats du syst�me
*	\return	matrix<double>	kalm_sys.predict_vector	--	matrice des valeurs estim�es

*	\test	test_Kalman		inclus dans le test_Kalman_rotation
*/
matrix<double> Kalman::update_step(matrix<double> value_measure){
	/****************************************/
	/* Hk*Xk-1								*/
	/****************************************/
	matrix<double> predict_measure = product_matrix(sys.mat_sortie, kalm_sys.predict_vector);

	/****************************************/
	/* Yk = Zk - Hk*Xk-1					*/
	/****************************************/
	matrix<double> innov_measure = value_measure - predict_measure;

	/****************************************/
	/* Hk*Pk-1*trans(Hk)					*
	* Sk = Hk*Pk-1*trans(Hk) + Rk			*/
	/****************************************/
	matrix<double> aux;
	matrix<double> innov_covariance;
	aux = product_matrix(sys.mat_sortie, kalm_sys.cov_estimate);
	aux = product_matrix(aux, trans(sys.mat_sortie));

	innov_covariance = aux + kalm_sys.cov_mesure;

	/****************************************/
	/* Kk = Pk-1*trans(Hk)*Sk^-1			*/
	/****************************************/
	matrix<double> aux_2 = product_matrix(kalm_sys.cov_estimate, trans(sys.mat_sortie));

	kalm_sys.kalman_gain = product_matrix(aux_2, conj(innov_covariance));

	/****************************************/
	/* ^Xk = ^Xk-1 +  Kk*Yk					*/
	/****************************************/
	matrix<double> aux_3 = product_matrix(kalm_sys.kalman_gain, innov_measure);

	kalm_sys.predict_vector = kalm_sys.predict_vector + aux_3;

	/****************************************/
	/* Pk = (I - Kk*Hk)*Pk-1				*/
	/****************************************/
	matrix<double> aux_4 = product_matrix(kalm_sys.kalman_gain, sys.mat_sortie);
	aux_4 = kalm_sys.matrix_ident - aux_4;

	kalm_sys.cov_estimate = product_matrix(aux_4, kalm_sys.cov_estimate);
	return kalm_sys.predict_vector;
}

/** \brief	Filtre de Kalman cas de la rotation - algorithme de D. Comotti , M. Ermidoro
Les �tats sont les param�tres d'un quaternion
Les valeurs � comparer viennent de l'acc�l�rom�tre
Le r�sultat du filtre est sortie sous forme quarternion<double>

*	\param	vect4D				v_angulaire		--	vitesse angulaire vient du gyroscope
\param	vect4D				acceleration	--	acc�l�ration selon 3 axes vient de l'acc�l�rom�tre
\param	double				dt				--	dur�e entre deux �chantillons
\param	Kalman				rotation		--	syst�me � appliquer le filtre

*	\return quaternion<double>	result_estimate	--	un quaternion apr�s avoir estim�

*	\test	test_Kalman_rotation	� tester les constants pour valider l'algorithme!
*/
quaternion<double> Kalman::kalman_rotation(vect4D v_angulaire, vect4D acceleration, vect4D magnetic, vect4D orientation, double dt, Kalman rotation){
	/********************************************************************/
	/*	D�finition les matrices A, B, C, Q, R							*
	*	Affectation les donn�es pour le filtre de Kalman + Syst�me		*/
	/********************************************************************/
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

	/********************************************************************/
	/* Mettre � jour la matrice de transition A							*/
	/********************************************************************/
	double wx, wy, wz, Ax, Ay, Az;
	wx = -v_angulaire.x;
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

	/*******************************************************************/


	/********************************************************************/
	/* D�clare le syst�me avec les matrices A, B, C, Q et R				*/
	/********************************************************************/
	rotation.declare_system(A, B, C);

	rotation.declare_noise(Q, R);
	/********************************************************************/


	/********************************************************************/
	/* Etape de pr�diction du syst�me									*/
	/********************************************************************/
	rotation.predict_step(zero_matrix<double>(4, 4));
	/********************************************************************/

	/********************************************************************/
	/*	R�cup�ration des acc�l�rations pour l'observation du syst�me	*
	*	Passage de l'acc�l�ration en angles d'Euler						*/
	/********************************************************************/
	vect3D angle_meas = { 0, 0, 0 };
	double ax, ay, az, mx, my, mz;
	ax = acceleration.x;
	ay = -acceleration.y;
	az = -acceleration.z;
	mx = magnetic.x;
	my = magnetic.y;
	mz = magnetic.z;
	quaternion<double> quat_meas;
	angle_meas.x = atan2(ay, az) * 180 / (atan(1) * 4);
	if ((ay*sin(angle_meas.x) + az*cos(angle_meas.x)) == 0){
		if (ax > 0){
			angle_meas.y = 90;
		}
		else{
			angle_meas.y = -90;
		}
	}
	else{
		angle_meas.y = atan2(-ay, (ay*sin(angle_meas.x) + az*cos(angle_meas.x))) * 180 / (atan(1) * 4);
	}
	angle_meas.z = atan2(mz*sin(angle_meas.x) - my*cos(angle_meas.x), mx*cos(angle_meas.y) + my*sin(angle_meas.y)*sin(angle_meas.x) + mz*sin(angle_meas.y)*cos(angle_meas.x)) * 180 / (atan(1) * 4);//atan(mz*cos(angle_meas.x) + my*sin(angle_meas.x) / mx*cos(angle_meas.z)+mz*sin(angle_meas.z)*sin(angle_meas.x)+my*sin(angle_meas.z)*cos(angle_meas.x))*180/(atan(1)*4);


	quat_meas = danglesToQuat(angle_meas.x, angle_meas.y, angle_meas.z);
	/********************************************************************/

	/********************************************************************/
	/*	Passage du quaternion en vecteur								*
	*	Etape update les donn�es du filtre de Kalman					*
	*	Passage du vecteur en quaternion retourner le resultat			*/
	/********************************************************************/
	matrix<double> mesuresQuat(4, 1, 0), estimate_result(4, 1, 0);

	vect3D measure_estimate;
	mesuresQuat(0, 0) = quat_meas.R_component_1();
	mesuresQuat(1, 0) = quat_meas.R_component_2();
	mesuresQuat(2, 0) = quat_meas.R_component_3();
	mesuresQuat(3, 0) = quat_meas.R_component_4();

	estimate_result = rotation.update_step(mesuresQuat);

	quaternion<double> quat_estimate(estimate_result(0, 0), estimate_result(1, 0), estimate_result(2, 0), estimate_result(3, 0));
	return quat_estimate;
	/********************************************************************/
}