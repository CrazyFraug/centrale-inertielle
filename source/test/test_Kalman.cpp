#include "Kalman.h"
#include "Structure.h"
#include "Quaternion.h"

/***************************************************************************************
	Test du filtre de Kalman cas de la rotation - algorithme de D. Comotti , M. Ermidoro
	Les états sont les paramètres d'un quaternion
	Les valeurs à comparer viennent de l'accéléromètre
	Le résultat du filtre est stocké dans la variable measure_estimate matrix<double>
	À évaluer : les bruits de mesure de l'accéléromètre - matrice R
			  : l'initialisation de la matrice de covariance			
*****************************************************************************************/
quaternion<double> test_kalman_rotation(double v_angulaire[3], double acc_trans[3], double dt){
	/* Initialisation du filtre de Kalman + Système */
	matrix<double> mat_cov(4, 4);
	for (int i=0; i < 4; i++)
		mat_cov(i, i) = 0.5;
	Kalman rotation(0, 4, 4, 100, mat_cov);

	/* Affectation les données pour le filtre de Kalman + Système */
	double sx, sy, sz; 
	sx = sy = sz = 0.5;
	matrix<double> A(4,4), B(0,0), C(4,4), Q(4,4), R(4,4);
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
	wx = v_angulaire[0];
	wy = v_angulaire[1];
	wz = v_angulaire[2];

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

	rotation.declare_system(A, B, C);
	rotation.declare_noise(Q, R);


	/* Etape de prédiction du système */
	matrix<double> mat_null(4, 4, 0);
	rotation.predict_step(mat_null);

	matrix<double> angle_meas(1,3) ,mat_meas(1, 4);
	angle_meas(0, 0) = pow(acc_trans[0] / (pow(acc_trans[0], 2) + pow(acc_trans[2], 2)), 2);
	angle_meas(0, 1) = pow(acc_trans[1] / (pow(acc_trans[1], 2) + pow(acc_trans[2], 2)), 2);
	angle_meas(0, 2) = 0;

	quaternion<double> quat_meas;
	
	quat_meas = anglesToQuat(angle_meas(0, 0), angle_meas(0, 1), angle_meas(0, 2));

	matrix<double> mesuresQuat(4, 1), estimate_result(4, 1, 0);

	vect3D measure_estimate;
	mesuresQuat(0, 0) = quat_meas.R_component_1();
	mesuresQuat(1, 0) = quat_meas.R_component_2();
	mesuresQuat(2, 0) = quat_meas.R_component_3();
	mesuresQuat(3, 0) = quat_meas.R_component_4();

	/* Etape update les données du filtre de Kalman */
	estimate_result = rotation.update_step(mesuresQuat);

	quaternion<double> quat_estimate(estimate_result(0, 0), estimate_result(1, 0), estimate_result(2, 0), estimate_result(3, 0));
	return quat_estimate;
}

//
///**
//*  Initialisation des matrices pour le filtre kalman
//*  \param      sx double   vitesse angulaire selon l'axe Ax
//*  \param      sy double   vitesse angulaire selon l'axe Ay
//*  \param      sz double   vitesse angulaire selon l'axe Az
//*
//*/
//void Kalman::initMatrices(double sx, double sy, double sz)
//{
//	double sx2, sy2, sz2;
//	sx2 = pow(sx, 2);
//	sy2 = pow(sy, 2);
//	sz2 = pow(sz, 2);
//
//	kalm_sys.cov_cmde(0, 0) = sx2 + sy2 + sz2; kalm_sys.cov_cmde(1, 1) = sx2 + sy2 + sz2; kalm_sys.cov_cmde(2, 2) = sx2 + sy2 + sz2; kalm_sys.cov_cmde(3, 3) = sx2 + sy2 + sz2; //diagonale
//	kalm_sys.cov_cmde(0, 1) = -sx2 + sy2 - sz2; kalm_sys.cov_cmde(0, 2) = -sx2 - sy2 + sz2; kalm_sys.cov_cmde(0, 3) = sx2 - sy2 - sz2;
//	kalm_sys.cov_cmde(1, 0) = -sx2 + sy2 - sz2; kalm_sys.cov_cmde(1, 2) = sx2 - sy2 - sz2; kalm_sys.cov_cmde(1, 3) = -sx2 - sy2 + sz2;
//	kalm_sys.cov_cmde(2, 0) = -sx2 - sy2 + sz2; kalm_sys.cov_cmde(2, 1) = sx2 - sy2 - sz2; kalm_sys.cov_cmde(2, 3) = -sx2 + sy2 - sz2;
//	kalm_sys.cov_cmde(3, 0) = sx2 - sy2 - sz2; kalm_sys.cov_cmde(3, 1) = -sx2 - sy2 + sz2; kalm_sys.cov_cmde(3, 2) = -sx2 + sy2 - sz2;
//
//	kalm_sys.cov_mesure(0, 0) = 0.05;
//	kalm_sys.cov_mesure(1, 1) = 0.05;
//	kalm_sys.cov_mesure(2, 2) = 0.05;
//	kalm_sys.cov_mesure(3, 3) = 0.05;
//
//
//	sys.mat_sortie(0, 0) = 1;
//	sys.mat_sortie(1, 1) = 1;
//	sys.mat_sortie(2, 2) = 1;
//	sys.mat_sortie(3, 3) = 1;
//
//}

//
///**
//*  Traitement avec le filtre Kalman
//*  \param      mesures[3]   vecteur de 3 mesures de vitesse angulaire selon l'axe Ax, Ay, Az
//*  \return measure_estimate vecteur de 3 angles de rotation selon l'axe Ax, Ay, Az estimés
//*
//*/
//
//double* Kalman::initSystem(double mesures[3], double dt) {
//
//	double* estimation;
//	double anglex, angley, anglez, wx, wy, wz, Ax, Ay, Az;
//	//CQRQuaternionHandle quat_meas, quat_estimate;
//
//	estimation = new double[3];
//
//	//CQRCreateEmptyQuaternion(&quat_meas);
//	//CQRCreateEmptyQuaternion(&quat_estimate);
//
//	wx = mesures[0];
//	wy = mesures[1];
//	wz = mesures[2];
//	/* Mettre à jour la matrice de transition A */
//	Ax = 0.5*wx*dt;
//	Ay = 0.5*wy*dt;
//	Az = 0.5*wz*dt;
//
//	sys.mat_transition(0, 0) = 1;
//	sys.mat_transition(0, 0) = 1; sys.mat_transition(1, 1) = 1; sys.mat_transition(2, 2) = 1; sys.mat_transition(3, 3) = 1; //diagonale
//	sys.mat_transition(0, 1) = -Ax; sys.mat_transition(0, 2) = -Ay; sys.mat_transition(0, 3) = -Az;
//	sys.mat_transition(1, 0) = Ax; sys.mat_transition(1, 2) = Az; sys.mat_transition(1, 3) = -Ay;
//	sys.mat_transition(2, 0) = Ay; sys.mat_transition(2, 1) = -Az; sys.mat_transition(2, 3) = Ax;
//	sys.mat_transition(3, 0) = Az; sys.mat_transition(3, 1) = Ay; sys.mat_transition(3, 2) = -Ax;
//
//	/* Mettre à jour le système à filtrer */
//	//filtre.declare_system(A, B, C);
//	//filtre.declare_noise(Q, R);
//
//	/* Etape prédiction du filtre Kalman */
//	predict_step(B);
//
//
//	/* Convertir les données mesurées(angles) en quaternion (type matrix) */
//	anglex = wx*dt;
//	angley = wy*dt;
//	anglez = wz*dt;
//
//	CQRAngles2Quaternion(quat_meas, anglex, angley, anglez);
//
//	matrix<double> mesuresQuat(4, 1);
//
//	mesuresQuat(0, 0) = quat_meas->w;
//	mesuresQuat(1, 0) = quat_meas->x;
//	mesuresQuat(2, 0) = quat_meas->y;
//	mesuresQuat(3, 0) = quat_meas->z;
//
//	matrix<double> estimate_result(4, 1, 0);
//
//	/* Etape update l'estimation */
//	estimate_result = filtre.update_step(mesuresQuat);
//
//	quat_estimate->w = estimate_result(0, 0);
//	quat_estimate->x = estimate_result(1, 0);
//	quat_estimate->y = estimate_result(2, 0);
//	quat_estimate->z = estimate_result(3, 0);
//
//	/* Convertir les données estimées (quaternion) en angles pour l'affichage */
//	matrix<double> measure_estimate(3, 1);
//	CQRQuaternion2Angles(&measure_estimate(0, 1), &measure_estimate(1, 1), &measure_estimate(2, 1), quat_estimate);
//
//	return estimation;
//}