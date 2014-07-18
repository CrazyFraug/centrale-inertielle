#include "Traitement.h"


/**
 *  Déclaration les données pour le traitement
 *  \param 	sx double   vitesse angulaire selon l'axe Ax
 *  \param 	sy double   vitesse angulaire selon l'axe Ay
 *  \param 	sz double   vitesse angulaire selon l'axe Az
 *
*/
Traitement::Traitement(double sx, double sy, double sz, matrix_double init_cov_estimate)
{
	double sx2, sy2, sz2;
	sx2 = pow(sx,2);
	sy2 = pow(sy,2);
	sz2 = pow(sz,2);

	filtre = kalman(0,4,4,100, init_cov_estimate);
	A.mat.get_matrix(4,4);
	B.mat.get_matrix(0,0,0);
	Q.mat.get_matrix(4,4);
	R.mat.get_matrix(4,4,0);
	C.mat.get_matrix(4,4,0);

	Q.mat(0,0) = sx2 + sy2 + sz2; Q.mat(1,1) = sx2 + sy2 + sz2; Q.mat(2,2) = sx2 + sy2 + sz2; Q.mat(3,3) = sx2 + sy2 + sz2; //diagonale
	Q.mat(0,1) = -sx2 + sy2 - sz2; Q.mat(0,2) = -sx2 -sy2 + sz2; Q.mat(0,3) = sx2 - sy2 - sz2;
	Q.mat(1,0) = -sx2 + sy2 - sz2; Q.mat(1,2) = sx2 - sy2 - sz2; Q.mat(1,3) = -sx2 - sy2 + sz2;
	Q.mat(2,0) = -sx2 -sy2 + sz2; Q.mat(2,1) = sx2 - sy2 - sz2; Q.mat(2,3) = -sx2 + sy2 - sz2;
	Q.mat(3,0) = sx2 - sy2 - sz2; Q.mat(3,1) = -sx2 - sy2 + sz2; Q.mat(3,2) = -sx2 + sy2 - sz2;

	R.mat(0,0) = 0.05;
	R.mat(1,1) = 0.05;
	R.mat(2,2) = 0.05;
	R.mat(3,3) = 0.05;


	C.mat(0,0) = 1;
	C.mat(1,1) = 1;
	C.mat(2,2) = 1;
	C.mat(3,3) = 1;

}
/**
 *  Traitement avec le filtre Kalman
 *  \param 	mesures[3]   vecteur de 3 mesures de vitesse angulaire selon l'axe Ax, Ay, Az
 *  \return measure_estimate vecteur de 3 angles de rotation selon l'axe Ax, Ay, Az estimés
 *
*/

matrix_type Traitement::initSystem(double* mesures[3]) {

	double anglex, angley, anglez, wx,wy,wz,dt, Ax, Ay, Az;
	CQRQuaternionHandle *quat_meas, *quat_estimate;
	wx = mesures[0];
	wy = mesures[1];
	wz = mesures[2];
	/* Mettre à jour la matrice de transition A */
	Ax = 0.5*wx*dt;
	Ay = 0.5*wy*dt;
	Az = 0.5*wz*dt;

	A.mat(0,0) = 1; A.mat(1,1) = 1; A.mat(2,2) = 1; A.mat(3,3) = 1; //diagonale
	A.mat(0,1) = -Ax; A.mat(0,2) = -Ay; A.mat(0,3) = -Az;
	A.mat(1,0) = Ax; A.mat(1,2) = Az; A.mat(1,3) = -Ay;
	A.mat(2,0) = Ay; A.mat(2,1) = -Az; A.mat(2,3) = Ax;
	A.mat(3,0) = Az; A.mat(3,1) = Ay; A.mat(3,2) = -Ax;

	/* Mettre à jour le système à filtrer */
	filtre.declare_system(A, B, C);
	filtre.declare_noise(Q, R);

	/* Etape prédiction du filtre Kalman */
	filtre.predict_step(B);


	/* Convertir les données mesurées(angles) en quaternion (type matrix) */
	anglex = wx*dt;
	angley = wy*dt;
	anglez = wz*dt;

	CQRAngles2Quaternion (quat_meas, anglex, angley, anglez );

	matrix_type mesuresQuat;
	mesuresQuat = get_matrix(4,1,0);
	mesuresQuat[0] = quatrotation.w;
	mesuresQuat[1] = quatrotation.x;
	mesuresQuat[2] = quatrotation.y;
	mesuresQuat[3] = quatrotation.z;

	matrix_type estimate_result;

	/* Etape update l'estimation */
	estimate_result = filtre.update_step(mesuresQuat);

	quat_estimate.w = estimate_result.mat(0,0);
	quat_estimate.x = estimate_result.mat(1,0);
	quat_estimate.y = estimate_result.mat(2,0);
	quat_estimate.z = estimate_result.mat(3,0);

	/* Convertir les données estimées (quaternion) en angles pour l'affichage */
	matrix_type measure_estimate;
	measure_estimate.get_matrix(3,1);
	CQRQuaternion2Angles(measure_estimate.mat(0,1), measure_estimate.mat(1,1), measure_estimate.mat(2,1), quat_estimate);

	return measure_estimate;
}
