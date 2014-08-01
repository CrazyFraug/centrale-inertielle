#include "Kalman.h"
#include "Structure.h"
#include "Quaternion.h"
#include "Traitement.h"
#include <iostream>

/***************************************************************************************
Test du filtre de Kalman cas de la rotation - algorithme de D. Comotti , M. Ermidoro
Les états sont les paramètres d'un quaternion
Les valeurs à comparer viennent de l'accéléromètre
Le résultat du filtre est sortie sous forme quarternion<double>
À évaluer : les bruits de mesure de l'accéléromètre - matrice R
: l'initialisation de la matrice de covariance
*****************************************************************************************/
quaternion<double> test_kalman_rotation(vect4D v_angulaire, vect4D acceleration, double dt, Kalman kalm){

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

	kalm.declare_system(A, B, C);
	kalm.declare_noise(Q, R);

	/* Etape de prédiction du système */
	kalm.predict_step(zero_matrix<double>(4, 4));

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
	estimate_result = kalm.update_step(mesuresQuat);

	quaternion<double> quat_estimate(estimate_result(0, 0), estimate_result(1, 0), estimate_result(2, 0), estimate_result(3, 0));
	return quat_estimate;
}

int main(){
	Serial link("COM8", 115200);
	Instrument_serie acce("acce", &link);
	Instrument_serie gyro("gyro", &link);

	Traitement trait_acce(&acce);
	Traitement trait_gyro(&gyro);

	quaternion<double> quat_test;
	std::string file_excel = "excel.csv";
	std::fstream fileexcel, file_gyro, file_acce;

	fileexcel.open(file_excel, std::ios::app);
	fileexcel << "temps;angleX_sensor;angleX_Kalman;angleY_sensor;angleY_Kalman;angleZ_sensor;angleZ_Kalman \n";
	int turn = 0;
	vect3D angles_test;
	vect3D angles_sensor = { 0, 0, 0 };
	matrix<double> mat_test(4, 1, 0);
	//trait_gyro.writeHeading(gyro.getnomfichier());
	//trait_acce.writeHeading(acce.getnomfichier());
	_RPT0(0, "DECLARATION VARIABLE OK \n");

	/* Initialisation du filtre de Kalman + Système */
	matrix<double> mat_cov(4, 4, 0), init_predict(4, 1, 0);
	for (int i = 0; i < 4; i++)
		mat_cov(i, i) = 1.3;
	for (int i = 0; i < 4; i++)
		init_predict(i, 0) = 0.05;
	Kalman rotation(0, 4, 4, 100, init_predict, mat_cov);

	while (1){
		trait_acce.stockerValeurs();
		trait_gyro.stockerValeurs();
		acce.afficherMesures();
		gyro.afficherMesures();
		vect4D v_angulaire, acceleration;

		trait_acce.filefromSensor(file_acce, "acce.csv", &acce);
		trait_gyro.filefromSensor(file_gyro, "gyro.csv", &gyro);

		if (trait_gyro.tabFull() == true)
		{
			fileexcel.open(file_excel, std::ios::app);
			angles_sensor = angles_sensor + trait_gyro.calculerAngle_deg();
			v_angulaire = gyro.getMesures();
			acceleration = acce.getMesures();

			_RPT3(0, "acceleration_t x : %f, y : %f, z : %f \n", acceleration.x, acceleration.y, acceleration.z);

			_RPT3(0, "v_angulaire_t x : %f, y : %f, z : %f \n", v_angulaire.x, v_angulaire.y, v_angulaire.z);

			quat_test = test_kalman_rotation(v_angulaire, acceleration, trait_gyro.get_dt(), rotation);
			angles_test = quatToAngles(quat_test);
			fileexcel << acceleration.temps << ";";
			fileexcel << angles_sensor.x << ";" << angles_test.x << ";";
			fileexcel << angles_sensor.y << ";" << angles_test.y << ";";
			fileexcel << angles_sensor.z << ";" << angles_test.z << "\n";
			fileexcel.close();
		}
		else{
			_RPT0(0, " FALSE \n");
		}
		turn++;
	}

}

