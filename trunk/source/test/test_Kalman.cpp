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
quaternion<double> test_kalman_rotation(vect4D v_angulaire, vect4D acceleration, double dt){
	/* Initialisation du filtre de Kalman + Système */
	matrix<double> mat_cov(4, 4,0), init_predict(4,1,0);
	for (int i=0; i < 4; i++)
		mat_cov(i, i) = 1.5;
	for (int i = 0; i < 4; i++)
		init_predict(i, 0) = 0.05;
	Kalman rotation(0, 4, 4, 100, init_predict, mat_cov);
	/* Affectation les données pour le filtre de Kalman + Système */
	double sx, sy, sz; 
	sx = sy = sz = 0.5;
	matrix<double> A(4,4,0), B(0,0,0), C(4,4,0), Q(4,4,0), R(4,4,0);
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

	rotation.declare_system(A, B, C);
	rotation.declare_noise(Q, R);

	/* Etape de prédiction du système */
	rotation.predict_step(zero_matrix<double>(4,4));

	/*	
	 *	Récupération des accélérations pour l'observation du système 
	 *	Passage de l'accélération en angles d'Euler
	 */
	vect3D angle_meas;
	angle_meas.x = atan(pow(acceleration.x/ (pow(acceleration.x, 2) + pow(acceleration.y, 2)), 2))*180/(atan(1)*4);
	angle_meas.y= 0;
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
	estimate_result = rotation.update_step(mesuresQuat);

	quaternion<double> quat_estimate(estimate_result(0, 0), estimate_result(1, 0), estimate_result(2, 0), estimate_result(3, 0));
	return quat_estimate;
}

//int main(){
//	Serial link("COM8", 115200);
//	Instrument acce("acce", &link);
//	Instrument gyro("gyro", &link);
//
//	Traitement trait_acce(&acce);
//	Traitement trait_gyro(&gyro);
//
//	quaternion<double> quat_test;
//	std::string filename_test = "test_kalman.txt", file_sensor_name = "test_sensor.txt", file_excel = "excel.csv";
//	std::fstream filetest, filesensor, fileexcel;
//	filetest.open(filename_test,std::ios::app);
//	filesensor.open(file_sensor_name);
//	filetest << "|| Angles X  || Angle Y || Angle Z  || \n";
//	filesensor << "|| Angles X  || Angle Y || Angle Z  || \n";
//	filetest.close();
//	filesensor.close();
//	int turn = 1;
//	vect3D angles_test;
//	vect3D angles_sensor;
//	matrix<double> mat_test(4, 1, 0);
//	trait_gyro.writeHeading(gyro.getnomfichier());
//	trait_acce.writeHeading(acce.getnomfichier());
//	_RPT0(0, "DECLARATION VARIABLE OK \n");
//	while (1){
//		trait_acce.stockerValeurs();
//		trait_gyro.stockerValeurs();
//		vect4D v_angulaire, acceleration;
//
//		trait_acce.filefromSensor(acce.getnomfichier(), &acce);
//		trait_gyro.filefromSensor(gyro.getnomfichier(), &gyro);
//
//		if (trait_gyro.tabFull() == true)
//		{
//			fileexcel.open(file_excel, std::ios::app);
//			filetest.open(filename_test,std::ios::app);
//			filesensor.open(file_sensor_name,std::ios::app);
//			angles_sensor = trait_gyro.calculerAngle_deg();
//			filesensor << "||  " << angles_sensor.x << "  ||  " << angles_sensor.y << "  ||  " << angles_sensor.z << "  ||\n";
//			v_angulaire = trait_gyro.readDatafromFile(gyro.getnomfichier(), turn);
//			acceleration = trait_acce.readDatafromFile(acce.getnomfichier(), turn);
//			
//			quat_test = test_kalman_rotation(v_angulaire, acceleration, trait_gyro.get_dt());
//			angles_test = quatToAngles(quat_test);
//			filetest << "||  ;" << angles_test.x << "  || ; " << angles_test.y << "  || ; " << angles_test.z << "  ||;\n";
//			fileexcel << acceleration.temps << ";";
//			fileexcel << angles_sensor.x << ";" << angles_test.x << ";";
//			fileexcel << angles_sensor.y << ";" << angles_test.y << ";";
//			fileexcel << angles_sensor.z << ";" << angles_test.z << "\n";
//			filetest.close();
//			filesensor.close();	
//			fileexcel.close();
//		}
//		else{ 
//			_RPT0(0, " FALSE \n");
//		}
//		turn++;
//	}
//
//}

