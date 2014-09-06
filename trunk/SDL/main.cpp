#include "SceneOpenGL.h"
#include "Tools.h"

int main(int argc, char *argv[]) {

	/* Déclaration des variables locaux */
	int _mode = 0;		// Mode de simulation -- affecter dans la partie déclaration des instruments
	int _step = 0;		// 
	double _wx, _wy, _wz, _Ax, _Ay, _Az, _dt;
	vect4D _vAngulaire_t, _acceleration_t, _magnetic_t, _orientation_t, _vAngulaire_t_deg;
	vect4D _initKalman = { 0, 0, 0, 0 };
	vect4D _vAngulaireInit = { 0, 0, 0, 0 };
	vect3D	angles;		// Les 3 angles de résultats
	vect3D	orientationData;	// Orientations lits du capteur 

	bool _headingResult(false);		// Variable pour le test du heading du fichier de résultat
	bool terminer(false);		// Indique la fin de la simulation

	quaternion<double> _quatResult;		// Quaternion contenant le résultat
	quaternion<double> _quatMeasure;		// Quaternion d'observation

	matrix<double> _matObs, _matResult, _matCmde;	// Les matrices contenant les valeurs d'observation, du résultat et de la commande

	/* L'objet de OpenGL pour l'affichage */
	SceneOpenGL scene("letitre", 800, 600);

	/* Parametre du Serial d'Arduino */
	std::string port = PORTSERIE;
	int baudRate = BAUD;

	/* Déclaration des instrument et des traitements */
	_mode = choiceMode();

	Instrument accel("acce", _mode);
	Instrument gyros("gyro", _mode);
	Instrument magne("mnet", _mode);
	Instrument orient("orie", _mode);

	Traitement trait_accel(&accel);
	Traitement trait_gyros(&gyros);
	Traitement trait_magne(&magne);
	Traitement trait_orient(&orient);

	/* Initialisation du filtre de Kalman + Système */
	matrix<double> A(4, 4, 0), B(0, 0, 0), C(4, 4, 0), Q(4, 4, 0), R(4, 4, 0);
	matrix<double> mat_cov(4, 4, 0), init_predict(4, 1, 0);

	C(0, 0) = C(1, 1) = C(2, 2) = C(3, 3) = 1;
	R(0, 0) = 0.05;
	R(1, 1) = 0.05;
	R(2, 2) = 0.05;
	R(3, 3) = 0.05;


	for (int i = 0; i < 4; i++)
		mat_cov(i, i) = 0.5;
	init_predict(0, 0) = 0.7071067811865476;
	init_predict(1, 0) = 0;
	init_predict(2, 0) = -0.7071067811865476;
	init_predict(3, 0) = 0;


	Kalman une_rotation(0, 4, 4, 100);
	une_rotation.initSystem(A, B, C, Q, R, init_predict, mat_cov);


	_matCmde = zero_matrix<double>(une_rotation.getSizeIn(), 1);
	_matObs.resize(une_rotation.getSizeState(), 1);
	_matResult.resize(une_rotation.getSizeState(), 1);

	/* Déclaration des fichiers d'écriture/lecture des valeurs */
	std::fstream filegyro;
	std::fstream fileacce;
	std::fstream filemnet;
	std::fstream fileorie;

	writeHeading("acce.csv", fileacce);
	writeHeading("gyro.csv", filegyro);
	writeHeading("mnet.csv", filemnet);
	writeHeading("orie.csv", fileorie);

	/* Initialisation du SDL */
	if (scene.InitialiserFenetre() == false) { system("PAUSE"); return -1; }

	if (!scene.iniGL()) 	{ system("PAUSE"); return -1; }

	/* Boucle Principale (avec Traitement Filtre Kalman + Affichage SDL */
	while (!terminer){

		/* Récupère les mesures d'accélération, de vitesse angulaire et du magnétomètre */
		if (_mode == 1){
			trait_gyros.stockerValeurs();
			trait_accel.stockerValeurs();
			trait_magne.stockerValeurs();
			trait_orient.stockerValeurs();
			_vAngulaire_t = gyros.getMesures();
			_acceleration_t = accel.getMesures();
			_magnetic_t = magne.getMesures();
			_orientation_t = orient.getMesures();
			filefromSensor("gyro.csv", filegyro, _vAngulaire_t);
			filefromSensor("acce.csv", fileacce, _acceleration_t);
			filefromSensor("mnet.csv", filemnet, _magnetic_t);
			filefromSensor("orie.csv", fileorie, _orientation_t);
		}
		else{
			_vAngulaire_t = readDatafromFile("gyro.csv", filegyro, _step);
			_acceleration_t = readDatafromFile("acce.csv", fileacce, _step);
			_magnetic_t = readDatafromFile("mnet.csv", filemnet, _step);
			_orientation_t = readDatafromFile("orie.csv", fileorie, _step);
		}

		/* Traitement des données et le filtre de Kalman */
		_wx = _vAngulaire_t.x;
		_wy = _vAngulaire_t.y;
		_wz = _vAngulaire_t.z;
		_dt = trait_gyros.get_dt();

		_Ax = 0.5*_wx*_dt;
		_Ay = 0.5*_wy*_dt;
		_Az = 0.5*_wz*_dt;
		A(0, 0) = 1;
		A(0, 0) = 1; A(1, 1) = 1; A(2, 2) = 1; A(3, 3) = 1; //diagonale
		A(0, 1) = -_Ax; A(0, 2) = -_Ay; A(0, 3) = -_Az;
		A(1, 0) = _Ax; A(1, 2) = _Az; A(1, 3) = -_Ay;
		A(2, 0) = _Ay; A(2, 1) = -_Az; A(2, 3) = _Ax;
		A(3, 0) = _Az; A(3, 1) = _Ay; A(3, 2) = -_Ax;

		une_rotation.majSystem(true, 'A', A, B, C);

		_quatMeasure = anglesToQuat(_orientation_t.x, _orientation_t.y, _orientation_t.z);

		_matObs(0, 0) = _quatMeasure.R_component_1();
		_matObs(1, 0) = _quatMeasure.R_component_2();
		_matObs(2, 0) = _quatMeasure.R_component_3();
		_matObs(3, 0) = _quatMeasure.R_component_4();

		/* Filtre de Kalman */
		if (_step < 100){
			_matResult = kalmanTraitement(une_rotation, _matCmde, _matObs);
		}
		else{
			/*if ((v_angulaire_t.x > 0.001 || v_angulaire_t.x < -0.001)  &&
			(v_angulaire_t.y > 0.001 || v_angulaire_t.y < -0.001)  &&
			(v_angulaire_t.z > 0.001 || v_angulaire_t.z < -0.001)){
			*/
			_matResult = kalmanTraitement(une_rotation, _matCmde, _matObs);
		}
		_quatResult = quaternion<double>(_matResult(0, 0), _matResult(1, 0), _matResult(2, 0), _matResult(3, 0));
		angles = quatToAngles_deg(_quatResult);


		/* Ecriture du résultat (angles calculés avec Kalman + orientation écriture capteur) */
		orientationData.x = _orientation_t.x;
		orientationData.y = _orientation_t.y;
		orientationData.z = _orientation_t.z;


		/*	Ecriture des valeurs d'angle et d'angle_sensor dans le fichier excel	 */
		writeResult(_vAngulaire_t.temps, orientationData, angles, "result.csv", _headingResult);
		_headingResult = true;
		_step++;

		/* Affichage SDL */
		terminer = scene.bouclePrincipale(angles);
	}

	return 0;
}
