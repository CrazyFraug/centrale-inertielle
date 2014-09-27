#include "Tools.h"
#include "SceneOpenGL.h"
#include "Traitement.h"
#include "Quaternion.h"
#include "source\test\test_Kalman.cpp"
#include "source\test\test_quat.cpp"
#include "source\test\test_Tools.cpp"

#define FRAME_RATE_WITH_SENSOR	5.0
#define FRAME_RATE_WITH_FILE	30.0

#define	MODE_SIMULATION_SENSOR	1
#define	MODE_SIMULATION_FILE	2
#define	MODE_CREATE_FILE		3
#define	MODE_CREATE_FILE_SENSOR	4
#define	MODE_TEST				5

#define	TEST_KALMAN				1
#define	TEST_CONVERSION_QUAT	2
#define	TEST_CHGT_REPERE		3
#define	TEST_STRING_TO_DOUBLE	4

int main(int argc, char *argv[]) {

	/* Déclaration des variables locaux */
	int mode = 0;		// Mode de simulation -- affecter dans la partie déclaration des instruments
	int step = 1;		// Compteur d'échantillon de Simulation 
	double wx, wy, wz, Ax, Ay, Az;
	double angle;
	double dt, tPred, tAct;
	static int frameRate;		// Période de mettre à jour du SDL

	vect4D vAngulaireLocal_t, acceleration_t, magnetic_t, orientation_t, vAngulaire_t_deg;
	vect4D initKalman = { 0, 0, 0, 0 };
	vect4D vAngulaireInit = { 0, 0, 0, 0 };

	vect3D	angles;		// Les 3 angles de résultats
	vect3D	orientationData;	// Orientations lits du capteur 
	vect3D	vAngulaireGlobal;
	vect3D	axe;

	bool tabMajsys[3];			// Tableau contenant la maj du système 0->A, 1->B, 2->C
	bool terminer(false);		// Indique la fin de la simulation

	quaternion<double> quatResult;		// Quaternion contenant le résultat
	quaternion<double> quatMeasure;		// Quaternion d'observation
	quaternion<double> qBase(1, 0, 0, 0);
	quaternion<double> quatInit;		// Quaternion initialisation (déclaré avec la conversion d'angle initial)
	quaternion<double> quatBias;		// Biais de mesure convertis en quaternion

	matrix<double> matObs, matResult, matCmde;	// Les matrices contenant les valeurs d'observation, du résultat et de la commande
	matrix<double> obsInit;
	/* L'objet de OpenGL pour l'affichage */
	SceneOpenGL scene("letitre", 800, 600);

	/* Parametre du Serial d'Arduino */
	std::string port = PORTSERIE;
	int baudRate = BAUD;

	/* Déclaration des instrument et des traitements */
	mode = choiceMode();

	Instrument accel("acce", mode);
	Instrument gyros("gyro", mode);
	Instrument magne("mnet", mode);
	Instrument orient("orie", mode);

	Traitement trait_accel(&accel);
	Traitement trait_gyros(&gyros);
	Traitement trait_magne(&magne);
	Traitement trait_orient(&orient);

	/* Initialisation du filtre de Kalman + système */
	matrix<double> A(4, 4, 0), B(0, 0, 0), C(4, 4, 0), Q(4, 4, 0), R(4, 4, 0);
	matrix<double> mat_cov(4, 4, 0), init_predict(4, 1, 0);

	C(0, 0) = C(1, 1) = C(2, 2) = C(3, 3) = 1;

	quatBias = anglesToQuat(0.02, 0.01, 0);

	R(0, 0) = quatBias.R_component_1();
	R(1, 1) = quatBias.R_component_2();
	R(2, 2) = quatBias.R_component_3();
	R(3, 3) = quatBias.R_component_4();

	quatInit = anglesToQuat(90, 1.5, 0);

	for (int i = 0; i < 4; i++)
		mat_cov(i, i) = 0.2;
	init_predict(0, 0) = quatInit.R_component_1();
	init_predict(1, 0) = quatInit.R_component_2();
	init_predict(2, 0) = quatInit.R_component_3();
	init_predict(3, 0) = quatInit.R_component_4();


	Kalman une_rotation(0, 4, 4, 100);
	une_rotation.initsystem(A, B, C, Q, R, init_predict, mat_cov);


	matCmde = zero_matrix<double>(une_rotation.getSizeIn(), 1);
	matObs.resize(une_rotation.getSizeState(), 1);
	matResult.resize(une_rotation.getSizeState(), 1);
	obsInit.resize(une_rotation.getSizeState(), 1);
	obsInit(0, 0) = 1;
	obsInit(1, 0) = obsInit(2, 0) = obsInit(3, 0) = 0;

	dt = tPred = tAct = 0;
	tabMajsys[0] = true;
	tabMajsys[1] = false;
	tabMajsys[2] = false;


	/* Déclaration des fichiers d'écriture/lecture des valeurs */
	std::fstream filegyro;
	std::fstream fileacce;
	std::fstream filemnet;
	std::fstream fileorie;
	std::fstream fileResult;

	/*	Detect Mode Utilisation	*/

	if (mode == MODE_SIMULATION_SENSOR || mode == MODE_SIMULATION_FILE)			/*	cas de Simulation	*/
	{
		/*	Frame rate du SDL	*/
		if (mode == MODE_SIMULATION_SENSOR){
			frameRate = FRAME_RATE_WITH_SENSOR;
		}
		else if (mode == MODE_SIMULATION_FILE){
			frameRate = FRAME_RATE_WITH_FILE;
		}

		/* Initialisation du SDL */
		if (scene.InitialiserFenetre() == false) { system("PAUSE"); return -1; }

		if (!scene.iniGL()) 	{ system("PAUSE"); return -1; }

		if (mode == MODE_SIMULATION_SENSOR){
			writeHeading(accel.getnomfichier(), fileacce);
			writeHeading(gyros.getnomfichier(), filegyro);
			writeHeading(magne.getnomfichier(), filemnet);
			writeHeading(orient.getnomfichier(), fileorie);
		}
	}
	else if (mode == MODE_CREATE_FILE)					/*	cas création de fichier	*/
	{
		createMeasureFile_separate(FILENAME, DIRECTIONS, SAMPLETIME);
		srand(time(NULL));
	}
	else if (mode == MODE_CREATE_FILE_SENSOR)					/*	cas création de fichier à partir de la liaison série	*/
	{
		Serial link(PORTSERIE, BAUD);
		int nbMes;
		std::cout << "entrez le nombre de mesures a copier : " << std::endl;
		std::cin >> nbMes;
		fileFromSerial("serial.txt", link, nbMes);
	}
	else if (mode == MODE_TEST)					/*	cas Test	*/
	{
		int test = 0;
		std::cout << "Rentrez le test à faire : \n";
		std::cout << "  1 - Kalman\n";
		std::cout << "  2 - Conversion Quaternion \n";
		std::cout << "  3 - Changement Repère Vitesse Angulaire \n";
		std::cout << "  4 - Conversion String To Double\n";
		std::cin >> test;
		if (test == TEST_KALMAN){
			testKalman();
		}
		else if (test == TEST_CONVERSION_QUAT){
			test_eulerQuat();
		}
		else if (test == TEST_CHGT_REPERE){
			test_changeRepere();
		}
		else if (test == TEST_STRING_TO_DOUBLE){
			testStringToDouble("3.25", 3.25);
		}
	}

	/*	Simulation Mode	*/
	/* Boucle Principale (avec Traitement Filtre Kalman + Affichage SDL) */
	while (!terminer && (mode == MODE_SIMULATION_SENSOR || mode == MODE_SIMULATION_FILE)){

		_RPT1(0, "----------	STEP %d		---------------\n", step);

		/* Récupère les mesures d'accélération, de vitesse angulaire et du magnétomètre */

		gyros.majMesures();
		accel.majMesures();
		magne.majMesures();
		orient.majMesures();
		vAngulaireLocal_t = gyros.getMesures();/*trait_gyros.moyenner(2);*/
		acceleration_t = accel.getMesures();/*trait_accel.moyenner(2);*/
		magnetic_t = magne.getMesures();/*trait_magne.moyenner(2);*/
		orientation_t = orient.getMesures();/* trait_orient.moyenner(2);*/

		trait_accel.stockerValeurs();
		trait_gyros.stockerValeurs();
		trait_magne.stockerValeurs();
		trait_orient.stockerValeurs();

		/* Ecriture les données dans le cas simulation avec capteur */
		if (mode == MODE_SIMULATION_SENSOR){
			filefromSensor(gyros.getnomfichier(), filegyro, vAngulaireLocal_t);
			filefromSensor(accel.getnomfichier(), fileacce, acceleration_t);
			filefromSensor(magne.getnomfichier(), filemnet, magnetic_t);
			filefromSensor(orient.getnomfichier(), fileorie, orientation_t);
		}

		/* Traitement des données et le filtre de Kalman */
		/* Vitesse angulaire ->> dans le repère global */
		vAngulaireGlobal.x = vAngulaireLocal_t.x;
		vAngulaireGlobal.y = vAngulaireLocal_t.x;
		vAngulaireGlobal.z = vAngulaireLocal_t.x;

		if (step < 100){		/*	Fixer le cube dans les 100 premiers étapes	*/
			wx = wy = wz = 0;
		}
		else{
			wx = vAngulaireGlobal.x;
			wy = vAngulaireGlobal.y;
			wz = vAngulaireGlobal.z;
		}
		tAct = vAngulaireLocal_t.temps;
		dt = (tAct - tPred) / 1000;
		tPred = tAct;

		Ax = 0.5*wx*dt;
		Ay = 0.5*wy*dt;
		Az = 0.5*wz*dt;
		A(0, 0) = 1;
		A(0, 0) = 1; A(1, 1) = 1; A(2, 2) = 1; A(3, 3) = 1; //diagonale
		A(0, 1) = -Ax; A(0, 2) = -Ay; A(0, 3) = -Az;
		A(1, 0) = Ax; A(1, 2) = Az; A(1, 3) = -Ay;
		A(2, 0) = Ay; A(2, 1) = -Az; A(2, 3) = Ax;
		A(3, 0) = Az; A(3, 1) = Ay; A(3, 2) = -Ax;

		une_rotation.majsystem(tabMajsys, A, B, C);

		quatMeasure = anglesToQuat(orientation_t.x, orientation_t.y, orientation_t.z);

		matObs(0, 0) = quatMeasure.R_component_1();
		matObs(1, 0) = quatMeasure.R_component_2();
		matObs(2, 0) = quatMeasure.R_component_3();
		matObs(3, 0) = quatMeasure.R_component_4();

		/* Filtre de Kalman */
		if (step < 100){
			matResult = kalmanTraitement(une_rotation, matCmde, obsInit);
		}
		else{
			/*if ((v_angulaire_t.x > 0.001 || v_angulaire_t.x < -0.001)  &&
			(v_angulaire_t.y > 0.001 || v_angulaire_t.y < -0.001)  &&
			(v_angulaire_t.z > 0.001 || v_angulaire_t.z < -0.001)){
			*/
			matResult = kalmanTraitement(une_rotation, matCmde, matObs);
		}
		quatResult = quaternion<double>(matResult(0, 0), matResult(1, 0), matResult(2, 0), matResult(3, 0));
		angles = quatToAngles_deg(quatResult);
		qBase = qBase*quatResult;
		quatComp(quatResult, axe, angle);

		/* Ecriture du résultat (angles calculés avec Kalman + orientation écriture capteur) */
		orientationData.x = orientation_t.x;
		orientationData.y = orientation_t.y;
		orientationData.z = orientation_t.z;


		/*	Ecriture des valeurs d'angle et d'angle_sensor dans le fichier excel	 */
		writeResult(vAngulaireLocal_t.temps, orientationData, angles, "result.csv", fileResult);
		step++;

		/* Affichage SDL */
		if ((acceleration_t.x == 0 && acceleration_t.y == 0 && acceleration_t.z == 0) &&
			(vAngulaireLocal_t.x == 0 && vAngulaireLocal_t.y == 0 && vAngulaireLocal_t.z == 0) &&
			(magnetic_t.x == 0 && magnetic_t.y == 0 && magnetic_t.z == 0) &&
			(orientation_t.x == 0 && orientation_t.y == 0 && orientation_t.z == 0)){
			terminer = true;
		}
		else{
			terminer = scene.bouclePrincipale(axe, angle, frameRate);
		}
		_RPT1(0, "----------	FIN STEP	---------------\n", step);
	}

	return 0;
}
