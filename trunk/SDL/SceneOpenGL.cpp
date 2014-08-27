#include "SceneOpenGL.h"
#include "Kalman.h"
#include <boost/numeric/ublas/operation.hpp>
//#include "source\test\test_Traitement.cpp"
//#include "Mobile.h"

#pragma comment (lib, "glew32.lib")
#pragma comment (lib, "OpenGL32.lib")
#pragma comment (lib, "SDL2.lib")
#pragma comment (lib, "SDL2main.lib")

using namespace std;
using namespace glm;


SceneOpenGL::SceneOpenGL(string titre, int largeur, int hauteur) :
m_titreFenetre(titre), m_largeurFenetre(largeur), m_hauteurFenetre(hauteur),
m_fenetre(0), m_contexteOpenGL(0)
{

}

SceneOpenGL::~SceneOpenGL()
{
	SDL_GL_DeleteContext(m_contexteOpenGL);
	SDL_DestroyWindow(m_fenetre);
	SDL_Quit();
}


bool SceneOpenGL::InitialiserFenetre() {

	//initialisation SDL
	if (SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		cout << "erreur lors de l'initialisation de la SDL" << SDL_GetError() << endl;
		SDL_Quit();
		return false;
	}


	// Version d'OpenGL
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);

	// Double Buffer
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

	// Création de la fenêtre

	m_fenetre = SDL_CreateWindow("Test SDL 2.0", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
		800, 600, SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL);

	if (m_fenetre == 0)
	{
		cout << "Erreur lors de la creation de la fenetre : " << SDL_GetError() << endl;
		SDL_Quit();

		return false;
	}

	// Création du contexte OpenGL

	m_contexteOpenGL = SDL_GL_CreateContext(m_fenetre);

	if (m_contexteOpenGL == 0)
	{
		std::cout << "Erreur a la creation du contexte OpenGL :  " << SDL_GetError() << std::endl;
		SDL_DestroyWindow(m_fenetre);
		SDL_Quit();

		return false;
	}

	return true;

}


bool SceneOpenGL::iniGL() {

	// On initialise GLEW
	GLenum initialisationGLEW(glewInit());

	// Si l'initialisation a échoué :
	if (initialisationGLEW != GLEW_OK)
	{
		// On affiche l'erreur grâce à la fonction : glewGetErrorString(GLenum code)
		std::cout << "Erreur d'initialisation de GLEW : " << glewGetErrorString(initialisationGLEW) << std::endl;


		// On quitte la SDL

		SDL_GL_DeleteContext(m_contexteOpenGL);
		SDL_DestroyWindow(m_fenetre);
		SDL_Quit();

		return false;
	}

	// #endif

	glEnable(GL_DEPTH_TEST);

	return true;

}


void SceneOpenGL::bouclePrincipaleSensor()
{

	bool terminer(false);
	unsigned int frameRate(1000 / 200);
	Uint32 debutBoucle(0), finBoucle(0), tempsEcoule(0);
	std::string port = PORTSERIE;
	int baudRate = BAUD;
	quaternion<double> un_quaternion;
	/* Initialisation du filtre de Kalman + Système */
	matrix<double> mat_cov(4, 4, 0), init_predict(4, 1, 0);
	for (int i = 0; i < 4; i++)
		mat_cov(i, i) = 0.5;
	init_predict(0, 0) = 0.7071067811865476;
	init_predict(1, 0) = 0;
	init_predict(2, 0) = -0.7071067811865476;
	init_predict(3, 0) = 0;
	Kalman une_rotation(0, 4, 4, 100, init_predict, mat_cov);
	//declareKalman(&une_rotation);

	vect4D v_angulaire_t, acceleration_t, magnetic_t, orientation_t, v_angulaire_t_deg;
	vect4D initKalman = { 0, 0, 0, 0 };
	vect4D vAngulaireInit = { 0, 0, 0, 0 };

	Serial link(port, baudRate);
	Instrument_serie accel("acce", &link);
	Traitement trait_accel(&accel);
	Instrument_serie gyros("gyro", &link);
	Traitement trait_gyros(&gyros);
	Instrument_serie magne("mnet", &link);
	Traitement trait_magne(&magne);
	Instrument_serie orient("orie", &link);
	Traitement trait_orient(&orient);

	vect3D angle = { 0.0, 0.0, 0.0 };
	vect3D angle_sensor = { 0.0, 0.0, 0.0 };
	// Matrices
	mat4 projection;
	mat4 modelview;

	COORD pos = { 0, 1 };
	HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);

	Cube lecube(2.0, "Shaders/couleur3D.vert", "Shaders/couleur3D.frag");
	int step = 0;
	vect3D orientation_data;
	double ax, ay, az, mx, my, mz;
	bool headingResult(false);
	projection = perspective(1.22, (double)m_largeurFenetre / m_hauteurFenetre, 1.0, 100.0);
	modelview = mat4(1.0);
	// Boucle principale
	std::fstream filegyro;
	std::fstream fileacce;
	std::fstream filemnet;
	std::fstream fileorie;

	writeHeading("acce.csv", fileacce);
	writeHeading("gyro.csv", filegyro);
	writeHeading("mnet.csv", filemnet);
	writeHeading("orie.csv", fileorie);
	while (!terminer)
	{
		debutBoucle = SDL_GetTicks();

		// Gestion des évènements
		SDL_PollEvent(&m_evenements);
		if (m_evenements.window.event == SDL_WINDOWEVENT_CLOSE)
			terminer = true;

		SetConsoleCursorPosition(console, pos);

		// Nettoyage de l'écran
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		/* Récupère les mesures d'accélération, de vitesse angulaire et du magnétomètre */

		trait_gyros.stockerValeurs();
		trait_accel.stockerValeurs();
		trait_magne.stockerValeurs();
		trait_orient.stockerValeurs();
		trait_gyros.filefromSensor("gyro.csv", filegyro, &gyros);
		trait_accel.filefromSensor("acce.csv", fileacce, &accel);
		trait_magne.filefromSensor("mnet.csv", filemnet, &magne);
		trait_orient.filefromSensor("orie.csv", fileorie, &orient);
		acceleration_t = accel.getMesures();
		v_angulaire_t = gyros.getMesures();
		magnetic_t = magne.getMesures();
		orientation_t = orient.getMesures();
		_RPT4(0, "ACCELERATION - X : %f - Y : %f - Z : %f - T : %f \n", acceleration_t.x, acceleration_t.y, acceleration_t.z, acceleration_t.temps);
		_RPT4(0, "GYROSCOPE - X : %f - Y : %f - Z : %f - T : %f \n", v_angulaire_t.x, v_angulaire_t.y, v_angulaire_t.z, v_angulaire_t.temps);
		_RPT4(0, "MAGNETIC - X : %f - Y : %f - Z : %f - T : %f\n", magnetic_t.x, magnetic_t.y, magnetic_t.z, magnetic_t.temps);
		_RPT4(0, "ORIENTATION - X : %f - Y : %f - Z : %f - T : %f\n", orientation_t.x, orientation_t.y, orientation_t.z, orientation_t.temps);
		// Placement de la caméra
		modelview = lookAt(vec3(4, 0, 0), vec3(0, 0, 0), vec3(0, 1, 0));
		/* Filtre de Kalman */
		if (step < 100){
			un_quaternion = kalman_rotation(vAngulaireInit, acceleration_t, magnetic_t, initKalman, trait_gyros.get_dt(), une_rotation);
		}
		else{
			/*if ((v_angulaire_t.x > 0.001 || v_angulaire_t.x < -0.001)  &&
			(v_angulaire_t.y > 0.001 || v_angulaire_t.y < -0.001)  &&
			(v_angulaire_t.z > 0.001 || v_angulaire_t.z < -0.001)){
			*/
			un_quaternion = kalman_rotation(v_angulaire_t, acceleration_t, magnetic_t, orientation_t, trait_gyros.get_dt(), une_rotation);
		}
		/*	angle_sensor	--	Angle calculé directement par les mesures du gyroscope	*
		*	angle			--	Angle convertit à partir du quaternion après filtré		*/
		/*angle_sensor = angle_sensor + trait_gyro.calculerAngle_deg();*/
		angle = quatToAngles_deg(un_quaternion);

		_RPT4(0, "QUATERNION %f  %f  %f  %f \n", un_quaternion.R_component_1(), un_quaternion.R_component_2(), un_quaternion.R_component_3(), un_quaternion.R_component_4());
		_RPT3(0, "ANGLES EULER X: %f Y: %f Z: %f \n", angle.x, angle.y, angle.z);

		/*}*/
		orientation_data.x = orientation_t.x;
		orientation_data.y = orientation_t.y;
		orientation_data.z = orientation_t.z;
		/*	Ecrire les valeurs d'angle et d'angle_sensor dans le fichier excel	 */
		writeResult(v_angulaire_t.temps, orientation_data, angle, "result.csv", headingResult);
		headingResult = true;

		cout << angle.x << endl;
		cout << angle.y << endl;
		cout << angle.z << endl;

		modelview = rotate(modelview, (float)(angle.x*M_2PI / 360.0), vec3(1, 0, 0));
		modelview = rotate(modelview, (float)(angle.y*M_2PI / 360.0), vec3(0, 1, 0));
		modelview = rotate(modelview, (float)(angle.z*M_2PI / 360.0), vec3(0, 0, 1));
		// Rotation du repère

		//} //end if(tabFull == true)

		/*else _RPT0(0, " FALSE \n");*/
		lecube.afficher(projection, modelview);

		// Actualisation de la fenêtre
		SDL_GL_SwapWindow(m_fenetre);

		//framerate
		finBoucle = SDL_GetTicks();
		tempsEcoule = finBoucle - debutBoucle;
		step++;
		if (tempsEcoule < frameRate)
			SDL_Delay(frameRate - tempsEcoule);
	}
}

void SceneOpenGL::bouclePrincipaleSimu()
{

	bool terminer(false);
	unsigned int frameRate(1000 / 200);
	Uint32 debutBoucle(0), finBoucle(0), tempsEcoule(0);
	int turn = 1;

	static double t_Pred, t_Act, dt;
	t_Pred = t_Act = dt = 0;
	quaternion<double> un_quaternion, quaternion_test;
	/* Initialisation du filtre de Kalman + Système */
	matrix<double> mat_cov(4, 4, 0), init_predict(4, 1, 0);
	for (int i = 0; i < 4; i++)
		mat_cov(i, i) = 0.5;
	init_predict(0, 0) = 0.7071067811865476;
	init_predict(1, 0) = 0;
	init_predict(2, 0) = -0.7071067811865476;
	init_predict(3, 0) = 0;
	Kalman une_rotation(0, 4, 4, 100, init_predict, mat_cov);
	vect4D v_angulaire_t, acceleration_t, magnetic_t, orientation_t;
	vect4D initKalman = { 0, 0, 0, 0 };
	vect4D vAngulaireInit = { 0, 0, 0, 0 };

	Instrument accel("acce");
	Traitement trait_accel(&accel);
	Instrument gyros("gyro");
	Traitement trait_gyros(&gyros);
	Instrument magne("mnet");
	Traitement trait_magne(&magne);
	Instrument orient("orie");
	Traitement trait_orient(&orient);

	vect3D angle = { 0.0, 0.0, 0.0 };
	vect3D angle_sensor = { 0.0, 0.0, 0.0 };

	vect3D angle_test = { 0.0, 0.0, 0.0 };
	// Matrices
	mat4 projection;
	mat4 modelview;

	COORD pos = { 0, 1 };
	HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);

	Cube lecube(2.0, "Shaders/couleur3D.vert", "Shaders/couleur3D.frag");


	std::fstream filegyro;
	std::fstream fileacce;
	std::fstream filemnet;
	std::fstream fileorie;
	vect3D orientation_data;
	bool headingResult(false);
	projection = perspective(1.22, (double)m_largeurFenetre / m_hauteurFenetre, 1.0, 100.0);
	modelview = mat4(1.0);
	// Boucle principale
	while (!terminer)
	{
		debutBoucle = SDL_GetTicks();

		// Gestion des évènements
		SDL_PollEvent(&m_evenements);
		if (m_evenements.window.event == SDL_WINDOWEVENT_CLOSE)
			terminer = true;

		SetConsoleCursorPosition(console, pos);

		// Nettoyage de l'écran
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		/* Récupère les mesures d'accélération, de vitesse angulaire et du magnétomètre */
		acceleration_t = trait_accel.readDatafromFile("acce.csv", fileacce, turn);
		v_angulaire_t = trait_gyros.readDatafromFile("gyro.csv", filegyro, turn);
		magnetic_t = trait_magne.readDatafromFile("mnet.csv", filemnet, turn);
		orientation_t = trait_orient.readDatafromFile("orie.csv", fileorie, turn);
		t_Act = acceleration_t.temps;
		dt = (t_Act - t_Pred) / 1000;
		t_Pred = acceleration_t.temps;
		_RPT4(0, "ACCELERATION - X : %f - Y : %f - Z : %f - T : %f \n", acceleration_t.x, acceleration_t.y, acceleration_t.z, acceleration_t.temps);
		_RPT4(0, "GYROSCOPE - X : %f - Y : %f - Z : %f - T : %f \n", v_angulaire_t.x, v_angulaire_t.y, v_angulaire_t.z, v_angulaire_t.temps);
		_RPT4(0, "MAGNETIC - X : %f - Y : %f - Z : %f - T : %f\n", magnetic_t.x, magnetic_t.y, magnetic_t.z, magnetic_t.temps);
		_RPT4(0, "ORIENTATION - X : %f - Y : %f - Z : %f - T : %f\n", orientation_t.x, orientation_t.y, orientation_t.z, orientation_t.temps);
		// Placement de la caméra
		modelview = lookAt(vec3(4, 0, 0), vec3(0, 0, 0), vec3(0, 1, 0));
		/* Filtre de Kalman */
		_RPT1(0, "DT : %f \n", dt);

		if (turn < 100){
			un_quaternion = kalman_rotation(vAngulaireInit, acceleration_t, magnetic_t, initKalman, dt, une_rotation);
		}
		else{
			/*if ((v_angulaire_t.x > 0.001 || v_angulaire_t.x < -0.001)  &&
			(v_angulaire_t.y > 0.001 || v_angulaire_t.y < -0.001)  &&
			(v_angulaire_t.z > 0.001 || v_angulaire_t.z < -0.001)){
			*/

			un_quaternion = kalman_rotation(v_angulaire_t, acceleration_t, magnetic_t, orientation_t, dt, une_rotation);

		}
		/*	angle_sensor	--	Angle calculé directement par les mesures du gyroscope	*
		*	angle			--	Angle convertit à partir du quaternion après filtré		*/
		/*angle_sensor = angle_sensor + trait_gyro.calculerAngle_deg();*/
		angle = quatToAngles_deg(un_quaternion);
		quaternion_test = anglesToQuat(orientation_t.x, orientation_t.y, orientation_t.z);
		_RPT4(0, "QUATERNION %f  %f  %f  %f \n", un_quaternion.R_component_1(), un_quaternion.R_component_2(), un_quaternion.R_component_3(), un_quaternion.R_component_4());
		_RPT4(0, "QUATERNION TEST %f  %f  %f  %f \n", quaternion_test.R_component_1(), quaternion_test.R_component_2(), quaternion_test.R_component_3(), quaternion_test.R_component_4());
		angle_test = quatToAngles_deg(quaternion_test);
		_RPT3(0, "ANGLES TEST X: %f Y: %f Z: %f \n", angle_test.x, angle_test.y, angle_test.z);
		_RPT3(0, "ANGLES EULER X: %f Y: %f Z: %f \n", angle.x, angle.y, angle.z);
		orientation_data.x = orientation_t.x;
		orientation_data.y = orientation_t.y;
		orientation_data.z = orientation_t.z;
		/*	Ecrire les valeurs d'angle et d'angle_sensor dans le fichier excel	 */
		writeResult(v_angulaire_t.temps, orientation_data, angle, "result.csv", headingResult);
		headingResult = true;
		cout << angle.x << endl;
		cout << angle.y << endl;
		cout << angle.z << endl;

		modelview = rotate(modelview, (float)(angle.x*M_2PI / 360.0), vec3(1, 0, 0));
		modelview = rotate(modelview, (float)(angle.y*M_2PI / 360.0), vec3(0, 1, 0));
		modelview = rotate(modelview, (float)(angle.z*M_2PI / 360.0), vec3(0, 0, 1));
		// Rotation du repère

		//} //end if(tabFull == true)

		/*else _RPT0(0, " FALSE \n");*/
		turn++;
		_RPT1(0, "TURN : %d\n", turn);
		lecube.afficher(projection, modelview);

		// Actualisation de la fenêtre
		SDL_GL_SwapWindow(m_fenetre);

		//framerate
		finBoucle = SDL_GetTicks();
		tempsEcoule = finBoucle - debutBoucle;

		if (tempsEcoule < frameRate)
			SDL_Delay(frameRate - tempsEcoule);
		if ((acceleration_t.x == 0 && acceleration_t.y == 0 && acceleration_t.z == 0) &&
			(v_angulaire_t.x == 0 && v_angulaire_t.y == 0 && v_angulaire_t.z == 0) &&
			(magnetic_t.x == 0 && magnetic_t.y == 0 && magnetic_t.z == 0) &&
			(orientation_t.x == 0 && orientation_t.y == 0 && orientation_t.z == 0)){
			terminer = true;
			std::cout << "FIN DE SIMULATION" << std::endl;
			_RPT0(0, "FIN DE SIMULATION");
		}
	}
}