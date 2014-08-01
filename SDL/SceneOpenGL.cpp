#include "SceneOpenGL.h"
#include "Kalman.h"
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
	if (SDL_Init(SDL_INIT_VIDEO) <0)
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


void SceneOpenGL::bouclePrincipale()
{

	bool terminer(false);
	unsigned int frameRate(1000 / 200);
	Uint32 debutBoucle(0), finBoucle(0), tempsEcoule(0);
	int turn = 1;
	std::string port = PORTSERIE;
	int baudRate = BAUD;
	quaternion<double> un_quaternion;

	/* Initialisation du filtre de Kalman + Système */
	matrix<double> mat_cov(4, 4, 0), init_predict(4, 1, 0);
	for (int i = 0; i < 4; i++)
		mat_cov(i, i) = 1.5;
	for (int i = 0; i < 4; i++)
		init_predict(i, 0) = 0.05;
	Kalman rotation(0, 4, 4, 100, init_predict, mat_cov);
	vect4D v_angulaire_t, acceleration_t;

	Serial link(port, baudRate);
	Instrument_serie gyro("gyro", &link);
	Instrument_serie acce("acce", &link);
	Traitement trait_gyro(&gyro);
	Traitement trait_acce(&acce);
	/* l'objet qui servira a récupérer les valeurs de l'arduino puis a faire un moyenne sur plusieurs valeurs pour des résultats plus stables*/

	/* Fichier sauvegarder données*/
	//	Mobile gant;

	fstream file_gyro;
	fstream file_acce;

	trait_gyro.openfile_readwrite(file_gyro, "gyro.txt");
	trait_acce.openfile_readwrite(file_acce, "acce.txt");
	_RPT0(0, "ouverture du fichier \n");

	vect3D angle = { 0.0, 0.0, 0.0 };

	// Matrices
	mat4 projection;
	mat4 modelview;

	COORD pos = { 0, 1 };
	HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);

	Cube lecube(2.0, "Shaders/couleur3D.vert", "Shaders/couleur3D.frag");

	projection = perspective(1.22, (double)m_largeurFenetre / m_hauteurFenetre, 1.0, 100.0);
	modelview = mat4(1.0);
	//trait_gyro.writeHeading(filename);
	//trait_acce.writeHeading(filename2);
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

		// Placement de la caméra
		modelview = lookAt(vec3(4, 0, 0), vec3(0, 0, 0), vec3(0, 1, 0));

		trait_gyro.stockerValeurs();
		trait_acce.stockerValeurs();
		trait_gyro.afficherValeurs();
		trait_acce.afficherValeurs();

		/* 
		*  Ecrire les données du capteur dans le fichier de filename 
		*  Lecture les données du fichier filename (stocker dans data)
		*  Ecrire les donnnées (data) dans le fichier filename2
		*/
		
		trait_gyro.filefromSensor(file_gyro,&gyro);

		trait_acce.filefromSensor(file_acce, &acce);

		/*

		acceleration.x = acceleration_t.x;
		acceleration.y = acceleration_t.y;
		acceleration.z = acceleration_t.z;
		_RPT3(0, "acceleration_t x : %f, y : %f, z : %f \n", acceleration_t.x, acceleration_t.y, acceleration_t.z);

		v_angulaire.x = v_angulaire_t.x;
		v_angulaire.y = v_angulaire_t.y;
		v_angulaire.z = v_angulaire_t.z;
	
		_RPT3(0, "v_angulaire_t x : %f, y : %f, z : %f \n", v_angulaire_t.x, v_angulaire_t.y, v_angulaire_t.z);
		*/
		if (trait_gyro.tabFull() == true)
		{
			acceleration_t = acce.getMesures();
			v_angulaire_t = gyro.getMesures();

			_RPT1(0, "DELTA TEMPS : %f \n", trait_gyro.get_dt());
			un_quaternion = rotation.kalman_rotation(v_angulaire_t, acceleration_t, trait_gyro.get_dt());
			
			angle = quatToAngles(un_quaternion);
			//angle = angle + trait_gyro.calculerAngle_deg();
			_RPT4(0, "QUATERNION %f  %f  %f  %f \n", un_quaternion.R_component_1(), un_quaternion.R_component_2(), un_quaternion.R_component_3(), un_quaternion.R_component_4());
			_RPT3(0, "ANGLES EULER X: %f Y: %f Z: %f \n", angle.x, angle.y, angle.z);

			cout << angle.x << endl;
			cout << angle.y << endl;
			cout << angle.z << endl;

			modelview = rotate(modelview, (float)(angle.x*M_2PI / 360.0), vec3(1, 0, 0));
			modelview = rotate(modelview, (float)(angle.y*M_2PI / 360.0), vec3(0, 1, 0));
			modelview = rotate(modelview, (float)(angle.z*M_2PI / 360.0), vec3(0, 0, 1));
			// Rotation du repère

		} //end if(tabFull == true)

		else _RPT0(0, " FALSE \n" );
		lecube.afficher(projection, modelview);

		// Actualisation de la fenêtre
		SDL_GL_SwapWindow(m_fenetre);

		//framerate
		finBoucle = SDL_GetTicks();
		tempsEcoule = finBoucle - debutBoucle;

		if (tempsEcoule < frameRate)
			SDL_Delay(frameRate - tempsEcoule);


	}
}