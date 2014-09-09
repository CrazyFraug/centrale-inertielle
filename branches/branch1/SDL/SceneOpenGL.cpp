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

	m_fenetre = SDL_CreateWindow("SDL 2.0", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
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


void SceneOpenGL::bouclePrincipale(Mobile &gant, Traitement **tabTrait)
{

	/* Gestion de la fermeture de la fenetre et du framerate */
	bool terminer(false);
	unsigned int frameRate(20);
	Uint32 debutBoucle(0), finBoucle(0), tempsEcoule(0);

	vect3D angle = {0,0,0};
	quaternion<double> q(1,0,0,0);

	// Matrices
	mat4 projection;
	mat4 modelview, modelview2, modelview3;

	// Gestion de la console
	COORD pos = { 0, 1 }; // position du curseur (lieu de départ où les messages seront écrits)
	HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);

	//Chargement des shaders
	Cube lecube(2.0, "Shaders/couleur3D.vert", "Shaders/couleur3D.frag");
	Axe axeX(2.0, 0.1, "Shaders/couleur3D.vert", "Shaders/couleur3D.frag");
	Axe axeY(2.0, 0.1, "Shaders/couleur3D.vert", "Shaders/couleur3D.frag");
	Axe axeZ(2.0, 0.1, "Shaders/couleur3D.vert", "Shaders/couleur3D.frag");

	//Matrices de projection et de transformation
	projection = perspective(1.22, (double)m_largeurFenetre / m_hauteurFenetre, 1.0, 100.0);
	modelview = mat4(1.0);
	modelview3 = mat4(1.0);
	modelview2 = mat4(1.0);


	system("cls");//effacer la console
	// Boucle principale
	while (!terminer)
	{
		debutBoucle = SDL_GetTicks();

		// Gestion des évènements
		SDL_PollEvent(&m_evenements);
		if (m_evenements.window.event == SDL_WINDOWEVENT_CLOSE)
			terminer = true;

		SetConsoleCursorPosition(console, pos);

		std::cout << "Simulation en cours..." << std::endl;
		// Nettoyage de l'écran
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Placement de la caméra
		modelview = lookAt(vec3(4, 0, 0), vec3(0, 0, 0), vec3(0, 1, 0));

		/* Filtre de Kalman */
		//un_quaternion = rotation.kalman_rotation(v_angulaire_t, acceleration_t, magnetic_t, orientation_t, dt, rotation);

		//angle = quatToAngles_deg(un_quaternion);*/


		///* Test équations de calculs d'orientation avec accélération */
		//angle.x = (float)atan2(ay, az) * 360 / M_2PI;
		//if ((ay*sin(angle.x) + az*cos(angle.x)) == 0){
		//	if (ax > 0){
		//		angle.y = 90;
		//	}
		//	else{
		//		angle.y = -90;
		//	}
		//}
		//else{
		//	angle.y = (float)atan2(-ax, (ay*sin(angle.x) + az*cos(angle.x))) * 360 / M_2PI;
		//}
		//angle.z = (float)atan2(mz*sin(angle.x) - my*cos(angle.x), mx*cos(angle.y) + my*sin(angle.y)*sin(angle.x) + mz*sin(angle.y)*cos(angle.x)) * 360 / M_2PI;//atan(mz*cos(angle_meas.x) + my*sin(angle_meas.x) / mx*cos(angle_meas.z)+mz*sin(angle_meas.z)*sin(angle_meas.x)+my*sin(angle_meas.z)*cos(angle_meas.x))*180/(atan(1)*4);


		vect3D anglePlus = {0,0,0};
		vect3D quat_axe = {0,0,0};
		double quat_angle = 0;
		_RPT0(0, "GYROSCOPE\n");
		

		//anglePlus = tabTrait[0]->renvoyerVal(1);
		//_RPT0(0, "ACCELEROMETRE\n");
		//vect3D acce = tabTrait[1]->renvoyerVal(1);
		//_RPT0(0, "MAGNETOMETRE\n");
		//vect3D mnet = tabTrait[2]->renvoyerVal(1);
		//_RPT0(0, "ORIENTATION\n");
		//vect3D orie = tabTrait[3]->renvoyerVal(1);
		//angle = angle + anglePlus;

		tabTrait[0]->renvoyerQuat(1, q, quat_axe, quat_angle);

		//_RPT1(0, "$$Valeurs d'angle en x : %f\n", angle.x);
		//_RPT1(0, "$$Valeurs d'angle en y : %f\n", angle.y);
		//_RPT1(0, "$$Valeurs d'angle en z : %f\n", angle.z);

		_RPT1(0, "$$$ angle du quaternion = %f\n", quat_angle);
		_RPT3(0, "$$$ axe du quaternion = %f  %f  %f\n", quat_axe.x, quat_axe.y, quat_axe.z);

		modelview = rotate(modelview, (float)(quat_angle), vec3(quat_axe.x, quat_axe.y, quat_axe.z));

		lecube.afficher(projection, modelview);
		axeX.afficher(projection, modelview);

		// Actualisation de la fenêtre
		SDL_GL_SwapWindow(m_fenetre);

		if (Traitement::get_finFichier())
			terminer = true;

		/* Gestion framerate */
		finBoucle = SDL_GetTicks();
		tempsEcoule = finBoucle - debutBoucle; // calcul du temps écoulé
		if (tempsEcoule < frameRate)
			SDL_Delay(frameRate - tempsEcoule); // mise en attente de l'affichage 

	}//fin while(!terminer)

	Traitement::resetCursor();

	system("PAUSE");
}