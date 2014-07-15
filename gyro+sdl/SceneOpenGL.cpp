#include "SceneOpenGL.h"
#include "Serial.cpp"

#pragma comment (lib, "glew32.lib")
#pragma comment (lib, "OpenGL32.lib")

using namespace std;
using namespace glm;

SceneOpenGL::SceneOpenGL(string titre, int largeur, int hauteur):
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
	if(SDL_Init(SDL_INIT_VIDEO) <0)
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

    if(m_fenetre == 0)
    {
        cout << "Erreur lors de la creation de la fenetre : " << SDL_GetError() << endl;
        SDL_Quit();

        return false;
    }

	// Création du contexte OpenGL

    m_contexteOpenGL = SDL_GL_CreateContext(m_fenetre);

    if(m_contexteOpenGL == 0)
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
        GLenum initialisationGLEW( glewInit() );

        // Si l'initialisation a échoué :
        if(initialisationGLEW != GLEW_OK)
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
    // Tout s'est bien passé, on retourne true
    return true;

	}


void SceneOpenGL::bouclePrincipale()
{
    // Booléen terminer
    bool terminer(false);
	
	float angleX(0.0), angleY(0.0), angleZ(0.0), tmp(0.0), dt(0.0);
	int axe;
	SimpleSerial serie8("COM8",115200);
	string valeur;
    // Matrices
    mat4 projection;
    mat4 modelview;

	COORD pos = {0,1};
	HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);

	Cube lecube(2.0, "Shaders/couleur3D.vert", "Shaders/couleur3D.frag");

    projection = perspective(1.22, (double) m_largeurFenetre / m_hauteurFenetre, 1.0, 100.0);
    modelview = mat4(1.0);

	t_mesX = clock(); t_mesY = clock(); t_mesZ = clock();

    // Boucle principale
    while(!terminer)
    {
			SetConsoleCursorPosition(console, pos);

			tmp = serie8.readDatas(axe);
			if (axe == 1)
				{cout << "Wx = " << tmp << endl;

				//calcul du temps écoulé
				dt = ((float)(clock()-t_mesX))/CLOCKS_PER_SEC;
				t_mesX = clock();
				
				angleX -= tmp*dt;
				pos.Y = 3;
			}

				if(axe == 2){
					cout << "Wy = " << tmp << endl;

					//calcul du temps
					dt = ((float)(clock()-t_mesY))/CLOCKS_PER_SEC;
					t_mesY = clock();
				
					angleY += tmp*dt;
					pos.Y = 4;
				}

					if (axe == 3){
						cout << "Wz = " << tmp << endl;

						//calcul du temps
						dt = ((float)(clock()-t_mesZ))/CLOCKS_PER_SEC;
						t_mesZ = clock();
				
						angleZ -= tmp*dt;
						pos.Y = 1;
					}


        // Gestion des évènements
        //SDL_WaitEvent(&m_evenements);
        if(m_evenements.window.event == SDL_WINDOWEVENT_CLOSE)
            terminer = true;

        // Nettoyage de l'écran
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Placement de la caméra
		modelview = lookAt(vec3(4, 0, 0), vec3(0, 0, 0), vec3(0, 1, 0));
		// Incrémentation de l'angle
		//angle += 0.05f;

		if(angleX >= 360.0)
			angleX -= 360.0;
		if(angleY >= 360.0)
			angleY -= 360.0;
		if(angleZ >= 360.0)
			angleZ -= 360.0;

		modelview = rotate(modelview, angleX, vec3(1, 0, 0));
		modelview = rotate(modelview, angleY, vec3(0, 1, 0));
		modelview = rotate(modelview, angleZ, vec3(0, 0, 1));
		// Rotation du repère

		lecube.afficher(projection, modelview);

        // Actualisation de la fenêtre
        SDL_GL_SwapWindow(m_fenetre);

		Sleep(10);
    }
}