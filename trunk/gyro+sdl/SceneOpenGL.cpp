#include "SceneOpenGL.h"
#include "Mobile.h"

#pragma comment (lib, "glew32.lib")
#pragma comment (lib, "OpenGL32.lib")
#pragma comment (lib, "SDL2.lib")
#pragma comment (lib, "SDL2main.lib")

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

    bool terminer(false);
	unsigned int frameRate (1000 / 100);
    Uint32 debutBoucle(0), finBoucle(0), tempsEcoule(0);

	float value = 0;
	int axe = 0;

	SimpleSerial serieTest("COM8",115200);
	value = serieTest.readDatas(axe);

	cout << value << endl;

	//Instrument accel("accelerometre");
	Instrument gyro("gyroscope",&serieTest);

	Mobile gant;

	vect3D angle = {0.0,0.0,0.0};
	clock_t* temps = new clock_t[3];
	double* dt = new double[3];

    // Matrices
    mat4 projection;
    mat4 modelview;

	COORD pos = {0,1};
	HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);

	Cube lecube(2.0, "Shaders/couleur3D.vert", "Shaders/couleur3D.frag");

    projection = perspective(1.22, (double) m_largeurFenetre / m_hauteurFenetre, 1.0, 100.0);
    modelview = mat4(1.0);

    // Boucle principale
    while(!terminer)
    {
		debutBoucle = SDL_GetTicks();

			SetConsoleCursorPosition(console, pos);

			gyro.majSerial();
			temps = gyro.getTemps();
			gyro.afficherMesures();
			gyro.afficherTemps();

        // Gestion des évènements
        SDL_PollEvent(&m_evenements);
        if(m_evenements.window.event == SDL_WINDOWEVENT_CLOSE)
            terminer = true;

        // Nettoyage de l'écran
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Placement de la caméra
		modelview = lookAt(vec3(4, 0, 0), vec3(0, 0, 0), vec3(0, 1, 0));

		angle = gyro.getMesures();
		dt = gyro.getdt();

		for (int i=0;i<3;i++)
		{
			//dt[i] = (clock() - temps[i])/CLOCKS_PER_SEC;
			cout << "dt " << i << " : " << dt[i] << endl;
		}

		if(angle.x >= 360.0)
			angle.x -= 360.0;
		if(angle.y >= 360.0)
			angle.y -= 360.0;
		if(angle.z >= 360.0)
			angle.z -= 360.0;

		modelview = rotate(modelview, (float)(angle.x*dt[0]), vec3(1, 0, 0));
		modelview = rotate(modelview, (float)(angle.y*dt[1]), vec3(0, 1, 0));
		modelview = rotate(modelview, (float)(angle.z*dt[2]), vec3(0, 0, 1));
		// Rotation du repère

		lecube.afficher(projection, modelview);

        // Actualisation de la fenêtre
        SDL_GL_SwapWindow(m_fenetre);

		finBoucle = SDL_GetTicks();
		tempsEcoule = finBoucle - debutBoucle;

		if(tempsEcoule < frameRate)
			SDL_Delay(frameRate - tempsEcoule);
    }
}