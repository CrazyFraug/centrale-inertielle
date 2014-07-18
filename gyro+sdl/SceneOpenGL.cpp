#include "SceneOpenGL.h"
#include "Mobile.h"

#pragma comment (lib, "glew32.lib")
#pragma comment (lib, "OpenGL32.lib")
#pragma comment (lib, "SDL2.lib")
#pragma comment (lib, "SDL2main.lib")

using namespace std;
using namespace glm;

//surcharge de l'operateur + pour les vecteur 3D
vect3D operator+(vect3D v1, vect3D v2)
{
	vect3D v;
	v.x = v1.x + v2.x;
	v.y = v1.y + v2.y;
	v.z = v1.z + v2.z;
	return v;
}

vect3D operator*(vect3D v2, double* v1) 
{
	vect3D result;
	result.x = v2.x * v1[0];	
	result.y = v2.y * v1[1];
	result.z = v2.z * v1[2];
	return result;
}

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

    // Cr�ation de la fen�tre

    m_fenetre = SDL_CreateWindow("Test SDL 2.0", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 
									800, 600, SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL);

    if(m_fenetre == 0)
    {
        cout << "Erreur lors de la creation de la fenetre : " << SDL_GetError() << endl;
        SDL_Quit();

        return false;
    }

	// Cr�ation du contexte OpenGL

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

        // Si l'initialisation a �chou� :
        if(initialisationGLEW != GLEW_OK)
        {
            // On affiche l'erreur gr�ce � la fonction : glewGetErrorString(GLenum code)
            std::cout << "Erreur d'initialisation de GLEW : " << glewGetErrorString(initialisationGLEW) << std::endl;


            // On quitte la SDL

            SDL_GL_DeleteContext(m_contexteOpenGL);
            SDL_DestroyWindow(m_fenetre);
            SDL_Quit();

            return false;
        }

   // #endif

	glEnable(GL_DEPTH_TEST);
    // Tout s'est bien pass�, on retourne true
    return true;

	}


void SceneOpenGL::bouclePrincipale()
{

    bool terminer(false);
	bool init(false);
	unsigned int frameRate (1000 / 200);
    Uint32 debutBoucle(0), finBoucle(0), tempsEcoule(0);

	float value = 0;
	int axe = 0;

	SimpleSerial serieTest("COM8",115200);

	//Instrument accel("accelerometre");
	Instrument gyro("gyroscope",&serieTest);

	Mobile gant;

	vect3D angle = {0.0,0.0,0.0};
	clock_t* temps = new clock_t[3];
	double* dt = new double[3];
	vect3D compteur = {0,0,0};
	double** mesures = new double*[3]; //tableau de 3 lignes
	for(int i = 0; i<3; i++)
	{
		mesures[i] = new double[bufferSize]; //avec nbColonnes = bufferSize
	}

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

		// Gestion des �v�nements
        SDL_PollEvent(&m_evenements);
        if(m_evenements.window.event == SDL_WINDOWEVENT_CLOSE)
            terminer = true;


		SetConsoleCursorPosition(console, pos);

		gyro.majSerial();
		temps = gyro.getTemps();
		gyro.afficherMesures();

        // Nettoyage de l'�cran
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Placement de la cam�ra
		modelview = lookAt(vec3(4, 0, 0), vec3(0, 0, 0), vec3(0, 1, 0));

		//mise a jour de la matrice de valeurs (mesures)
		/*value = (float)gyro.getMesure(axe);
		cout << value << endl;
		if(axe == 1){
			mesures[0][(int)compteur.x] = value;
			compteur.x++;
		}
		if(axe == 2){
			mesures[1][(int)compteur.y] = value;
			compteur.y++;
		}
		if(axe == 3){
			mesures[2][(int)compteur.z] = value;
			compteur.z++;
		}*/
					
		//cout <<"mesure x : " << mesures[0][0] << endl;
		//cout <<"mesure x : " << mesures[0][1] << endl;
		//cout <<"mesure x : " << mesures[0][2] << endl;
		//cout <<"mesure x : " << mesures[0][3] << endl;

		//remise a zero des compteurs + mise a jour des angles de rotation + relev� du temps
		/*if(compteur.x == bufferSize) {
			compteur.x = 0;
			angle.x = angle.x + gant.meanValue(bufferSize, mesures[0])*(temps[1]-(gyro.getTemps())[0]);
			temps[0] = (gyro.getTemps())[0];
		}
		if(compteur.y == bufferSize) {
			compteur.y = 0;
			angle.x = angle.x + gant.meanValue(bufferSize, mesures[0])*(temps[1]-(gyro.getTemps())[1]);			
			temps[1] = (gyro.getTemps())[1];
		}
		if(compteur.z == bufferSize) {
			compteur.z = 0;
			angle.x = angle.x + gant.meanValue(bufferSize, mesures[0])*(temps[2]-(gyro.getTemps())[2]);
			temps[2] = (gyro.getTemps())[2];
		}
		*/

		for (int i=0;i<3;i++)
		{
			dt[i] = (clock() - temps[i])/1000.0;
		}
			angle.x += gyro.getMesure(1)*dt[0];
			angle.y += gyro.getMesure(2)*dt[1];
			angle.z += gyro.getMesure(3)*dt[2];

			cout <<"angle x : " << angle.x << "  dt "<< " : " << dt[0] << endl;
			cout <<"angle y : " << angle.y << "  dt "<< " : " << dt[1] << endl;
			cout <<"angle z : " << angle.z << "  dt "<< " : " << dt[2] << endl;
			

		if (init == true) 
		{
			modelview = rotate(modelview, (float)(angle.x), vec3(1, 0, 0));
			modelview = rotate(modelview, (float)(angle.y), vec3(0, 1, 0));
			modelview = rotate(modelview, (float)(angle.z), vec3(0, 0, 1));
			// Rotation du rep�re
		}
		else //initialisation des valeurs initiales
		{
			angle.x = 0;
			angle.y = 0;
			angle.z = 0;
			temps[0] = clock();
			temps[1] = clock();
			temps[2] = clock();
			gyro.setTemps(temps);
			gyro.calibrer();
			gyro.afficherVI();
			init = true;		
		}

		lecube.afficher(projection, modelview);

        // Actualisation de la fen�tre
        SDL_GL_SwapWindow(m_fenetre);

		finBoucle = SDL_GetTicks();
		tempsEcoule = finBoucle - debutBoucle;

		if(tempsEcoule < frameRate)
			SDL_Delay(frameRate - tempsEcoule);

    }
}