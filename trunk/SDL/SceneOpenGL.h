#ifndef DEF_SCENEOPENGL
#define DEF_SCENEOPENGL

#include <iostream>
#include <string>

//include OpenGL
#include <SDL2\SDL.h>
#include <GL\glew.h>

//glm libs
#define GLM_FORCE_RADIANS
#include <glm.hpp>
#include <gtx\transform.hpp>
#include <gtc\type_ptr.hpp>

#include <time.h>
#include "Shader.h" 
#include "Cube.h"
#include "Traitement.h"	
//#include "Instrument.h"

#define M_2PI 6.28318530718 

//serial port parameters:
#define PORTSERIE "COM9"
#define BAUD 115200
class SceneOpenGL
{
    public:

    SceneOpenGL(std::string titre, int largeur, int hauteur);
    ~SceneOpenGL();
	
	//méthodes:
	bool InitialiserFenetre();
	bool iniGL();
	void bouclePrincipale();

    private:

    std::string m_titreFenetre;
    int m_largeurFenetre;
    int m_hauteurFenetre;

	clock_t t_mesX, t_mesY, t_mesZ;

    SDL_Window* m_fenetre;
    SDL_GLContext m_contexteOpenGL;	
    SDL_Event m_evenements;

};

#endif