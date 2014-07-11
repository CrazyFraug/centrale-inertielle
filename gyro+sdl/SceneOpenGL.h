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

#include "Shader.h" 
#include "Cube.h"

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

    SDL_Window* m_fenetre;
    SDL_GLContext m_contexteOpenGL;	
    SDL_Event m_evenements;

};

#endif