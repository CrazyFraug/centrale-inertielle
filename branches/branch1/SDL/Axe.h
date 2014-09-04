#ifndef DEF_AXE
#define DEF_AXE

#include <iostream>
#include <string>

// Includes OpenGL
#include <SDL2\SDL.h>
#include <GL\glew.h>

//glm libs
#define GLM_FORCE_RADIANS
#include <glm.hpp>
#include <gtx\transform.hpp>
#include <gtc\type_ptr.hpp>

#include "Shader.h"

// Classe Cube
class Axe
{
    public:

    Axe(float hauteur, float largeur, std::string const vertexShader, std::string const fragmentShader);
    ~Axe();
	void afficher(glm::mat4 &projection, glm::mat4 &modelview); 

    private:

    Shader m_shader;
    float m_vertices[108];
    float m_couleurs[108];
};

#endif