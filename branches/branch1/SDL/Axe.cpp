#include "Axe.h"

using namespace glm;
using namespace std;

Axe::Axe(float hauteur, float largeur, std::string const vertexShader, std::string const fragmentShader) : m_shader(vertexShader, fragmentShader)
{
    // Chargement du shader
    m_shader.charger();

    // Division de la taille
    largeur /= 2;

    // Vertices temporaires
    float verticesTmp[] = {-largeur, -largeur, 0,			largeur, -largeur, 0,			largeur, largeur, 0,		// Face 1 = face du bas
                           -largeur, -largeur, 0,			-largeur, largeur, 0,			largeur, largeur, 0,		// Face 1

                           largeur, -largeur, 0,			largeur, -largeur, hauteur,		largeur, largeur, 0,		// Face 2 = face de droite
                           largeur, -largeur, hauteur,		largeur, largeur, 0,			largeur, largeur, -hauteur, // Face 2

                           -largeur, -largeur, hauteur,		largeur, -largeur, hauteur,		largeur, -largeur, 0,		// Face 3 = face du fond
                           -largeur, -largeur, hauteur,		-largeur, -largeur, 0,			largeur, -largeur, 0,		// Face 3

                           -largeur, -largeur, hauteur,		largeur, -largeur, hauteur,		largeur, largeur, hauteur,  // Face 4 = face du  haut
                           -largeur, -largeur, hauteur,		-largeur, largeur, hauteur,		largeur, largeur, hauteur,  // Face 4

                           -largeur, -largeur, 0,			-largeur, -largeur, hauteur,	-largeur, largeur, hauteur,	// Face 5 = face de gauche
                           -largeur, -largeur, 0,			-largeur, largeur, 0,			-largeur, largeur, hauteur,	// Face 5

                           -largeur, largeur, hauteur,		largeur, largeur, hauteur,		largeur, largeur, 0,		// Face 6
                           -largeur, largeur, hauteur,		-largeur, largeur, 0,			largeur, largeur, 0};		// Face 6


    // Couleurs temporaires
    float couleursTmp[] = {
		                   0.0, 1.0, 0.0,   0.0, 1.0, 0.0,   0.0, 1.0, 0.0,          
                           0.0, 1.0, 0.0,   0.0, 1.0, 0.0,   0.0, 1.0, 0.0, 
						   
						   0.0, 0.0, 1.0,   0.0, 0.0, 1.0,   0.0, 0.0, 1.0,
						   0.0, 0.0, 1.0,   0.0, 0.0, 1.0,   0.0, 0.0, 1.0,
		
						   1.0, 0.0, 0.0,   1.0, 0.0, 0.0,   1.0, 0.0, 0.0,           // Face 1
                           1.0, 0.0, 0.0,   1.0, 0.0, 0.0,   1.0, 0.0, 0.0,           // Face 1

                           0.0, 1.0, 0.0,   0.0, 1.0, 0.0,   0.0, 1.0, 0.0,           // Face 2
                           0.0, 1.0, 0.0,   0.0, 1.0, 0.0,   0.0, 1.0, 0.0,           // Face 2

                           0.0, 0.0, 1.0,   0.0, 0.0, 1.0,   0.0, 0.0, 1.0,           // Face 3
                           0.0, 0.0, 1.0,   0.0, 0.0, 1.0,   0.0, 0.0, 1.0,           // Face 3

                           1.0, 0.0, 0.0,   1.0, 0.0, 0.0,   1.0, 0.0, 0.0,           // Face 4
                           1.0, 0.0, 0.0,   1.0, 0.0, 0.0,   1.0, 0.0, 0.0,           // Face 4

                           0.0, 1.0, 0.0,   0.0, 1.0, 0.0,   0.0, 1.0, 0.0,           // Face 5
                           0.0, 1.0, 0.0,   0.0, 1.0, 0.0,   0.0, 1.0, 0.0           // Face 5
                           };         


    // Copie des valeurs dans les tableaux finaux

    for(int i(0); i < 108; i++)
    {
        m_vertices[i] = verticesTmp[i];
        m_couleurs[i] = couleursTmp[i];
    }
}

Axe::~Axe() { }



void Axe::afficher(glm::mat4 &projection, glm::mat4 &modelview)
{
    // Activation du shader
    glUseProgram(m_shader.getProgramID());


        // Envoi des vertices
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, m_vertices);
        glEnableVertexAttribArray(0);

        // Envoi de la couleur
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, m_couleurs);
        glEnableVertexAttribArray(1);

        // Envoi des matrices
        glUniformMatrix4fv(glGetUniformLocation(m_shader.getProgramID(), "projection"), 1, GL_FALSE, value_ptr(projection));
        glUniformMatrix4fv(glGetUniformLocation(m_shader.getProgramID(), "modelview"), 1, GL_FALSE, value_ptr(modelview));

        // Rendu
        glDrawArrays(GL_TRIANGLES, 0, 36);

        // Désactivation des tableaux
        glDisableVertexAttribArray(1);
        glDisableVertexAttribArray(0);


    // Désactivation du shader
    glUseProgram(0);
}