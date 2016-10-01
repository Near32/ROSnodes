#ifndef DEF_CUBE
#define DEF_CUBE


// Includes OpenGL

#ifdef WIN32
#include <GL/glew.h>

#else
#define GL_PROTOTYPES 1
#include <GL/gl.h>

#endif


// Includes GLM

#include <gl/glm.h>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>


// Includes

#include "Shader.h"


// Classe Cube

class Cube
{
    public:

    Cube(float taille, std::string const vertexShader, std::string const fragmentShader);
    ~Cube();

    void afficher(glm::mat4 &projection, glm::mat4 &modelview);


    protected:

    Shader m_shader;
    float m_vertices[108];
    float m_couleurs[108];
};

#endif
