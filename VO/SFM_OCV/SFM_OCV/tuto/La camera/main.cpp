#include "SceneOpenGL.h"


int main(int argc, char **argv)
{
    // Création de la sène

    SceneOpenGL scene("OpenGL", 800, 600);


    // Initialisation de la scène

    if(scene.initialiserFenetre() == false)
	return -1;

    if(scene.initGL() == false)
	return -1;


    // Boucle Principale

    scene.bouclePrincipale();


    // Fin du programme

    return 0;
}
