#include "SceneOpenGL.h"

#define NB_SENSOR 4

int main(int argc, char *argv[]) {

	int mode = 0;
	int nb_capteur;
	SceneOpenGL scene("letitre", 800, 600);

	if (scene.InitialiserFenetre() == false) { system("PAUSE"); return -1; }

	if (!scene.iniGL()) 	{ system("PAUSE"); return -1; }

	mode = choiceMode();
	if (mode == 1){
		scene.bouclePrincipaleSensor();
	}
	else if (mode == 2){
		scene.bouclePrincipaleSimu();
	}


	return 0;
}
