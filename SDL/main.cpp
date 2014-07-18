#include "SceneOpenGL.h"

int main(int argc, char *argv[]) {

	SceneOpenGL scene("letitre", 800, 600);

	if(scene.InitialiserFenetre()==false) {system("PAUSE"); return -1; }

	if(!scene.iniGL()) 	{system("PAUSE"); return -1;}

	scene.bouclePrincipale();
	
	return 0 ;
}
