#include "Traitement.h"


/**
*	\brief	Test de la fonction filefromSensor
*	\param	filename	string		le nom du fichier dans lequel on va �crire les donn�es - doit �tre en format "nom.txt"
*	\param	inst		Instrument*	l'instrument dont on va extraire les donn�es
*	\test	le test n'est pas encore valid�
*/
void test_filefromSensor(std::string* filename, Instrument* inst){
	/* D�claration des variables locaux */
	Traitement un_traitement(inst);
	char c = 0;
	bool test_non_valid(false), end_of_file(false);
	vect4D donnees;
	double result[4];
	std::string premier_ligne;
	std::fstream infile;
	/* Ouverture du fichier et r�cup�ration du premier ligne de pr�sentation */
	infile.open(*filename);
	getline(infile, premier_ligne);
	end_of_file = infile.eof();
	/* Tant que le test n'est pas valid� ou le fichier n'est pas totalement trait� */
	while (!test_non_valid && !end_of_file){
		un_traitement.filefromSensor(*filename, inst);
		donnees = inst->getMesures();
		/* On traite chaque ligne du fichier */
		while (c != '\n'){
			infile >> c;
			for (int i = 0; i < 4; i++){
				infile >> result[i];
				if (c == '|'){
					infile >> c;
					if (c == '|'){
						//infile >> result[i];
					}
				}
			}
		}
		/* Test si le r�sultat re�u correspond aux donn�es */
		if (result[1] != donnees.x || result[2] != donnees.y || result[3] != donnees.z || result[4] != donnees.temps){
			test_non_valid = true;
		}
	}
	infile.close();
	if (test_non_valid){
		std::cout << "Test non valid�, fichier mal �crit " << std::endl;
	}
	else{
		std::cout << "Test valid�, fichier bien �crit " << std::endl;
	}

}


/**
*	\brief	Test de la fonction readDatafromFile
*	\param	filename	string		le nom du fichier dont on va r�cup�rer les donn�es - doit �tre en format "nom.txt"
*	
*	\test	le test n'est pas encore valid�
*/

void test_readDatafromFile(std::string* filename){
	vect4D value_in,test_value;
	Traitement un_traitement(*filename);
	bool end_of_file(false);
	std::fstream myfile;
	myfile.open(*filename);

	std::cout << "Rentrer une valeur pour le test : " << std::endl;
	std::cout << "Valeur pour l'axe X : ";
	std::cin >> value_in.x;
	std::cout << "Valeur pour l'axe Y : ";
	std::cin >> value_in.y;
	std::cout << "Valeur pour l'axe Z : ";
	std::cin >> value_in.z;
	std::cout << "Valeur pour le temps : ";
	std::cin >> value_in.temps;
	myfile << "\n";
	
	while (!end_of_file){
		test_value = un_traitement.readDatafromFile(*filename);
	}

	if (test_value.x != value_in.x || test_value.y != value_in.y || test_value.z != value_in.z || test_value.temps != value_in.temps)
	{
		std::cout << " Test non valid�, erreur de lecture du fichier" << std::endl;
	}
	else{
		std::cout << " Test valid� " << std::endl;
	}
}