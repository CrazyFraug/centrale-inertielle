#ifndef _TEST_TRAITEMENT_H
#define _TEST_TRAITEMENT_H
#include "Traitement.h"


/**
*	\brief	Test de la fonction filefromSensor
*	\param	filename	string		le nom du fichier dans lequel on va �crire les donn�es - doit �tre en format "nom.txt"
*	\param	inst		Instrument*	l'instrument dont on va extraire les donn�es
*	\test	le test		TEST VALIDE
*/
bool test_filefromSensor(std::string filename, Instrument* inst){
	/* D�claration des variables locaux */
	Traitement un_traitement(inst);
	char c = 0;
	bool test_non_valid(false), finish_line(false);
	vect4D donnees;
	double result[4];
	std::string premier_ligne;
	std::fstream infile;
	/* Ouverture du fichier et r�cup�ration du premier ligne de pr�sentation */
	infile.open(filename);
	getline(infile, premier_ligne);
	/* Tant que le test n'est pas valid� ou le fichier n'est pas totalement trait� */
	while (!test_non_valid || !finish_line){
		un_traitement.filefromSensor(filename, inst);
		donnees = inst->getMesures();
		/* On traite chaque ligne du fichier */
		for (int i = 1; i <= 4; i++){
			infile >> c;
			if (c == '|'){
				infile>> c;
				if (c == '|'){
					infile >> result[i - 1];
				}
			}
			else{
				infile >> result[i - 1];
			}
		}
		finish_line == true;
		/* Test si le r�sultat re�u correspond aux donn�es */
		if (result[0] != donnees.x || result[1] != donnees.y || result[2] != donnees.z || result[3] != donnees.temps){
			test_non_valid = true;
			_RPT0(_CRT_ERROR,"Test non valid� \n");
		}
		else{
			_RPT0(0, "Test ok \n");
		}
	}
	return (!test_non_valid);
}


/**
*	\brief	Test de la fonction readDatafromFile
*	\param	filename	string		le nom du fichier dont on va r�cup�rer les donn�es - doit �tre en format "nom.txt"
*	
*	\test	le test n'est pas encore valid�
*/

void test_readDatafromFile(std::string filename){
	vect4D value_in,test_value;
	Traitement un_traitement(filename);
	bool end_of_file(false);
	std::fstream myfile;
	myfile.open(filename);
	int turn = 0;
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
		test_value = un_traitement.readDatafromFile(filename,turn);
	}

	if (test_value.x != value_in.x || test_value.y != value_in.y || test_value.z != value_in.z || test_value.temps != value_in.temps)
	{
		_RPT0(0, "Test non valid�, probl�me lecture du fichier \n");
	}
	else{
		_RPT0(0, "Test valid�");
	}
}

#endif //_TEST_TRAITEMENT_H