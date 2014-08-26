#ifndef _TEST_TRAITEMENT_H
#define _TEST_TRAITEMENT_H
#include "Traitement.h"


/**
*	\brief	Test de la fonction filefromSensor
*	\param	filename	string		le nom du fichier dans lequel on va écrire les données - doit être en format "nom.txt"
*	\param	inst		Instrument*	l'instrument dont on va extraire les données
*	\test	le test		TEST VALIDE
*/
bool test_filefromSensor(std::string filename,Instrument* inst, Traitement un_traitement){
	/* Déclaration des variables locaux */
	char c = 0;
	bool test_non_valid(false), finish_line(false);
	vect4D donnees;
	double result[4];
	std::string premier_ligne,end_line;
	std::fstream infile;
	/* Ouverture du fichier et récupération du premier ligne de présentation */
	infile.open(filename);
	un_traitement.writeHeading(filename);
	getline(infile, premier_ligne);
	/* Tant que le test n'est pas validé ou le fichier n'est pas totalement traité */
	while (!test_non_valid || !finish_line){
		un_traitement.filefromSensor(filename, inst);
		donnees = inst->getMesures();
		/* On traite chaque ligne du fichier */
		for (int i = 1; i <= 4; i++){
			infile >> c;
			if (c == '|'){
				infile >> c;
				if (c == '|'){
					infile >> result[i - 1];
				}
			}
			else{
				infile >> result[i - 1];
			}
		}
		getline(infile, end_line);
		finish_line == true;
		_RPT1(0, "donnees_x : %f \n", donnees.x);
		_RPT1(0, "donnees_y : %f \n", donnees.y);
		_RPT1(0, "donnees_z : %f \n", donnees.z);
		_RPT1(0, "donnees_temps : %f \n", donnees.temps);
		_RPT1(0, "result_x : %f \n", result[0]);
		_RPT1(0, "result_y : %f \n", result[1]);
		_RPT1(0, "result_z : %f \n", result[2]);
		_RPT1(0, "result_temps : %f \n", result[3]);
		/* Test si le résultat reçu correspond aux données */
		if (result[0] != donnees.x || result[1] != donnees.y || result[2] != donnees.z || result[3] != donnees.temps){
			test_non_valid = true;
			_RPT0(_CRT_ERROR,"Test non validé \n");
		}
		else{
			_RPT0(0, "Test ok \n");
		}
	}
	return (!test_non_valid);
}


/**
*	\brief	Test de la fonction readDatafromFile
*	\param	filename	string		le nom du fichier dont on va récupérer les données - doit être en format "nom.txt"
*	
*	\test	le test n'est pas encore validé
*/

void test_readDatafromFile(std::string filename,int turn){
	vect4D value_in,test_value;
	Traitement un_traitement(filename);
	bool end_of_file(false);
	std::fstream myfile;
	myfile.open(filename);
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
		_RPT0(0, "Test non validé, problème lecture du fichier \n");
	}
	else{
		_RPT0(0, "Test validé");
	}
}

//int main(){
//	std::string port = "COM8";
//	int baud = 115200;
//	Serial data_serie(port, baud);
//	Instrument inst("gyro",&data_serie);
//	Traitement un_traitement(&inst);
//
//	std::string filename = "gyro.txt";
//	_RPT0(0, "Test de l'écriture de fichier \n");
//	while (1){
//		un_traitement.stockerValeurs();
//		test_filefromSensor(filename, &inst, un_traitement);
//	}
//}
#endif //_TEST_TRAITEMENT_H