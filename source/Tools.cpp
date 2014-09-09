#include "Tools.h"
#include <fstream>
#include <hash_map>

#define _DEFINE_DEPRECATED_HASH_CLASSES 0
/** A CHANGER
*	\brief Ecrire les données récupérées d'un capteur dans un fichier
*
*	\param filename	string		le nom du fichier - doit être en format "nom.txt" et doit être ouvert
*   \param inst		Instrument	le capteur  dont on récupère les données
*
*   \test  test_filefromSensor
*/
void writeHeading(std::string filename, std::fstream &myfile){
	myfile.open(filename.c_str(), std::ios::app);
	if (filename == "gyro.csv"){
		myfile << "Gyr_X;Gyr_Y;Gyr_Z;temps\n";
	}
	else if (filename == "acce.csv"){
		myfile << "Acc_X;Acc_Y;Acc_Z;temps\n";
	}
	else if (filename == "mnet.csv"){
		myfile << "Mag_X;Mag_Y;Mag_Z;temps\n";
	}
	else if (filename == "orie.csv"){
		myfile << "Ori_X;Ori_Y;Ori_Z;temps\n";
	}
	myfile.close();
}

/**
* \brief Ecrit dans un fichier les mesures prises par le capteur
*/
void filefromSensor(std::string filename, std::fstream &myfile, vect4D data){

	if (!myfile.is_open()){
		myfile.open(filename, std::ios::app);
	}

	myfile << data.x;
	myfile << ";";
	myfile << data.y;
	myfile << ";";
	myfile << data.z;
	myfile << ";";
	myfile << data.temps;
	myfile << ";";

	myfile << "\n";
	myfile.flush();
}

/**
*	\brief Lire les données à partir d'un fichier
*	\param	filename	string		le nom du fichier - doit être en format "nom.txt"
*   \return data		vect4D		vecteur de 4 éléments (données selon l'axe x,y,z et le temps) pour un traitement
*   \test  test_readDatafromFile
*/
vect4D readDatafromFile(std::string filename, std::fstream &myfile, int cursor)
{
	vect4D data = { 0, 0, 0, 0 };
	static stdext::hash_map<int, std::string> dataFromFile;
	int i = 0;
	std::string chaine;
	if (!myfile.is_open()){
		myfile.open(filename, std::ios::in || std::ios::out);
		while (myfile.eof() == false)
		{
			/* Enlève l'entête du fichier */
			std::getline(myfile, chaine);
			dataFromFile[i] = chaine;
			/* Récupération des données du fichier tant que ce n'est pas la fin d'une ligne */
			i++;
		}
	}

	int m = 0;
	std::string val_x, val_y, val_z, val_t;
	if (cursor < dataFromFile.size() - 1){
		for (int n = 0; n < 4; n++){

			while (dataFromFile[cursor][m] != ';')
			{
				if (n == 0){
					val_x += dataFromFile[cursor][m];
				}
				else if (n == 1){
					val_y += dataFromFile[cursor][m];
				}
				else if (n == 2){
					val_z += dataFromFile[cursor][m];
				}
				else if (n == 3){
					val_t += dataFromFile[cursor][m];
				}

				m++;
			}
			m++;
		}
		data.x = string_to_double(val_x);
		data.y = string_to_double(val_y);
		data.z = string_to_double(val_z);
		data.temps = string_to_double(val_t);
	}
	else{
		data.x = data.y = data.z = data.temps = 0;
	}
#ifdef TEST
	_RPT1(0, "valeur x = %f\n", data.x);
	_RPT1(0, "valeur y = %f\n", data.y);
	_RPT1(0, "valeur z = %f\n", data.z);
	_RPT1(0, "valeur t = %f\n", data.temps);
#endif

	return data;
}



int choiceMode(){

	int mode;
	HWND hwnd = GetConsoleWindow();
	SetForegroundWindow(hwnd);

	system("cls");
	std::cout << "MODE DE SIMULATION : " << std::endl;
	std::cout << "0 - Quitter" << std::endl;
	std::cout << "1 - Simulation avec capteur" << std::endl;
	std::cout << "2 - Simulation avec fichier de valeurs" << std::endl;
	std::cout << "3 - Generer fichier de valeurs" << std::endl;
	std::cout << "4 - Generer fichier a partir de la liaison serie" << std::endl;
	std::cout << "5 - Test conversion quaternion" << std::endl;
	std::cout << "Le mode de simulation choisi : ";
	std::cin >> mode;
	std::cout << std::endl;

	while (mode != 1 && mode != 2 && mode != 3 && mode != 0 && mode != 4 && mode != 5)
	{
		std::cout << "Le mode choisi est incorrect : ";
		std::cin >> mode;
		std::cout << std::endl;
	}
	return mode;
}

void writeResult(double temps, vect3D angles_measure, vect3D angles_calcul, std::string fileResult, bool headingWrited)
{
	static std::fstream file_excel;
	if (!file_excel.is_open())
		file_excel.open(fileResult, std::ios::app);
	if (!headingWrited){
		file_excel << "Temps;X_Mesure;X_Calcul;Y_Mesure;Y_Calcul;Z_Mesure;Z_Calcul\n";
	}
	file_excel << temps << ";";
	file_excel << angles_measure.x << ";" << angles_calcul.x << ";";
	file_excel << angles_measure.y << ";" << angles_calcul.y << ";";
	file_excel << angles_measure.z << ";" << angles_calcul.z << "\n";
	file_excel.flush();
}

/**
* Convert a string into a double type variable
* return 0 if function failed
*/
float string_to_double(const std::string& s)
{
	std::stringstream convert(s);
	float x = 0;

	if (!(convert >> x))
	{
		_RPT0(_CRT_ERROR, "Problem Conversion string to double \n");
		return 0;
	}
	return x;
}

double normAngle(double alpha)
{
	double result = 0;
	while (alpha > 180 || alpha <= -180)
	{
		if (alpha > 180)
			result = alpha - 180;
		if (alpha <= -180)
			result = alpha + 180;
	}
	return result;
}

/** \brief	Filtre de Kalman avec les étapes de prédiction et mettre à jour d'état

*	\param	Kalman			&rotation		--	Système à filtrer
\param	matrix<double>	value_cmd		--	matrice contenant les données de la commande
\param	matrix<double>	value_obs		--	matrice contenant les données d'observation

*	\return matrix<double>	estimate_result	--	matrice contenant l'état estimé

*	\test	test_Kalman_rotation	à tester les constants pour valider l'algorithme!

*/

matrix<double> kalmanTraitement(Kalman &rotation, matrix<double> value_cmd, matrix<double> value_obs){

	/********************************************************************/
	/*	Etape update les données du filtre de Kalman					*
	/********************************************************************/
	rotation.predict_step(value_cmd);
	/********************************************************************/


	/********************************************************************/
	/*	Etape update les données du filtre de Kalman					*
	/********************************************************************/
	matrix<double> estimate_result(4, 1, 0);
	estimate_result = rotation.update_step(value_obs);
	return estimate_result;
	/********************************************************************/
}


/** \brief	Produit de deux matrices

*	\param	matrix<double>	M1		--	Matrice gauche de taille m x n
\param	matrix<double>	M2		--	Matrice gauche de taille n x o

*	\return	matrix<double>	result	-- Résultat du produit de taille m x o (cas normal) || Message d'erreur si la taille des matrices ne convient pas
*/
matrix<double> product_matrix(matrix<double> M1, matrix<double> M2){
	if (M1.size2() != M2.size1()){
		_RPT0(_CRT_ERROR, "ERROR SIZE MATRIX \n");
	}
	else{
		matrix<double> result(M1.size1(), M2.size2(), 0);
		for (int i = 0; i < M1.size1(); i++){
			for (int j = 0; j < M2.size2(); j++){
				for (int k = 0; k < M1.size2(); k++){
					result(i, j) += M1(i, k)*M2(k, j);
				}
			}
		}
		return result;
	}
}

void printMatrix(matrix<double> A){
	for (int i = 0; i < A.size1(); i++){
		_RPT0(0, "| ");
		for (int j = 0; j < A.size2(); j++){
			_RPT1(0, " %f ", A(i, j));
		}
		_RPT0(0, " |\n");
	}
}


void createMeasureFile(std::string filename, std::string direction, double sampleTime, double variation, double bias)
{
	std::fstream fDirection, fMeas;
	vect3D orientation = { 0, 0, 0 };
	fDirection.open(direction, std::ios::in);
	fMeas.open(filename, std::fstream::out);
	if ((fDirection.rdstate() && std::ifstream::failbit) != 0)
	{
		_RPT0(_CRT_ERROR, "Erreur lors de l'ouverture du fichier direction\n");
	}
	else if ((fMeas.rdstate() && std::ifstream::failbit) != 0)
	{
		_RPT0(0, "Fichier directions ouvert correctement\n");
		_RPT0(_CRT_ERROR, "Erreur lors de l'ouverture du fichier de mesure\n");
		fDirection.close();
	}
	else
	{
		_RPT0(0, "Fichier mesures ouvert correctement\n");
		double val1, val2, val3, tEcoule(0.0), temps, duree;
		double val1e, val2e, val3e;
		while (fDirection.eof() == false)
		{
			getDirection(fDirection, val1, val2, val3, temps, duree);
			std::cout << "valeur 1 : " << val1 << " ; valeur 2 : " << val2 << " ; valeur 3 : " << val3 << " ; temps : " << temps << " ; duree : " << duree << std::endl;

			if (temps >= tEcoule)
			{
				while (tEcoule < temps)
				{
					fMeas << "gyro:" << '\n';
					fMeas << 'x' << addError(0, variation, bias) << ';' << 'y' << addError(0, variation, bias) << ';' << 'z' << addError(0, variation, bias) << ';' << 't' << tEcoule << ';' << '\n';
					fMeas << "orie;" << '\n';
					fMeas << 'x' << 0 << ';' << 'y' << 0 << ';' << 'z' << 0 << ';' << 't' << tEcoule << ';' << '\n';
					tEcoule += sampleTime;
				}

				while (tEcoule < temps + duree)
				{
					//Ajout des erreurs de mesure:
					val1e = addError(val1, variation, bias);
					val2e = addError(val2, variation, bias);
					val3e = addError(val3, variation, bias);
					fMeas << "gyro:" << '\n';
					fMeas << 'x' << val1e << ';' << 'y' << val2e << ';' << 'z' << val3e << ';' << 't' << tEcoule << ';' << '\n';
					orientation.x += val1*SAMPLETIME / 1000;
					orientation.y += val2*SAMPLETIME / 1000;
					orientation.z += val3*SAMPLETIME / 1000;
					fMeas << "orie:" << '\n';
					fMeas << 'x' << orientation.x << ';' << 'y' << orientation.y << ';' << 'z' << orientation.z << ';' << 't' << tEcoule << ';' << '\n';
					tEcoule += sampleTime;
				}
			}

			else
			{
				_RPT0(0, "temps inferieur a celui de la derniere ligne + duree, temps remanié");
				temps = tEcoule;

				while (tEcoule < temps + duree)
				{
					val1e = addError(val1, variation, bias);
					val2e = addError(val2, variation, bias);
					val3e = addError(val3, variation, bias);
					fMeas << "gyro:" << '\n';
					fMeas << 'x' << val1e << ';' << 'y' << val2e << ';' << 'z' << val3e << ';' << 't' << tEcoule << ';' << '\n';
					tEcoule += sampleTime;
				}
			}
		}

		fMeas.close();
		fDirection.close();
		system("PAUSE");
	}

}

void fileFromSerial(std::string filename, Serial &link, int nbMes)
{
	std::fstream file;
	std::string tmp;
	int i = 0;
	file.open(filename, std::ios::out);
	static HANDLE h = NULL;
	h = GetStdHandle(STD_OUTPUT_HANDLE);
	COORD c = { 0, 7 };
	while (i++ < nbMes)
	{
		tmp = link.readLine();
		file << tmp << '\n';
		SetConsoleCursorPosition(h, c);
		std::cout << tmp << std::endl;
	}
	file.close();
}

void getDirection(std::fstream &file, double &val1, double &val2, double &val3, double &temps, double &duree)
{
	char c;
	file >> temps;
	file >> c;
	file >> duree;
	file >> c;
	file >> val1;
	file >> c;
	file >> val2;
	file >> c;
	file >> val3;

	val1 /= (duree / 1000.0);
	val2 /= (duree / 1000.0);
	val3 /= (duree / 1000.0);

}


double addError(double baseValeur, double variation, double bias)
{
	double meas = baseValeur - variation;
	double erreur = 0;
	erreur = (double)rand() / RAND_MAX; //generate random number between 0 and 1
	meas += erreur*(variation * 2);
	return (meas + bias);
}
