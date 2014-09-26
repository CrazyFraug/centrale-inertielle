#include "Tools.h"
#include <fstream>
#include <hash_map>

#define _DEFINE_DEPRECATED_HASH_CLASSES 0

/*****************************************************************************************/

void writeHeading(std::string filename, std::fstream &myfile){
	myfile.open(filename.c_str(), std::ios::app);
	if (filename == "gyro.csv"){
		myfile << "Temps;Gyr_X;Gyr_Y;Gyr_Z\n";
	}
	else if (filename == "acce.csv"){
		myfile << "Temps;Acc_X;Acc_Y;Acc_Z\n";
	}
	else if (filename == "mnet.csv"){
		myfile << "Temps;Mag_X;Mag_Y;Mag_Z\n";
	}
	else if (filename == "orie.csv"){
		myfile << "Temps;Ori_X;Ori_Y;Ori_Z\n";
	}
	myfile.close();
}

/*****************************************************************************************/

void filefromSensor(std::string filename, std::fstream &myfile, vect4D data){

	if (!myfile.is_open()){
		myfile.open(filename, std::ios::app);
	}

	myfile << data.temps;
	myfile << ";";
	myfile << data.x;
	myfile << ";";
	myfile << data.y;
	myfile << ";";
	myfile << data.z;
	myfile << ";";

	myfile << "\n";
	myfile.flush();
}

/*****************************************************************************************/

vect4D readDatafromFile(std::string filename, std::fstream &myfile, int cursor)
{
	vect4D data = { 0, 0, 0, 0 };
	static stdext::hash_map<int, std::string> dataFromFile;	// Hash map contenant les valeurs du fichier
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
					val_t += dataFromFile[cursor][m];
				}
				else if (n == 1){
					val_x += dataFromFile[cursor][m];
				}
				else if (n == 2){
					val_y += dataFromFile[cursor][m];
				}
				else if (n == 3){
					val_z += dataFromFile[cursor][m];
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

/*****************************************************************************************/

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
	std::cout << "5 - Test " << std::endl;
	std::cout << "Le mode de simulation choisi : ";
	std::cin >> mode;
	std::cout << std::endl;

	while (mode != 0 && mode != 1 && mode != 2 && mode != 3 && mode != 4 && mode != 5)
	{
		std::cout << "Le mode choisi est incorrect : ";
		std::cin >> mode;
		std::cout << std::endl;
	}
	return mode;
}

/*****************************************************************************************/

void writeResult(double temps, vect3D angles_measure, vect3D angles_calcul, std::string fileResult, std::fstream &file_excel)
{
	if (!file_excel.is_open())
	{
		file_excel.open(fileResult, std::ios::app);
		file_excel << "Temps;X_Mesure;X_Calcul;Y_Mesure;Y_Calcul;Z_Mesure;Z_Calcul\n";
	}
	file_excel << temps << ";";
	file_excel << angles_measure.x << ";" << angles_calcul.x << ";";
	file_excel << angles_measure.y << ";" << angles_calcul.y << ";";
	file_excel << angles_measure.z << ";" << angles_calcul.z << "\n";
	file_excel.flush();
}

/*****************************************************************************************/

float string_to_double(const std::string& s)
{
	float x = 0;
	std::string::size_type sz;
	x = std::stod(s, &sz);
	return x;
}

/*****************************************************************************************/

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

/*****************************************************************************************/

matrix<double> kalmanTraitement(Kalman &rotation, matrix<double> value_cmd, matrix<double> value_obs){

	/********************************************************************/
	/*	Etape update les données du filtre de Kalman					*
	/********************************************************************/
	rotation.predictStep(value_cmd);
	/********************************************************************/


	/********************************************************************/
	/*	Etape update les données du filtre de Kalman					*
	/********************************************************************/
	matrix<double> estimate_result(4, 1, 0);
	estimate_result = rotation.updateStep(value_obs);
	return estimate_result;
	/********************************************************************/
}

/*****************************************************************************************/

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

/*****************************************************************************************/

void printMatrix(matrix<double> A){
	for (int i = 0; i < A.size1(); i++){
		_RPT0(0, "| ");
		for (int j = 0; j < A.size2(); j++){
			_RPT1(0, " %f ", A(i, j));
		}
		_RPT0(0, " |\n");
	}
}

/*****************************************************************************************/

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

/*****************************************************************************************/

void createMeasureFile_separate(std::string filename, std::string direction, double sampleTime)
{
	std::fstream fDirection, fMeas_gyro, fMeas_gyro_vrai, fMeas_acce, fMeas_acce_vrai, fMeas_orie, fMeas_orie_vrai;
	double variation1, variation2, variation3, bias1, bias2, bias3;
	vect3D orientation = {0,0,0};
	vect3D orientation_err = {0,0,0};
	vect3D acce = {0,0,0};
	bool bGyro_vrai(false), bAcce_vrai(false), bOrie_vrai(false);
	bool bGyro(true), bOrie(true), bAcce(true);
	quaternion<double> qOrie(1,0,0,0);
	quaternion<double> qOrie_err(1,0,0,0);
	quaternion<double> qRot(1,0,0,0);
	int i(0), j(0);

	fDirection.open(direction, std::ios::in);
	verifierOuverture(fDirection, "direction", _CRT_ERROR);
	
	getHeader(fDirection, variation1, variation2, variation3, bias1, bias2, bias3);


	fMeas_gyro.open((filename+"_gyro.txt"), std::fstream::out);
	if(!verifierOuverture(fMeas_gyro, "_gyro"))
		bGyro = false;

	fMeas_orie.open((filename+"_orie.txt"), std::fstream::out);
	if (!verifierOuverture(fMeas_gyro, "_orie"))
		bOrie = false;

	fMeas_acce.open((filename+"_acce.txt"), std::fstream::out);
	if(!verifierOuverture(fMeas_gyro, "_acce"))
		bAcce = false;


	double val1,val2,val3, tEcoule(0.0), temps, duree;
	double vitX, vitY, vitZ;

	if(bGyro)
		writeHeader(fMeas_gyro,filename, variation1, bias1);
	if(bOrie)
		writeHeader(fMeas_orie,filename, variation1, bias1);
	if(bAcce)
		writeHeader(fMeas_acce,filename, variation2, bias2);

	std::cout << "bGyro = " << bGyro << std::endl; 
	std::cout << "bOrie = " << bOrie << std::endl;
	std::cout << "bAcce = " << bAcce << std::endl;

	if (bGyro_vrai = ((variation1 !=0 || bias1 !=0) && bGyro)) //attribution + condition
	{
		std::cout << "bool gyro_vrai = true " << std::endl;
		fMeas_gyro_vrai.open(filename+"gyro_vrai.txt",std::fstream::out);
		if (!verifierOuverture(fMeas_gyro_vrai, "gyro_vrai"))
			bGyro_vrai = false;
		else
			writeHeader(fMeas_gyro_vrai, filename+"gyro_vrai", 0, 0);
	}
				
	if (bOrie_vrai = ((variation1 !=0 || bias1 !=0) && bOrie)) //attribution + condition
	{
		std::cout << "bool orie_vrai = true " << std::endl;
		fMeas_orie_vrai.open(filename+"orie_vrai.txt",std::fstream::out);
		if (!verifierOuverture(fMeas_orie_vrai, "orie_vrai"))
			bOrie_vrai = false;
		else
			writeHeader(fMeas_orie_vrai, filename+"orie_vrai.txt", 0, 0);
	}

	if (bAcce_vrai = ((variation2 !=0 || bias2 !=0) && bAcce)) //attribution + condition
	{
		std::cout << "bool acce_vrai = true " << std::endl;
		fMeas_acce_vrai.open(filename+"acce_vrai.txt",std::fstream::out);
		if (!verifierOuverture(fMeas_acce_vrai, "acce_vrai"))
			bAcce_vrai = false;
		else
			writeHeader(fMeas_acce_vrai, filename+"acce_vrai", 0, 0);
	}
		
	while (1)
	{
		system("PAUSE");
		if (!getDirection(fDirection, temps, duree, val1, val2, val3, vitX, vitY, vitZ))
		{
			j++;

			std::cout << "valeur 1 : "<< val1 << " ; valeur 2 : " << val2 << " ; valeur 3 : " << val3 << " ; temps : " << temps << " ; duree : " << duree << std::endl;
			std::cout << "vitesse x : "<< vitX << " ; vitesse y : " << vitY << " ; vitesse z : " << vitZ << std::endl;

			// Conversion des angles en vitesse angulaire //
			val1 /= duree/1000;
			val2 /= duree/1000;
			val3 /= duree/1000;

			if (temps < tEcoule) 
			{
				temps = tEcoule;
				_RPT0(_CRT_WARN, "temps indique inferieur au temps ecoule, temps remanie...\n");
			}

			while ( tEcoule < temps)
			{
				if(bGyro)
				{
					fMeas_gyro << 'x' << addError(0,variation1,bias1) << ';' << 'y' << addError(0,variation1,bias1) << ';' << 'z' << addError(0,variation1,bias1) << ';' << 't' << tEcoule << ';' << '\n';
					if(bGyro_vrai)
					{
						fMeas_gyro_vrai << 'x' << 0 << ';' << 'y' << 0 << ';' << 'z' << 0 << ';' << 't' << tEcoule << ';' << '\n';
					}
				}

				if(bOrie)
				{

					qRot = danglesToQuat(addError(0,variation2, bias2)*SAMPLETIME/1000, addError(0,variation2, bias2)*SAMPLETIME/1000, addError(0,variation2, bias2)*SAMPLETIME/1000);
					qOrie_err = qOrie_err*qRot;
					orientation_err = quatToAngles_deg(qOrie_err);
					fMeas_orie << 'x' << orientation_err.x << ';' << 'y' << orientation_err.y << ';' << 'z' << orientation_err.z << ';' << 't' << tEcoule << ';' << '\n';
					
					if(bGyro_vrai)
						fMeas_orie_vrai << 'x' << orientation.x << ';' << 'y' << orientation.y << ';' << 'z' << orientation.z << ';' << 't' << tEcoule << ';' << '\n';
				}

				if(bAcce)
				{
					calculateAccel(orientation.x, orientation.y, acce.x, acce.y, acce.z);
					fMeas_acce << 'x' << acce.x << ';' << 'y' << acce.y << ';' << 'z' << acce.z << ';' << 't' << tEcoule << ';' << '\n';

				}
		
				tEcoule += sampleTime;

			}

			while (tEcoule < temps+duree)
			{
				if (bGyro)
				{
					//Ajout des erreurs de mesure:
					fMeas_gyro << 'x' << addError(val1,variation1,bias1) << ';' << 'y' << addError(val2,variation1,bias1) << ';' << 'z' << addError(val3,variation1,bias1) << ';' << 't' << tEcoule << ';' << '\n';

					if(bGyro_vrai)
					{
						fMeas_gyro_vrai << 'x' << val1 << ';' << 'y' << val2 << ';' << 'z' << val3 << ';' << 't' << tEcoule << ';' << '\n';
					}
				}
				if(bOrie)
				{
					i++;
					qRot = danglesToQuat(addError(val1,variation2,bias2)*SAMPLETIME/1000, addError(val2,variation2,bias2)*SAMPLETIME/1000, addError(val3,variation2,bias2)*SAMPLETIME/1000);
					qOrie_err = qOrie_err*qRot;
					orientation_err = quatToAngles_deg(qOrie_err);
					fMeas_orie << 'x' << orientation_err.x << ';' << 'y' << orientation_err.y << ';' << 'z' << orientation_err.z << ';' << 't' << tEcoule << ';' << '\n';
					
					if(bGyro_vrai)
					{
						qRot = danglesToQuat(val1*SAMPLETIME/1000, val2*SAMPLETIME/1000, val3*SAMPLETIME/1000);
						qOrie = qOrie*qRot;
						orientation = quatToAngles_deg(qOrie);
						fMeas_orie_vrai << 'x' << orientation.x << ';' << 'y' << orientation.y << ';' << 'z' << orientation.z << ';' << 't' << tEcoule << ';' << '\n';
					}
				}
				if(bAcce)
				{
					calculateAccel(orientation.x, orientation.y, acce.x, acce.y, acce.z);
					acce.x += vitX*1000/duree;
					acce.y += vitY*1000/duree;
					acce.z += vitZ*1000/duree;
					fMeas_acce << 'x' << acce.x << ';' << 'y' << acce.y << ';' << 'z' << acce.z << ';' << 't' << tEcoule << ';' << '\n';

				}
		
				tEcoule += sampleTime;
			}


		} //end if(!getDirection())

		else break; // getDirection = true quand on atteint la fin du fichier ( file.eof() ). On sort alors de la boucle

	} //end while

	std::cout << i << "---i" <<std::endl;

	if(bGyro_vrai)
		fMeas_gyro_vrai.close();
	if(bOrie_vrai)
		fMeas_orie_vrai.close();
	if(bAcce_vrai)
		fMeas_acce_vrai.close();
	if(bGyro)
		fMeas_gyro.close();
	if(bOrie)
		fMeas_orie.close();
	if(bAcce)
		fMeas_acce.close();

	fDirection.close();
	system("PAUSE");

}

/*****************************************************************************************/

bool verifierOuverture (const std::fstream &file1, const char* name, int reportType)
{
	if ((file1.rdstate() && std::ifstream::failbit) != 0)
	{
		_RPT1(reportType, "Erreur lors de l'ouverture du fichier \'%s\'\n", name);
		return false;
	}
	else
		return true;
}

/*****************************************************************************************/

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

/*****************************************************************************************/

bool getHeader( std::fstream &file, double &variation1, double &variation2, double &variation3, double &bias1, double &bias2, double &bias3)
{
	char buffer[128];
	file >> buffer[0];

	while (buffer[0] == '%')
	{
		file.getline(buffer, 128);
		if (strcmp("variation", buffer) == 0) {
			file.get(buffer[0]);
			file >> variation1;
			file.get(buffer[0]);
			file >> variation2;
			file.get(buffer[0]);
			file >> variation3;
			std::cout << "variation1 = " << variation1 << "--- variation2 = " << variation2 << "--- variation3 = " << variation3 << std::endl;
			file.get(buffer[0]);//recupere le caractere de fin de ligne
		}
		else if (strcmp("biais",buffer) == 0) {
			file.get(buffer[0]);
			file >> bias1;
			file.get(buffer[0]);
			file >> bias2;
			file.get(buffer[0]);
			file >> bias3;
			std::cout << "biais1 = " << bias1 << "--- biais2 = " << bias2 << "---bias3 = " << bias3 << std::endl;
			file.get(buffer[0]);//recupere le caractere de fin de ligne
		}
		file.get(buffer[0]);
	}

	file.unget(); //recule le curseur de 2 caractères si le dernier caractère lu n'est pas un '%' (fin de ligne + premier caractère de la nouvelle ligne)
	file.unget(); 

	return true;
}

/*****************************************************************************************/

void writeHeader(std::fstream& file, std::string& name, const double& variation, const double& biais)
{
	file << '%' << name << '\n';
	file << '%' << "variation = " << variation << "; biais = " << biais << '\n';
}

/*****************************************************************************************/

bool getDirection(std::fstream &file, double &temps, double &duree, double &val1, double &val2, double &val3, double &vitX, double &vitY, double &vitZ)
{
	char buffer[128];

	file >> temps;
	file >> buffer[0];
	file >> duree;
	file >> buffer[0];
	file >> val1;
	file >> buffer[0];
	file >> val2;
	file >> buffer[0];
	file >> val3;
	file >> buffer[0];
	file >> vitX;
	file >> buffer[0];
	file >> vitY;
	file >> buffer[0];
	file >> vitZ;

	val1 /= (duree/1000.0);
	val2 /= (duree/1000.0);
	val3 /= (duree/1000.0);

	return file.eof();

}

/*****************************************************************************************/

double addError(double baseValeur, double variation, double bias)
{
	double meas = baseValeur - variation;
	double erreur = 0;
	erreur = (double)rand() / RAND_MAX; //generate random number between 0 and 1
	meas += erreur*(variation*2);
	return (meas+bias);
}

/*****************************************************************************************/

void calculateAccel(const double &phi, const double &teta, double &accX, double &accY, double &accZ)
{
	accX -= sin(teta)*cos(phi)*G;
	accY -= sin(phi)*G; 
	accZ -= cos(teta)*cos(phi)*G;
}

/*****************************************************************************************/


//Instrument *createInstrument(char* nomSensor, int mode){
//	Instrument *test;
//	if (mode == 1){
//		InstrumentSerie inst(nomSensor);
//		test = &inst;
//	}
//	else {
//		Instrument inst(nomSensor, 2);
//		test = &inst;
//	}
//	return test;
//}