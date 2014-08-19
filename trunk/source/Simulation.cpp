#include "Simulation.h"


Simulation::Simulation(char* name, Traitement *un_Traitement)
{
	using namespace std;
	int mode = 0;
	cout << "MODE DE SIMULATION  DE  " << endl;
	cout << "1 - Simulation avec capteur " << endl;
	cout << "2 - Simulation avec fichier de donnees" << endl;
	cout << "Veuillez choisir le mode de simulation :" << endl;
	while (mode != 1 && mode != 2){
		cout << "Le mode de simulation doit etre 1 ou 2 : ";
		cin >> mode;
	}
	_nameSensor = name;
	if (mode == 1){
		_traitement = un_Traitement;
		_traitement->afficherTraitement();
		writeHeading(_traitement->getInstrument()->getnomfichier());
		_modeSimulation = mode;
	}
	else{
		Instrument un_Instrument(name);
		Traitement trait_file(&un_Instrument);
		_traitement = &trait_file;
		_modeSimulation = mode;
	}
}


Simulation::~Simulation()
{

}

void Simulation::afficherSimulation(void){


}

vect4D Simulation::getData(int turn)
{
	vect4D data;
	std::string filename;
	if (_nameSensor == "acce"){
		filename = "acce.csv";
	}
	else if (_nameSensor == "gyro"){
		filename = "gyro.csv";
	}
	else if (_nameSensor == "mnet"){
		filename = "magn.csv";
	}
	else if (_nameSensor == "orie"){
		filename = "orientation.csv";
	}
	/* Mode de simulation avec capteur */
	if (_modeSimulation == 1){
		_traitement->afficherTraitement();
		_traitement->stockerValeurs();

		data = _traitement->getInstrument()->getMesures();
		_RPT4(0, "VALUE : %f %f %f %f \n", data.x, data.y, data.z, data.temps);
	}
	/* Mode de simulation avec fichier de données */
	else
	{
		data = _traitement->readDatafromFile(filename, turn);
		_RPT1(0, "x: %f \n", data.x);
		_RPT1(0, "y: %f \n", data.y);
		_RPT1(0, "z: %f \n", data.z);
		_RPT1(0, "t: %f \n", data.temps);
	}
	return data;
}

void writeResult(double temps, vect3D angles_measure, vect3D angles_calcul, std::string fileResult, bool headingWrited)
{
	std::fstream file_excel;
	file_excel.open(fileResult, std::ios::app);
	if (!headingWrited){
		file_excel << "Temps;X_Mesure;X_Calcul;Y_Mesure;Y_Calcul;Z_Mesure;Z_Calcul\n";
	}
	file_excel << temps << ";";
	file_excel << angles_measure.x << ";" << angles_calcul.x << ";";
	file_excel << angles_measure.y << ";" << angles_calcul.y << ";";
	file_excel << angles_measure.z << ";" << angles_calcul.z << "\n";
	file_excel.close();
}
