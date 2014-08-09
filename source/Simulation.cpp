#include "Simulation.h"


Simulation::Simulation(char* name)
{
	using namespace std;
	int mode = 0;
	cout << "MODE DE SIMULATION  DE  " << name << endl;
	cout << "1 - Simulation avec capteur " << endl;
	cout << "2 - Simulation avec fichier de donnees" << endl;
	cout << "Veuillez choisir le mode de simulation :" << endl;
	while (mode != 1 && mode != 2){
		cout << "Le mode de simulation doit etre 1 ou 2 : ";
		cin >> mode;
	}
	_nameSensor = name;
	if (mode == 1){
		Serial link_serie("COM8", 115200);
		_link = &link_serie;
		if (_nameSensor == "acce"){
			Instrument_serie acce(name, _link);
			_inst = &acce;
			Traitement traitement_acce(_inst);
			_traitement = &traitement_acce;
			writeHeading(acce.getnomfichier());
		}
		else if (_nameSensor == "gyro"){
			Instrument_serie gyro(name, _link);
			_inst = &gyro;
			Traitement traitement_gyro(_inst);
			_traitement = &traitement_gyro;
			writeHeading(gyro.getnomfichier());
		}
		else if (_nameSensor == "mnet"){
			Instrument_serie magn(name, _link);
			_inst = &magn;
			Traitement traitement_magn(_inst);
			_traitement = &traitement_magn;
			writeHeading(magn.getnomfichier());
		}
		else if (_nameSensor == "orie"){
			Instrument_serie orie(name, _link);
			_inst = &orie;
			Traitement traitement_orie(_inst);
			_traitement = &traitement_orie;
			writeHeading(orie.getnomfichier());
		}
	}
	else{
		Instrument un_Instrument(name);
		Traitement trait_file(&un_Instrument);
		_traitement = &trait_file;
	}
	_modeSimulation = mode;
}


Simulation::~Simulation()
{
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
	Instrument *un_Instrument = _traitement->getInstrument();
	/* Mode de simulation avec capteur */
	if (_modeSimulation == 1){
		_traitement->stockerValeurs();
		_traitement->filefromSensor(filename, un_Instrument);
		un_Instrument->afficherMesures();
		data = un_Instrument->getMesures();
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

void writeResult(double temps, vect3D angles_measure, vect3D angles_calcul, std::string fileResult)
{
	std::fstream file_excel;
	file_excel.open(fileResult, std::ios::app);
	file_excel << temps << ";";
	file_excel << angles_measure.x << ";" << angles_calcul.x << ";";
	file_excel << angles_measure.y << ";" << angles_calcul.y << ";";
	file_excel << angles_measure.z << ";" << angles_calcul.z << "\n";
	file_excel.close();
}