#pragma once
#include "Traitement.h"
class Simulation
{
public:
	Simulation(char *name);
	~Simulation();
	vect4D getData(int turn);
private:
	Instrument *_inst;
	Traitement *_traitement;
	int _modeSimulation;
	char *_nameSensor;
	Serial *_link;
};
void writeResult(double temps, vect3D angles_measure, vect3D angles_calcul, std::string fileResult);
