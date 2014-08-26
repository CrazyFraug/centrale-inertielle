#ifndef SIMULATION_H
#define SIMULATION_H


#pragma once
#include "Traitement.h"
class Simulation
{
public:
	Simulation(char *name, Traitement *un_Traitement);
	~Simulation();
	vect4D getData(int turn);
	void afficherSimulation(void);
private:
	Traitement *_traitement;
	int _modeSimulation;
	char *_nameSensor;
};
void writeResult(double temps, vect3D angles_measure, vect3D angles_calcul, std::string fileResult, bool headingWrited);
#endif // SIMULATION_H