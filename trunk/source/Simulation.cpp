#include "Simulation.h"



Simulation::~Simulation()
{

}

void Simulation::afficherSimulation(void){


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