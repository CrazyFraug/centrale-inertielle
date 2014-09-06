#ifndef TOOLS_H
#define TOOLS_H

#include "Mobile.h"
#include "Kalman.h"

void writeHeading(std::string filename, std::fstream &myfile);
void filefromSensor(std::string filename, std::fstream &myfile, vect4D data);
vect4D readDatafromFile(std::string filename, std::fstream &myfile, int turns);
int choiceMode();
void writeResult(double temps, vect3D angles_measure, vect3D angles_calcul, std::string fileResult, bool headingWrited);
float string_to_double(const std::string& s);
double norm_Angle(double alpha);


matrix<double> product_matrix(matrix<double> M1, matrix<double> M2);
void printMatrix(matrix<double> A);

matrix<double> kalmanTraitement(Kalman &rotation, matrix<double> value_cmd, matrix<double> value_obs);

#endif	//TOOLS_H
