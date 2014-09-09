#ifndef TOOLS_H
#define TOOLS_H

#include "Serial.h"
#include "Mobile.h"
#include "Kalman.h"

#define SAMPLETIME 20
#define FILENAME "simMeasures.txt"
#define DIRECTIONS "directions.txt"


void writeHeading(std::string filename, std::fstream &myfile);
void filefromSensor(std::string filename, std::fstream &myfile, vect4D data);
vect4D readDatafromFile(std::string filename, std::fstream &myfile, int turns);
int choiceMode();
void writeResult(double temps, vect3D angles_measure, vect3D angles_calcul, std::string fileResult, bool headingWrited);
float string_to_double(const std::string& s);
double normAngle(double alpha);

matrix<double> product_matrix(matrix<double> M1, matrix<double> M2);
void printMatrix(matrix<double> A);

matrix<double> kalmanTraitement(Kalman &rotation, matrix<double> value_cmd, matrix<double> value_obs);



/**
* \brief Créé un fichier de mesures à partir des directions indiquée dans un autre fichier
* la fonction recupere les indication dans le fichier direction
* et les mets sous forme de fichier pouvant etre lu par la classe Traitement
* \param filename : nom du fichier de mesures à créer
* \param direction : nom du fichier dans lequel la fonction va puiser ces information
* \param sampleTime : temps d'échantillonage que l'on veut utiliser lors de la simulation (en millisecondes)
* \param variation (défaut = 0) : erreur absolue qui peut s'ajouter ou se soustraire à chaque mesure
* \param bias (défaut = 0) : biais à ajouter à chaque mesure
*/
void createMeasureFile(std::string filename, std::string direction, double sampleTime, double variation, double bias);

void fileFromSerial(std::string filename, Serial &link, int nbMes);


/**
* \brief recupere les informations du fichier file et les stocke dans les differentes variables
*/
void getDirection(std::fstream &file, double &val1, double &val2, double &val3, double &temps, double &duree);


/**
* \brief rajoute une erreur sur la mesure pour simuler une instabilité ou un biais
* \param baseValeur : valeur de base, "parfaite", à laquelle une erreur va être rajoutée
* \param variation : valeur maximale qui peut être rajoutée ou enlevée à la valeur de base
* \param bias : biais rajouté, le biais vaut 0 par défaut
*/
double addError(double baseValeur, double variation, double bias);
#endif	//TOOLS_H
