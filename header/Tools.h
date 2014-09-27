#ifndef TOOLS_H
#define TOOLS_H

#include "Serial.h"
#include "Mobile.h"
#include "Kalman.h"

#define SAMPLETIME 20
#define FILENAME "simMeasures.txt"
#define DIRECTIONS "directions.txt"

/*	\brief	Ecriture de l'ent�te des fichiers sauvegardant les donn�es
	\param	std::string		filename	Nom du fichier || Doit �tre "acce.csv", "gyro.csv", "mnet.csv", "orient.csv"
	\param	std::fstream	&myfile		R�f�rence du fichier
*/
void writeHeading(std::string filename, std::fstream &myfile);


/*	\brief	Ecriture d'une donn�es dans un fichier sauvegardant les donn�es
	\param	std::string		filename	Nom du fichier || Doit �tre "acce.csv", "gyro.csv", "mnet.csv", "orient.csv"
	\param	std::fstream	&myfile		R�f�rence du fichier
	\param	vect4D			data		Les donn�es � �crire dans le fichier
*/
void filefromSensor(std::string filename, std::fstream &myfile, vect4D data);


/*	\brief	Lecture des fichiers sauvegardant les donn�es
	\param	std::string		filename	Nom du fichier || Doit �tre "acce.csv", "gyro.csv", "mnet.csv", "orient.csv"
	\param	std::fstream	&myfile		R�f�rence du fichier
	\param	int				turns		La ligne de donn�es du fichier � r�cup�rer
	\return	vect4D						Les donn�es r�cup�r�es du fichier
*/
vect4D readDatafromFile(std::string filename, std::fstream &myfile, int turns);


/*	\brief	Choix du mode de simulation 
*/
int choiceMode();


/*	\brief	Ecriture du r�sultat dans un fichier 
	\param	double			temps			La valeur du moment de chaque �chantillon
	\param	vect3D			angles_measure	La valeur mesur�e vient du capteur
	\param	vect3D			angles_calcul	La valeur calcul�e avec le filtre de Kalman
	\param	std::string		fileResult		Nom du fichier de r�sultat
	\param	std::fstream	file_excel		R�f�rence du fichier de r�sultat
*/

void writeResult(double temps, vect3D angles_measure, vect3D angles_calcul, std::string fileResult, std::fstream &file_excel);


/*	\brief	Convert a string into a double type variable
	\return float	Valeur du string || Message error au cas probl�me
*/
float string_to_double(const std::string& s);


/*	\brief	Normaliser l'angle dans entre -180� et 180�
*/
double normAngle(double alpha);


/*	\brief	Produit de deux matrices

	\param	matrix<double>	M1		--	Matrice gauche de taille m x n
	\param	matrix<double>	M2		--	Matrice gauche de taille n x o

	\return	matrix<double>	result	-- R�sultat du produit de taille m x o (cas normal) || Message d'erreur si la taille des matrices ne convient pas
*/
matrix<double> product_matrix(matrix<double> M1, matrix<double> M2);

/*	\brief	Affichage d'une matrice
	\param	matrix<double>	A	Matrice � afficher
*/
void printMatrix(matrix<double> A);


/*	\brief	Filtre de Kalman avec les �tapes de pr�diction et mettre � jour d'�tat
	\param	Kalman			&rotation		syst�me � filtrer
	\param	matrix<double>	value_cmd		Matrice contenant les donn�es de la commande
	\param	matrix<double>	value_obs		Matrice contenant les donn�es d'observation
	\return matrix<double>	estimate_result	Matrice contenant l'�tat estim�
	\test	test_Kalman		A tester les constants pour valider l'algorithme!

*/
matrix<double> kalmanTraitement(Kalman &rotation, matrix<double> value_cmd, matrix<double> value_obs);


/*	\brief	Cr�� un fichier de mesures � partir des directions indiqu�e dans un autre fichier
			la fonction recupere les indication dans le fichier direction
			et les mets sous forme de fichier pouvant etre lu 
	\param	std::string	filename	Nom du fichier de mesures � cr�er
	\param	std::string	direction	Nom du fichier dans lequel la fonction va puiser ces information
	\param	double		sampleTime	Temps d'�chantillonage que l'on veut utiliser lors de la simulation (en millisecondes)
	\param	double		variation	(d�faut = 0) : erreur absolue qui peut s'ajouter ou se soustraire � chaque mesure
	\param	double		bias		(d�faut = 0) : biais � ajouter � chaque mesure
*/
void createMeasureFile(std::string filename, std::string direction, double sampleTime, double variation, double bias);

void createMeasureFile_separate(std::string filename, std::string direction, double sampleTime);

bool verifierOuverture (const std::fstream &file1, const char* name, int reportType);

void fileFromSerial(std::string filename, Serial &link, int nbMes);

bool getHeader( std::fstream &file, double &variation1, double &variation2, double &variation3, double &bias1, double &bias2, double &bias3);

void writeHeader(std::fstream& file, std::string& name, const double& variation, const double& biais);

/*	\brief	Recupere les informations du fichier file et les stocke dans les differentes variables
*/
bool getDirection(std::fstream &file, double &temps, double &duree, double &val1, double &val2, double &val3, double &vitX, double &vitY, double &vitZ);


/*	\brief	Rajoute une erreur sur la mesure pour simuler une instabilit� ou un biais
	\param	double	baseValeur	valeur de base, "parfaite", � laquelle une erreur va �tre rajout�e
	\param	double	variation	valeur maximale qui peut �tre rajout�e ou enlev�e � la valeur de base
	\param	double	bias		biais rajout�, le biais vaut 0 par d�faut
*/
double addError(double baseValeur, double variation, double bias);

void calculateAccel(const double &phi, const double &teta, double &accX, double &accY, double &accZ);

#endif	//TOOLS_H
