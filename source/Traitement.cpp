#include "Traitement.h"
#include <iostream>
/* test SVN modif */

/** Constructeur **/
Traitement::Traitement(Instrument* inst) :_compteur(0), _dt(0)
{
	_capteur = inst;
	/** Allocation mémoire de la matrice de valeurs **/
	for (int i = 0; i<3; i++)
	{
		_valeurs[i] = new double[NB_VALEURS];
	}
}
Traitement::Traitement() :_compteur(0), _dt(0)
{
	/** Allocation mémoire de la matrice de valeurs **/
	for (int i = 0; i<3; i++)
	{
		_valeurs[i] = new double[NB_VALEURS];
	}
}


/** Destructeur **/
Traitement::~Traitement()
{
	for (int i = 0; i<3; i++)
	{
		delete _valeurs[i];
	}
}

Instrument* Traitement::getInstrument(void){
	return (_capteur);
}

double	Traitement::get_dt(){
	return _dt;
}

void Traitement::setInstrument(Instrument_serie *un_Instrument){
	_capteur = un_Instrument;
}

void Traitement::stockerValeurs(vect4D val)
{
	_tempsAct = val.temps;
	_dt = (_tempsAct - _tempsPrec) / 1000.0;
	_tempsPrec = _tempsAct;

#ifdef TEST
	_RPT1(0, "nouvelles valeurs à stocker : \r\nval.x = %f\n", val.x);
	_RPT1(0, "val.y = %f\n", val.y);
	_RPT1(0, "val.x = %f\n", val.z);
#endif

	if (_compteur < NB_VALEURS) // cas ou le tableau n'est pas plein
	{
		_valeurs[0][_compteur] = val.x;
		_valeurs[1][_compteur] = val.y;
		_valeurs[2][_compteur] = val.z;
		_compteur++;
	}

	else // cas ou le tableau est deja rempli
	{
		//_RPT0(0, "tableau rempli !\n");
		for (int i = 0; i<NB_VALEURS - 1; i++)
		{
			_valeurs[0][i] = _valeurs[0][i + 1];
			_valeurs[1][i] = _valeurs[1][i + 1];
			_valeurs[2][i] = _valeurs[2][i + 1];
		}

		if (_compteur < 0)
			_compteur = 0;
		_valeurs[0][_compteur - 1] = val.x;
		_valeurs[1][_compteur - 1] = val.y;
		_valeurs[2][_compteur - 1] = val.z;

	}
	//_RPT1(0, "compteur = %d\n", _compteur);
}

/**
*      \brief stocke des valeurs passées en argument dans la matrice de valeur attribut
*      la variable compteur permet de savoir si la matrice est remplie ou pas
*      si la matrice est deja remplie, elle se contente de remplacer la premiere valeur de chaque ligne avec les valeurs en argument
*      \param val vect3D défini dans structure.h contenant les mesures a stocker
*/
void Traitement::stockerValeurs()
{
	_capteur->majMesures(/*_capteur->getID()*/);
	_tempsAct = _capteur->getMesure(4); /* 4 correspond à l'axe temporel (mesures.temps) */
	_dt = (_tempsAct - _tempsPrec) / 1000.0;
	_tempsPrec = _tempsAct;

	if (_compteur < NB_VALEURS) // cas ou le tableau n'est pas plein
	{
		_valeurs[0][_compteur] = _capteur->getMesure(1);
		_valeurs[1][_compteur] = _capteur->getMesure(2);
		_valeurs[2][_compteur] = _capteur->getMesure(3);
		_compteur++;
	}

	else // cas ou le tableau est deja rempli
	{
		//_RPT0(0, "tableau rempli !\n");
		for (int i = 0; i < NB_VALEURS - 1; i++)
		{
			_valeurs[0][i] = _valeurs[0][i + 1];
			_valeurs[1][i] = _valeurs[1][i + 1];
			_valeurs[2][i] = _valeurs[2][i + 1];
		}

		_valeurs[0][_compteur - 1] = _capteur->getMesure(1);
		_valeurs[1][_compteur - 1] = _capteur->getMesure(2);
		_valeurs[2][_compteur - 1] = _capteur->getMesure(3);

	}
	//_RPT1(0, "compteur = %d\n", _compteur);
}

/**
*  \brief realise la moyenne des valeurs d'une ligne de la matrice
*  \param axe numero de la ligne que l'on veut moyenner
*  \return la moyenne d'une ligne
*/
double Traitement::moyenner(int axe)
{
	double moyenne = 0;
	for (int i = 0; i < _compteur; i++)
	{
		moyenne += _valeurs[axe - 1][i];
	}

	return (moyenne / NB_VALEURS);
}


/**
* \brief calcul la variation d'angle (en degré)
* la fonction utilise les attributs privés _dt et appelle la fonction Traitement::moyenner pour calculer l'angle
* \return angles d'euler sous forme d'un vect3D
*/
vect3D Traitement::calculerAngle_deg()
{
	vect3D angles;
	angles.x = moyenner(1)*_dt * 180 / (atan(1) * 4);
	angles.y = moyenner(2)*_dt * 180 / (atan(1) * 4);
	angles.z = moyenner(3)*_dt * 180 / (atan(1) * 4);
	_RPT1(0, " _dt = %f ms\n", _dt);
	std::cout << " _dt = " << _dt << " ms " << std::endl;
	return angles;
}

/** A CHANGER
* \brief Soustraction de l'influence de l'accélération g sur chacun des axes de l'accéléromètre
*/
void substractG(double matrice[3][3], double* accel_x, double* accel_y, double* accel_z)
{
	(*accel_x) = G*matrice[0][2];
	(*accel_y) = G*matrice[1][2];
	(*accel_z) = G*matrice[2][2];
}

/**
* \brief affiche les moyennes de chaque axe sur la sortie et sur la console
*/
void Traitement::afficherValeurs()
{
	_RPT1(0, "CAPTEUR %s : \n", _capteur->getID());
	std::cout << "CAPTEUR " << _capteur->getID() << ": " << std::endl;
	if (_compteur == NB_VALEURS) {
		_RPT1(0, "moyenne X = %f\n", moyenner(1));
		_RPT1(0, "moyenne Y = %f\n", moyenner(2));
		_RPT1(0, "moyenne Z = %f\n", moyenner(3));

		std::cout << "moyenne X = " << moyenner(1) << "                      " << std::endl;
		std::cout << "moyenne Y = " << moyenner(2) << "                      " << std::endl;
		std::cout << "moyenne Z = " << moyenner(3) << "                      " << std::endl;
	}
}


/**
* \brief indique si le tableau _valeurs est rempli
* i.e. si le nombres de mesures relevées suffit à remplir le tableau au moins une fois
* \return boolean
*/
bool Traitement::tabFull()
{
	if (_compteur == NB_VALEURS)
		return true;
	else
		return false;
}


/** A CHANGER
*	\brief Ecrire les données récupérées d'un capteur dans un fichier
*
*	\param filename	string		le nom du fichier - doit être en format "nom.txt" et doit être ouvert
*   \param inst		Instrument	le capteur  dont on récupère les données
*
*   \test  test_filefromSensor
*/
void writeHeading(std::string filename){
	std::fstream myfile;
	myfile.open(filename.c_str(), std::ios::app);
	if (filename == "gyro.csv"){
		myfile << "Gyr_X;Gyr_Y;Gyr_Z;temps\n";
	}
	else if (filename == "acce.csv"){
		myfile << "Acc_X;Acc_Y;Acc_Z;temps\n";
	}
	else if (filename == "magn.csv"){
		myfile << "Mag_X;Mag_Y;Mag_Z;temps\n";
	}
	else if (filename == "orie.csv"){
		myfile << "Ori_X,Ori_Y;Ori_Z;temps\n";
	}
	myfile.close();
}

/**
* \brief Ecrit dans un fichier les mesures prises par le capteur
*/
void Traitement::filefromSensor(std::string filename, Instrument* inst){
	std::fstream myfile;
	myfile.open(filename, std::ios::app);
	for (int i = 1; i <= 4; i++){
		myfile << inst->getMesure(i);
		myfile << ";";
	}
	myfile << "\n";
	myfile.close();
}

/**
*	\brief Lire les données à partir d'un fichier
*	\param	filename	string		le nom du fichier - doit être en format "nom.txt"
*   \return data		vect4D		vecteur de 4 éléments (données selon l'axe x,y,z et le temps) pour un traitement
*   \test  test_readDatafromFile
*/
vect4D Traitement::readDatafromFile(std::string filename, int cursor)
{
	vect4D data = { 0, 0, 0, 0 };
	std::fstream myfile;
	char c;
	std::string chaine;

	myfile.open(filename, std::ios::in || std::ios::out);
	if ((myfile.rdstate() && std::ifstream::failbit) != 0)
		_RPT0(_CRT_ERROR, "erreur lors de l'ouverture du fichier");

	if (myfile.eof() == false)
	{
		/* Enlève l'entête du fichier */
		for (int i = 0; i < cursor; i++)
			std::getline(myfile, chaine);

		/* Récupération des données du fichier tant que ce n'est pas la fin d'une ligne */
		myfile >> data.x;
		myfile >> c;
		myfile >> data.y;
		myfile >> c;
		myfile >> data.z;
		myfile >> c;
		myfile >> data.temps;
		myfile >> c;
	}
	myfile.close();
#ifdef TEST
	_RPT1(0, "valeur x = %f\n", data.x);
	_RPT1(0, "valeur y = %f\n", data.y);
	_RPT1(0, "valeur z = %f\n", data.z);
	_RPT1(0, "valeur t = %f\n", data.temps);
#endif

	return data;
}


/** \brief ou==Ouverture d'un fichier en mode lecture */
void Traitement::openfile_readwrite(std::fstream& myfile, std::string filename)
{
	myfile.open(filename, std::fstream::in | std::fstream::out | std::fstream::app);
	if ((myfile.rdstate() && std::ifstream::failbit) != 0)
		_RPT0(_CRT_ERROR, "erreur lors de l'ouverture du fichier");
}

void Traitement::afficherTraitement(void){
	_capteur->afficherCapteur();
	_RPT1(0, "dt : %f \n", _dt);
}

int choiceMode(){
	int mode;
	std::cout << "MODE DE SIMULATION : " << std::endl;
	std::cout << "1 - Simulation avec capteur" << std::endl;
	std::cout << "2 - Simulation avec fichier de valeurs" << std::endl;
	std::cout << "La mode de simulation choisie : ";
	std::cin >> mode;
	std::cout << std::endl;
	while (mode != 1 && mode != 2){
		std::cout << "La mode choisie doit etre 1 ou 2 : ";
		std::cin >> mode;
		std::cout << std::endl;
	}
	return mode;
}