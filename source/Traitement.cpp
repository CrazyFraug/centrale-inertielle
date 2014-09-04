#include "Traitement.h"
#include <iostream>

/** Constructeur **/

Traitement::Traitement(std::string id, std::string filename) :_compteur(0), _dt(0), _id(id), _tempsPrec(0)
{
	/** Allocation mémoire de la matrice de valeurs **/
	for (int i = 0; i<3; i++)
	{
		_valeurs[i] = new double[NB_VALEURS];
	}
	_filename = filename;
}


/** Destructeur **/
Traitement::~Traitement()
{
	for (int i = 0; i<3; i++)
	{
		delete _valeurs[i];
	}
}

/** Setter **/
void Traitement::set_dt(double t) {_dt = t;}

void Traitement::set_tempsPrec(double t) {_tempsPrec = t;}

/** Getter **/
double	Traitement::get_dt() {return _dt;}

int Traitement::get_compteur(void) {return _compteur;}

double Traitement::get_tempsPrec(void) {return _tempsPrec;}

bool Traitement::get_finFichier(void) {return _finFichier;}


/** Autres méthodes **/

vect4D Traitement::acquisition(void)
{
	return readDatafromFile();
}


/**
*      \brief stocke des valeurs passées en argument dans la matrice de valeur attribut
*      la variable compteur permet de savoir si la matrice est remplie ou pas
*      si la matrice est deja remplie, elle se contente de remplacer la premiere valeur de chaque ligne avec les valeurs en argument
*      \param mesures vect4D défini dans structure.h contenant les mesures a stocker et le temps 
*/
void Traitement::stockerValeurs(void)
{
	vect4D mesures = acquisition();
	_tempsAct = mesures.temps; /* axe temporel (mesures.temps) */
	_dt = (_tempsAct - _tempsPrec) / 1000.0;
	_RPT1(0,"(stockerValeurs) _tempsPrec= %f\n",_tempsPrec);
	_RPT1(0,"(stockerValeurs) _tempsAct = %f\n",_tempsAct);
	_tempsPrec = _tempsAct;

	_RPT1(0,"(stockerValeurs) _dt = %f\n",_dt);
	if (_compteur < NB_VALEURS) // cas ou le tableau n'est pas plein
	{
		_valeurs[0][_compteur] = mesures.x;
		_valeurs[1][_compteur] = mesures.y;
		_valeurs[2][_compteur] = mesures.z;
		_compteur++;
	}
	else // cas ou le tableau est deja rempli
	{
		for (int i = 0; i < NB_VALEURS - 1; i++)
		{
			_valeurs[0][i] = _valeurs[0][i + 1];
			_valeurs[1][i] = _valeurs[1][i + 1];
			_valeurs[2][i] = _valeurs[2][i + 1];
		}

		_valeurs[0][_compteur - 1] = mesures.x;
		_valeurs[1][_compteur - 1] = mesures.y;
		_valeurs[2][_compteur - 1] = mesures.z;
		std::cout << mesures.x << " ; " << mesures.y << " ; " << mesures.z << " ; " << mesures.temps << "                                 " << std::endl;

	}
	//_RPT1(0, "stocker val: mesures.x = %d\n", mesures.x);

}

/**
*  \brief realise la moyenne des valeurs d'une ligne de la matrice
*  \param axe numero de la ligne que l'on veut moyenner
*  \return la moyenne d'une ligne
*/
double Traitement::moyennerAxe(int axe)
{
	double moyenne = 0;
	for (int i = 0; i < _compteur; i++)
	{
		moyenne += _valeurs[axe - 1][i];
	}

	return (moyenne / _compteur);
}

/**
* \brief renvoie la moyenne des valeurs de chaque axe sous forme de vect3D
* \param nb : nombre de valeurs à moyenner. La fonction réalisera la moyenne des "nb" dernières valeurs mesurées
*/
vect3D Traitement::moyenner(int nb)
{
	vect3D res = {0,0,0};
	if(nb>_compteur)
	{
		nb = _compteur;
	}
	//_RPT1(0,"(moyenner) _tempsPrec= %f\n",_tempsPrec);
	//_RPT1(0,"(moyenner) valeur de nb : %d\n", nb);
	for (int i =0; i<nb; i++)
	{
		res.x += _valeurs[0][_compteur-i-1];
		res.y += _valeurs[1][_compteur-i-1];
		res.z += _valeurs[2][_compteur-i-1];
		_RPT1(0,"(moyenner) valeur de res.x : %f\n", res.x);
	}
	res.x /= nb;
	res.y /= nb;
	res.z /= nb;
	//_RPT1(0,"(moyenner) valeur de res.x : %f\n", res.x);
	return res;
}

/**
*	\brief renvoie la dernière valeurs stockée par chaque axe ainsi que le temps auquel elles ont été stockées
*/
vect4D Traitement::lastVal(void)
{
	vect4D res;
	res.temps = _tempsAct;
	res.x = _valeurs[0][_compteur-1];
	res.y = _valeurs[1][_compteur-1];
	res.z = _valeurs[2][_compteur-1];

	return res;
}


/**
* \brief renvoie une moyenne des valeurs de mesures apres une intégration
* la fonction utilise les attributs privés _dt et appelle la fonction Traitement::moyenner pour calculer l'angle
* \param nb : nombre de colonne de _valeurs qui sont moyennées. Attribut de la fonction Traitement::moyenner(int)
* \return val sous forme d'un vect3D
*/
vect3D Traitement::renvoyerVal(int nb)
{

	if (nb<1)
		_RPT0(_CRT_ERROR,"le parametre int nb doit etre superieur ou egal a 1\n");

	stockerValeurs();
	_RPT1(0,"(renvoyerVal)_valeurs.y = %f\n", _valeurs[1][0]); 
	//_RPT1(0,"(renvoyerVal)_valeurs.x = %f\n", _valeurs[0][1]); 

	vect3D val = {0,0,0};
	if (_compteur>1)
	{
		val = moyenner(nb);
		//_RPT1(0,"(renvoyerVal) moyenne val.x = %f\n", val.x); 
		//Sans conversion :
		val.x *= (_dt);
		val.y *= (_dt);
		val.z *= (_dt);
		_RPT1(0,"(renvoyerVal) val.y*_dt= %f\n", val.y); 
	}

	val.x = -val.x;
	return val;
}

/**
* \brief renvoie un rotation par quaternion
*/
void Traitement::renvoyerQuat(int nb, quaternion<double> &q_base, vect3D &axe, double &angle)
{

	quaternion<double> q(0,0,0,0);

	if (nb<1)
		_RPT0(_CRT_ERROR,"le parametre int nb doit etre superieur ou egal a 1\n");

	stockerValeurs();

	vect3D val = {0,0,0};
	if (_compteur>1)
	{
		val = moyenner(nb);
		//Sans conversion :
		val.x *= (_dt);
		val.y *= (_dt);
		val.z *= (_dt);
	}

	//_RPT1(0, "(renvoyerQuat) val.x*dt = %f\n", val.x);
	//_RPT1(0, "(renvoyerQuat) val.y*dt = %f\n", val.y);
	//_RPT1(0, "(renvoyerQuat) val.z*dt = %f\n", val.z);

	q = danglesToQuat(val.x, val.y, val.z);
	q_base = q_base*q;
	quatComp(q_base, axe, angle);
	//val.x = -val.x;
}


/** A CHANGER
* \brief Soustraction de l'influence de l'accélération g (intensité de la pesanteur) sur chacun des axes de l'accéléromètre
* \param matrice : matrice de rotation
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
	if (_compteur == NB_VALEURS) {
		_RPT1(0, "moyenne X = %f\n", moyennerAxe(1));
		_RPT1(0, "moyenne Y = %f\n", moyennerAxe(2));
		_RPT1(0, "moyenne Z = %f\n", moyennerAxe(3));

		std::cout << "moyenne X = " << moyennerAxe(1) << "                      " << std::endl;
		std::cout << "moyenne Y = " << moyennerAxe(2) << "                      " << std::endl;
		std::cout << "moyenne Z = " << moyennerAxe(3) << "                      " << std::endl;
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
*	\param	&type	std::string		modifié dans cette fonction selon à quel capteur appartiennent les mesures
*   \return data	vect4D			vecteur de 4 éléments (données selon l'axe x,y,z et le temps) pour un traitement
*   \test  test_readDatafromFile
*/
vect4D Traitement::readDatafromFile()
{
	vect4D data = { 0, 0, 0, 0 };
	std::fstream myfile;
	char c;
	std::string chaine;
	bool typeFound(false);

	myfile.open(_filename, std::ios::in);
	if ((myfile.rdstate() && std::ifstream::failbit) != 0)
		_RPT0(_CRT_ERROR, "Erreur lors de l'ouverture du fichier");

	if (!_finFichier)
	{
		/* Enlève l'entête du fichier */
		for (int i = 0; i < _cursor; i++)
			std::getline(myfile, chaine);

		while (!typeFound && !_finFichier)
		{
			std::getline(myfile, chaine);

			if (chaine == _id + ":")
				typeFound = true;
			else 
				typeFound = false;

			if (myfile.eof()) //on verifie que l'on n'est pas à la fin du fichier (eof)
				_finFichier = true;

			_cursor++;
			_RPT2(0,"curseur %s : %d\n",_id.c_str(),_cursor);
		}
		
			/* Récupération des données */
			myfile >> c;	
			myfile >> data.x;
			myfile >> c;	
			myfile >> c;
			myfile >> data.y;
			myfile >> c;
			myfile >> c;
			myfile >> data.z;
			myfile >> c;
			_RPT1(0, "char 2 = %c\n", c);
			myfile >> c;
			_RPT1(0, "char 3 = %c\n", c);
			myfile >> data.temps;
			myfile >> c;

	}
	else 
	{
		_RPT0(0, "end of file \n");
		std::cout <<  " END OF FILE                                        " << std::endl;
	}
	myfile.close();
#define TEST
#ifdef TEST
	_RPT1(0, "[readData] valeur x = %f\n", data.x);
	_RPT1(0, "[readData] valeur y = %f\n", data.y);
	_RPT1(0, "[readData] valeur z = %f\n", data.z);
	_RPT1(0, "[readData] valeur t = %f\n", data.temps);
#endif
	_RPT0(0,"fin readData \n"); 

	return data;
}

/** \brief reset _cursor */
void Traitement::resetCursor(void)
{
	_cursor = 0;
	_finFichier = false;
}

/** \brief reset _compteur pour effectuer une nouvelle simulation */
void Traitement::resetCompteur(void)
{
	_compteur = 0;
}


/** \brief Ouverture d'un fichier en mode lecture */
void Traitement::openfile_readwrite(std::fstream& myfile, std::string filename)
{
	myfile.open(filename, std::fstream::in | std::fstream::out | std::fstream::app);
	if ((myfile.rdstate() && std::ifstream::failbit) != 0)
		_RPT0(_CRT_ERROR, "erreur lors de l'ouverture du fichier");
}


//*************************************************************//
//********************* Traitement_serie **********************//
//*************************************************************//

/** Constructeur **/
Traitement_serie::Traitement_serie(std::string id, std::string filename, Serial* link): Traitement(filename)
{
	_capteur = new Instrument_serie(id, link);
}

/** Destructeur **/
Traitement_serie::~Traitement_serie()
{
	delete _capteur;
}

/** Setter **/


/** Getter **/
Instrument_serie* Traitement_serie::getInstrument(void){
	return (_capteur);
}

/**
*      \brief stocke des valeurs passées en argument dans la matrice de valeur attribut
*      la variable compteur permet de savoir si la matrice est remplie ou pas
*      si la matrice est deja remplie, elle se contente de remplacer la premiere valeur de chaque ligne avec les valeurs en argument
*      \param val vect3D défini dans structure.h contenant les mesures a stocker
*/
vect4D Traitement_serie::acquisition(void)
{
	return _capteur->getMesures();
}


void Traitement_serie::afficherTraitSerie(void){
	_capteur->afficherCapteur();
	_RPT1(0, "dt : %f \n", get_dt());
}

//*************************************************************//
//******************** Fonctions globales *********************//
//*************************************************************//

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
	std::cout << "Le mode de simulation choisi : ";
	std::cin >> mode;
	std::cout << std::endl;

	while (mode != 1 && mode != 2 && mode != 3 && mode != 0 && mode != 4)
	{
		std::cout << "Le mode choisi est incorrect : ";
		std::cin >> mode;
		std::cout << std::endl;
	}
	return mode;
}