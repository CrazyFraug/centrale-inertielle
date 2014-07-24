#include "Traitement.h"


/**Constructeur**/
Traitement::Traitement(Instrument* inst):_compteur(0), _dt(0)
{
		_capteur = inst;
        /** Allocation mémoire de la matrice de valeurs **/
        for (int i = 0; i<3; i++)
        {
                _valeurs[i] = new double[NB_VALEURS];
        }
}

Traitement::Traitement(std::string filename) :_compteur(0), _dt(0)
{
	char c;
	std::fstream myfile;
	myfile.open(filename);
	myfile >> c;
	
	if (c == 'a'){
		Instrument mon_instrument("acce", "COM8", 11820);
		_capteur = &mon_instrument;
	}
	else{
		Instrument mon_instrument("gyro", "COM8", 11820);
		_capteur = &mon_instrument;
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


/** 
 *      \brief stocke des valeurs passées en argument dans la matrice de valeur attribut
 *      la variable compteur permet de savoir si la matrice est remplie ou pas
 *      si la matrice est deja remplie, elle se contente de remplacer la premiere valeur de chaque ligne avec les valeurs en argument
 *      \param val vect3D défini dans structure.h contenant les mesures a stocker
*/
void Traitement::stockerValeurs() 
{
		_capteur->majSerial();
		_tempsAct = _capteur->getMesure(4); /* 4 correspond à l'axe temporel (mesures.temps) */
		_dt = (_tempsAct - _tempsPrec)/1000.0;
		_tempsPrec = _tempsAct;

        if (_compteur < NB_VALEURS)
        {
                _valeurs[0][_compteur] = _capteur->getMesure(1);
                _valeurs[1][_compteur] = _capteur->getMesure(2);
                _valeurs[2][_compteur] = _capteur->getMesure(3);
                _compteur++;
        }

        else
        {
                for(int i = NB_VALEURS-1; i>0; i--)
                {
                        _valeurs[0][i] = _valeurs[0][i-1];
                        _valeurs[1][i] = _valeurs[1][i-1];
                        _valeurs[2][i] = _valeurs[2][i-1];
                }

                _valeurs[0][0] = _capteur->getMesure(1);
                _valeurs[1][0] = _capteur->getMesure(2);
                _valeurs[2][0] = _capteur->getMesure(3);

        }
}

/**
 *  \brief realise la moyenne des valeurs d'une ligne de la matrice
 *  \param axe numero de la ligne que l'on veut moyenner
 *  \return la moyenne d'une ligne
*/
double Traitement::moyenner(int axe)
{
        double moyenne = 0;
        for (int i =0; i < _compteur; i++)
        {
                moyenne += _valeurs[axe-1][i];
        }

        return (moyenne/NB_VALEURS);
}

/**
* \brief calcul la variation d'angle (en degré)
* la fonction utilise les attributs privés _dt et appelle la fonction Traitement::moyenner pour calculer l'angle
* \return angles d'euler sous forme d'un vect3D 
*/
vect3D Traitement::calculerAngle_deg()
{
	vect3D angles;
	angles.x = moyenner(1)*_dt;
	angles.y = moyenner(2)*_dt;
	angles.z = moyenner(3)*_dt;
	std::cout << " _dt = " << _dt << " ms " << std::endl;
	return angles;
}


void Traitement::afficherValeurs()
{
	if (_compteur == NB_VALEURS) {
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


/**
\brief fonction inutile
*/
void Traitement::testd (void){
        std::cout << "test" << std::endl;
}


/**
*	\brief Ecrire les données récupérées d'un capteur dans un fichier
*
*	\param filename	string		le nom du fichier - doit être en format "nom.txt"
*   \param inst		Instrument	le capteur  dont on récupère les données
*
*   \test  test_filefromSensor	
*/
void Traitement::filefromSensor(std::string filename, Instrument* inst){
	
		std::fstream myfile;
		myfile.open(filename);
		char c;
		myfile >> c;
		/* Entête du fichier */
		if (c == NULL){
			if (inst->getID() == "gyro"){
				myfile << "||  Gyr_X  ||  Gyr_Y  ||  Gyr_Z  ||  dt  || \n";
			}
			else if (inst->getID() == "acce"){
				myfile << "||  Acc_X  ||  Acc_Y  ||  Acc_Z  ||  dt  || \n";
			}
		}

		for (int i = 0; i < 4; i++){
			myfile << "|| ";
			myfile << inst->getMesure(1);
		}
		myfile <<" || \n";
}

/**
*	\brief Lire les données à partir d'un fichier
*
*	\param	filename	string		le nom du fichier - doit être en format "nom.txt"
*   \return data		vect4D		vecteur de 4 éléments (données selon l'axe x,y,z et le temps) pour un traitement
*
*   \test  test_readDatafromFile
*/
vect4D Traitement::readDatafromFile(std::string filename){
	double value_recup[4];
	vect4D data;
	std::string premier_ligne;
	std::fstream myfile;
	char c = NULL;

	myfile.open(filename);
	/* Enlève l'entête du fichier */
	getline(myfile, premier_ligne);	


	/* Récupération des données du fichier tant que ce n'est pas la fin d'une ligne */
	while (c != '\n'){
		for (int i = 0; i < 4; i++){
			myfile >> c;
			if (c == '|'){
				if (c == '|'){
					myfile >> value_recup[i];
				}
			}
			else{
				myfile >> value_recup[i];
			}
		}
	}

	data.x = value_recup[0]; data.y = value_recup[1]; data.z = value_recup[2]; data.temps = value_recup[3];
	return data;
}