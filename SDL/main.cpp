#include "SceneOpenGL.h"
#include "Mobile.h"
#include <string.h>

#define NB_SENSOR 4
#define PORTSERIE "COM9"
#define BAUD 115200
#define SAMPLETIME 20


double addError(double average, double variation, double bias=0);
void getDirection(std::fstream &file, double &val1, double &val2, double &val3, double &temps, double &duree);
void createMeasureFile(std::string filename, std::string direction, double sampleTime, double variation=0, double bias=0);
void fileFromSerial(std::string filename, Serial &link, int nbMes);

int Traitement::_cursor = 0;
bool Traitement::_finFichier = false;

int main(int argc, char *argv[]) {

	int mode = 0;
	Mobile gant;

	Traitement *tab[4];
	Traitement_serie *tabInst[4];

	SceneOpenGL scene("letitre", 800, 600);

	if (scene.InitialiserFenetre() == false) { system("PAUSE"); return -1; }

	if (!scene.iniGL()) { system("PAUSE"); return -1; }

	mode = choiceMode();
	while (mode != 0)
	{
		if (mode == 1){
			Serial link(PORTSERIE,BAUD);
			tabInst[0] = new Traitement_serie ("gyro","filename.txt", &link);
			tabInst[1] = new Traitement_serie ("acce","filename.txt", &link);
			tabInst[2] = new Traitement_serie ("mnet","filename.txt", &link);
			tabInst[3] = new Traitement_serie ("orie","filename.txt", &link);
			scene.bouclePrincipaleSensor(tabInst);
		}
		else if (mode == 2)
		{
			tab[0] = new Traitement("gyro","serial.txt");
			tab[1] = new Traitement("acce","serial.txt");
			tab[2] = new Traitement("mnet","serial.txt");
			tab[3] = new Traitement("orie","serial.txt");

			scene.bouclePrincipaleSimu(gant, tab);
		}

		else if (mode == 3)
		{
			createMeasureFile(FILENAME, DIRECTIONS, SAMPLETIME,1);
			srand(time(NULL));
			/*matrix<double> predict_vector(4,1,0);
			matrix<double> cov_estimate(4,4,0);
			for (int i = 0; i < 4; i++)
			{
				predict_vector(i, 0) = 1.0;
				cov_estimate(i, i) = 1.0;
			}*/
		}
		else if (mode == 4)
		{
			Serial link(PORTSERIE,BAUD);
			int nbMes;
			std::cout << "entrez le nombre de mesures a copier : " << std::endl;
			std::cin >> nbMes;
			fileFromSerial("serial.txt", link, nbMes);
		}

		tab[0]->resetCompteur();
		tab[1]->resetCompteur();
		tab[2]->resetCompteur();
		tab[3]->resetCompteur();

		
		delete tab[0];
		delete tab[1];
		delete tab[2];
		delete tab[3];

		mode = choiceMode();
	}
	//system("PAUSE");

	return 0;
}

//Kalman init_kalman(int nb_in, int nb_out, int nb_state, int nb_step)
//{
//	matrix<double> *_p_cov, *_p_predict;
//	/* Initialisation du filtre de Kalman + Système */
//	//matrix<double> mat_cov(nb_state, nb_state, 0), init_predict(nb_state, 1, 0);
//	_p_cov = new matrix<double>(nb_state, nb_state, 0);
//	_p_predict = new matrix<double>(nb_state, 1, 0);
//
//	for (int i = 0; i < nb_state; i++){
//	(*_p_cov)(i, i) = 1;
//	(*_p_predict)(i, 0) = 1;
//	}
//
//	/* Instanciation */
//	Kalman rotation(nb_in, nb_out, nb_state, nb_step, *_p_predict, *_p_cov);
//
//	return rotation;
//}

/**
* \brief Créé un fichier de mesures à partir des directions indiquée dans un autre fichier
* la fonction recupere les indication dans le fichier direction 
* et les mets sous forme de fichier pouvant etre lu par la classe Traitement
* \param filename : nom du fichier de mesures à créer
* \param direction : nom du fichier dans lequel la fonction va puiser ces information
* \param sampleTime : temps d'échantillonage que l'on veut utiliser lors de la simulation
* \param variation (défaut = 0) : erreur absolue qui peut s'ajouter ou se soustraire à chaque mesure
* \param bias (défaut = 0) : biais à ajouter à chaque mesure
*/
void createMeasureFile(std::string filename, std::string direction, double sampleTime, double variation, double bias)
{
	std::fstream fDirection, fMeas;
	fDirection.open(direction, std::ios::in);
	fMeas.open(filename, std::fstream::out);
	if ((fDirection.rdstate() && std::ifstream::failbit) != 0)
	{
		_RPT0(_CRT_ERROR, "Erreur lors de l'ouverture du fichier direction\n");
	}
	else if ((fMeas.rdstate() && std::ifstream::failbit) != 0)
	{
		_RPT0(0,"Fichier directions ouvert correctement\n");
		_RPT0(_CRT_ERROR, "Erreur lors de l'ouverture du fichier de mesure\n");
		fDirection.close();
	}
	else
	{
		_RPT0(0,"Fichier mesures ouvert correctement\n");
		double val1,val2,val3, tEcoule(0.0), temps, duree;
		double val1e, val2e, val3e;
		while (fDirection.eof() == false)
		{
			getDirection(fDirection, val1, val2, val3, temps, duree);
			std::cout << "valeur 1 : "<< val1 << " ; valeur 2 : " << val2 << " ; valeur 3 : " << val3 << " ; temps : " << temps << " ; duree : " << duree << std::endl;

			if (temps >= tEcoule)
			{
				while ( tEcoule < temps)
				{
					fMeas << "gyro:" << '\n';
					fMeas << addError(0,variation,bias) << ';' << addError(0,variation,bias) << ';' << addError(0,variation,bias) << ';' << tEcoule << ';' << '\n';
					tEcoule += sampleTime;
				}

				while (tEcoule < temps+duree)
				{
					//Ajout des erreurs de mesure:
					val1e = addError(val1,variation,bias);
					val2e = addError(val2,variation,bias);
					val3e = addError(val3,variation,bias);
					fMeas << "gyro:" << '\n';
					fMeas << val1e << ';' << val2e << ';' << val3e << ';' << tEcoule << ';' << '\n';
					tEcoule += sampleTime;
				}
			}
		
			else 
			{
					_RPT0(0, "temps inferieur a celui de la derniere ligne + duree, temps remanié");
					temps = tEcoule;

					while (tEcoule < temps+duree)
					{
					//Ajout des erreurs de mesure:
					val1e = addError(val1,variation,bias);
					val2e = addError(val2,variation,bias);
					val3e = addError(val3,variation,bias);
					fMeas << val1e << ';' << val2e << ';' << val3e << ';' << tEcoule << ';' << '\n';
					tEcoule += sampleTime;
					}
			}
		}

		fMeas << std::ios::eofbit;
		fMeas.close();
		fDirection.close();
		system("PAUSE");
	}

}

void fileFromSerial(std::string filename, Serial &link, int nbMes)
{
	std::fstream file;
	std::string tmp;
	int i =0;
	file.open(filename, std::ios::out);
	static HANDLE h = NULL;  
	h = GetStdHandle(STD_OUTPUT_HANDLE);
	COORD c = { 0, 7 };  
	while(i++ < nbMes)
	{
		tmp = link.readLine();
		file << tmp << '\n';
		SetConsoleCursorPosition(h,c);
		std::cout << tmp << std::endl;
	}
	file.close();
}

/**
* \brief recupere les informations du fichier file et les stocke dans les differentes variables
*/
void getDirection(std::fstream &file, double &val1, double &val2, double &val3, double &temps, double &duree)
{
	char c;
	file >> temps;
	file >> c;
	file >> duree;
	file >> c;
	file >> val1;
	file >> c;
	file >> val2;
	file >> c;
	file >> val3;

	val1 /= (duree/1000.0);
	val2 /= (duree/1000.0);
	val3 /= (duree/1000.0);

}

/**
* \brief rajoute une erreur sur la mesure pour simuler une instabilité ou un biais
* \param baseValeur : valeur de base, "parfaite", à laquelle une erreur va être rajoutée
* \param variation : valeur maximale qui peut être rajoutée ou enlevée à la valeur de base
* \param bias : biais rajouté, le biais vaut 0 par défaut
*/
double addError(double baseValeur, double variation, double bias)
{
	double meas = baseValeur - variation;
	double erreur = 0;
	erreur = (double)rand() / RAND_MAX; //generate random number between 0 and 1
	meas += erreur*(variation*2);
	return (meas+bias);
}

