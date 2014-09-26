#include "SceneOpenGL.h"
#include "Mobile.h"
#include <string.h>

#define NB_SENSOR 4
#define PORTSERIE "COM9"
#define BAUD 115200
#define SAMPLETIME 20


double addError(double average, double variation, double bias=0);
bool getHeader(std::fstream &file, double &variation1, double &variation2, double &bias1, double &bias2, bool &bGyro, bool &bAcce, bool &bOrie);
void writeHeader(std::fstream& file, std::string& name, const double& variation, const double& biais);
bool getDirection(std::fstream &file, double &temps, double &duree, double &val1, double &val2, double &val3, double &vitX, double &vitY, double &vitZ);
void createMeasureFile(std::string filename, std::string direction, double sampleTime, double variation=0, double bias=0);
void createMeasureFile_separate(std::string filename, std::string direction, double sampleTime);
void fileFromSerial(std::string filename, Serial &link, int nbMes);
void calculateAccel(const double &phi, const double &teta, double &accX, double &accY, double &accZ);
bool getVitesse(std::fstream &file, double &val1, double &val2, double &val3, const double duree);
bool verifierOuverture (const std::fstream &file1, const char* name, int reportType=_CRT_WARN);

int Traitement::_cursor = 0;
bool Traitement::_finFichier = false;

int main(int argc, char *argv[]) {

	int mode = 0;
	Mobile gant;
	bool tabDefine = false;

	Traitement *tab[4];

	SceneOpenGL scene("letitre", 800, 600);

	if (scene.InitialiserFenetre() == false) { system("PAUSE"); return -1; }

	if (!scene.iniGL()) { system("PAUSE"); return -1; }

	mode = choiceMode();
	while (mode != 0)
	{
		if (mode == 1){
			Serial link(PORTSERIE,BAUD);
			tab[0] = new Traitement_serie ("gyro","filename.txt", &link);
			tab[1] = new Traitement_serie ("acce","filename.txt", &link);
			tab[2] = new Traitement_serie ("mnet","filename.txt", &link);
			tab[3] = new Traitement_serie ("orie","filename.txt", &link);
			tabDefine = true;
		}
		else if (mode == 2)
		{
			tab[0] = new Traitement("gyro","simMeasures.txt");
			tab[1] = new Traitement("acce","simMeasures.txt");
			tab[2] = new Traitement("mnet","simMeasures.txt");
			tab[3] = new Traitement("orie","simMeasures.txt");
			tabDefine = true;
		}
		else if (mode == 3)
		{
			createMeasureFile_separate(FILENAME, DIRECTIONS, SAMPLETIME);
			srand(time(NULL));
		}
		else if (mode == 4)
		{
			Serial link(PORTSERIE,BAUD);
			int nbMes;
			std::cout << "entrez le nombre de mesures a copier : " << std::endl;
			std::cin >> nbMes;
			fileFromSerial("serial.txt", link, nbMes);
		}
		else if (mode == 5)
		{
			test_eulerQuat();
			test_changeRepere();
		}

		if (tabDefine)
		{
			scene.bouclePrincipale(gant,tab);

			tab[0]->resetCompteur();
			tab[1]->resetCompteur();
			tab[2]->resetCompteur();
			tab[3]->resetCompteur();

			delete tab[0];
			delete tab[1];
			delete tab[2];
			delete tab[3];
			
			tabDefine = false;
		}

		mode = choiceMode();
	}
	//system("PAUSE");

	return 0;
}


/**
* \brief Créé un fichier de mesures à partir des directions indiquée dans un autre fichier
* la fonction recupere les indication dans le fichier direction 
* et les mets sous forme de fichier pouvant etre lu par la classe Traitement
* si les parametres variation et biais sont différents de 0, un deuxième fichier ("fichier_vrai.txt") est créé,
* ce fichier contient les mm mesures mais sans les erreurs. Il servira de référence
* \param filename : nom du fichier de mesures à créer
* \param direction : nom du fichier dans lequel la fonction va puiser ces information
* \param sampleTime : temps d'échantillonage que l'on veut utiliser lors de la simulation (en millisecondes)
* \param variation (défaut = 0) : erreur absolue maximale qui peut s'ajouter ou se soustraire à chaque mesure
* \param bias (défaut = 0) : biais à ajouter à chaque mesure
*/
void createMeasureFile(std::string filename, std::string direction, double sampleTime, double variation, double bias)
{
	std::fstream fDirection, fMeas;
	vect3D orientation = {0,0,0};
	bool endOfFile(false);
	int i = 0;
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
		double vitX, vitY, vitZ;
		while (1)
		{
			if (!getDirection(fDirection, temps, duree, val1, val2, val3, vitX, vitY, vitZ))
			{

				std::cout << "valeur 1 : "<< val1 << " ; valeur 2 : " << val2 << " ; valeur 3 : " << val3 << " ; temps : " << temps << " ; duree : " << duree << std::endl;

				if (temps >= tEcoule)
				{
					while ( tEcoule < temps)
					{
						fMeas << "gyro:" << '\n';
						fMeas << 'x' << addError(0,variation,bias) << ';' << 'y' << addError(0,variation,bias) << ';' << 'z' << addError(0,variation,bias) << ';' << 't' << tEcoule << ';' << '\n';
						fMeas << "orie;" << '\n';
						fMeas << 'x' << 0 << ';' << 'y' << 0 << ';' << 'z' << 0 << ';' << 't' << tEcoule << ';' << '\n';
						tEcoule += sampleTime;
					}

					while (tEcoule < temps+duree)
					{
						//Ajout des erreurs de mesure:
						val1e = addError(val1,variation,bias);
						val2e = addError(val2,variation,bias);
						val3e = addError(val3,variation,bias);
						fMeas << "gyro:" << '\n';
						fMeas << 'x' << val1e << ';' << 'y' << val2e << ';' << 'z' << val3e << ';' << 't' << tEcoule << ';' << '\n';
						orientation.x += val1*SAMPLETIME/1000;
						orientation.y += val2*SAMPLETIME/1000;
						orientation.z += val3*SAMPLETIME/1000;
						fMeas << "orie:" << '\n';
						fMeas << 'x' << orientation.x << ';' << 'y' << orientation.y << ';' << 'z' << orientation.z << ';' << 't' << tEcoule << ';' << '\n';
						tEcoule += sampleTime;
					}
				}
		
				else 
				{
						_RPT0(0, "temps inferieur a celui de la derniere ligne + duree, temps remanié");
						temps = tEcoule;

						while (tEcoule < temps+duree)
						{
						val1e = addError(val1,variation,bias);
						val2e = addError(val2,variation,bias);
						val3e = addError(val3,variation,bias);
						fMeas << "gyro:" << '\n';
						fMeas << 'x' << val1e << ';' << 'y' << val2e << ';' << 'z' << val3e << ';' << 't' << tEcoule << ';' << '\n';
						tEcoule += sampleTime;
						}
				}

			} //end if(!geDirection())

			else break;

		} //end while

		fMeas.close();
		fDirection.close();
		system("PAUSE");
	}

}

void createMeasureFile_separate(std::string filename, std::string direction, double sampleTime)
{
	std::fstream fDirection, fMeas_gyro, fMeas_gyro_vrai, fMeas_acce, fMeas_acce_vrai, fMeas_orie, fMeas_orie_vrai;
	double variation1, variation2, bias1, bias2;
	vect3D orientation = {0,0,0};
	vect3D orientation_err = {0,0,0};
	vect3D acce = {0,0,0};
	bool bGyro_vrai(false), bAcce_vrai(false), bOrie_vrai;
	bool bGyro(false), bOrie(false), bAcce(false);
	int i(0), j(0);

	fDirection.open(direction, std::ios::in);
	verifierOuverture(fDirection, "direction", _CRT_ERROR);
	
	getHeader(fDirection, variation1, variation2, bias1, bias2, bGyro, bAcce, bOrie);

	if(bGyro) {
		fMeas_gyro.open((filename+"_gyro.txt"), std::fstream::out);
		if(!verifierOuverture(fMeas_gyro, "_gyro"))
			bGyro = false;
	}

	if(bOrie) {
		fMeas_orie.open((filename+"_orie.txt"), std::fstream::out);
		if (!verifierOuverture(fMeas_gyro, "_orie"))
			bOrie = false;
	}

	if(bAcce) {
		fMeas_acce.open((filename+"_acce.txt"), std::fstream::out);
		if(!verifierOuverture(fMeas_gyro, "_acce"))
			bAcce = false;
	}

		double val1,val2,val3, tEcoule(0.0), temps, duree;
		double val1e, val2e, val3e;
		double vitX, vitY, vitZ;

		if(bGyro)
			writeHeader(fMeas_gyro,filename, variation1, bias1);
		if(bOrie)
			writeHeader(fMeas_orie,filename, variation1, bias1);
		if(bAcce)
			writeHeader(fMeas_acce,filename, variation2, bias2);

		std::cout << "bGyro = " << bGyro << std::endl; 
		std::cout << "bOrie = " << bOrie << std::endl;
		std::cout << "bAcce = " << bAcce << std::endl;

		if (bGyro_vrai = ((variation1 !=0 || bias1 !=0) && bGyro)) //attribution + condition
		{
			std::cout << "bool gyro_vrai = true " << std::endl;
			fMeas_gyro_vrai.open(filename+"gyro_vrai.txt",std::fstream::out);
			if (!verifierOuverture(fMeas_gyro_vrai, "gyro_vrai"))
				bGyro_vrai = false;
			else
				writeHeader(fMeas_gyro_vrai, filename+"gyro_vrai", 0, 0);
		}
				
		if (bOrie_vrai = ((variation1 !=0 || bias1 !=0) && bOrie)) //attribution + condition
		{
			std::cout << "bool orie_vrai = true " << std::endl;
			fMeas_orie_vrai.open(filename+"orie_vrai.txt",std::fstream::out);
			if (!verifierOuverture(fMeas_orie_vrai, "orie_vrai"))
				bOrie_vrai = false;
			else
				writeHeader(fMeas_orie_vrai, filename+"orie_vrai.txt", 0, 0);
		}

		if (bAcce_vrai = ((variation2 !=0 || bias2 !=0) && bAcce)) //attribution + condition
		{
			std::cout << "bool acce_vrai = true " << std::endl;
			fMeas_acce_vrai.open(filename+"acce_vrai.txt",std::fstream::out);
			if (!verifierOuverture(fMeas_acce_vrai, "acce_vrai"))
				bAcce_vrai = false;
			else
				writeHeader(fMeas_acce_vrai, filename+"acce_vrai", 0, 0);
		}
		
		while (1)
		{
			system("PAUSE");
			if (!getDirection(fDirection, temps, duree, val1, val2, val3, vitX, vitY, vitZ))
			{
				j++;
				if (bAcce) {
					//getVitesse(fDirection, vitX, vitY, vitZ, duree);
					std::cout << "vitesse x : "<< vitX << " ; vitesse y : " << vitY << " ; vitesse z : " << vitZ << std::endl;
				}

				std::cout << "valeur 1 : "<< val1 << " ; valeur 2 : " << val2 << " ; valeur 3 : " << val3 << " ; temps : " << temps << " ; duree : " << duree << std::endl;

				if (temps < tEcoule) 
				{
					temps = tEcoule;
					_RPT0(_CRT_WARN, "temps indique inferieur au temps ecoule, temps remanie...\n");
				}


				std::cout << tEcoule << '\t' << temps+duree << std::endl;
				while ( tEcoule < temps)
				{
					if(bGyro)
					{
						val1e = addError(0,variation1,bias1);
						val2e = addError(0,variation1,bias1);
						val3e = addError(0,variation1,bias1);
						i++;
						fMeas_gyro << 'x' << addError(0,variation1,bias1) << ';' << 'y' << addError(0,variation1,bias1) << ';' << 'z' << addError(0,variation1,bias1) << ';' << 't' << tEcoule << ';' << '\n';
						if(bGyro_vrai)
						{
							//fMeas_gyro_vrai << filename+"_vrai:" << '\n';
							fMeas_gyro_vrai << 'x' << 0 << ';' << 'y' << 0 << ';' << 'z' << 0 << ';' << 't' << tEcoule << ';' << '\n';
						}
					}

					if(bOrie)
					{
						if(!bGyro)
						{
							val1e = addError(0,variation1,bias1);
							val2e = addError(0,variation1,bias1);
							val3e = addError(0,variation1,bias1);
						}
						orientation_err.x += val1e*SAMPLETIME/1000;
						orientation_err.y += val2e*SAMPLETIME/1000;
						orientation_err.z += val3e*SAMPLETIME/1000;
						fMeas_orie << 'x' << orientation_err.x << ';' << 'y' << orientation_err.y << ';' << 'z' << orientation_err.z << ';' << 't' << tEcoule << ';' << '\n';
						if(bGyro_vrai)
							fMeas_orie_vrai << 'x' << orientation.x << ';' << 'y' << orientation.y << ';' << 'z' << orientation.z << ';' << 't' << tEcoule << ';' << '\n';
					}

					if(bAcce)
					{
						int i = 0;
						calculateAccel(orientation.x, orientation.y, acce.x, acce.y, acce.z);
						fMeas_acce << 'x' << acce.x << ';' << 'y' << acce.y << ';' << 'z' << acce.z << ';' << 't' << tEcoule << ';' << '\n';

					}
		
					tEcoule += sampleTime;

				}

				while (tEcoule < temps+duree)
				{
					if (bGyro)
					{
						//Ajout des erreurs de mesure:
						val1e = addError(val1,variation1,bias1);
						val2e = addError(val2,variation1,bias1);
						val3e = addError(val3,variation1,bias1);
						//fMeas_gyro << filename+":" << '\n';
						fMeas_gyro << 'x' << val1e << ';' << 'y' << val2e << ';' << 'z' << val3e << ';' << 't' << tEcoule << ';' << '\n';
						i++;
						if(bGyro_vrai)
						{
							//fMeas_gyro_vrai << filename+"_vrai:" << '\n';
							fMeas_gyro_vrai << 'x' << val1 << ';' << 'y' << val2 << ';' << 'z' << val3 << ';' << 't' << tEcoule << ';' << '\n';
						}
					}
					if(bOrie)
					{
						if(!bGyro)
						{
							val1e = addError(val1,variation1,bias1);
							val2e = addError(val2,variation1,bias1);
							val3e = addError(val3,variation1,bias1);
						}
						orientation_err.x += val1e*SAMPLETIME/1000;
						orientation_err.y += val2e*SAMPLETIME/1000;
						orientation_err.z += val3e*SAMPLETIME/1000;
						fMeas_orie << 'x' << orientation_err.x << ';' << 'y' << orientation_err.y << ';' << 'z' << orientation_err.z << ';' << 't' << tEcoule << ';' << '\n';
						if(bGyro_vrai)
						{
							orientation.x += val1*SAMPLETIME/1000;
							orientation.y += val2*SAMPLETIME/1000;
							orientation.z += val3*SAMPLETIME/1000;
							fMeas_orie_vrai << 'x' << orientation.x << ';' << 'y' << orientation.y << ';' << 'z' << orientation.z << ';' << 't' << tEcoule << ';' << '\n';
						}
					}
					if(bAcce)
					{
						calculateAccel(orientation.x, orientation.y, acce.x, acce.y, acce.z);
						fMeas_acce << 'x' << acce.x << ';' << 'y' << acce.y << ';' << 'z' << acce.z << ';' << 't' << tEcoule << ';' << '\n';

					}
		
					tEcoule += sampleTime;
				}


			} //end if(!getDirection())

			else break; // getDirection = true quand on atteint la fin du fichier ( file.eof() ). On sort alors de la boucle

		} //end while

		std::cout << "i = " << i << std::endl << "j = " << j << std::endl;;

		if(bGyro_vrai)
			fMeas_gyro_vrai.close();
		if(bOrie_vrai)
			fMeas_orie_vrai.close();
		if(bAcce_vrai)
		fMeas_gyro.close();
		fDirection.close();
		system("PAUSE");

}

bool verifierOuverture (const std::fstream &file1, const char* name, int reportType)
{
	if ((file1.rdstate() && std::ifstream::failbit) != 0)
	{
		_RPT1(reportType, "Erreur lors de l'ouverture du fichier \'%s\'\n", name);
		return false;
	}
	else
		return true;
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

bool getHeader( std::fstream &file, double &variation1, double &variation2, double &bias1, double &bias2, bool &bGyro, bool &bAcce, bool &bOrie)
{
	char buffer[128];
	file >> buffer[0];

	while (buffer[0] == '%')
	{
		file.getline(buffer, 128);
		if(strcmp("orie",buffer) == 0)
			bOrie = 1;
		else if(strcmp("gyro",buffer) == 0)
			bGyro = 1;
		else if(strcmp("acce",buffer) == 0)
			bAcce = 1;
		else if (strcmp("variation", buffer) == 0) {
			file.get(buffer[0]);
			file >> variation1;
			file.get(buffer[0]);
			file >> variation2;
			std::cout << "variation1 = " << variation1 << "--- variation2 = " << variation2 << std::endl;
			file.get(buffer[0]);//recupere le caractere de fin de ligne
		}
		else if (strcmp("biais",buffer) == 0) {
			file.get(buffer[0]);
			file >> bias1;
			file.get(buffer[0]);
			file >> bias2;
			std::cout << "biais1 = " << bias1 << "--- biais2 = " << bias2 << std::endl;
			file.get(buffer[0]);//recupere le caractere de fin de ligne
		}
		file.get(buffer[0]);
		std::cout << "caractere :" << buffer[0] << "---" << std::endl;
	}

	file.unget(); //recule le curseur de 2 caractères si le dernier caractère lu n'est pas un '%' (fin de ligne + premier caractère de la nouvelle ligne)
	file.unget(); 

	return true;
}

void writeHeader(std::fstream& file, std::string& name, const double& variation, const double& biais)
{
	file << '%' << name << '\n';
	file << '%' << "variation = " << variation << "; biais = " << biais << '\n';
}


/**
* \brief recupere les informations du fichier file et les stocke dans les differentes variables
*/
bool getDirection(std::fstream &file, double &temps, double &duree, double &val1, double &val2, double &val3, double &vitX, double &vitY, double &vitZ)
{
	char buffer[128];

	//file >> buffer[0]; //caractère saut de ligne
	std::cout << "00000get 1 :" << buffer[0] << "---" << std::endl;
	file >> temps;
	file >> buffer[0];
	std::cout << "00000get 2 :" << buffer[0] << "---" << std::endl;
	file >> duree;
	file >> buffer[0];
	file >> val1;
	file >> buffer[0];
	file >> val2;
	file >> buffer[0];
	file >> val3;
	file >> buffer[0];
	file >> vitX;
	file >> buffer[0];
	file >> vitY;
	file >> buffer[0];
	file >> vitZ;

	val1 /= (duree/1000.0);
	val2 /= (duree/1000.0);
	val3 /= (duree/1000.0);

	return file.eof();

}

bool getVitesse(std::fstream &file, double &val1, double &val2, double &val3, const double duree)
{
	char buffer[128];

	file >> buffer[0];
	file >> val1;
	file >> buffer[0];
	file >> val2;
	file >> buffer[0];
	file >> val3;

	val1 /= (duree/1000.0);
	val2 /= (duree/1000.0);
	val3 /= (duree/1000.0);
	std::cout << "buffer[0] = " << buffer[0] << std::endl;
	return file.eof();
}


void calculateAccel(const double &phi, const double &teta, double &accX, double &accY, double &accZ)
{
	accX -= sin(teta)*cos(phi)*G;
	accY -= sin(phi)*G; 
	accZ -= cos(teta)*cos(phi)*G;
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

