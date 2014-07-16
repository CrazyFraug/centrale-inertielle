#include "Serial.cpp"
#include <Windows.h>
#include <time.h>
#include <iostream>
#include "Structure.h"


class Instrument {

private : 
		char* ID;
		vect3D mesures;
		vect3D valeursInitiales;
		double* dt;
		clock_t t_acq[3];
		SimpleSerial* serialLink;
		
		//retire les valeurs initiales des mesures
		void soustraireVI(void) 
		{
			mesures.x -= valeursInitiales.x;
			mesures.y -= valeursInitiales.y;
			mesures.z -= valeursInitiales.z;
		}

public:
		//constructor
	Instrument(char* nom, SimpleSerial* link):ID(nom),serialLink(link)
		{
			mesures.x = 0;
			mesures.y = 0;
			mesures.z = 0;
			valeursInitiales.x = 0;
			valeursInitiales.y = 0;
			valeursInitiales.z = 0;
			t_acq[0] = clock();
			t_acq[1] = clock();
			t_acq[2] = clock();
			dt = new double[3];
		}

		//destructor//
		~Instrument() 
		{
		}

		//getter//
		vect3D getMesures(void) {return mesures;}

		clock_t* getTemps(void) {return t_acq;}
		
		double* getdt(void) {return dt;}

		//setter//
		void setTemps(clock_t* nvTemps) 
		{ 
			t_acq[0] = nvTemps[0];
			t_acq[1] = nvTemps[1];
			t_acq[2] = nvTemps[2];
		}

		void setVI(vect3D valeurs)
		{
			valeursInitiales.x = valeurs.x;
			valeursInitiales.y = valeurs.y;
			valeursInitiales.z = valeurs.z;
		}

		//met a jour les valeurs de l'instrument
		void majMesures(vect3D nvMesures)
		{
			mesures = nvMesures;
			soustraireVI();
		}

		//mise a jour des valeurs de l'instrument avec les données envoyées via le port série
		void majSerial()
		{
			int axe;
			double value;

			value = serialLink->readDatas(axe);

			switch (axe)
			{
			case 1:
				mesures.x = -value;
				//dt[0] = (clock()-t_acq[0])/CLOCKS_PER_SEC;
				t_acq[0] = clock();
				break;
			case 2:
				mesures.y = value;
				//dt[1] = (clock()-t_acq[1])/CLOCKS_PER_SEC;
				t_acq[1] = clock();
				break;
			case 3:
				mesures.z = -value;
				//dt[2] = (clock()-t_acq[2])/CLOCKS_PER_SEC;
				t_acq[2] = clock();
				break;
			}
			soustraireVI();

		}
		
		void calibrer(void)
		{
			vect3D VI;
			while((VI.x == 0) || (VI.y != 0) || (VI.z != 0))
			{

			}

		}

		void afficherMesures() const
		{
			std::cout << "valeur pour " << ID << " : " << std::endl;
			std::cout << "x = " << mesures.x << std::endl;
			std::cout << "y = " << mesures.y << std::endl;
			std::cout << "z = " << mesures.z << std::endl;

		}

		void afficherTemps() const
		{
			std::cout << "temps : " << std::endl;
			std::cout << "t1 = " << t_acq[0] << std::endl;
			std::cout << "t2 = " << t_acq[1] << std::endl;
			std::cout << "t3 = " << t_acq[2] << std::endl;

		}

		void afficherVI() const
		{
			std::cout << "valeurs initiales : " << std::endl;
			std::cout << "vi1 = " << valeursInitiales.x << std::endl;
			std::cout << "vi2 = " << valeursInitiales.y << std::endl;
			std::cout << "vi3 = " << valeursInitiales.z << std::endl;

		}
};