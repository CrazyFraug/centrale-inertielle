#include "Serial.cpp"
#include <Windows.h>
#include <time.h>
#include <iostream>
//#include <string.h>
#include "Structure.h"


class Instrument {

private : 
		char* ID;
		vect3D mesures;
		vect3D valeursInitiales;
		clock_t t_acq[3];
		SimpleSerial *serialLink;
		
		//retire les valeurs initiales des mesures
		void soustraireVI(void) 
		{
			mesures.x -= valeursInitiales.x;
			mesures.y -= valeursInitiales.y;
			mesures.z -= valeursInitiales.z;
		}

public:
		//constructor
		Instrument(char* nom)
		{
			serialLink = new SimpleSerial("COM8",115200);
			ID = new char[sizeof(nom)/sizeof(*nom)];
			ID = nom;
		}

		//destructor
		~Instrument() 
		{
		}

		//getter
		vect3D getMesures(void) {return mesures;}

		clock_t* getTemps(void) {return t_acq;}
		
		//setter
		void setTemps(clock_t* nvTemps) 
		{ 
			t_acq[0] = nvTemps[0];
			t_acq[1] = nvTemps[1];
			t_acq[2] = nvTemps[2];
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
				mesures.x = value;
				break;
			case 2:
				mesures.y = value;
				break;
			case 3:
				mesures.z = value;
				break;
			}
		}

		void afficherMesures()
		{
			std::cout << "valeur pour " << ID << " : " << std::endl;
			std::cout << "x = " << mesures.x << std::endl;
			std::cout << "y = " << mesures.y << std::endl;
			std::cout << "z = " << mesures.z << std::endl;

		}
};