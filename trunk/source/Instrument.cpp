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
		
		double getMesure(int axe)
		{
			switch (axe)
			{
			case 1:
				return mesures.x;
				break;
			case 2:
				return mesures.y;
				break;
			case 3:
				return mesures.z;
				break;
			default:
				return 0;
				break;
			}
		}

		clock_t* getTemps(void) {return t_acq;}

		clock_t getTemps(int axe) {return t_acq[axe-1];}
		
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

		//met a jour les valeurs des mesures de l'instrument avec les valeurs passées en paramètre
		void majMesures(vect3D nvMesures)
		{
			mesures = nvMesures;
			soustraireVI();
		}

		/** 
		 * \brief mise a jour des valeurs de l'instrument avec les données envoyées via le port série
		 * lit forcément les valeurs de chaque axe au moins une fois
		 */
		void majSerial()
		{
			double value;
			int axe=0;
			bool axe1(false), axe2(false), axe3(false);
			while((axe1==false) || (axe2==false) || (axe3==false))
			{
				value = serialLink->readDatas(axe);
				switch (axe)
				{
				case 1:
					mesures.x = -value;
					t_acq[0] = clock();
					axe1 = true;
					//std::cout << "test x : " << mesures.x << std::endl << std::endl << std::endl;
					break;
				case 2:
					mesures.y = value;
					t_acq[1] = clock();
					axe2 = true;
					//std::cout << std::endl << "test y : " << mesures.y << std::endl << std::endl;
					break;
				case 3:
					mesures.z = -value;
					t_acq[2] = clock();
					axe3 = true;
					//std::cout << std::endl << std::endl <<"test z : " << mesures.z << std::endl;
					break;
				}
			}
			soustraireVI();
		}
		
		//calibrer l'instrument en initialisant les valeurs initiales
		void calibrer(void)
		{
			majSerial();
			mesures.x = 0;
			mesures.y = 0;
			mesures.z = 0;

			while((mesures.x == 0) || (mesures.y == 0) || (mesures.z == 0))
			{
				majSerial();
		
			}
			valeursInitiales = mesures;
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