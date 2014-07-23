#ifndef DEF_INSTRUMENT
#define DEF_INSTRUMENT

#include "Serial.h"
//#include <Windows.h>
#include <time.h>
#include <iostream>
#include "Structure.h"

vect3D operator+(vect3D v1, vect3D v2);

vect3D operator*(vect3D v2, double* v1);

vect4D operator+(vect4D v1, vect3D v2);

class Instrument {

private : 
		char* ID;
		vect4D _mesures;
		vect3D _valeursInitiales;
		clock_t _t_acq[3];
		Serial _serialLink;
		
		//retire les valeurs initiales des mesures
		void soustraireVI(void) ;

public:
		//constructor
		Instrument(char* nom, std::string port, int baudRate);

		//destructor//
		~Instrument();

		//getter//

		vect4D getMesures(void);
		
		double getMesure(int axe);
		

		clock_t* getTemps(void) ;

		/**
		* \brief return the time of the last measure according to the parameter axe
		*/
		clock_t getTemps(int axe);


		//setter//

		/** \brief fills the _t_acq table with the new values (as parameter)
		*/
		void setTemps(clock_t* nvTemps) ;

		/** \brief fills the _valeursInitiales table with the new values (as parameter)
		*/
		void setVI(vect3D valeurs);

		/**met a jour les valeurs des mesures de l'instrument avec les valeurs passées en paramètre*/
		void majMesures(vect3D nvMesures);

		/** 
		 * \brief mise a jour des valeurs de l'instrument avec les données envoyées via le port série 
		 * renvoie aussi l'heure à laquelle ces valeurs ont été mises à jour;
		 * lit forcément les valeurs de chaque axe au moins une fois (axe4 = axe du temps)
		 */
		void majSerial();
		
		//calibrer l'instrument en initialisant les valeurs initiales
		void calibrer(void);

		void afficherMesures() const;

		void afficherTemps() const;

		void afficherVI() const;

};


#endif