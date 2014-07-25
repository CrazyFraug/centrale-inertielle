#include "Instrument.h"
/**Surcharge d'operateur les vecteur 3D et 4D**/

/* \brief Surcharge operateur+ pour 2 vecteurs 3D*/
vect3D operator+(vect3D v1, vect3D v2)
{
	vect3D v;
	v.x = v1.x + v2.x;
	v.y = v1.y + v2.y;
	v.z = v1.z + v2.z;
	return v;
}

/* \brief Surcharge operateur* pour un vecteur 3D et un nombre*/
vect3D operator*(vect3D v2, double* v1) 
{
	vect3D result;
	result.x = v2.x * v1[0];	
	result.y = v2.y * v1[1];
	result.z = v2.z * v1[2];
	return result;
}

/* \brief Surcharge operateur+ pour un vecteur 4D et un 3D
* le temps reste inchangé */
vect4D operator+(vect4D v1, vect3D v2) 
{
	vect4D result;
	result.x = v2.x + v1.x;	
	result.y = v2.y + v1.y;
	result.z = v2.z + v1.z;
	return result;
}


/** 
* \brief Constructeur d'Instrument
* prend en argument un objet de type Serial
* \param nom : chaine de caractere qui identifie l'instrument
* \param link : communication série associée
*/
Instrument::Instrument(char* nom, Serial* link):ID(nom),_p_serialLink(link)
{
	_mesures.x = 0;
	_mesures.y = 0;
	_mesures.z = 0;
	_mesures.temps = clock();
	_valeursInitiales.x = 0;
	_valeursInitiales.y = 0;
	_valeursInitiales.z = 0;
	_t_acq[0] = clock();
	_t_acq[1] = clock();
	_t_acq[2] = clock();
}


/** 
* \brief Constructeur d'Instrument
* prend en argument les parametre de la communication série
* \param nom : chaine de caractere qui identifie l'instrument
* \param port : nom du port serie utilisé, défini en tant que constante dans SceneOpenGL.h
* \param baudRate : baudrate de la laison série
*/
Instrument::Instrument(char* nom, std::string port, int baudRate):ID(nom)
{
	_p_serialLink = new Serial(port, baudRate);
	_mesures.x = 0;
	_mesures.y = 0;
	_mesures.z = 0;
	_mesures.temps = clock();
	_valeursInitiales.x = 0;
	_valeursInitiales.y = 0;
	_valeursInitiales.z = 0;
	_t_acq[0] = clock();
	_t_acq[1] = clock();
	_t_acq[2] = clock();
}


/** \brief Destructeur
*/
Instrument::~Instrument() 
{
}


/**Getter**/
		
/** brief return the "mesures" 4D vector
*/
vect4D Instrument::getMesures(void) {return _mesures;}
		
/**
* \brief Return the last measure of the Instrument relative to an axe
* \param [in] axe is the axe you want the measure from (axe = 1 -> X axis | ... | axe = 4 -> time axis
*/
double Instrument::getMesure(int axe)
{
	switch (axe)
	{
	case 1:
		return _mesures.x;
		break;
	case 2:
		return _mesures.y;
		break;
	case 3:
		return _mesures.z;
		break;
	case 4:
		return _mesures.temps;
		break;
	default:
		return 0;
		break;
	}
}

clock_t* Instrument::getTemps(void) {return _t_acq;}

/**
* \brief return the time of the last measure according to the parameter axe
*/
clock_t Instrument::getTemps(int axe) {return _t_acq[axe-1];}

char* Instrument::getID(void){
	return ID;
}
/**Setter**/

/** \brief fills the _t_acq table with the new values (as parameter)
*/
void Instrument::setTemps(clock_t* nvTemps) 
{ 
	_t_acq[0] = nvTemps[0];
	_t_acq[1] = nvTemps[1];
	_t_acq[2] = nvTemps[2];
}

/** \brief fills the _valeursInitiales table with the new values (as parameter)
*/
void Instrument::setVI(vect3D valeurs)
{
	_valeursInitiales.x = valeurs.x;
	_valeursInitiales.y = valeurs.y;
	_valeursInitiales.z = valeurs.z;
}


/** Autres methodes **/


/** \brief retire les conditions initiales (in progress)*/
void Instrument::soustraireVI(void) 
{
	_mesures.x -= _valeursInitiales.x;
	_mesures.y -= _valeursInitiales.y;
	_mesures.z -= _valeursInitiales.z;
}
/**
* \brief met a jour les mesures de l'instrument avec les valeurs passées en paramètre
*/
void Instrument::majMesures(vect4D nvMesures)
{
	_mesures.x = nvMesures.x;
	_mesures.y = nvMesures.y;
	_mesures.z = nvMesures.z;
	_mesures.temps = nvMesures.temps;
	soustraireVI();
}

/** 
	* \brief mise a jour des valeurs de l'instrument avec les données envoyées via le port série 
	* renvoie aussi l'heure à laquelle ces valeurs ont été mises à jour;
	* lit forcément les valeurs de chaque axe au moins une fois (axe4 = axe du temps)
	*/
void Instrument::majSerial(char* type)
{
	double value;
	bool capteur(false);
	int axe=0;
	bool axe1(false), axe2(false), axe3(false), axe4(false);
	while((axe1==false) || (axe2==false) || (axe3==false))
	{
		value = _p_serialLink->readDatas(axe, type, capteur);
		switch (axe)
		{
		case 1:
			_mesures.x = -value;
			_t_acq[0] = clock();
			axe1 = true;
			break;
		case 2:
			_mesures.y = value;
			_t_acq[1] = clock();
			axe2 = true;
			break;
		case 3:
			_mesures.z = -value;
			_t_acq[2] = clock();
			axe3 = true;
			break;
		case 4 :
			_mesures.temps = value;
			axe4 = true;
			break;
		}
	}
	soustraireVI();
}
		
//calibrer l'instrument en initialisant les valeurs initiales
void Instrument::calibrer(void)
{
	majSerial(ID);
	_mesures.x = 0;
	_mesures.y = 0;
	_mesures.z = 0;

	while((_mesures.x == 0) || (_mesures.y == 0) || (_mesures.z == 0))
	{
		majSerial(ID);
		
	}
	_valeursInitiales.x = _mesures.x;
	_valeursInitiales.y = _mesures.y;
	_valeursInitiales.z = _mesures.z;

}

void Instrument::afficherMesures() const
{
	_RPT1(0,"valeur pour %s : \n", ID);	
	_RPT1(0, " x = %f \n", _mesures.x);
	_RPT1(0, " y = %f \n", _mesures.y);
	_RPT1(0, " z = %f \n", _mesures.z);

	std::cout << "valeur pour " << ID << " : " << std::endl;
	std::cout << "x = " << _mesures.x << std::endl;
	std::cout << "y = " << _mesures.y << std::endl;
	std::cout << "z = " << _mesures.z << std::endl;

}

void Instrument::afficherTemps() const
{
	_RPT0(0, " temps : \n");
	_RPT1(0, " t1 = %f \n", _t_acq[0]);
	_RPT1(0, " t2 = %f \n", _t_acq[1]);
	_RPT1(0, " t3 = %f \n", _t_acq[2]);

	std::cout << "temps : " << std::endl;
	std::cout << "t1 = " << _t_acq[0] << std::endl;
	std::cout << "t2 = " << _t_acq[1] << std::endl;
	std::cout << "t3 = " << _t_acq[2] << std::endl;

}

void Instrument::afficherVI() const
{
	_RPT0(0, " valeurs initiales : \n");
	_RPT1(0, " vi1 = %f \n", _valeursInitiales.x);
	_RPT1(0, " vi2 = %f \n", _valeursInitiales.y);
	_RPT1(0, " vi3 = %f \n", _valeursInitiales.z);

	std::cout << "valeurs initiales : " << std::endl;
	std::cout << "vi1 = " << _valeursInitiales.x << std::endl;
	std::cout << "vi2 = " << _valeursInitiales.y << std::endl;
	std::cout << "vi3 = " << _valeursInitiales.z << std::endl;

}
