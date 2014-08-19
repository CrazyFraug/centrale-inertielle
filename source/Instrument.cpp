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

/* \brief Surcharge operateur* pour un vecteur 3D et un double*/
vect3D operator*(vect3D v2, double v1)
{
	vect3D result;
	result.x = v2.x * v1;
	result.y = v2.y * v1;
	result.z = v2.z * v1;
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
* prend en argument une chaine de caracteres
* \param nom : chaine de caractere qui identifie l'instrument
*/
Instrument::Instrument(char* nom) :ID(nom)
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

/** \brief Destructeur
*/
Instrument::~Instrument()
{
}


/**Getter**/

/** brief return the "mesures" 4D vector
*/
vect4D Instrument::getMesures(void) { return _mesures; }

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

clock_t* Instrument::getTemps(void) { return _t_acq; }

/**
* \brief return the time of the last measure according to the parameter axe
*/
clock_t Instrument::getTemps(int axe) { return _t_acq[axe - 1]; }

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


void Instrument::setMesuresX(double val){ _mesures.x = val; }
void Instrument::setMesuresY(double val){ _mesures.y = val; }
void Instrument::setMesuresZ(double val){ _mesures.z = val; }
void Instrument::setMesuresT(double val){ _mesures.temps = val; }
/** Autres methodes **/

void Instrument::majMesures()
{

}


std::string Instrument::getnomfichier(void){
	return NULL;
}

/** \brief retire les conditions initiales (in progress)*/
void Instrument::soustraireVI(void)
{
	_mesures.x -= _valeursInitiales.x;
	_mesures.y -= _valeursInitiales.y;
	_mesures.z -= _valeursInitiales.z;
}




//calibrer l'instrument en initialisant les valeurs initiales
void Instrument::calibrer(void)
{
	majMesures();
	_mesures.x = 0;
	_mesures.y = 0;
	_mesures.z = 0;

	while ((_mesures.x == 0) || (_mesures.y == 0) || (_mesures.z == 0))
	{
		majMesures();

	}
	_valeursInitiales.x = _mesures.x;
	_valeursInitiales.y = _mesures.y;
	_valeursInitiales.z = _mesures.z;

}

void Instrument::afficherMesures() const
{
	_RPT1(0, "valeur pour %s : \n", ID);
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

/** Sous-classe Instrument_serie **/


/**
* \brief Constructeur d'Instrument_serie
* prend en argument un objet de type Serial
* \param nom : chaine de caractere qui identifie l'instrument
* \param link : communication série associée
*/
Instrument_serie::Instrument_serie(char* nom, Serial* link) :Instrument(nom), _serialLink(link)
{
	std::string filename = getID();
	filename += ".txt";
	nom_fichier = filename;
}

Instrument_serie::~Instrument_serie() { }

void Instrument_serie::majMesures()
{
	majSerial();
}

/**
* \brief mise a jour des valeurs de l'instrument avec les données envoyées via le port série
*/
void Instrument_serie::majSerial(/*char *IDSensor*/)
{
	_RPT1(0, "SENSOR NAME: %s \n", getID());
	packDatas data = _serialLink->readData_s(getID());
	if (strcmp(getID(), data.sensor_name.c_str()) == 0){
		setMesuresX(data.datas.x);
		setMesuresY(data.datas.y);
		setMesuresZ(data.datas.z);
		setMesuresT(data.datas.temps);
	}
}


std::string Instrument_serie::getnomfichier(void){
	return nom_fichier;
}


void Instrument::afficherCapteur(void){
	_RPT1(0, "CAPTEUR: %s \n", ID);
	afficherMesures();
}