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

/************************************************************************/

/*	Constructeur	*/
Instrument::Instrument(char* nom, int mode) :ID(nom)
{
	if (mode == 1){
		static Serial link("COM8", 115200);
		_serialLink = &link;
	}
	else {
		_serialLink = NULL;
	}
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

/************************************************************************/

/*	Destructeur	*/
Instrument::~Instrument()
{
}

/************************************************************************/

/*	Getter	*/

vect4D Instrument::getMesures(void) { return _mesures; }

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

clock_t Instrument::getTemps(int axe) { return _t_acq[axe - 1]; }

std::string Instrument::getnomfichier(void){
	return fileName;
}
char* Instrument::getID(void){
	return ID;
}

/************************************************************************/

/*	Setter	*/

void Instrument::setTemps(clock_t* nvTemps)
{
	_t_acq[0] = nvTemps[0];
	_t_acq[1] = nvTemps[1];
	_t_acq[2] = nvTemps[2];
}

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

/************************************************************************/

void Instrument::majMesures()
{
	if (_serialLink != NULL){
		_RPT1(0, "SENSOR NAME: %s \n", getID());
		packDatas data = _serialLink->readDatas(getID());
		if (strcmp(getID(), data.sensor_name.c_str()) == 0){
			setMesuresX(data.datas.x);
			setMesuresY(data.datas.y);
			setMesuresZ(data.datas.z);
			setMesuresT(data.datas.temps);
		}
		else{
			_RPT2(0, "CAPTEUR COM %s N'EST PAS LE CAPTEUR DEMANDE %s \n", data.sensor_name.c_str(), getID());
		}
	}
}

/************************************************************************/

void Instrument::soustraireVI(void)
{
	_mesures.x -= _valeursInitiales.x;
	_mesures.y -= _valeursInitiales.y;
	_mesures.z -= _valeursInitiales.z;
}

/************************************************************************/

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

/************************************************************************/

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

/************************************************************************/

