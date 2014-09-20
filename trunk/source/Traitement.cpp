#include "Traitement.h"
#include "Tools.h"
#include <iostream>

/** Constructeur **/
Traitement::Traitement(Instrument* inst) :_compteur(0), _dt(0)
{
	_capteur = inst;
	/** Allocation mémoire de la matrice de valeurs **/
	for (int i = 0; i < 3; i++)
	{
		_valeurs[i] = new double[NB_VALEURS];
	}
}

/** Destructeur **/
Traitement::~Traitement()
{
	for (int i = 0; i < 3; i++)
	{
		delete _valeurs[i];
	}
}

/*	Getter	*/
double	Traitement::get_dt(){
	return _dt;
}

/***************************************************************************/

void Traitement::stockerValeurs(vect4D val)
{
	_tempsAct = val.temps;
	_dt = (_tempsAct - _tempsPrec) / 1000.0;
	_tempsPrec = _tempsAct;

#ifdef TEST
	_RPT1(0, "nouvelles valeurs à stocker : \r\nval.x = %f\n", val.x);
	_RPT1(0, "val.y = %f\n", val.y);
	_RPT1(0, "val.x = %f\n", val.z);
#endif
	tabData[val.temps].x = val.x;
	tabData[val.temps].y = val.y;
	tabData[val.temps].z = val.z;
	if (_compteur < NB_VALEURS) // cas ou le tableau n'est pas plein
	{
		_valeurs[0][_compteur] = val.x;
		_valeurs[1][_compteur] = val.y;
		_valeurs[2][_compteur] = val.z;
		_compteur++;
	}

	else // cas ou le tableau est deja rempli
	{
		//_RPT0(0, "tableau rempli !\n");
		for (int i = 0; i < NB_VALEURS - 1; i++)
		{
			_valeurs[0][i] = _valeurs[0][i + 1];
			_valeurs[1][i] = _valeurs[1][i + 1];
			_valeurs[2][i] = _valeurs[2][i + 1];
		}

		if (_compteur < 0)
			_compteur = 0;
		_valeurs[0][_compteur - 1] = val.x;
		_valeurs[1][_compteur - 1] = val.y;
		_valeurs[2][_compteur - 1] = val.z;

	}
	//_RPT1(0, "compteur = %d\n", _compteur);
}

/***************************************************************************/

void Traitement::stockerValeurs()
{
	_tempsAct = _capteur->getMesure(4); /* 4 correspond à l'axe temporel (mesures.temps) */
	_dt = (_tempsAct - _tempsPrec) / 1000.0;
	_tempsPrec = _tempsAct;

	tabData[_capteur->getMesure(4)].x = _capteur->getMesure(1);
	tabData[_capteur->getMesure(4)].y = _capteur->getMesure(2);
	tabData[_capteur->getMesure(4)].z = _capteur->getMesure(3);

	if (_compteur < NB_VALEURS) // cas ou le tableau n'est pas plein
	{
		_valeurs[0][_compteur] = _capteur->getMesure(1);
		_valeurs[1][_compteur] = _capteur->getMesure(2);
		_valeurs[2][_compteur] = _capteur->getMesure(3);
		_compteur++;
	}

	else // cas ou le tableau est deja rempli
	{

		//_RPT0(0, "tableau rempli !\n");
		for (int i = 0; i < NB_VALEURS - 1; i++)
		{
			_valeurs[0][i] = _valeurs[0][i + 1];
			_valeurs[1][i] = _valeurs[1][i + 1];
			_valeurs[2][i] = _valeurs[2][i + 1];
		}

		_valeurs[0][_compteur - 1] = _capteur->getMesure(1);
		_valeurs[1][_compteur - 1] = _capteur->getMesure(2);
		_valeurs[2][_compteur - 1] = _capteur->getMesure(3);

	}
	//_RPT1(0, "compteur = %d\n", _compteur);
}

/***************************************************************************/

double Traitement::moyennerAxe(int axe)
{
	double moyenne = 0;
	for (int i = 0; i < _compteur; i++)
	{
		moyenne += _valeurs[axe - 1][i];
	}

	return (moyenne / _compteur);
}


vect4D Traitement::moyenner(int nb)
{
	vect4D res = { 0, 0, 0, 0 };
	if (nb > _compteur)
	{
		nb = _compteur;
	}
	//_RPT1(0,"(moyenner) _tempsPrec= %f\n",_tempsPrec);
	//_RPT1(0,"(moyenner) valeur de nb : %d\n", nb);
	for (int i = 0; i < nb; i++)
	{
		res.x += _valeurs[0][_compteur - i - 1];
		res.y += _valeurs[1][_compteur - i - 1];
		res.z += _valeurs[2][_compteur - i - 1];
		_RPT1(0, "(moyenner) valeur de res.x : %f\n", res.x);
	}
	res.x /= nb;
	res.y /= nb;
	res.z /= nb;
	res.temps = _tempsAct;
	//_RPT1(0,"(moyenner) valeur de res.x : %f\n", res.x);
	return res;
}

vect4D Traitement::moyenner(double temps, int nb)
{
	double tpsNext;
	vect4D res = { 0, 0, 0, 0 };
	while (tabData.count(temps) == 0){
		temps++;
	}
	res.x = tabData[temps].x;
	res.y = tabData[temps].y;
	res.z = tabData[temps].z;

	tpsNext = temps;

	for (int i = 0; i < nb; i++){
		tpsNext += 30;
		while (tabData.count(tpsNext) == 0){
			tpsNext++;
		}
		res.x += tabData[tpsNext].x;
		res.y += tabData[tpsNext].y;
		res.z += tabData[tpsNext].z;
	}

	return{ res.x / nb, res.y / nb, res.z / nb, tpsNext };
}

/***************************************************************************/

vect3D Traitement::calculerAngle_deg()
{
	vect3D angles;
	angles.x = moyennerAxe(1)*_dt * 180 / (atan(1) * 4);
	angles.y = moyennerAxe(2)*_dt * 180 / (atan(1) * 4);
	angles.z = moyennerAxe(3)*_dt * 180 / (atan(1) * 4);
	_RPT1(0, " _dt = %f ms\n", _dt);
	std::cout << " _dt = " << _dt << " ms " << std::endl;
	return angles;
}

/***************************************************************************/

void Traitement::afficherValeurs()
{
	_RPT1(0, "CAPTEUR %s : \n", _capteur->getID());
	std::cout << "CAPTEUR " << _capteur->getID() << ": " << std::endl;
	if (_compteur == NB_VALEURS) {
		_RPT1(0, "moyenne X = %f\n", moyennerAxe(1));
		_RPT1(0, "moyenne Y = %f\n", moyennerAxe(2));
		_RPT1(0, "moyenne Z = %f\n", moyennerAxe(3));

		std::cout << "moyenne X = " << moyennerAxe(1) << "                      " << std::endl;
		std::cout << "moyenne Y = " << moyennerAxe(2) << "                      " << std::endl;
		std::cout << "moyenne Z = " << moyennerAxe(3) << "                      " << std::endl;
	}
}

/***************************************************************************/

bool Traitement::tabFull()
{
	if (_compteur == NB_VALEURS)
		return true;
	else
		return false;
}

/***************************************************************************/

