#include "Mobile.h"

using namespace std;

Mobile::Mobile(void)
	{
		_orientation.x = 1;
		_orientation.y = 0;
		_orientation.z = 0;
		_angles.x = 0;
		_angles.y = 0;
		_angles.z = 0;
		_position.x = 0;
		_position.y = 0;
		_position.z = 0;
		_vitesse.x = 0;
		_vitesse.y = 0;
		_vitesse.z = 0;
	}

Mobile::~Mobile() {}

vect3D Mobile::getAngles() {return _angles;}

vect3D Mobile::getPosition() {return _position;}

vect3D Mobile::getOrientation() {return _orientation;}

vect3D Mobile::getVitesse() {return _vitesse;}

void Mobile::norm_Angle(double alpha)
{
        if (alpha > 180)
			{alpha -= 360;}
		else if (alpha< -180)
			{alpha += 360;}
}


double Mobile::norm_Vecteur()
{
	return sqrt(pow(_orientation.x,2)+pow(_orientation.y,2)+pow(_orientation.z,2)); 
}

void Mobile::rotate(vect3D rot) 
{

}

void Mobile::rotate(quaternion<double> q)
{
	rotateVector(q,_orientation);
}

void Mobile::majAngles()
{
	_angles.x = acos(_orientation.x/norm_Vecteur());
	_angles.y = acos(_orientation.y/norm_Vecteur());
	_angles.z = acos(_orientation.z/norm_Vecteur());
}

void Mobile::deplacer(double dx, double dy, double dz)
{
	_position.x += dx;
	_position.y += dy;
	_position.z += dz;
}

void Mobile::setVitesse (double v_x, double v_y, double v_z)
{
    _vitesse.x = v_x;
    _vitesse.y = v_y;
    _vitesse.z = v_z;
}

void Mobile::setPosition(double x, double y, double z)
{
	_position.x = x;
	_position.y = y;
	_position.z = z;
}

void Mobile::afficher_position()
{
	cout << "Mobile: position : " << endl ;
	cout << "X = " << _position.x << endl ;
	cout << "Y = " << _position.y << endl ;
	cout << "Z = " << _position.z << endl ;

	_RPT1(0, "Mobile: position en x : %f\n", _position.x);
	_RPT1(0, "Mobile: position en y : %f\n", _position.y);
	_RPT1(0, "Mobile: position en z : %f\n", _position.z);
}

void Mobile::afficher_vitesse()
{
	cout << "Mobile: vitesse : " << endl ;
	cout << "X = " << _vitesse.x << endl ;
	cout << "Y = " << _vitesse.y << endl ;
	cout << "Z = " << _vitesse.z << endl ;

	_RPT1(0, "Mobile: vitesse en x : %f\n", _vitesse.x);
	_RPT1(0, "Mobile: vitesse en y : %f\n", _vitesse.y);
	_RPT1(0, "Mobile: vitesse en z : %f\n", _vitesse.z);
}

void Mobile::afficher_angles(void) 
{
	cout << "Mobile: angles : " << endl ;
	cout << "X = " << _angles.x << endl ;
	cout << "Y = " << _angles.y << endl ;
	cout << "Z = " << _angles.z << endl ;

	_RPT1(0, "Mobile: angles en x : %f\n", _angles.x);
	_RPT1(0, "Mobile: angles en y : %f\n", _angles.y);
	_RPT1(0, "Mobile: angles en z : %f\n", _angles.z);
}

void Mobile::afficher_orientation(void) 
{
	cout << "Mobile: orientation : " << endl ;
	cout << "X = " << _orientation.x << endl ;
	cout << "Y = " << _orientation.y << endl ;
	cout << "Z = " << _orientation.z << endl ;

	_RPT1(0, "Mobile: orientation en x : %f\n", _orientation.x);
	_RPT1(0, "Mobile: orientation en y : %f\n", _orientation.y);
	_RPT1(0, "Mobile: orientation en z : %f\n", _orientation.z);
}