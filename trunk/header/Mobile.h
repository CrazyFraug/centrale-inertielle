#ifndef DEF_MOBILE
#define DEF_MOBILE

#include <Windows.h>
#include <iostream>
#include <iomanip>
#include <math.h>

#include "Quaternion.h"
#include "Structure.h"

void substractG(double matrice[3][3], double* accel_x, double* accel_y, double* accel_z);

class Mobile {

private :
	vect3D _orientation ;
	vect3D _angles ;
	vect3D _position ;
	vect3D _vitesse ;

public:
	Mobile(void);
	~Mobile();
	/**Getter**/
	vect3D getOrientation();
	vect3D getAngles();
	vect3D getPosition();
	vect3D getVitesse();
	/**Setter**/
	void setVitesse (double v_x, double v_y, double v_z);
	void setPosition(double x, double y, double z) ;

	void norm_Angle(double alpha);
	double norm_Vecteur();
	void rotate(vect3D rotation);
	void rotate(quaternion<double> q);
	void majAngles(void);
	void deplacer(double dx, double dy, double dz);

	/*Affichage*/
	void afficher_position(void);
	void afficher_angles(void);
	void afficher_vitesse(void);
	void afficher_orientation (void);
	
};

#endif

