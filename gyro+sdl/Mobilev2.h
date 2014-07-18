#ifndef DEF_MOBILE
#define DEF_MOBILE

#include <Windows.h>
#include <time.h>
#include <iostream>
#include <iomanip>
#include <math.h>


#include "Structure.h"

#define G 9.81

void substractG(double matrice[3][3], double* accel_x, double* accel_y, double* accel_z);

class Mobile {

private :
	repere_angle orientation ;
	repere_distance position_absolue ;
	repere_distance position_relative;
	vitesse_rotation v_rot;
	vitesse_translation v_tr;
	clock_t t_acquisition;
    void norm_Angle(double alpha);
	double vecteur_orientation;

public:
	Mobile(void);
	~Mobile();
	//getter//
	double* get_VectOrient() ;
	void get_angles(double vAngle_phi, double vAngle_teta, double vAngle_psi);
	//setter//
	void set_vitesse (double v_x, double v_y, double v_z);
	void set_VectOrient(double* val) ;

	void maj_orientation(double phi, double teta, double psi);
	void maj_position(double x, double y, double z);

	void afficher_position(void);
	void afficher_vitesse(void);
	void afficher_angle (void);
	

	
};

#endif

