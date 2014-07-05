#ifndef DEF_MOBILE
#define DEF_MOBILE

#include <Windows.h>
#include <time.h>
#include <iostream>
#include <iomanip>
#include "repere.h"

struct accel_translation {
	double accel_x;
	double accel_y;
	double accel_z;
};

struct accel_rotation {
	float accel_phi;
	float accel_teta;
	float accel_psi;
};


struct vitesse_translation {
	double v_x;
	double v_y;
	double v_z;
};

struct vitesse_rotation {
	float v_phi;
	float v_teta;
	float v_psi;
};


struct repere_angle {
	double phi;
	double teta;
	double psi;
};


struct repere_distance {
	double x;
	double y;
	double z;
};


class Mobile {

private :
    accel_translation acc_trans;
	repere_angle orientation ;
	repere_distance position_absolue ;
	repere_distance position_relative;
	vitesse_rotation v_rot;
	vitesse_translation v_tr;
	clock_t t_acquisition, t_pred, t_act, t_orientation, t_position, t_v_rot, t_v_tr;
    void norm_Angle(double alpha);
public:
	Mobile();
	void set_Acceleration (double acc_x, double acc_y, double acc_z);
	void maj_orientation(double phi, double teta, double psi);
	void set_vitesse (double v_x, double v_y, double v_z);
	void calcul_vitesse(accel_translation translation, double dt);
	void chgt_repere_rotation(double teta_pitch, double teta_roll, double teta_yaw);
	void chgt_repere_translation(double acc_x, double acc_y, double acc_z);
	//void calcul_v_rot(accel_rotation rotation);
	void afficher_mobile(void);
	void afficher_position(void);
	void afficher_vitesse(void);

};

#endif
