#ifndef DEF_MOBILE
#define DEF_MOBILE

#include <Windows.h>
#include <time.h>
#include <iostream>
#include <iomanip>

struct accel_translation {
	float accel_x;
	float accel_y;
	float accel_z;
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
	float phi;
	float teta;
	float psi;
};

struct repere_distance {
	double x;
	double y;
	double z;
};


class Mobile {

private :
	repere_angle orientation ;
	repere_distance position ;
	vitesse_rotation v_rot;
	vitesse_translation v_tr;
	clock_t t_orientation, t_position, t_v_rot, t_v_tr;

public:
	Mobile();
	void maj_position(double x, double y, double z);
	void maj_orientation(double phi, double teta, double psi);
	void calcul_vitesse(accel_translation translation, float dt);
	void chgt_repere_rotation(float teta_pitch, float teta_roll, float teta_yaw);
	void chgt_repere_translation(float acc_x, float acc_y, float acc_z);
	//void calcul_v_rot(accel_rotation rotation);
	void afficher_mobile(void);
	void afficher_position(void);

};

#endif
