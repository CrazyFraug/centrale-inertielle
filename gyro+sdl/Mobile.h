#ifndef DEF_MOBILE
#define DEF_MOBILE

#include <Windows.h>
#include <time.h>
#include <iostream>
#include <iomanip>
#include <math.h>

#include "cqrlib.h"
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
	clock_t t_acquisition, t_acqGyro, t_pred, t_act, t_orientation, t_position, t_v_rot, t_v_tr;
    void norm_Angle(double alpha);
	double* vecteur_orientation;

public:
	Mobile(void);
	~Mobile();
	//void set_Acceleration (double acc_x, double acc_y, double acc_z);
	void get_Acceleration(double acc_x, double acc_y, double acc_z, int i);
	void maj_orientation(double phi, double teta, double psi);
	void set_vitesse (double v_x, double v_y, double v_z);
	void calcul_vitesse(acc_translation translation, double dt);
	int rotate_vector(CQRQuaternionHandle, double* out, double* in);
	void chgt_repere_translation(double acc_x, double acc_y, double acc_z);
	//void calcul_v_rot(accel_rotation rotation);
	void afficher_mobile(void);
	void afficher_position(void);
	void afficher_vitesse(void);
	CQRQuaternionHandle* calculerOrientation(double teta_pitch, double teta_roll, double teta_yaw, double matrice[3][3]);
	double best_Value_x (void);
	double best_Value_y (void);
	double best_Value_z (void);
	//acc_translation acc_trans;
    void afficher_acc_stock(void);
	double meanValue (int tailleTab, double* mesures);
    double acc_x_stock[4];
    double acc_y_stock[4];
    double acc_z_stock[4];
	void afficher_angle (void);
	double* get_VectOrient() ;
	void set_VectOrient(double* val) ;
	void get_angles(double vAngle_phi, double vAngle_teta, double vAngle_psi);
};

#endif

