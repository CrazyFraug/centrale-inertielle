#include "Mobile.h"
#include <math.h>

using namespace std;

	Mobile::Mobile() {

		double tmp[3] = {-1,0,0}; //sert à initialiser le tableau dynamique
		position_absolue.x = 0;
		position_absolue.y = 0;
		position_absolue.z = 0;

        position_relative.x = 0;
		position_relative.y = 0;
		position_relative.z = 0;

		v_tr.v_x=0;
		v_tr.v_y=0;
		v_tr.v_z=0;

        t_acquisition = 0;
		t_pred = clock();
		t_act = clock();

		vecteur_orientation = new double[3];
		vecteur_orientation = tmp;

	}

	Mobile::~Mobile() {
		delete[] vecteur_orientation;
	}

	double* Mobile::get_VectOrient() {return vecteur_orientation;}

	void Mobile::set_VectOrient(double* val) {vecteur_orientation = val;}

/** Fonction mettre à jour l'attribut accélération
 *  In : acc_x, acc_y, acc_z - les accélérations selon 3 axes
 *  Remarques : mettre à jour le temps de l'acquisition
 */

void Mobile::set_Acceleration (double acc_x, double acc_y, double acc_z){
		t_pred = t_act;
		acc_trans.accel_x = acc_x;
		acc_trans.accel_y = acc_y;
		acc_trans.accel_z = acc_z;
        Sleep(20);
		t_act = clock();
}

void Mobile::norm_Angle(double alpha){
        if (alpha > 180){
            alpha -= 360;
	    }else if (alpha< -180){
            alpha += 360;
	    }
}


/** Fonction mettra à jour les angles d'orientation
 * In : phi,teta,psi - les angles reçus du gyroscope
 * Remarque : Ajout la normalisation - 05/07/2014
*/
	void Mobile::maj_orientation(double phi, double teta, double psi) {
	    /* Normaliser les angles entre -360;360 */
	    norm_Angle(orientation.phi);
	    norm_Angle(orientation.teta);
	    norm_Angle(orientation.psi);
		orientation.phi = phi;
		orientation.teta = teta;
		orientation.psi = psi;
	}


CQRQuaternionHandle* Mobile::calculerOrientation(double teta_pitch, double teta_roll, double teta_yaw, double matrice[3][3])
{
	CQRQuaternionHandle* quat_rotation;
	CQRCreateEmptyQuaternion(quat_rotation);
	if (CQRAngles2Quaternion(*quat_rotation, teta_pitch,teta_roll,teta_yaw) == CQR_SUCCESS) {
		CQRQuaternion2Matrix (matrice, *quat_rotation);
	}
	return quat_rotation;
}


/** Fonction calcule le changement de coordonnées du vecteur à partir d'un quat rotation
 * In : CQRQuaternionHandle quat_rotation, double* w, double* v -> v le vecteur d'origine, et w le vecteur après rotation
 * Out: 0 if succcess
 * Remarque : 
*/

int Mobile::rotate_vector(CQRQuaternionHandle quat_rotation, double* w, double* v)
{
        return CQRRotateByQuaternion(w,quat_rotation,v);
}


/** Soustraction de l'influence de l'accélération g sur chacun des axes de l'accéléromètre*/

void substractG(double matrice[3][3], double* accel_x, double* accel_y, double* accel_z)
{
	(*accel_x) = G*matrice[0][2];
	(*accel_y) = G*matrice[1][2];
	(*accel_z) = G*matrice[2][2];
}

void Mobile :: set_vitesse (double v_x, double v_y, double v_z){
        v_tr.v_x = v_x;
        v_tr.v_y = v_y;
        v_tr.v_z = v_z;
}


/** Fonction calcule la vitesse à partir de l'accélération
 * In : accel_translation - accélération de translation
        dt                - la différence de temps entre 2 acquisitions
 * Remarque : la vitesse est calculé en cm/s (*100)  - 05/07/2014
 */
void Mobile::calcul_vitesse(accel_translation translation, double dt) {
		v_tr.v_x += translation.accel_x*dt*100/CLOCKS_PER_SEC;
		v_tr.v_y += translation.accel_y*dt*100/CLOCKS_PER_SEC;
		v_tr.v_z += translation.accel_z*dt*100/CLOCKS_PER_SEC;
	}

	void Mobile::afficher_mobile()
	{
		cout << "angles : " << endl ;
		cout << "phi  = " << orientation.phi << endl ;
		cout << "teta = " << orientation.teta << endl ;
		cout << "psi  = " << orientation.psi << endl ;
	}


/**
 *  Fonction pour récupérer plusieurs valeur d'accélération afin d'avoir une meilleur précision
 *
*/
void Mobile::get_Acceleration(double acc_x, double acc_y, double acc_z, int i){
    acc_x_stock[i] = acc_x;
    acc_y_stock[i] = acc_y;
    acc_z_stock[i] = acc_z;
}

/**
 * Fonction pour trouver la moyenne des valeurs d'un tableau de valeur
 * À généraliser!!!!!!!!!!!!!!!!!
 */
double Mobile::best_Value_x (void){
    double mean_value;
    double sum_table = 0;
    for (int i = 0; i <= 4 ; i++){
        sum_table += acc_x_stock[i];
    }
    mean_value = sum_table/4;
    return mean_value;
}
double Mobile::best_Value_y (void){
    double mean_value;
    double sum_table = 0;
    for (int i = 0; i <= 4 ; i++){
        sum_table += acc_y_stock[i];
    }
    mean_value = sum_table/4;
    return mean_value;
}

double Mobile::best_Value_z (void){
    double mean_value;
    double sum_table = 0;
    for (int i = 0; i <= 4 ; i++){
        sum_table += acc_z_stock[i];
    }
    mean_value = sum_table/4;
    return mean_value;
}
/** Fonction calcule le changement du repère absolu avec une translation
 * In : acc_x, acc_y, acc_z - les accélérations reçus de l'accéléromètre
 * Remarques : Erreur accéléromètre -> erreur de calcul -> Filtre de Kalman à calculer - 14h22 03/07/2014
 *             Redéfinir le dt (temps entre deux acquisitions d'accélération) - 05/07/2014
*/
void Mobile::chgt_repere_translation(double acc_x, double acc_y, double acc_z){
    double dt = (t_act - t_pred);
    t_acquisition = dt;
    set_Acceleration(acc_x,acc_y,acc_z);
    calcul_vitesse(acc_trans,dt);
    //cout << "dt : " << dt << endl;
    position_absolue.x += v_tr.v_x*dt;
    position_absolue.y += v_tr.v_y*dt;
    position_absolue.z += v_tr.v_z*dt;
}

void Mobile::afficher_position()
	{
		//cout.width(4);
		//cout.fill('0');
		cout << "Position : " << endl ;
		cout << "X  = " << position_absolue.x << endl ;
		cout << "Y = " << position_absolue.y<< endl ;
		cout << "Z  = " << position_absolue.z << endl ;
	}

void Mobile::afficher_vitesse()
	{
		cout << "Vitesse : " << endl ;
		cout << "Vitesse X  = " << v_tr.v_x<< endl ;
		cout << "Vitesse Y = " << v_tr.v_y<< endl ;
		cout << "Vitesse Z  = " << v_tr.v_z<< endl ;
		cout << "t_acquisition :" << t_acquisition << endl;
		cout << "t_pred :" << t_pred<< endl;
		cout << "t_actuel :" << t_act << endl;
	}

void Mobile::afficher_angle(void) {
	cout << "phi : " << orientation.phi << endl;
	cout << "teta : " << orientation.teta << endl;
	cout << "psi : " << orientation.psi << endl;
}
