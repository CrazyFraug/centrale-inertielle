#include "Mobile.h"

using namespace std;

	Mobile::Mobile(void) {

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

		acc_x_stock[0] = 0;acc_x_stock[1] = 0;acc_x_stock[2] = 0;acc_x_stock[3] = 0;
		acc_y_stock[0] = 0;acc_y_stock[1] = 0;acc_y_stock[2] = 0;acc_y_stock[3] = 0;
		acc_z_stock[0] = 0;acc_z_stock[1] = 0;acc_z_stock[2] = 0;acc_z_stock[3] = 0;

        t_acquisition = 0;
		t_pred = clock();
		t_act = clock();
		t_acqGyro = 0;

		vecteur_orientation = new double[3];
		vecteur_orientation = tmp;

	}

	Mobile::~Mobile() {
		//delete[] vecteur_orientation;
	}

	double* Mobile::get_VectOrient() {return vecteur_orientation;}

	void Mobile::set_VectOrient(double* val) {vecteur_orientation = val;}


void Mobile::norm_Angle(double alpha){
        if (alpha > 180){
            alpha -= 360;
	    }else if (alpha< -180){
            alpha += 360;
	    }
}

void Mobile::get_angles(double vAngle_phi, double vAngle_teta, double vAngle_psi) {
	double phi,teta,psi;
	double dt = clock() - t_acqGyro;
	t_acqGyro = clock();
	phi = vAngle_phi*dt;
	teta = vAngle_teta*dt;
	psi = vAngle_psi*dt;
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
void Mobile::calcul_vitesse(acc_translation translation, double dt) {
		v_tr.v_x += translation.acc_x*dt*100/CLOCKS_PER_SEC;
		v_tr.v_y += translation.acc_y*dt*100/CLOCKS_PER_SEC;
		v_tr.v_z += translation.acc_z*dt*100/CLOCKS_PER_SEC;
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

void Mobile::afficher_acc_stock(){
    for (int i = 0;i<4;i++){
        cout << "Acc_X : " << acc_x_stock[i]<<endl;
    }
        for (int i = 0;i<4;i++){
        cout << "Acc_Y : " << acc_y_stock[i]<<endl;
    }
        for (int i = 0;i<4;i++){
        cout << "Acc_Z : " << acc_z_stock[i]<<endl;
    }
}



double Mobile::meanValue (int tailleTab, double* mesures)
{
	double sum = 0;
	for (int i = 0; i<tailleTab; i++)
	{
		sum += mesures[i];
	}
	return sum/tailleTab;
}

/**
 * Fonction pour trouver la moyenne des valeurs d'un tableau de valeur
 * À généraliser!!!!!!!!!!!!!!!!!
 */
double Mobile::best_Value_x (void){
    double mean_value;
    double sum_table = 0;
    for (int i = 0; i < 4 ; i++){
        sum_table += acc_x_stock[i];
    }
    mean_value = sum_table/4;
    return mean_value;
}
double Mobile::best_Value_y (void){
    double mean_value;
    double sum_table = 0;
    for (int i = 0; i < 4 ; i++){
        sum_table += acc_y_stock[i];
    }
    mean_value = sum_table/4;
    return mean_value;
}

double Mobile::best_Value_z (void){
    double mean_value;
    double sum_table = 0;
    for (int i = 0; i < 4 ; i++){
        sum_table += acc_z_stock[i];
    }
    mean_value = sum_table/4;
    return mean_value;
}

void Mobile::afficher_position()
{
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
}

void Mobile::afficher_angle(void) 
{
	cout << "angles : " << endl;
	cout << "phi : " << orientation.phi << endl;
	cout << "teta : " << orientation.teta << endl;
	cout << "psi : " << orientation.psi << endl;
}
