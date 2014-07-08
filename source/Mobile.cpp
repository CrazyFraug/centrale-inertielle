#include "Mobile.h"
#include <math.h>
#include "cqrlib.h"

using namespace std;

	Mobile::Mobile() {
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

	}

/** Fonction mettre à jour l'attribut accélération
 *  In : acc_x, acc_y, acc_z - les accélérations selon 3 axes
 *  Remarques : mettre à jour le temps de l'acquisition
 */


void Mobile::set_Acceleration (double acc_x, double acc_y, double acc_z){
		acc_trans.accel_x = acc_x;
		acc_trans.accel_y = acc_y;
		acc_trans.accel_z = acc_z;
		t_pred = t_act;
		t_act = clock();
}

void Mobile::norm_Angle(double alpha){
        if (alpha > 360){
            alpha -=360;
	    }else if (alpha< -360){
            alpha +=360;
	    }
}

/*
 * Fonction mettra à jour les angles d'orientation
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

void Mobile :: set_vitesse (double v_x, double v_y, double v_z){
        v_tr.v_x = v_x;
        v_tr.v_y = v_y;
        v_tr.v_z = v_z;
}


/*
 * Fonction calcule la vitesse à partir de l'accélération
 * In : accel_translation - accélération de translation
        dt                - la différence de temps entre 2 acquisitions
 * Remarque : la vitesse est calculé en cm/s (*100)  - 05/07/2014
 */
void Mobile::calcul_vitesse(accel_translation translation, double dt) {
		//float dt = (clock() - t_position)/CLOCKS_PER_SEC;
		v_tr.v_x += translation.accel_x*dt*100/CLOCKS_PER_SEC;
		v_tr.v_y += translation.accel_y*dt*100/CLOCKS_PER_SEC;
		v_tr.v_z += translation.accel_z*dt*100/CLOCKS_PER_SEC;
		/*v_tr.v_x /= 1000;
		v_tr.v_y /= 1000;
		v_tr.v_z /= 1000;*/
		//cout << "dt ---> " << dt << endl;
	}

	void Mobile::afficher_mobile()
	{
		//cout.width(4);
		//cout.fill('0');
		cout << "angles : " << endl ;
		cout << "phi  = " << orientation.phi << endl ;
		cout << "teta = " << orientation.teta << endl ;
		cout << "psi  = " << orientation.psi << endl ;

	}


/*
 * Fonction calcule le changement du repère absolu avec une rotation
 * In : teta_pitch, teta_roll, teta_yaw - les angles reçus du gyroscope
 * Remarque : Erreur d'algorithme -> Refaire des calculs!!!!! - 14h21 03/07/2014
 *            Passage en Quaternion -> Revoir la modif du repère - 05/07/2014
*/
void Mobile::chgt_repere_rotation(double teta_pitch, double teta_roll, double teta_yaw)
{
    CQRQuaternionHandle quat_rotation;
    CQRCreateEmptyQuaternion(&quat_rotation);
    if (CQRAngles2Quaternion(quat_rotation, teta_pitch,teta_roll,teta_yaw) == 0){
        double result[3] = {0,0,0};
        double vect [3] = {position_absolue.x, position_absolue.y, position_absolue.z};
        CQRRotateByQuaternion(result,quat_rotation,vect);
        //maj_position(result[0], result[1], result[2]);
        //return 0;
    }else {
        //return 1;
    }
    /*if (teta_pitch!=0){
        //cout << "Il y a une rotation selon l'axe Ox" << endl;
        position.y *= cos(teta_pitch);
        position.z *= cos(teta_pitch);
    }
    if (teta_roll!=0){
        //cout << "Il y a une rotation selon l'axe Oy" << endl;
        position.x *= cos(teta_roll);
        position.z *= cos(teta_roll);
    }
    if (teta_yaw!=0){
        //cout << "Il y a une rotation selon l'axe Oz" << endl;
        position.x *= cos(teta_yaw);
        position.y *= cos(teta_yaw);
    }*/
}

/*
 * Fonction calcule le changement du repère absolu avec une translation
 * In : acc_x, acc_y, acc_z - les accélérations reçus de l'accéléromètre
 * Remarques : Erreur accéléromètre -> erreur de calcul -> Filtre de Kalman à calculer - 14h22 03/07/2014
 *             Redéfinir le dt (temps entre deux acquisitions d'accélération) - 05/07/2014
*/
void Mobile::chgt_repere_translation(float acc_x, float acc_y, float acc_z){
    double dt = t_act - t_pred/CLOCKS_PER_SEC;
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
		cout << "t_position :" << t_position << endl;
	}
