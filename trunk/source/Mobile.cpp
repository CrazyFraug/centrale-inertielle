#include "Mobile.h"
#include <math.h>

using namespace std;

	Mobile::Mobile() {
		position.x = 0;
		position.y = 0;
		position.z = 0;

		v_tr.v_x=0;
		v_tr.v_y=0;
		v_tr.v_z=0;

		t_position = clock();
	}

	void Mobile::maj_position(double x, double y, double z) {
		position.x = x;
		position.y = y;
		position.z = z;

	}

	void Mobile::maj_orientation(double phi, double teta, double psi) {
		orientation.phi = phi;
		orientation.teta = teta;
		orientation.psi = psi;
	}

	void Mobile::calcul_vitesse(accel_translation translation, double dt) {
		//float dt = (clock() - t_position)/CLOCKS_PER_SEC;
		v_tr.v_x += translation.accel_x*dt;
		v_tr.v_y += translation.accel_y*dt;
		v_tr.v_z += translation.accel_z*dt;
		v_tr.v_x /= 1000;
		v_tr.v_y /= 1000;
		v_tr.v_z /= 1000;
		cout << "dt ---> " << dt << endl;
	}


	void Mobile::afficher_mobile()
	{
		//cout.width(4);
		//cout.fill('0');
		cout << "angles : " << endl ;
		cout << "phi  = " << setw(4) << right << (int)orientation.phi << endl ;
		cout << "teta = " << setw(4) << right << (int)orientation.teta << endl ;
		cout << "psi  = " << orientation.psi << endl ;

	}


/*
 * Fonction calcule le changement du repère absolu avec une rotation
 * In : teta_pitch, teta_roll, teta_yaw - les angles reçus du gyroscope
 * Remarque : Erreur d'algorithme -> Refaire des calculs!!!!! - 14h21 03/07/2014
*/
void Mobile::chgt_repere_rotation(float teta_pitch, float teta_roll, float teta_yaw)
{
    if (teta_pitch!=0){
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
    }
}

/*
 * Fonction calcule le changement du repère absolu avec une translation
 * In : acc_x, acc_y, acc_z - les accélérations reçus de l'accéléromètre
 * Remarques : Erreur accéléromètre -> erreur de calcul -> Filtre de Kalman à calculer - 14h22 03/07/2014
*/
void Mobile::chgt_repere_translation(float acc_x, float acc_y, float acc_z){
    double dt = (clock() - t_position);
	t_position = clock();
    accel_translation accelerometre;
    accelerometre.accel_x = acc_x;
    accelerometre.accel_y = acc_y;
    accelerometre.accel_z = acc_z;
    calcul_vitesse(accelerometre,dt);
    position.x += v_tr.v_x*dt;
    position.y += v_tr.v_y*dt;
    position.z += v_tr.v_z*dt;
}

void Mobile::afficher_position()
	{
		//cout.width(4);
		//cout.fill('0');
		cout << "Position : " << endl ;
		cout << "X  = " << position.x << endl ;
		cout << "Y = " << position.y<< endl ;
		cout << "Z  = " << position.z << endl ;

	}

void Mobile::afficher_vitesse()
	{
		cout << "Vitesse : " << endl ;
		cout << "Vitesse X  = " << v_tr.v_x<< endl ;
		cout << "Vitesse Y = " << v_tr.v_y<< endl ;
		cout << "Vitesse Z  = " << v_tr.v_z<< endl ;
		cout << "t_position :" << t_position << endl;

	}