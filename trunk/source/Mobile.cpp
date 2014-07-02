#include "Mobile.h"

using namespace std;

	Mobile::Mobile() {
		orientation.phi = 0;
		orientation.teta = 0;
		orientation.psi = 0;

		position.x = 0;
		position.y = 0;
		position.z = 0;

		v_rot.v_phi=0;
		v_rot.v_teta=0;
		v_rot.v_psi=0;

		v_tr.v_x=0;
		v_tr.v_y=0;
		v_tr.v_z=0;

		t_orientation = clock();
		t_position = clock();
		t_v_rot= clock();
		t_v_tr = clock();
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
	
	void Mobile::calcul_vitesse(accel_translation translation) {
		double dt = (clock() - t_v_tr)/CLOCKS_PER_SEC;
		v_tr.v_x += translation.accel_x*dt;
		v_tr.v_y += translation.accel_y*dt;
		v_tr.v_z += translation.accel_z*dt;

	}
	

	void Mobile::afficher_mobile()
	{
		cout << "angles : " << endl << "phi = " << orientation.phi << endl ;
		cout << "angles : " << endl << "teta = " << orientation.teta<< endl ;
		cout << "angles : " << endl << "psi = " << orientation.psi << endl ;

	}