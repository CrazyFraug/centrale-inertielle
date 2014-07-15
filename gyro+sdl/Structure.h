#ifndef STRUCTURE_H
#define STRUCTURE_H

struct acc_translation {
	double acc_x;
	double acc_y;
	double acc_z;
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

struct vect3D {
	double x;
	double y;
	double z;
};

#endif