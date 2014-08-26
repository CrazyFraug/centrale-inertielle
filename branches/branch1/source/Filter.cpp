#include "Quaternion.h"
#include "Mobile.h"

/*	\brief	Orientation from angular rate
	\param	quaternion<double>	qPred		Quaternion step k-1
	\param	vect4D				vAngulaire	Vitesse angulaire mesurée par gyroscope (en rad/s)
	\param	double				dt			Période échantillonnage
	\return	quaternion<double>	qResult		Quaternion from angular rate step k
 */
quaternion<double> filterAngularRate(quaternion<double> qPred, vect4D vAngulaire, double dt){
	/* Déclaration des variables utilisées */
	double wx, wy, wz;
	quaternion<double> qDot, qResult;
	wx = vAngulaire.x;
	wy = vAngulaire.y;
	wz = vAngulaire.z;
	quaternion<double> rotation(0, wx, wy, wz);
	
	/* Calcul */
	qDot = 0.5*hamiltonProduct(qPred, rotation);
	qResult = qPred + qDot*dt;
	return qResult;
}

/*	\brief	Orientation from vector observations 
	\param	quaternion<double>	qPred	Quaternion step k-1
	\param	vect4D				fSensor	Field in sensor frame
	\param	vect4D				fEarth	Field in earth frame
	\param	double				muy		Pas de descent du gradient
	\return	quaternion<double>	qResult	Quaternion from vector observations step k
*/
quaternion<double> filterObservations(quaternion<double> qPred, vect4D fSensor, vect4D fEarth, double muy)
{
	/* Déclaration des variables utilisées */
	quaternion<double> f, aux;
	double _qw, _qx, _qy, _qz;
	double _qw2, _qx2, _qy2, _qz2;
	double _dx, _dy, _dz;
	double _sx, _sy, _sz;
	double _j00, _j01, _j02, _j03;
	double _j10, _j11, _j12, _j13;
	double _j20, _j21, _j22, _j23;
	double _j30, _j31, _j32, _j33;
	double _f1, _f2, _f3, _f4;
	double _qR1, _qR2, _qR3, _qR4;
	double _gradf1, _gradf2, _gradf3, _gradf4;
	double normGradf;
	quaternion<double> qSensor(0, _dx, _dy, _dz);
	quaternion<double> qEarth(0, _sx, _sy, _sz);
	_qw = qPred.R_component_1();
	_qx = qPred.R_component_2();
	_qy = qPred.R_component_3();
	_qz = qPred.R_component_4();
	_qw2 = pow(_qw, 2);
	_qx2 = pow(_qx, 2);
	_qy2 = pow(_qy, 2);
	_qz2 = pow(_qz, 2);

	/* Calcul */
	aux = hamiltonProduct(conj(qPred), qEarth);
	aux = hamiltonProduct(aux, qPred);
	f = aux - qSensor;
	_f1 = f.R_component_1();
	_f2 = f.R_component_2();
	_f3 = f.R_component_3();
	_f4 = f.R_component_4();
	/* Jacobien de f */
	_j00 = _j01 = _j02 = _j03 = 0;
	_j10 = 2 * _dy*_qz;
	_j11 = 2 * _dy*_qy + 2 * _dz*_qz;
	_j12 = -4 * _dx*_qy + 2 * _dy*_qx - 2 * _dz*_qw;
	_j13 = -4 * _dx*_qz + 2 * _dy*_qw + 2 * _dz*_qx;
	_j20 = -2 * _dx*_qz + 2 * _dz*_qx;
	_j21 = 2 * _dx*_qy - 4 * _dy*_qx + 2 * _dz*_qw;
	_j22 = 2 * _dx*_qx + 2 * _dz*_qz;
	_j23 = -2 * _dx*_qw - 4 * _dy*_qz + 2 * _dz*_qy;
	_j30 = 2 * _dx*_qy - 2 * _dy*_qx;
	_j31 = 2 * _dx*_qz - 2 * _dy*_qw - 4 * _dz*_qx;
	_j32 = 2 * _dx*_qw + 2 * _dy*_qz - 4 * _dz*_qy;
	_j33 = 2 * _dx*_qx + 2 * _dy*_qy;

	_gradf1 = 0;
	_gradf2 = _j10*_f1 + _j11*_f2 + _j12*_f3 + _j13*_f4;
	_gradf3 = _j20*_f1 + _j21*_f2 + _j22*_f3 + _j23*_f4;
	_gradf4 = _j30*_f1 + _j31*_f2 + _j32*_f3 + _j33*_f4;
	normGradf = sqrt(pow(_gradf1, 2) + pow(_gradf2, 2) + pow(_gradf3, 2) + pow(_gradf4, 2));

	_qR1 = _qw - muy*(_gradf1 / normGradf);
	_qR2 = _qx - muy*(_gradf2 / normGradf);
	_qR3 = _qy - muy*(_gradf3 / normGradf);
	_qR4 = _qz - muy*(_gradf4 / normGradf);
	quaternion<double> qResult(_qR1, _qR2, _qR3, _qR4);
	return qResult;
}

