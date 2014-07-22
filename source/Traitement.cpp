#include "Traitement.h"


/**Constructeur**/
/*Traitement::Traitement(matrix_double init_cov_estimate):compteur(0), filtre(0,4,4,100, init_cov_estimate), A(4,4,0), B(0,0,0), C(4,4,0), Q(4,4), R(4,4,0)
{
        /** Allocation mémoire de la matrice de valeurs **/
        /*for (int i = 0; i<3; i++)
        {
                valeurs[i] = new double[nbValeurs];
        }
}*/

/**Constructeur**/
Traitement::Traitement(Instrument* inst):_compteur(0)
{
		_capteur = inst;
        /** Allocation mémoire de la matrice de valeurs **/
        for (int i = 0; i<3; i++)
        {
                _valeurs[i] = new double[NB_VALEURS];
        }
}


/** Destructeur **/
Traitement::~Traitement()
{
        for (int i = 0; i<3; i++)
        {
                delete _valeurs[i];
        }
}


/**
 *  Initialisation des matrices pour le filtre kalman
 *  \param      sx double   vitesse angulaire selon l'axe Ax
 *  \param      sy double   vitesse angulaire selon l'axe Ay
 *  \param      sz double   vitesse angulaire selon l'axe Az
 *
*/
/*void Traitement::initMatrices(double sx, double sy, double sz)
{
        double sx2, sy2, sz2;
        sx2 = pow(sx,2);
        sy2 = pow(sy,2);
        sz2 = pow(sz,2);

        Q(0,0) = sx2 + sy2 + sz2; Q(1,1) = sx2 + sy2 + sz2; Q(2,2) = sx2 + sy2 + sz2; Q(3,3) = sx2 + sy2 + sz2; //diagonale
        Q(0,1) = -sx2 + sy2 - sz2; Q(0,2) = -sx2 -sy2 + sz2; Q(0,3) = sx2 - sy2 - sz2;
        Q(1,0) = -sx2 + sy2 - sz2; Q(1,2) = sx2 - sy2 - sz2; Q(1,3) = -sx2 - sy2 + sz2;
        Q(2,0) = -sx2 -sy2 + sz2; Q(2,1) = sx2 - sy2 - sz2; Q(2,3) = -sx2 + sy2 - sz2;
        Q(3,0) = sx2 - sy2 - sz2; Q(3,1) = -sx2 - sy2 + sz2; Q(3,2) = -sx2 + sy2 - sz2;

        R(0,0) = 0.05;
        R(1,1) = 0.05;
        R(2,2) = 0.05;
        R(3,3) = 0.05;


        C(0,0) = 1;
        C(1,1) = 1;
        C(2,2) = 1;
        C(3,3) = 1;     

}*/


/**
 *  Traitement avec le filtre Kalman
 *  \param      mesures[3]   vecteur de 3 mesures de vitesse angulaire selon l'axe Ax, Ay, Az
 *  \return measure_estimate vecteur de 3 angles de rotation selon l'axe Ax, Ay, Az estimés
 *
*/

/*double* Traitement::initSystem(double mesures[3], double dt) {

        double* estimation;
        double anglex, angley, anglez, wx,wy,wz, Ax, Ay, Az;
        CQRQuaternionHandle quat_meas, quat_estimate;

        estimation = new double[3];

        CQRCreateEmptyQuaternion(&quat_meas);
        CQRCreateEmptyQuaternion(&quat_estimate);

        wx = mesures[0];
        wy = mesures[1];
        wz = mesures[2];*/
        /* Mettre à jour la matrice de transition A */
        /*Ax = 0.5*wx*dt;
        Ay = 0.5*wy*dt;
        Az = 0.5*wz*dt;

        A(0,0)=1;
        A(0,0) = 1; A(1,1) = 1; A(2,2) = 1; A(3,3) = 1; //diagonale
        A(0,1) = -Ax; A(0,2) = -Ay; A(0,3) = -Az;
        A(1,0) = Ax; A(1,2) = Az; A(1,3) = -Ay;
        A(2,0) = Ay; A(2,1) = -Az; A(2,3) = Ax;
        A(3,0) = Az; A(3,1) = Ay; A(3,2) = -Ax;*/

        /* Mettre à jour le système à filtrer */
        /*filtre.declare_system(A, B, C);
        filtre.declare_noise(Q, R);*/

        /* Etape prédiction du filtre Kalman */
        //filtre.predict_step(B);


        /* Convertir les données mesurées(angles) en quaternion (type matrix) */
        /*anglex = wx*dt;
        angley = wy*dt;
        anglez = wz*dt;

        CQRAngles2Quaternion (quat_meas, anglex, angley, anglez );

        matrix<double> mesuresQuat(4,1);

        mesuresQuat(0,0) = quat_meas->w;
        mesuresQuat(1,0) = quat_meas->x;
        mesuresQuat(2,0) = quat_meas->y;
        mesuresQuat(3,0) = quat_meas->z;

        matrix<double> estimate_result(4,1,0);*/

        /* Etape update l'estimation */
/*		estimate_result = filtre.update_step(mesuresQuat);

        quat_estimate->w = estimate_result(0,0);
        quat_estimate->x = estimate_result(1,0);
        quat_estimate->y = estimate_result(2,0);
        quat_estimate->z = estimate_result(3,0);*/

        /* Convertir les données estimées (quaternion) en angles pour l'affichage */
        /*matrix<double> measure_estimate(3,1);
        CQRQuaternion2Angles(&measure_estimate(0,1), &measure_estimate(1,1), &measure_estimate(2,1), quat_estimate);

        return estimation;
}*/


/** 
 *      \brief stocke des valeurs passées en argument dans la matrice de valeur attribut
 *      la variable compteur permet de savoir si la matrice est remplie ou pas
 *      si la matrice est deja remplie, elle se contente de remplacer la premiere valeur de chaque ligne avec les valeurs en argument
 *      \param val vect3D défini dans structure.h contenant les mesures a stocker
*/
void Traitement::stockerValeurs() 
{
		_capteur->majSerial();

        if (_compteur < NB_VALEURS)
        {
                _valeurs[0][_compteur] = _capteur->getMesure(1);
                _valeurs[1][_compteur] = _capteur->getMesure(2);
                _valeurs[2][_compteur] = _capteur->getMesure(3);
                _compteur++;
        }

        else
        {
                for(int i = NB_VALEURS-1; i>0; i--)
                {
                        _valeurs[0][i] = _valeurs[0][i-1];
                        _valeurs[1][i] = _valeurs[1][i-1];
                        _valeurs[2][i] = _valeurs[2][i-1];
                }

                _valeurs[0][0] = _capteur->getMesure(1);
                _valeurs[1][0] = _capteur->getMesure(2);
                _valeurs[2][0] = _capteur->getMesure(3);

				_t[0] = _capteur->getTemps(1);
				_t[1] = _capteur->getTemps(2);
				_t[2] = _capteur->getTemps(3);
        }
}

/**
 *  \brief realise la moyenne des valeurs d'une ligne de la matrice
 *  \param axe numero de la ligne que l'on veut moyenner
 *  \return la moyenne d'une ligne
*/
double Traitement::moyenner(int axe)
{
        double _moyenne = 0;
        for (int i =0; i < _compteur; i++)
        {
                _moyenne += _valeurs[axe-1][i];
        }

        return (_moyenne/NB_VALEURS);
}

vect3D Traitement::calculerAngle()
{
	vect3D angles;
	/*angles.x = moyenner(1)*(clock()-t[0])/1000;
	angles.y = moyenner(2)*(clock()-t[1])/1000;
	angles.z = moyenner(3)*(clock()-t[2])/1000;*/
	angles.x = moyenner(1)*(20)/1000;
	angles.y = moyenner(2)*(20)/1000;
	angles.z = moyenner(3)*(20)/1000;
	std::cout <<  "clock = " << clock() << std::endl;
	std::cout << "| t[0] = " << _t[0] << "| dt = " << (clock()-_t[0])/1000 << std:: endl;
	std::cout << "| t[1] = " << _t[1] << "| dt = " << (clock()-_t[1])/1000 << std:: endl;
	std::cout << "| t[2] = " << _t[2] << "| dt = " << (clock()-_t[2])/1000 << std:: endl;
	return angles;
}


void Traitement::afficherValeurs()
{
	if (_compteur == NB_VALEURS) {
		std::cout << "moyenne X = " << moyenner(1) << std::endl;
		std::cout << "moyenne Y = " << moyenner(2) << std::endl;
		std::cout << "moyenne Z = " << moyenner(3) << std::endl;
	}
}

bool Traitement::tabFull()
{
	if (_compteur == NB_VALEURS)
		return true;
	else
		return false;
}


/**
\brief fonction inutile
*/
void Traitement::testd (void){
        std::cout << "test" << std::endl;
}