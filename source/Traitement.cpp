#include "Traitement.h"


/**Constructeur**/
Traitement::Traitement(matrix_double init_cov_estimate):compteur(0), filtre(0,4,4,100, init_cov_estimate)
{
	/** Allocation mémoire de la matrice de valeurs **/
	for (int i = 0; i<3; i++)
	{
		valeurs[i] = new double[nbValeurs];
	}
}


/** Destructeur **/
Traitement::~Traitement()
{
	for (int i = 0; i<3; i++)
	{
		delete valeurs[i];
	}
}


/**
 *  Initialisation des matrices pour le filtre kalman
 *  \param 	sx double   vitesse angulaire selon l'axe Ax
 *  \param 	sy double   vitesse angulaire selon l'axe Ay
 *  \param 	sz double   vitesse angulaire selon l'axe Az
 *
*/
void initMatrices(double sx, double sy, double sz)
{
	
	double sx2, sy2, sz2;
	sx2 = pow(sx,2);
	sy2 = pow(sy,2);
	sz2 = pow(sz,2);

	A.mat.get_matrix(4,4);
	B.mat.get_matrix(0,0,0);
	Q.mat.get_matrix(4,4);
	R.mat.get_matrix(4,4,0);
	C.mat.get_matrix(4,4,0);

	Q.mat(0,0) = sx2 + sy2 + sz2; Q.mat(1,1) = sx2 + sy2 + sz2; Q.mat(2,2) = sx2 + sy2 + sz2; Q.mat(3,3) = sx2 + sy2 + sz2; //diagonale
	Q.mat(0,1) = -sx2 + sy2 - sz2; Q.mat(0,2) = -sx2 -sy2 + sz2; Q.mat(0,3) = sx2 - sy2 - sz2;
	Q.mat(1,0) = -sx2 + sy2 - sz2; Q.mat(1,2) = sx2 - sy2 - sz2; Q.mat(1,3) = -sx2 - sy2 + sz2;
	Q.mat(2,0) = -sx2 -sy2 + sz2; Q.mat(2,1) = sx2 - sy2 - sz2; Q.mat(2,3) = -sx2 + sy2 - sz2;
	Q.mat(3,0) = sx2 - sy2 - sz2; Q.mat(3,1) = -sx2 - sy2 + sz2; Q.mat(3,2) = -sx2 + sy2 - sz2;

	R.mat(0,0) = 0.05;
	R.mat(1,1) = 0.05;
	R.mat(2,2) = 0.05;
	R.mat(3,3) = 0.05;


	C.mat(0,0) = 1;
	C.mat(1,1) = 1;
	C.mat(2,2) = 1;
	C.mat(3,3) = 1;	A.mat.get_matrix(4,4);
	B.mat.get_matrix(0,0,0);
	Q.mat.get_matrix(4,4);
	R.mat.get_matrix(4,4,0);
	C.mat.get_matrix(4,4,0);

	Q.mat(0,0) = sx2 + sy2 + sz2; Q.mat(1,1) = sx2 + sy2 + sz2; Q.mat(2,2) = sx2 + sy2 + sz2; Q.mat(3,3) = sx2 + sy2 + sz2; //diagonale
	Q.mat(0,1) = -sx2 + sy2 - sz2; Q.mat(0,2) = -sx2 -sy2 + sz2; Q.mat(0,3) = sx2 - sy2 - sz2;
	Q.mat(1,0) = -sx2 + sy2 - sz2; Q.mat(1,2) = sx2 - sy2 - sz2; Q.mat(1,3) = -sx2 - sy2 + sz2;
	Q.mat(2,0) = -sx2 -sy2 + sz2; Q.mat(2,1) = sx2 - sy2 - sz2; Q.mat(2,3) = -sx2 + sy2 - sz2;
	Q.mat(3,0) = sx2 - sy2 - sz2; Q.mat(3,1) = -sx2 - sy2 + sz2; Q.mat(3,2) = -sx2 + sy2 - sz2;

	R.mat(0,0) = 0.05;
	R.mat(1,1) = 0.05;
	R.mat(2,2) = 0.05;
	R.mat(3,3) = 0.05;


	C.mat(0,0) = 1;
	C.mat(1,1) = 1;
	C.mat(2,2) = 1;
	C.mat(3,3) = 1;


}

/** 
 *	\brief stocke des valeurs passées en argument dans la matrice de valeur attribut
 *	la variable compteur permet de savoir si la matrice est remplie ou pas
 *	si la matrice est deja remplie, elle se contente de remplacer la premiere valeur de chaque ligne avec les valeurs en argument
 *	\param val vect3D défini dans structure.h contenant les mesures a stocker
*/
void Traitement::stockerValeurs(vect3D val) 
{
	if (compteur < nbValeurs)
	{
		valeurs[0][compteur] = val.x;
		valeurs[1][compteur] = val.y;
		valeurs[2][compteur] = val.z;
		compteur++;
	}

	else
	{
		for(int i = nbValeurs-1; i>0; i--)
		{
			valeurs[0][i] = valeurs[0][i-1];
			valeurs[1][i] = valeurs[1][i-1];
			valeurs[2][i] = valeurs[2][i-1];
		}

		valeurs[0][0] = val.x;
		valeurs[1][0] = val.y;
		valeurs[2][0] = val.z;
	}
}

/**
 *	\brief realise la moyenne des valeurs d'une ligne de la matrice
 *	\param axe numero de la ligne que l'on veut moyenner
 *	\return la moyenne d'une ligne
*/
double Traitement::moyenner(int axe)
{
	double moyenne;
	for (int i =0; i < nbValeurs; i++)
	{
		moyenne += valeurs[axe][i];
	}

	return (moyenne/nbValeurs);
}


CQRQuaternionHandle* Traitement::calculerOrientation(double pitch, double roll, double yaw, double* matrice[3][3])
{
	CQRQuaternionHandle* quat_rotation;
	quat_rotation = new CQRQuaternionHandle;
	CQRCreateEmptyQuaternion(quat_rotation);
	CQRAngles2Quaternion(*quat_rotation, pitch,roll,yaw); //if enlevé
	return quat_rotation;

}

/** Fonction calcule le changement de coordonnées du vecteur à partir d'un quat rotation
 *	\param [In] : CQRQuaternionHandle quat_rotation, double* w, double* v -> v le vecteur d'origine, et w le vecteur après rotation
 *	\param [Out]: 0 if succcess
 *	Remarque : 
*/

int Traitement::rotate_vector(CQRQuaternionHandle quat_rotation, double* w, double* v)
{
        return CQRRotateByQuaternion(w,quat_rotation,v);
}

/**
\brief fonction inutile
*/
void Traitement::testd (void){
	std::cout << "test" << std::endl;
}


#include "Traitement.h"


/**
 *  Traitement avec le filtre Kalman
 *  \param 	mesures[3]   vecteur de 3 mesures de vitesse angulaire selon l'axe Ax, Ay, Az
 *  \return measure_estimate vecteur de 3 angles de rotation selon l'axe Ax, Ay, Az estimés
 *
*/

matrix_type Traitement::initSystem(double* mesures[3]) {

	double anglex, angley, anglez, wx,wy,wz,dt, Ax, Ay, Az;
	CQRQuaternionHandle *quat_meas, *quat_estimate;
	wx = mesures[0];
	wy = mesures[1];
	wz = mesures[2];
	/* Mettre à jour la matrice de transition A */
	Ax = 0.5*wx*dt;
	Ay = 0.5*wy*dt;
	Az = 0.5*wz*dt;

	A(0,0) = 1; A(1,1) = 1; A(2,2) = 1; A(3,3) = 1; //diagonale
	A(0,1) = -Ax; A(0,2) = -Ay; A(0,3) = -Az;
	A(1,0) = Ax; A(1,2) = Az; A(1,3) = -Ay;
	A(2,0) = Ay; A(2,1) = -Az; A(2,3) = Ax;
	A(3,0) = Az; A(3,1) = Ay; A(3,2) = -Ax;

	/* Mettre à jour le système à filtrer */
	filtre.declare_system(A, B, C);
	filtre.declare_noise(Q, R);

	/* Etape prédiction du filtre Kalman */
	filtre.predict_step(B);


	/* Convertir les données mesurées(angles) en quaternion (type matrix) */
	anglex = wx*dt;
	angley = wy*dt;
	anglez = wz*dt;

	CQRAngles2Quaternion (quat_meas, anglex, angley, anglez );

	matrix_type mesuresQuat;
	mesuresQuat = get_matrix(4,1,0);
	mesuresQuat[0] = quatrotation.w;
	mesuresQuat[1] = quatrotation.x;
	mesuresQuat[2] = quatrotation.y;
	mesuresQuat[3] = quatrotation.z;

	matrix_type estimate_result;

	/* Etape update l'estimation */
	estimate_result = filtre.update_step(mesuresQuat);

	quat_estimate.w = estimate_result.mat(0,0);
	quat_estimate.x = estimate_result.mat(1,0);
	quat_estimate.y = estimate_result.mat(2,0);
	quat_estimate.z = estimate_result.mat(3,0);

	/* Convertir les données estimées (quaternion) en angles pour l'affichage */
	matrix_type measure_estimate;
	measure_estimate.get_matrix(3,1);
	CQRQuaternion2Angles(measure_estimate.mat(0,1), measure_estimate.mat(1,1), measure_estimate.mat(2,1), quat_estimate);

	return measure_estimate;
}