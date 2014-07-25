#ifndef KALMAN_H
#define KALMAN_H
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "Quaternion.h"

using namespace boost::numeric::ublas;

/** struct représentant l'etat du systeme 
* composé de 3 matrice ainsi que des dimensions de la représentation d'état et des entrée sorties
*/
typedef struct _systemstate{
	int size_state;
	int size_out;
	int size_in;
	matrix<double> mat_transition;  //matrice de transition (état k+1 - état k)            (A/F)
	matrix<double> mat_cmde;   //matrice de commande (état - commade)                      (B)
	matrix<double> mat_sortie; //matrice de sortie  (sortie - état)                        (C/H)
} system_state;

typedef struct _kalmanstate{

	matrix<double> matrix_ident;
	int    nb_step;           // nombre d'échantillon d'estimation
	matrix<double> noise_cmde;  //bruit de la commande                                    (v)
	matrix<double> cov_cmde;        // covariance de la commande                          (Q)
	matrix<double> noise_mesure;  // bruit de la mesure                                   (w)
	matrix<double> cov_mesure;      //covariance de la mesure                             (R)
	boost::numeric::ublas::matrix<double> cov_estimate;    // prédiction de la covariance	(P)
	matrix<double> predict_vector; //état prédit                                          (X^)
	matrix<double> kalman_gain;     // gain de Kalman                                     (K)

} kalmanstate;


class kalman
{
public:
	kalman(int nb_in, int nb_out, int nb_state, int step, matrix<double> init_cov_estimate);
	~kalman();

	void declare_system(matrix<double> A, matrix<double> B, matrix<double> C);
	void declare_noise(matrix<double> Q, matrix<double> R);
	void predict_step(matrix<double> value_cmd);
	matrix<double> update_step(matrix<double> value_mesure);

    void initMatrices(double sigma1, double sigma2, double sigma3);
    double* initSystem(double mesures[3], double dt);


private:
	system_state sys;
	kalmanstate kalm_sys;

};

#endif // KALMAN_H