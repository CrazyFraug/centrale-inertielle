#ifndef KALMAN_H
#define KALMAN_H
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "matrix_type.h"
using namespace boost::numeric::ublas;

/*typedef boost::numeric::ublas::matrix<double> matrix_type;
typedef boost::numeric::ublas::identity_matrix<double> identity_type;*/


typedef struct _systemstate{
        int size_state;
        int size_out;
        int size_in;
        matrix_type mat_transition;  //matrice de transition (état k+1 - état k)            (A/F)
        matrix_type mat_cmde;   //matrice de commande (état - commade)                      (B)
        matrix_type mat_sortie; //matrice de sortie  (sortie - état)                        (C/H)
} system_state;

typedef struct _kalmanstate{

        system_state filtre_sys;    // système à filtrer
        matrix_type matrix_ident ;
        int    nb_step;           // nombre d'échantillon d'estimation
        matrix_type noise_cmde ;  //bruit de la commande                                    (v)
        matrix_type cov_cmde ;        // covariance de la commande                          (Q)
        matrix_type noise_mesure ;  // bruit de la mesure                                   (w)
        matrix_type cov_mesure ;      //covariance de la mesure                             (R)
        matrix_type cov_estimate ;    // prédiction de la covariance                        (P)
        matrix_type predict_vector ; //état prédit                                          (X^)
        matrix_type kalman_gain ;     // gain de Kalman                                     (K)

} kalmanstate;


class kalman
{
    public:
        kalman(int nb_in, int nb_out, int nb_state, int step,matrix_double init_cov_estimate);
        void declare_system(matrix_double A, matrix_double B, matrix_double C);
        void declare_noise(matrix_double Q, matrix_double R);
        void predict_step(matrix_double value_cmd);
        matrix_type update_step(matrix_double value_mesure);
        virtual ~kalman();
    protected:
    private:
        system_state sys;
        kalmanstate kalm_sys;

};

#endif // KALMAN_H
