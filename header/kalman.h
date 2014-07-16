#ifndef KALMAN_H
#define KALMAN_H
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace boost::numeric::ublas;

typedef boost::numeric::ublas::matrix<double> matrix_type;
typedef boost::numeric::ublas::identity_matrix<double> identity_type;

matrix_type get_matrix(int sRow, int sColumn){
        matrix_type result(sRow,sColumn,0);
        return result;
}

identity_type get_ident(int sMat){
        identity_type result(sMat);
        return result;
}

matrix_type copy_matrix(matrix_type mat_source){
        matrix_type mat_dest(mat_source);
        return mat_dest;
}

typedef struct _systemstate{
        int size_state;
        int size_out;
        int size_in;
        matrix_type mat_transition = get_matrix(size_state,size_state);  //matrice de transition (état k+1 - état k)                    (A/F)
        matrix_type mat_cmde = get_matrix(size_in,size_state);   //matrice de commande (état - commade)                                 (B)
        matrix_type mat_sortie = get_matrix(size_out,size_state); //matrice de sortie  (sortie - état)                                  (C/H)
} system_state;

typedef struct _kalmanstate{

        system_state filtre_sys;    // système à filtrer
        identity_type  matrix_ident = get_ident(filtre_sys.size_state);
        int    nb_step;           // nombre d'échantillon d'estimation
        matrix_type noise_cmde = get_matrix(1,filtre_sys.size_in);  //bruit de la commande                                              (v)
        matrix_type cov_cmde = get_matrix(filtre_sys.size_in,filtre_sys.size_state);        // covariance de la commande                (Q)
        matrix_type noise_mesure = get_matrix(1, filtre_sys.size_out);  // bruit de la mesure                                           (w)
        matrix_type cov_mesure = get_matrix(filtre_sys.size_out, filtre_sys.size_state);      //covariance de la mesure                 (R)
        matrix_type cov_estimate = get_matrix(filtre_sys.size_out, filtre_sys.size_state);    // prédiction de la covariance            (P)
        matrix_type predict_vector = get_matrix(1,filtre_sys.size_state); //état prédit                                                 (X^)
        matrix_type kalman_gain = get_matrix(filtre_sys.size_state,filtre_sys.size_state);     // gain de Kalman                        (K)

} kalmanstate;


class kalman
{
    public:
        kalman(int nb_in, int nb_out, int nb_state);
        void declare_system(matrix_type A, matrix_type B, matrix_type C);
        void declare_noise(matrix_type Q, matrix_type R);
        void predict_step(matrix_type value_cmd);
        void update_step(matrix_type value_mesure);
        virtual ~kalman();
    protected:
    private:
        system_state sys;
        kalmanstate kalm_sys;

};

#endif // KALMAN_H
