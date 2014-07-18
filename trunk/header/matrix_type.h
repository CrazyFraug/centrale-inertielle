#ifndef MATRIX_TYPE_H
#define MATRIX_TYPE_H
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace boost::numeric::ublas;
typedef boost::numeric::ublas::matrix<double> matrix_double;

class matrix_type
{
    public:
        matrix_double mat;
        matrix_type();
        void get_matrix(int sRow, int sColumn);
        void get_ident(int sMat, double value_ident);
        void copy_matrix(matrix_double mat_source);
        matrix_double sum_matrix(matrix_double matA, matrix_double matB);
        virtual ~matrix_type();
    protected:
    private:
};

#endif // MATRIX_TYPE_H
