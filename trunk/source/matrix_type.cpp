#include "matrix_type.h"

matrix_type::matrix_type(){
        mat(0,0);
}

void matrix_type::get_matrix(int sRow, int sColumn){
        mat(sRow,sColumn,0);
}

void matrix_type::get_ident(int sMat, double value_ident){
        mat(sMat,sMat,0);
        for(int i = 0; i < sMat; i++)
        mat(i,i) = value_ident;
}

void matrix_type::copy_matrix(matrix_double mat_source){
        mat(mat_source);
}

matrix_double matrix_type::sum_matrix(matrix_double matA, matrix_double matB){
        return matA + matB;

}
