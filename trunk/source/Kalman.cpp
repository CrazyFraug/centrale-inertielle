
#include <boost/numeric/ublas/operation.hpp>
#include "Kalman.h"
#include "Tools.h"

/** \brief Constructor d'un objet Kalman
*/
Kalman::Kalman(int nbIn, int nbOut, int nbState, int step) 
{
	/********************************************************************/
	/* Affectation les nombres d'entrées, sorties et d'état du système	*
	*  Déclaration le nombre d'échantillon d'initialisation				*/
	/********************************************************************/
	_sys.sizeIn = nbIn;
	_sys.sizeOut = nbOut;
	_sys.sizeState = nbState;
	_kalmsys.nbstep = step;

	/********************************************************************/
	/* Affectation les matrices aux tailles du système					*/
	/********************************************************************/
	_sys.matTransition.resize(_sys.sizeState, _sys.sizeState);
#pragma warning(suppress: 6282)
	if (_sys.sizeIn != 0){
		_sys.matCommande.resize(_sys.sizeIn, _sys.sizeState);
		_kalmsys.noiseCommande.resize(1, _sys.sizeIn);
	}
	_sys.matSortie.resize(_sys.sizeOut, _sys.sizeState);

	_kalmsys.matIdent.resize(_sys.sizeState, _sys.sizeState);
	for (int i = 0; i < _kalmsys.matIdent.size1(); i++){
		for (int j = 0; j < _kalmsys.matIdent.size2(); j++){
			if (i == j){
				_kalmsys.matIdent(i, j) = 1;
			}
			else{
				_kalmsys.matIdent(i, j) = 0;
			}
		}
	}

	_kalmsys.covCommande.resize(_sys.sizeState, _sys.sizeState);
	_kalmsys.noiseMesure.resize(1, _sys.sizeOut);
	_kalmsys.covMesure.resize(_sys.sizeOut, _sys.sizeState);
	_kalmsys.covEstimate.resize(_sys.sizeState, _sys.sizeState);


	_kalmsys.predictVector.resize(_sys.sizeState, 1);
	_kalmsys.kalmanGain.resize(_sys.sizeState, _sys.sizeOut);

}

/*******************************************************************************************************/

/* \brief	Destructor
 */
Kalman::~Kalman()
{
}

/*******************************************************************************************************/

/* Getter */
int Kalman::getSizeIn(){
	return _sys.sizeIn;
}

int Kalman::getSizeOut(){
	return _sys.sizeOut;
}

int Kalman::getSizeState(){
	return _sys.sizeState;
}

matrix<double> Kalman::getKalmanGain(){
	return _kalmsys.kalmanGain;
}

matrix<double> Kalman::getCovEstimate(){
	return _kalmsys.covEstimate;
}
/*******************************************************************************************************/

void Kalman::declare_system(matrix<double> &A, matrix<double> &B, matrix<double> &C){
	_sys.matTransition = A;
	_sys.matCommande = B;
	_sys.matSortie = C;
}

/*******************************************************************************************************/

void Kalman::declare_noise(matrix<double> &Q, matrix<double> &R){
	_kalmsys.covCommande = Q;

	_kalmsys.covMesure = R;
}

/*******************************************************************************************************/

void Kalman::initsystem(matrix<double> &A, matrix<double> &B, matrix<double> &C, matrix<double> &Q, matrix<double> &R, matrix<double> &init_predict, matrix<double> &init_cov_estimate){
	_kalmsys.covEstimate = init_cov_estimate;
	_kalmsys.predictVector = init_predict;
	declare_system(A, B, C);
	declare_noise(Q, R);
}

/*******************************************************************************************************/

void Kalman::majsystem(bool maj[3], matrix<double> &A, matrix<double> &B, matrix<double> &C){
	if (maj[0] == true)
		_sys.matTransition = A;
	if (maj[1] == true)
		_sys.matCommande = B;
	if (maj[2] == true)
		_sys.matSortie = C;
}

/*******************************************************************************************************/

void Kalman::predictStep(matrix<double> &value_cmd)
{
	/****************************************/
	/* ^Xk = A*Xk-1 + B*u	(cas normal)	*
	 * ^Xk = A*Xk-1	(cas non entrée)		*/
	/****************************************/
	if (_sys.sizeIn == 0){
		_RPT0(0, "MAT TRANSITION : \n");
		printMatrix(_sys.matTransition);

		_RPT0(0, "PREDICT VECTOR : \n");
		printMatrix(_kalmsys.predictVector);

		_kalmsys.predictVector = product_matrix(_sys.matTransition, _kalmsys.predictVector);

	}
	else{
		_kalmsys.predictVector = product_matrix(_sys.matTransition, _kalmsys.predictVector) + product_matrix(_sys.matCommande, value_cmd);
	}

	/****************************************/
	/* Pk = F*Pk-1*trans(F) + Q				*/
	/****************************************/
	matrix<double> aux_2 = product_matrix(_sys.matTransition, _kalmsys.covEstimate);
	aux_2 = product_matrix(aux_2, trans(_sys.matTransition));
	_kalmsys.covEstimate = aux_2 + _kalmsys.covCommande;

	/*_RPT0(0, "MATRICE DE COVARIANCE : \n");
	printMatrix(_kalmsys.covEstimate);*/
}

/*******************************************************************************************************/

matrix<double> Kalman::updateStep(matrix<double> &value_measure){
	/****************************************/
	/* Hk*Xk-1								*/
	/****************************************/
	matrix<double> predict_measure = product_matrix(_sys.matSortie, _kalmsys.predictVector);
	_RPT0(0, "PREDICT MEASURE : \n");
	printMatrix(predict_measure);
	/****************************************/
	/* Yk = Zk - Hk*Xk-1					*/
	/****************************************/
	matrix<double> innov_measure = value_measure - predict_measure;
	_RPT0(0, "VALUE MEASURE : \n");
	printMatrix(value_measure);
	_RPT0(0, "INNOVATION MEASURE : \n");
	printMatrix(innov_measure);
	/****************************************/
	/* Hk*Pk-1*trans(Hk)					*
	 * Sk = Hk*Pk-1*trans(Hk) + Rk			*/
	/****************************************/
	matrix<double> aux;
	matrix<double> innov_covariance;
	aux = product_matrix(_sys.matSortie, _kalmsys.covEstimate);
	aux = product_matrix(aux, trans(_sys.matSortie));
	innov_covariance = aux + _kalmsys.covMesure;

	_RPT0(0, "INNOVATION COVARIANCE : \n");
	printMatrix(innov_covariance);
	/****************************************/
	/* Kk = Pk-1*trans(Hk)*Sk^-1			*/
	/****************************************/
	matrix<double> aux_2 = product_matrix(_kalmsys.covEstimate, trans(_sys.matSortie));

	_kalmsys.kalmanGain = product_matrix(aux_2, conj(innov_covariance));
	_RPT0(0, "KALMAN GAIN : \n");
	printMatrix(_kalmsys.kalmanGain);
	/****************************************/
	/* ^Xk = ^Xk-1 +  Kk*Yk					*/
	/****************************************/
	matrix<double> aux_3 = product_matrix(_kalmsys.kalmanGain, innov_measure);

	_kalmsys.predictVector = _kalmsys.predictVector + aux_3;
	_RPT0(0, "PREDICT VECTOR : \n");
	printMatrix(_kalmsys.predictVector);
	/****************************************/
	/* Pk = (I - Kk*Hk)*Pk-1				*/
	/****************************************/
	matrix<double> aux_4 = product_matrix(_kalmsys.kalmanGain, _sys.matSortie);
	aux_4 = _kalmsys.matIdent - aux_4;

	_kalmsys.covEstimate = product_matrix(aux_4, _kalmsys.covEstimate);
	/*_RPT0(0, "COVARIANCE ESTIMATE : \n");
	printMatrix(_kalmsys.covEstimate);*/
	return _kalmsys.predictVector;
}

/*******************************************************************************************************/

