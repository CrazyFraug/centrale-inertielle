#include "Kalman.h"
#include "Quaternion.h"
#include <fstream>
#include "Tools.h"

/***************************************************************************************
Test du filtre de Kalman - Selon l'exemple d'Alex Blekhman
Les états sont la position et la vitesse
L'observation est la position qui est calculé par la formule X = X + V*t
La vitesse est fixée à 10m/s
Test validé	-- Résultat présenté dans le fichier resultTestKalman.xlsx
*****************************************************************************************/

void testKalman(){
	/* Déclaration des variables locaux */
	double _dt;
	double sigmaMeas;
	double vTrue;
	double posTrue;

	/* Déclaration des matrices utilisées par le filtre Kalman */
	matrix<double> A(2, 2, 0), B(0, 0, 0), C(1, 2, 0);
	matrix<double> Q(2, 2, 0), R(1, 1, 0);
	matrix<double> X(2, 1, 0), P(2, 2, 0);
	matrix<double> matCmde, matObs;
	matrix<double> matResult;

	/* Affectation des valeurs initiaux */
	_dt = .1;
	vTrue = 10;
	posTrue = 0;
	Kalman kalmTest(0, 1, 2, 100);
	A(0, 0) = A(1, 1) = 1;
	A(0, 1) = _dt;
	A(1, 0) = 0;

	C(0, 0) = 1;
	C(0, 1) = 0;

	sigmaMeas = 1;
	R(0, 0) = sigmaMeas*sigmaMeas;

	X(0, 0) = 0;
	X(1, 0) = 0.5*vTrue;

	P(0, 0) = P(1, 1) = 1;

	kalmTest.initSystem(A, B, C, Q, R, X, P);

	matCmde.resize(kalmTest.getSizeIn(), 1, 0);
	matObs.resize(kalmTest.getSizeOut(), 1, 0);
	matResult.resize(kalmTest.getSizeState(), 1, 0);

	/* Fichier stocké le résultat du filtre */
	std::fstream fileResult;
	fileResult.open("resultTest.csv", std::ios::app);
	fileResult << "Step;PositionKalman;PositionTrue;VitesseKalman;VitesseTrue";
	fileResult << "\n";


	for (int step = 0; step < 100; step++){
		posTrue += vTrue*_dt;
		matObs(0, 0) = posTrue;
		matResult = kalmanTraitement(kalmTest, matCmde, matObs);
		fileResult << step << ";";
		fileResult << matResult(0, 0) << "; " << posTrue << ";";
		fileResult << matResult(1, 0) << ";" << vTrue << "\n";
	}
}
//
//int main(){
//	testKalman();
//	return 0;
//}