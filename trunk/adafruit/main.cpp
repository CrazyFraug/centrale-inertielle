#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include "Serial.cpp"
#include "Mobile.h"

using namespace std;

int main (int argc, char *argv[])
{
	int axe;
	Mobile manette;
	SimpleSerial serie8("COM8",115200);
	string valeur;
	while(1) {	

		valeur = serie8.readDatas(axe);
		if (axe == 1){cout << "Wx = " << valeur << endl;}
		else{ 
			if(axe == 2){cout << "Wy = " << valeur << endl;}
			else{ 
				if (axe == 3){cout << "Wz = " << valeur << endl;}
			}
		}
	}

	double matrice[3][3];
	double v[3] = {-1,0,0};
	double w[3] = {0,0,0};

	cout << cos(45) << endl;
	manette.rotate_vector(*manette.calculerOrientation(0, 3.1415/4, 0, matrice), w, v);
	cout << "w = " << w[0] << endl;
	cout << w[1] << endl;
	cout << w[2] << endl;

	system("PAUSE");


	return 0;

}
