#include <Tools.h>

bool testStringToDouble(const std::string stringDonne, const double doubleResult)
{
	bool test(false);
	double x;
	x = string_to_double(stringDonne);
	_RPT3(0, "Valeur donn�e : %s - Valeur esp�r� : %f - Valeur trouv�e : %f \n", stringDonne.c_str(), doubleResult, x);
	if (x == doubleResult)
	{
		_RPT0(0, "String_To_Double valid� \n");
	}
	else
	{
		_RPT0(0, "String_To_Double ERROR \n");
	}
	return test;
}

