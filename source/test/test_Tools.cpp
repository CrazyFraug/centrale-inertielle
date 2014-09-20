#include <Tools.h>

bool testStringToDouble(const std::string stringDonne, const double doubleResult)
{
	bool test(false);
	double x;
	x = string_to_double(stringDonne);
	_RPT3(0, "Valeur donnée : %s - Valeur espéré : %f - Valeur trouvée : %f \n", stringDonne.c_str(), doubleResult, x);
	if (x == doubleResult)
	{
		_RPT0(0, "String_To_Double validé \n");
	}
	else
	{
		_RPT0(0, "String_To_Double ERROR \n");
	}
	return test;
}

