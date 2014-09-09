#include "Serial.h"
#include "Tools.h"
//for arduino: SCL to A5, SDA to A4, GND to ground and Vin to 5V


/**
* Constructor.
* \param port device name, example "/dev/ttyUSB0" or "COM4"
* \param baud_rate communication speed, example 9600 or 115200
* \throws boost::system::system_error if cannot open the
* serial device
*/
Serial::Serial(std::string port, unsigned int baud_rate)
: io(), serial(io, port)
{
	serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
}

/**
* Write a string to the serial device.
* \param s string to write
* \throws boost::system::system_error on failure
*/
void Serial::writeString(std::string s)
{
	boost::asio::write(serial, boost::asio::buffer(s.c_str(), s.size()));
}

/**
* Blocks until a line is received from the serial device.
* Eventual '\n' or '\r\n' characters at the end of the string are removed.
* \return a string containing the received line
* \throws boost::system::system_error on failure
*/
std::string Serial::readLine()
{
	//Reading data char by char, code is optimized for simplicity, not speed
	using namespace boost;
	char c;
	std::string result;
	for (;;)
	{
		asio::read(serial, asio::buffer(&c, 1));
		switch (c)
		{
		case '\r':
			break;
		case '\n':
			return result;
		default:
			result += c;
		}
	}
}

/** version gyro seul
* \brief lit les valeurs qu'envoie l'arduino
* les valeurs doivent �tre sous un format sp�cifique (on pourra le changer apr�s):
* string + ":" + valeur1 + "y"+";" + valeur2 + "x"+";" + valeur3 + "z"+";" + endl
* exemple : "gyro:0.45y;0.12x;-5.2z;"
* \param [out] axe :la variable axe repr�sente l'axe dont la valeur � �t� mesur�e : axe = 1 -> axe X | axe = 2 -> axe Y | axe = 3 -> axe Z |
*/
double Serial::readDatas(int &axe)
{
	using namespace boost;
	using namespace std;
	char c;
	string result;
	axe = 0;
	double testo = 0;

	for (;;)

	{
		asio::read(serial, asio::buffer(&c, 1));
		switch (c)
		{//crlf \r\n

		case '\r':
			break;
		case ':':
			result.clear();
			break;
		case '\n':
			result.clear();
			break;
		case ';':
			break;

		case 'x':
			asio::read(serial, asio::buffer(&c, 1));
			if (c == ';')
			{
				axe = 1;
				return string_to_double(result);
			}
			else result.clear();

		case 'y':
			asio::read(serial, asio::buffer(&c, 1));
			if (c == ';')
			{
				axe = 2;
				return string_to_double(result);
			}
			else result.clear();

		case 'z':
			asio::read(serial, asio::buffer(&c, 1));
			if (c == ';')
			{
				axe = 3;
				return string_to_double(result);
			}
			else result.clear();

		case 't':
			asio::read(serial, asio::buffer(&c, 1));
			if (c == ';')
			{
				axe = 4;
				return string_to_double(result);
			}
			else result.clear();

		default:
			result += c;
		}
	}
}

/**
*	Lire les donn�es viennent de l'arduino
*	\return	packresult	packDatas	paquet de donn�es contient le nom du capteur, les donn�es selon les 3 axes et le temps
*/
packDatas Serial::readData_s(std::string name_sensor){
	static double t_pred_out, average_pred_out = 0, t_begin = 0;

	using namespace boost;
	using namespace std;
	char c;
	string name;
	string value_x, value_y, value_z, value_t;
	bool read_x(false), read_y(false), read_z(false), read_t(false);
	packDatas packresult;

	name.clear();
	t_begin = clock();
	average_pred_out = 0.8*average_pred_out + (1 - 0.8)*(clock() - t_pred_out);
	_RPT1(0, "AVERAGE PRED OUT : %f \n", average_pred_out);
	/* Tant qu'on n'a pas fini de lire tout le paquet de don�es */
	asio::read(serial, asio::buffer(&c, 1));
	if (name_sensor == "acce"){
		while (c != 'a'){
			asio::read(serial, asio::buffer(&c, 1));
		}
	}
	else if (name_sensor == "gyro"){
		while (c != 'g'){
			asio::read(serial, asio::buffer(&c, 1));
		}
	}
	else if (name_sensor == "mnet")
	{
		while (c != 'm'){
			asio::read(serial, asio::buffer(&c, 1));
		}
	}
	else if (name_sensor == "orie")
	{
		while (c != 'o'){
			asio::read(serial, asio::buffer(&c, 1));
		}
	}

	while (c != ':'){
		name += c;

		asio::read(serial, asio::buffer(&c, 1));
	}
	packresult.sensor_name = name;

	while (!read_x || !read_y || !read_z || !read_t)
	{
		/* Lire une charact�re du paquet */
		asio::read(serial, asio::buffer(&c, 1));
		switch (c){
			/* Si c'est un caract�re x, y, z, t, on r�cup�re les valeurs pour stocker dans le packresult*/
		case'x':
			while (c != ';')
			{
				asio::read(serial, asio::buffer(&c, 1));
				value_x += c;
			}
			packresult.datas.x = string_to_double(value_x);
			read_x = true;
			break;
		case'y':
			while (c != ';')
			{
				asio::read(serial, asio::buffer(&c, 1));
				value_y += c;
			}
			packresult.datas.y = string_to_double(value_y);
			read_y = true;
			break;
		case'z':
			while (c != ';')
			{
				asio::read(serial, asio::buffer(&c, 1));
				value_z += c;
			}
			packresult.datas.z = string_to_double(value_z);
			read_z = true;
			break;
		case't':
			while (c != ';')
			{
				asio::read(serial, asio::buffer(&c, 1));
				value_t += c;
			}
			packresult.datas.temps = string_to_double(value_t);
			read_t = true;
			break;
			/* Par d�fault, on r�cup�re le nom du paquet */
		default:
			break;
		}
	}
	_RPT1(0, "DUREE READ_DATA_S : %f \n", clock() - t_begin);
	_RPT1(0, "TOTAL : %f \n", clock() - t_pred_out);
	t_pred_out = clock();

	return packresult;
}

/**
* \brief lit les valeurs qu'envoie l'arduino
* les valeurs doivent �tre sous un format sp�cifique (on pourra le changer apr�s):
* type(char*) + ":" + valeur1 + "y"+";" + valeur2 + "x"+";" + valeur3 + "z"+";" + endl
* exemple : "gyro:0.45y;0.12x;-5.2z;"
* \param [out] axe :la variable axe repr�sente l'axe dont la valeur � �t� mesur�e : axe = 1 -> axe X | axe = 2 -> axe Y | axe = 3 -> axe Z |
*/
double Serial::readDatas(int &axe, char* type, bool &capteur)
{
	using namespace boost;
	using namespace std;
	char c;
	string result;
	axe = 0;

	for (;;)

	{
		asio::read(serial, asio::buffer(&c, 1));
		switch (c)
		{//crlf \r\n

		case '\r':
			break;

		case '\n':
			result.clear();
			break;

		case ';':
			result.clear();
			break;

		case 'x':
			asio::read(serial, asio::buffer(&c, 1));

			if (c == ';' && capteur)
			{
				axe = 1;
				return string_to_double(result);
			}
			//else result.clear();

			break;

		case 'y':
			asio::read(serial, asio::buffer(&c, 1));
			if (c == ';' && capteur)
			{
				axe = 2;
				return string_to_double(result);
			}
			else { result += 'y'; result += c; }
			break;

		case 'z':
			asio::read(serial, asio::buffer(&c, 1));
			if (c == ';' && capteur)
			{
				axe = 3;
				return string_to_double(result);
			}

			//else result.clear();
			break;

		case 't':
			asio::read(serial, asio::buffer(&c, 1));
			if (c == ';' && capteur)
			{
				axe = 4;
				return string_to_double(result);
			}
			else result.clear();
			break;

		case ':':
			if (result == type)
			{
				capteur = true;
			}
			else capteur = false;
			result.clear();
			break;
		default:
			result += c;
		}

	}
}