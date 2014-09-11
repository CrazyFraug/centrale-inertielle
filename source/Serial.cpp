#include "Serial.h"
#include "Tools.h"


/******************************************************************************/

/*	Constructor	*/
Serial::Serial(std::string port, unsigned int baud_rate)
: io(), serial(io, port)
{
	serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
}

/******************************************************************************/

void Serial::writeString(std::string s)
{
	boost::asio::write(serial, boost::asio::buffer(s.c_str(), s.size()));
}

/******************************************************************************/

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

/******************************************************************************/

packDatas Serial::readDatas(std::string name_sensor){
	static double t_pred_out, average_pred_out = 0, t_begin = 0;

	using namespace boost;
	using namespace std;
	char c;
	string name;
	string value_x, value_y, value_z, value_t;
	bool read_x(false), read_y(false), read_z(false), read_t(false);
	packDatas packresult;

	name.clear();
	//t_begin = clock();
	//average_pred_out = 0.8*average_pred_out + (1 - 0.8)*(clock() - t_pred_out);
	//_RPT1(0, "AVERAGE PRED OUT : %f \n", average_pred_out);
	/* Tant qu'on n'a pas fini de lire tout le paquet de donées */
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
		/* Lire une charactère du paquet */
		asio::read(serial, asio::buffer(&c, 1));
		switch (c){
			/* Si c'est un caractère x, y, z, t, on récupère les valeurs pour stocker dans le packresult*/
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
			/* Par défault, on récupère le nom du paquet */
		default:
			break;
		}
	}
	//_RPT1(0, "DUREE READ_DATA_S : %f \n", clock() - t_begin);
	//_RPT1(0, "TOTAL : %f \n", clock() - t_pred_out);
	//t_pred_out = clock();

	return packresult;
}

/******************************************************************************/
