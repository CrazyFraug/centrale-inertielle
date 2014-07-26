#include "Serial.h"

//for arduino: SCL to A5, SDA to A4, GND to ground and Vin to 5V


    /**
     * Constructor.
     * \param port device name, example "/dev/ttyUSB0" or "COM4"
     * \param baud_rate communication speed, example 9600 or 115200
     * \throws boost::system::system_error if cannot open the
     * serial device
     */
    Serial::Serial(std::string port, unsigned int baud_rate)
    : io(), serial(io,port)
    {
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    }


	/**
	* Convert a string into a double type variable
	* return 0 if function failed
	*/
	float Serial::string_to_double( const std::string& s )
	 {
	   std::stringstream convert(s);
	   float x = 0;

	   if (!(convert >> x))
	   {
		 //std::cout << " failed ";
		 return 0;
	   }
	   return x;
	 } 


    /**
     * Write a string to the serial device.
     * \param s string to write
     * \throws boost::system::system_error on failure
     */
    void Serial::writeString(std::string s)
    {
        boost::asio::write(serial,boost::asio::buffer(s.c_str(),s.size()));
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
        for(;;)
        {
            asio::read(serial,asio::buffer(&c,1));
            switch(c)
            {
                case '\r':
                    break;
                case '\n':
                    return result;
                default:
                    result+=c;
            }
        }
    }

	/** version gyro seul
* \brief lit les valeurs qu'envoie l'arduino
* les valeurs doivent être sous un format spécifique (on pourra le changer après):
* string + ":" + valeur1 + "y"+";" + valeur2 + "x"+";" + valeur3 + "z"+";" + endl
* exemple : "gyro:0.45y;0.12x;-5.2z;"
* \param [out] axe :la variable axe représente l'axe dont la valeur à été mesurée : axe = 1 -> axe X | axe = 2 -> axe Y | axe = 3 -> axe Z |
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
 *	Lire les données viennent de l'arduino
 *	\return	packresult	packDatas	paquet de données contient le nom du capteur, les données selon les 3 axes et le temps
 */
packDatas Serial::readData_s(void){
	using namespace boost;
	using namespace std;
	char c;
	string name;
	string value_x, value_y, value_z, value_t;
	bool read_x(false), read_y(false), read_z(false), read_t(false);
	packDatas packresult;

	name.clear();

	/* Tant qu'on n'a pas fini de lire tout le paquet de donées */
	asio::read(serial, asio::buffer(&c, 1));
	while (c != 'a' && c != 'g'){
		asio::read(serial, asio::buffer(&c, 1));
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
		default:
			break;
		}
	}
	return packresult;
}
///**
//* \brief lit les valeurs qu'envoie l'arduino
//* les valeurs doivent être sous un format spécifique (on pourra le changer après):
//* type(char*) + ":" + valeur1 + "y"+";" + valeur2 + "x"+";" + valeur3 + "z"+";" + endl
//* exemple : "gyro:0.45y;0.12x;-5.2z;"
//* \param [out] axe :la variable axe représente l'axe dont la valeur à été mesurée : axe = 1 -> axe X | axe = 2 -> axe Y | axe = 3 -> axe Z |
//*/
//double Serial::readDatas(int &axe, char* type, bool &capteur)
//{
//	using namespace boost;
//	using namespace std;
//	char c;
//	string result;
//	axe = 0;

//	for (;;)

//	{
//		asio::read(serial, asio::buffer(&c, 1));
//		switch (c)
//		{//crlf \r\n

//		case '\r':
//			break;

//		case '\n':
//			result.clear();
//			break;

//		case ';':
//			result.clear();
//			break;

//		case 'x':
//			asio::read(serial, asio::buffer(&c, 1));

//			if (c == ';' && capteur)
//			{
//				axe = 1;
//				return string_to_double(result);
//			}
//			//else result.clear();

//			break;

//		case 'y':
//			asio::read(serial, asio::buffer(&c, 1));
//			if (c == ';' && capteur)
//			{
//				axe = 2;
//				return string_to_double(result);
//			}
//			else { result += 'y'; result += c; }
//			break;

//		case 'z':
//			asio::read(serial, asio::buffer(&c, 1));
//			if (c == ';' && capteur)
//			{
//				axe = 3;
//				return string_to_double(result);
//			}

//			//else result.clear();
//			break;

//		case 't':
//			asio::read(serial, asio::buffer(&c, 1));
//			if (c == ';' && capteur)
//			{
//				axe = 4;
//				return string_to_double(result);
//			}
//			else result.clear();
//			break;

//		case ':':
//			if (result == type)
//			{
//				capteur = true;
//			}
//			else capteur = false;
//			result.clear();
//			break;
//		default:
//			result += c;
//		}

//	}
//}