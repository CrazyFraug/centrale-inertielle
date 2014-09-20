#ifndef DEF_SERIAL
#define DEF_SERIAL

#include <boost\asio.hpp>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include "Structure.h"
struct packDatas{
	std::string sensor_name;
	vect4D datas;
};

class Serial
{
public:
    /**
     * Constructor.
     * \param port device name, example "/dev/ttyUSB0" or "COM4"
     * \param baud_rate communication speed, example 9600 or 115200
     * \throws boost::system::system_error if cannot open the
     * serial device
     */
    Serial(std::string port, unsigned int baud_rate);

	/**
	* Convert a string into a double type variable
	* return 0 if function failed
	*/
	float string_to_double( const std::string& s );


    /**
     * Write a string to the serial device.
     * \param s string to write
     * \throws boost::system::system_error on failure
     */
    void writeString(std::string s);


    /**
     * Blocks until a line is received from the serial device.
     * Eventual '\n' or '\r\n' characters at the end of the string are removed.
     * \return a string containing the received line
     * \throws boost::system::system_error on failure
     */
    std::string readLine();

	/**
	* \brief lit les valeurs qu'envoie l'arduino
	* les valeurs doivent �tre sous un format sp�cifique (on pourra le changer apr�s):
	* string + ":" + valeur1 + "y"+";" + valeur2 + "x"+";" + valeur3 + "z"+";" + endl
	* exemple : "gyro:0.45y;0.12x;-5.2z;"
	* \param [out] axe :la variable axe repr�sente l'axe dont la valeur � �t� mesur�e : axe = 1 -> axe X | axe = 2 -> axe Y | axe = 3 -> axe Z |
	*/
	double readDatas(int &axe);

	double readDatas(int &axe, char* type, bool &capteur);

	packDatas readData_s(std::string name_sensor);

	void closePort(void);

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
};

#endif;