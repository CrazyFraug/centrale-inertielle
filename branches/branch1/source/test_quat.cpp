#include "Quaternion.h"
#include <stdlib.h>



void test_eulerQuat(void)
{
	quaternion<double> q1(0,0,0,0);
	double phi, teta, psi;
	vect3D angle;
	double quat_angle = 0;
	vect3D quat_axe = {0,0,0};

	system("cls");
	_RPT0(0, "\n");
	phi = 15;
	teta = 0;
	psi = 0;
	std::cout << std::endl << "Angle avant conversion :\nphi = " << phi << "\nteta = "<< teta << "\npsi = " << psi << std::endl;
	q1 = danglesToQuat(phi, teta, psi);
	angle = quatToAngles_deg(q1);
	quatComp(q1, quat_axe, quat_angle);
	_RPT3(0, "Euler = %f  %f  %f\n", angle.x, angle.y, angle.z);
	_RPT4(0, "quaternion : angle = %f, axe = (%f, %f, %f)\n\n", quat_angle*180/M_PI, quat_axe.x, quat_axe.y, quat_axe.z);
	std::cout << "Angle apres conversion :\nphi = " << angle.x << "\nteta = " << angle.y << "\npsi =" << angle.z << std::endl << std::endl;

	phi = 0;
	teta = -90;
	psi = 0;
	std::cout << "Angle avant conversion :\nphi = " << phi << "\nteta = "<< teta << "\npsi = " << psi << std::endl;
	q1 = danglesToQuat(phi, teta, psi);
	angle = quatToAngles_deg(q1);
	quatComp(q1, quat_axe, quat_angle);
	_RPT3(0, "Euler = %f  %f  %f\n", angle.x, angle.y, angle.z);
	_RPT4(0, "quaternion : angle = %f, axe = (%f, %f, %f)\n\n", quat_angle*180/M_PI, quat_axe.x, quat_axe.y, quat_axe.z);
	std::cout << "angle apres conversion :\nphi = " << angle.x << "\nteta = " << angle.y << "\npsi =" << angle.z << std::endl << std::endl;

	phi = 0;
	teta = 380;
	psi = 0;
	std::cout << "Angle avant conversion :\nphi = " << phi << "\nteta = "<< teta << "\npsi = " << psi << std::endl;
	q1 = danglesToQuat(phi, teta, psi);
	angle = quatToAngles_deg(q1);
	quatComp(q1, quat_axe, quat_angle);
	_RPT3(0, "Euler = %f  %f  %f\n", angle.x, angle.y, angle.z);
	_RPT4(0, "quaternion : angle = %f, axe = (%f, %f, %f)\n\n", quat_angle*180/M_PI, quat_axe.x, quat_axe.y, quat_axe.z);
	std::cout << "angle apres conversion :\nphi = " << angle.x << "\nteta = " << angle.y << "\npsi =" << angle.z << std::endl << std::endl;

	phi = 90;
	teta = 0;
	psi = 90;
	std::cout << "Angle avant conversion :\nphi = " << phi << "\nteta = "<< teta << "\npsi = " << psi << std::endl;
	q1 = danglesToQuat(phi, teta, psi);
	angle = quatToAngles_deg(q1);
	quatComp(q1, quat_axe, quat_angle);
	_RPT3(0, "Euler = %f  %f  %f\n", angle.x, angle.y, angle.z);
	_RPT4(0, "quaternion : angle = %f, axe = (%f, %f, %f)\n\n", quat_angle*180/M_PI, quat_axe.x, quat_axe.y, quat_axe.z);
	std::cout << "angle apres conversion :\nphi = " << angle.x << "\nteta = " << angle.y << "\npsi =" << angle.z << std::endl << std::endl;

	phi = 90;
	teta = -90;
	psi = 0;
	std::cout << "Angle avant conversion :\nphi = " << phi << "\nteta = "<< teta << "\npsi = " << psi << std::endl;
	q1 = danglesToQuat(phi, teta, psi);
	angle = quatToAngles_deg(q1);
	quatComp(q1, quat_axe, quat_angle);
	_RPT3(0, "Euler = %f  %f  %f\n", angle.x, angle.y, angle.z);
	_RPT4(0, "quaternion : angle = %f, axe = (%f, %f, %f)\n\n", quat_angle*180/M_PI, quat_axe.x, quat_axe.y, quat_axe.z);
	std::cout << "angle apres conversion :\nphi = " << angle.x << "\nteta = " << angle.y << "\npsi =" << angle.z << std::endl << std::endl;

	//system("PAUSE");

}
