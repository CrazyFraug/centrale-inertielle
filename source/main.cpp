#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "Mobile.h"
#include "test.h"
#include "Quaternion.h"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace std;
using namespace boost::numeric::ublas;

int main (int argc, char *argv[])
{

	double* tmp = eulerToQuat(0,90,0);
	Quaternion Q1(tmp[0], tmp[1], tmp[2], tmp[3]);
	matrix<double> p = Q1.matricePassage();
	cout << p <<endl;

	SetConsoleTitle(_T("- Wiimote: "));
	HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);
	wiimote remote;
	Mobile manette;

	remote.ChangedCallback = on_state_change;

	remote.CallbackTriggerFlags = (state_change_flags)(	CONNECTED |
														EXTENSION_CHANGED |
														MOTIONPLUS_CHANGED |
														ACCEL_CHANGED |
														ORIENTATION_CHANGED |
														BUTTONS_CHANGED);

//label for reconnection procedure
reconnect:

	COORD pos = { 0, 1 };
	COORD cursor_pos = { 0, 3 };

	//"Looking for a wiimote" waiting screen:
	SetConsoleCursorPosition(console, pos);
	WHITE; _tprintf(_T("  Looking for a Wiimote     "));
	static const TCHAR* wait_str[] = { _T(".  "), _T(".. "), _T("...") };
	unsigned count = 0;
	//"waiting dots" animation :
	while(!remote.Connect(wiimote::FIRST_AVAILABLE))
	{
		_tprintf(_T("\b\b\b%s"), wait_str[count%3]);
		count++;
		Sleep(1000);
	}

		//Switch LEDs on
	remote.SetLEDs(0x0f);
	BRIGHT_CYAN; _tprintf(_T("\b\b\b... Connected!"));

//end reconnect

	while(!remote.Button.Home())
	{
		Sleep (15);

		while(remote.RefreshState() == NO_CHANGE)
			Sleep(5);

		SetConsoleCursorPosition(console, cursor_pos);

		// In case of connection lost: jump to reconnect
		if(remote.ConnectionLost())
			{
			BRIGHT_RED; _tprintf(_T("*** connection lost! ***\n") BLANK_LINE );
			Sleep(2000);
			goto reconnect;
			}

				// Buttons:
		CYAN; _tprintf(_T("  Buttons: ")); WHITE; _tprintf(_T("["));
		for(unsigned bit=0; bit<16; bit++)
			{
			WORD mask = (WORD)(1 << bit);
			// skip unused bits
			if((wiimote_state::buttons::ALL & mask) == 0)
				continue;

			const TCHAR* button_name = wiimote::ButtonNameFromBit[bit];
			bool		 pressed	 = ((remote.Button.Bits & mask) != 0);
			if(bit > 0) {
				CYAN; _tprintf(_T("|")); // seperator
				}
			if(pressed) {
				BRIGHT_WHITE; _tprintf(_T("%s")  , button_name);
				}
			else{
				WHITE       ; _tprintf(_T("%*s"), _tcslen(button_name), _T(""));
				}
			}
		WHITE; _tprintf(_T("]\n"));


		//MAJ attributs de l'objet "manette"
		manette.maj_orientation(remote.Acceleration.Orientation.Pitch,
								remote.Acceleration.Orientation.Roll,
								remote.Acceleration.Orientation.Yaw);

		manette.set_Acceleration(remote.Acceleration.X,
								remote.Acceleration.Y,
								remote.Acceleration.Z);
		//affichage valeurs :
		manette.afficher_mobile();
		cout << "Acc_X : " << remote.Acceleration.X << endl;
		cout << "Acc_Y : " << remote.Acceleration.Y << endl;
		cout << "Acc_Z : " << remote.Acceleration.Z << endl;

		cout << "Gyr_Pitch : " << remote.Acceleration.Orientation.Pitch << endl;
		cout << "Gyr_Roll : " << remote.Acceleration.Orientation.Roll << endl;
		cout << "Gyr_Yaw : " << remote.Acceleration.Orientation.Yaw << endl;


		manette.afficher_vitesse();
		// CALCUL LA NOUVELLE POSITION

        if (remote.Acceleration.Orientation.UpdateAge != 0){
		manette.chgt_repere_translation(remote.Acceleration.X,
                                        remote.Acceleration.Y,
                                        remote.Acceleration.Z
                                        );
		manette.chgt_repere_rotation(remote.Acceleration.Orientation.Pitch,
                                     remote.Acceleration.Orientation.Roll,
                                     remote.Acceleration.Orientation.Yaw
                                    );



		}
		else{
		manette.set_vitesse(0,0,0);

		}
		manette.afficher_position();
         //Sleep(500);


	}

	// disconnect (auto-happens on wiimote destruction anyway, but let's play nice)
	remote.Disconnect();

	BRIGHT_WHITE; // for automatic 'press any key to continue' msg
	CloseHandle(console);



	return 0;

}
