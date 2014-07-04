#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "Mobile.h"
#include "test.h"

using namespace std;


int main (int argc, char *argv[])
{

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
	COORD cursor_pos = { 0, 5 };
	
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


	//Calibration de  la manette
	manette.calibrer(remote.Acceleration.X, remote.Acceleration.Y, remote.Acceleration.Z);
	cout << endl;
	CYAN; cout << "Acc_X initiale: " << (remote.Acceleration.X) << endl;
	cout << "Acc_Y : initiale " << (remote.Acceleration.Y) << endl;
	cout << "Acc_Z initiales : " << (remote.Acceleration.Z) << endl;

	manette.activer_clock();

	//boucle principale
	while(!remote.Button.Home())
	{
		
		SetConsoleCursorPosition(console, cursor_pos);

		RED; cout << "Acc_X initiale: " << (remote.Acceleration.X) << endl;
		cout << "clock " << clock() << endl;

		while(remote.RefreshState() == NO_CHANGE)
			Sleep(1);

		// In case of connection lost: jump to reconnect
		if(remote.ConnectionLost())
			{
			BRIGHT_RED; _tprintf(_T("*** connection lost! ***\n") BLANK_LINE );
			Sleep(2000);
			goto reconnect;
			}

		if(remote.Button.A()) {
			manette.calibrer(remote.Acceleration.X, remote.Acceleration.Y, remote.Acceleration.Z);
		cout << "calibrer";
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
								0);

		/*manette.calcul_vitesse(	remote.Acceleration.Orientation.X,
								remote.Acceleration.Orientation.Y,
								remote.Acceleration.Orientation.Z);*/
		//affichage valeurs :
		manette.afficher_mobile();
		

		manette.afficher_vitesse();

		// CALCUL LA NOUVELLE POSITION

       // if (remote.Acceleration.Orientation.UpdateAge != 0){
		manette.chgt_repere_translation(remote.Acceleration.X,
                                        remote.Acceleration.Y,
                                        remote.Acceleration.Z
                                        );
		//manette.chgt_repere_rotation(remote.Acceleration.Orientation.Pitch,
        //                             remote.Acceleration.Orientation.Roll,
        //                             remote.Acceleration.Orientation.Yaw
        //                            );



		//}
		//else{
		//manette.set_vitesse(0,0,0);

		//}

		manette.afficher_position();

	}

	// disconnect (auto-happens on wiimote destruction anyway, but let's play nice)
	remote.Disconnect();

	BRIGHT_WHITE; // for automatic 'press any key to continue' msg
	CloseHandle(console);



	return 0;

}