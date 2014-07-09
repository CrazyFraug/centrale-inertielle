#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include "Mobile.h"
#include "test.h"

using namespace std;

int main (int argc, char *argv[])
{
	cout << "fjkgjfseol"<< endl;
	SetConsoleTitle(_T("- Wiimote: "));
	HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);
	wiimote remote;
	Mobile manette;

	double matrice[3][3];
	double v[3] = {-1,0,0}; 
	double w[3] = {0,0,0};

	remote.ChangedCallback = on_state_change;

	remote.CallbackTriggerFlags = (state_change_flags)(	CONNECTED |
														EXTENSION_CHANGED |
														MOTIONPLUS_CHANGED |
														ACCEL_CHANGED |
														ORIENTATION_CHANGED |
														BUTTONS_CHANGED);
	cout << cos(45) << endl;
	manette.rotate_vector(*manette.calculerOrientation(0, 3.1415/4, 0, matrice), w, v);
	cout << "w = " << w[0] << endl;
	cout << w[1] << endl;
	cout << w[2] << endl;

	system("PAUSE");

//label for reconnection procedure
reconnect:

	COORD pos = {0, 6};
	COORD cursor_pos = {0, 7};

	//"Looking for a wiimote" waiting screen:
	SetConsoleCursorPosition(console, pos);
	CYAN; _tprintf(_T("  Looking for a Wiimote     "));
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
        int i = 0;
        while (i<=4){
            manette.get_Acceleration(remote.Acceleration.X,
								remote.Acceleration.Y,
								remote.Acceleration.Z,i);
			i++;
        }

        double best_x, best_y, best_z;
        best_x = manette.best_Value_x();
        best_y = manette.best_Value_y();
        best_z = manette.best_Value_z();

        cout << "Best_ACC_X :  " << best_x << endl;
        cout << "Best_ACC_Y :  " << best_y << endl;
        cout << "Best_ACC_Z :  " << best_z << endl;


		manette.set_Acceleration(best_x, best_y, best_z);
		/*manette.set_Acceleration(remote.Acceleration.X,
								remote.Acceleration.Y,
								remote.Acceleration.Z);*/
		//affichage valeurs :
		manette.afficher_mobile();
		cout << "Acc_X : " << manette.acc_trans.accel_x << endl;
		cout << "Acc_Y : " << manette.acc_trans.accel_y << endl;
		cout << "Acc_Z : " << manette.acc_trans.accel_z << endl;

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
		/*manette.chgt_repere_rotation(remote.Acceleration.Orientation.Pitch,
                                     remote.Acceleration.Orientation.Roll,
                                     remote.Acceleration.Orientation.Yaw
                                    );*/

		}
		else{
			manette.set_vitesse(0,0,0);
		}
		manette.afficher_position();
         //Sleep(40);


	}

	// disconnect (auto-happens on wiimote destruction anyway, but let's play nice)
	remote.Disconnect();

	BRIGHT_WHITE; // for automatic 'press any key to continue' msg
	CloseHandle(console);



	return 0;

}
