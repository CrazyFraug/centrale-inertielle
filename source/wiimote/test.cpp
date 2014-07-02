#include "test.h"


//work in progress
/*void display_state(string changed_state, int new_val)		
	{													

	switch(changed_state)								
			{	case _T("A"): cursor_pos.Y = 5;			
					break;								
				case _T("B"): cursor_pos.Y = 6;			
					break;								
				case _T("Plus"): cursor_pos.Y = 7;		
					break;								
				case _T("Minus"): cursor_pos.Y = 8;		
					break;								
				case _T("One"): cursor_pos.Y = 9;		
					break;								
				case _T("Two"): cursor_pos.Y = 10;		
					break;								
			}											

		SetConsoleCursorPosition(console, cursor_pos);	\
		_tprintf(_T(changed_state + " :"+ new_val + "\r\n"));	\			
		}		*/		



void on_state_change (wiimote			  &remote,
					  state_change_flags  changed,
					  const wiimote_state &new_state)
	{
	// we use this callback to set report types etc. to respond to key events
	//  (like the wiimote connecting or extensions (dis)connecting).
	
	// NOTE: don't access the public state from the 'remote' object here, as it will
	//		  be out-of-date (it's only updated via RefreshState() calls, and these
	//		  are reserved for the main application so it can be sure the values
	//		  stay consistent between calls).  Instead query 'new_state' only.

	// the wiimote just connected
	if(changed & CONNECTED)
		{
		// ask the wiimote to report everything (using the 'non-continous updates'
		//  default mode - updates will be frequent anyway due to the acceleration/IR
		//  values changing):

		// note1: you don't need to set a report type for Balance Boards - the
		//		   library does it automatically.
		
		// note2: for wiimotes, the report mode that includes the extension data
		//		   unfortunately only reports the 'BASIC' IR info (ie. no dot sizes),
		//		   so let's choose the best mode based on the extension status:
		if(new_state.ExtensionType != wiimote::BALANCE_BOARD)
			{
			if(new_state.bExtension)
				remote.SetReportType(wiimote::IN_BUTTONS_ACCEL_IR_EXT); // no IR dots
			else
				remote.SetReportType(wiimote::IN_BUTTONS_ACCEL_IR);		//    IR dots
			}
		}
	// a MotionPlus was detected
	if(changed & MOTIONPLUS_DETECTED)
		{
		// enable it if there isn't a normal extension plugged into it
		// (MotionPlus devices don't report like normal extensions until
		//  enabled - and then, other extensions attached to it will no longer be
		//  reported (so disable the M+ when you want to access them again).
		if(remote.ExtensionType == wiimote_state::NONE) {
			bool res = remote.EnableMotionPlus();
			_ASSERT(res);
			}
		}
	// an extension is connected to the MotionPlus
	else if(changed & MOTIONPLUS_EXTENSION_CONNECTED)
		{
		// We can't read it if the MotionPlus is currently enabled, so disable it:
		if(remote.MotionPlusEnabled())
			remote.DisableMotionPlus();
		}
	// an extension disconnected from the MotionPlus
	else if(changed & MOTIONPLUS_EXTENSION_DISCONNECTED)
		{
		// enable the MotionPlus data again:
		if(remote.MotionPlusConnected())
			remote.EnableMotionPlus();
		}
	// another extension was just connected:
	else if(changed & EXTENSION_CONNECTED)
		{
#ifdef USE_BEEPS_AND_DELAYS
		Beep(1000, 200);
#endif
		// switch to a report mode that includes the extension data (we will
		//  loose the IR dot sizes)
		// note: there is no need to set report types for a Balance Board.
		if(!remote.IsBalanceBoard())
			remote.SetReportType(wiimote::IN_BUTTONS_ACCEL_IR_EXT);
		}
	// extension was just disconnected:
	else if(changed & EXTENSION_DISCONNECTED)
		{
#ifdef USE_BEEPS_AND_DELAYS
		Beep(200, 300);
#endif
		// use a non-extension report mode (this gives us back the IR dot sizes)
		remote.SetReportType(wiimote::IN_BUTTONS_ACCEL_IR);
		}
	}


void update_accel (wiimote			  &remote,
				state_change_flags  changed,
				const wiimote_state &new_state) {

					if(changed && ACCEL_CHANGED)
					{
						//Mobile
					}
}


int test(accel_translation translation, repere_angle orientation)
{

	SetConsoleTitle(_T("- WiiYourself! - Demo: "));
	HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);

	wiimote remote;

		// in this demo we use a state-change callback to get notified of
	//  extension-related events, and polling for everything else
	// (note you don't have to use both, use whatever suits your app):
	remote.ChangedCallback = on_state_change;
	//  notify us only when the wiimote connected sucessfully, or something
	//   related to extensions changes
	remote.CallbackTriggerFlags = (state_change_flags)(	CONNECTED | 
														EXTENSION_CHANGED | 
														MOTIONPLUS_CHANGED);

	//action when a button is pressed/released:
	#define ON_PRESS_RELEASE(button, pressed_action, released_action)	\
	{																	\
	bool pressed = remote.Button.button();								\
	if(pressed)															\
		{				   												\
		if(!last_##button) pressed_action;		/* just pressed? */		\
		}																\
																		\
	else if(last_##button) /* just released */							\
		released_action;												\
																		\
	/* remember the current button state for next time */				\
	last_##button = pressed;											\
	}


reconnect:

	COORD pos = { 0, 1 };	
	COORD cursor_pos = { 0, 3 };
	
	SetConsoleCursorPosition(console, pos);

	//"Looking for a wiimote" waiting screen:
	WHITE; _tprintf(_T("  Looking for a Wiimote     "));
	static const TCHAR* wait_str[] = { _T(".  "), _T(".. "), _T("...") };
	unsigned count = 0;
	while(!remote.Connect(wiimote::FIRST_AVAILABLE)) {
		_tprintf(_T("\b\b\b%s"), wait_str[count%3]);
		count++;
		Sleep(1000);
	}

		//Switch LEDs on
		remote.SetLEDs(0x0f);
		BRIGHT_CYAN; _tprintf(_T("\b\b\b... Connected!"));

		//instructions:
		WHITE; _tprintf(_T("\r\n  -- Home = Exit --\n"));

					/** Begin loop **/
	// display the wiimote state data until 'Home' is pressed:
	while(!remote.Button.Home())// && !GetAsyncKeyState(VK_ESCAPE))
	{
		
		while(remote.RefreshState() == NO_CHANGE)
			Sleep(1);

		cursor_pos.Y = 3;
		SetConsoleCursorPosition(console, cursor_pos);

		// Battery level display:
		CYAN; _tprintf(_T("  Battery: "));
		BRIGHT_GREEN; _tprintf(_T("%3u%%   "), remote.BatteryPercent);

		// In case of connection lost: jump to reconnect
		if(remote.ConnectionLost())
			{
			BRIGHT_RED; _tprintf(_T("   *** connection lost! ***                                          \n")
				BLANK_LINE BLANK_LINE BLANK_LINE BLANK_LINE BLANK_LINE BLANK_LINE
				BLANK_LINE BLANK_LINE BLANK_LINE BLANK_LINE BLANK_LINE BLANK_LINE
				BLANK_LINE BLANK_LINE BLANK_LINE);
			//Beep(100, 1000);
			Sleep(2000);
			COORD pos = { 0, 6 };
			SetConsoleCursorPosition(console, pos);
			_tprintf(BLANK_LINE BLANK_LINE BLANK_LINE);
			goto reconnect;
			}

// Output method:
	    CYAN; _tprintf( _T("        using %s\n"), (remote.IsUsingHIDwrites()?
											   _T("HID writes") : _T("WriteFile()")));
		
		// 'Unique' IDs (not guaranteed to be unique, check the variable
		//  defintion for details)
		CYAN  ; _tprintf(_T("       ID: "));
		YELLOW; _tprintf(_T("%I64u")  , remote.UniqueID);
#ifdef ID2_FROM_DEVICEPATH		// (see comments in header)
		CYAN;  _tprintf(_T("   ID2: "));
		WHITE; _tprintf(_T("%I64u\n"), remote.UniqueID2);
#else
		_tprintf(_T("\n"));
#endif

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

		// Acceleration:
		CYAN ; _tprintf(_T("    Accel:"));
		remote.IsBalanceBoard()? RED : WHITE;
		_tprintf(_T("  X %+2.3f  Y %+2.3f  Z %+2.3f  \n"),
					remote.Acceleration.X,
					remote.Acceleration.Y,
					remote.Acceleration.Z);
	
		// Orientation estimate (shown red if last valid update is aging):
		CYAN ; _tprintf(_T("   Orient:"));
		remote.IsBalanceBoard()? RED : WHITE;
		_tprintf(_T("  UpdateAge %3u  "), remote.Acceleration.Orientation.UpdateAge);
		
		//  show if the last orientation update is considered out-of-date
		//   (using an arbitrary threshold)
		if(remote.Acceleration.Orientation.UpdateAge > 10)
			RED;
			
		_tprintf(_T("Pitch:%4ddeg  Roll:%4ddeg  \n")
			     _T("                           (X %+.3f  Y %+.3f  Z %+.3f)      \n"),
				 (int)remote.Acceleration.Orientation.Pitch,
				 (int)remote.Acceleration.Orientation.Roll ,
				 remote.Acceleration.Orientation.X,
				 remote.Acceleration.Orientation.Y,
  				 remote.Acceleration.Orientation.Z);


	} //end while(!home)

		// disconnect (auto-happens on wiimote destruction anyway, but let's play nice)
	remote.Disconnect();

	BRIGHT_WHITE; // for automatic 'press any key to continue' msg
	CloseHandle(console);

	return 0;
}