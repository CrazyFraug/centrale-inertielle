#ifndef TEST_H
#define TEST_H

#include <wiimote.h>
#include <mmsystem.h>	// for timeGetTime
#include <string.h>
#include "Mobile.h"

/**DEFINES**/
#define _SETCOL(flags) SetConsoleTextAttribute(console, flags)
//Colors (for console font)
#define WHITE			_SETCOL(FOREGROUND_RED|FOREGROUND_GREEN|FOREGROUND_BLUE)
#define BRIGHT_CYAN		_SETCOL(FOREGROUND_GREEN|FOREGROUND_BLUE|FOREGROUND_INTENSITY)
#define BRIGHT_RED		_SETCOL(FOREGROUND_RED|FOREGROUND_INTENSITY)
#define CYAN			_SETCOL(FOREGROUND_BLUE|FOREGROUND_GREEN)
#define RED				_SETCOL(FOREGROUND_RED)
#define BRIGHT_WHITE	_SETCOL(FOREGROUND_RED|FOREGROUND_GREEN|FOREGROUND_BLUE|FOREGROUND_INTENSITY)
#define YELLOW			_SETCOL(FOREGROUND_RED|FOREGROUND_GREEN)
#define BRIGHT_GREEN	_SETCOL(FOREGROUND_GREEN|FOREGROUND_INTENSITY)
//
#define BLANK_LINE _T("\n")

void on_state_change (wiimote			  &remote,
					  state_change_flags  changed,
					  const wiimote_state &new_state);

int test(accel_translation translation, repere_angle orientation);

void update_accel (wiimote			  &remote,
					  state_change_flags  changed,
					  const wiimote_state &new_state);

#endif