/****************************************************************************
 *
 *   Copyright (C) 2013, 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Driver for the PX4 audio alarm port, /dev/tone_alarm.
 *
 * The tone_alarm driver supports a set of predefined "alarm"
 * tunes and one user-supplied tune.
 *
 * The TONE_SET_ALARM ioctl can be used to select a predefined
 * alarm tune, from 1 - <TBD>.  Selecting tune zero silences
 * the alarm.
 *
 * Tunes follow the syntax of the Microsoft GWBasic/QBasic PLAY
 * statement, with some exceptions and extensions.
 *
 * From Wikibooks:
 *
 * PLAY "[string expression]"
 *
 * Used to play notes and a score ... The tones are indicated by letters A through G.
 * Accidentals are indicated with a "+" or "#" (for sharp) or "-" (for flat)
 * immediately after the note letter. See this example:
 *
 *   PLAY "C C# C C#"
 *
 * Whitespaces are ignored inside the string expression. There are also codes that
 * set the duration, octave and tempo. They are all case-insensitive. PLAY executes
 * the commands or notes the order in which they appear in the string. Any indicators
 * that change the properties are effective for the notes following that indicator.
 *
 * Ln     Sets the duration (length) of the notes. The variable n does not indicate an actual duration
 *        amount but rather a note type; L1 - whole note, L2 - half note, L4 - quarter note, etc.
 *        (L8, L16, L32, L64, ...). By default, n = 4.
 *        For triplets and quintets, use L3, L6, L12, ... and L5, L10, L20, ... series respectively.
 *        The shorthand notation of length is also provided for a note. For example, "L4 CDE L8 FG L4 AB"
 *        can be shortened to "L4 CDE F8G8 AB". F and G play as eighth notes while others play as quarter notes.
 * On     Sets the current octave. Valid values for n are 0 through 6. An octave begins with C and ends with B.
 *        Remember that C- is equivalent to B.
 * < >    Changes the current octave respectively down or up one level.
 * Nn     Plays a specified note in the seven-octave range. Valid values are from 0 to 84. (0 is a pause.)
 *        Cannot use with sharp and flat. Cannot use with the shorthand notation neither.
 * MN     Stand for Music Normal. Note duration is 7/8ths of the length indicated by Ln. It is the default mode.
 * ML     Stand for Music Legato. Note duration is full length of that indicated by Ln.
 * MS     Stand for Music Staccato. Note duration is 3/4ths of the length indicated by Ln.
 * Pn     Causes a silence (pause) for the length of note indicated (same as Ln).
 * Tn     Sets the number of "L4"s per minute (tempo). Valid values are from 32 to 255. The default value is T120.
 * .      When placed after a note, it causes the duration of the note to be 3/2 of the set duration.
 *        This is how to get "dotted" notes. "L4 C#." would play C sharp as a dotted quarter note.
 *        It can be used for a pause as well.
 *
 * Extensions/variations:
 *
 * MB MF  The MF command causes the tune to play once and then stop. The MB command causes the
 *        tune to repeat when it ends.
 *
 */

#include <px4_config.h>
#include <px4_log.h>
#include <debug.h>

#include <drivers/device/device.h>
#include <drivers/drv_tone_alarm.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <ctype.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <systemlib/err.h>
#include <circuit_breaker/circuit_breaker.h>

#include <px4_workqueue.h>

#include <lib/tunes/tunes.h>
#include <uORB/uORB.h>
#include <uORB/topics/tune_control.h>

#include "ar_gpio.h"
#include "ar_tim.h"
#include "chip/ar_tim.h"
#include "chip/ar_config.h"


#define TONE_ALARM_TIMER_BASE   AR_TIM0_BASE

/*
 * Timer register accessors
 */
#define REG_TONE(_reg)	    (TONE_ALARM_TIMER_BASE + _reg)

#define rTIMERLOADCOUNT_TONE        REG_TONE(AR_TIM_TIMER0LOADCOUNT_OFFSET   )
#define rTIMERLOADCOUNT2_TONE       REG_TONE(AR_TIM_TIMER0LOADCOUNT2_OFFSET  )
#define rTIMERCURRENTVALUE_TONE     REG_TONE(AR_TIM_TIMER0CURRENTVALUE_OFFSET)
#define rTIMERCONTROLREG_TONE       REG_TONE(AR_TIM_TIMER0CONTROLREG_OFFSET  )
#define rTIMEREOI_TONE              REG_TONE(AR_TIM_TIMER0EOI_OFFSET         )
#define rTIMERINTSTATUS_TONE        REG_TONE(AR_TIM_TIMER0INTSTATUS_OFFSET   )

#define TONE_ALARM_CLOCK            AR_BUS_CLK


#define CBRK_BUZZER_KEY 782097

class ToneAlarm : public device::CDev
{
public:
	_EXT_ITCM ToneAlarm();
	_EXT_ITCM ~ToneAlarm();

	_EXT_ITCM virtual int init();
	_EXT_ITCM void status();

	enum {
		CBRK_OFF = 0,
		CBRK_ON,
		CBRK_UNINIT
	};

private:
	volatile bool _running;
	volatile bool _should_run;
	bool _play_tone;

	Tunes _tunes;

	unsigned _silence_length; // if nonzero, silence before next note

	int _cbrk; ///< if true, no audio output
	int _tune_control_sub;

	tune_control_s _tune;

	static work_s _work;

	// Convert a frequency value into a divisor for the configured timer's clock.
	//
	_EXT_ITCM unsigned frequency_to_divisor(unsigned frequency);

	// Start playing the note
	//
	_EXT_ITCM void start_note(unsigned frequency);

	// Stop playing the current note and make the player 'safe'
	//
	_EXT_ITCM void stop_note();

	// Parse the next note out of the string and play it
	//
	_EXT_ITCM void next_note();

	// work queue trampoline for next_note
	//
	_EXT_ITCM static void next_trampoline(void *arg);

};

struct work_s ToneAlarm::_work = {};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int tone_alarm_main(int argc, char *argv[]);


ToneAlarm::ToneAlarm() :
	CDev("tone_alarm", TONEALARM0_DEVICE_PATH),
	_running(false),
	_should_run(true),
	_play_tone(false),
	_tunes(),
	_silence_length(0),
	_cbrk(CBRK_UNINIT),
	_tune_control_sub(-1)
{
	// enable debug() calls
	//_debug_enabled = true;
}

ToneAlarm::~ToneAlarm()
{
	_should_run = false;
	int counter = 0;

	while (_running && ++counter < 10) {
		usleep(100000);
	}
}

int ToneAlarm::init()
{
	int ret;

	ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	/* configure the GPIO to the idle state */
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);

	/* clock/power on our timer */
	// AR8020 do nothing

	/* initialise the timer */
	/*freerun timer*/
	putreg32(AR_TIMERCONTROLREG_ENABLE_DISABLE, rTIMERCONTROLREG_TONE);

	/* clear cnt */
	putreg32(0x0000FFFF, rTIMERLOADCOUNT_TONE);

	/* clear cnt2 */
	putreg32(0x0000FFFF, rTIMERLOADCOUNT2_TONE);

	getreg32(rTIMEREOI_TONE);	/* clear inturput */

	putreg32(AR_TIMERCONTROLREG_PWM_DISABLE	| \
		 AR_TIMERCONTROLREG_INTMASK_MASKED 	| \
		 AR_TIMERCONTROLREG_MODE_USERDEFINED | \
		 AR_TIMERCONTROLREG_ENABLE_ENABLE, \
		 rTIMERCONTROLREG_TONE);

	DEVICE_DEBUG("ready");

	_running = true;
	work_queue(HPWORK, &_work, (worker_t)&ToneAlarm::next_trampoline, this, 0);
	return OK;
}

void ToneAlarm::status()
{
	if (_running) {
		PX4_INFO("running");

	} else {
		PX4_INFO("stopped");
	}
}

unsigned ToneAlarm::frequency_to_divisor(unsigned frequency)
{
	float period = 0.5f / frequency;

	// and the divisor, rounded to the nearest integer
	unsigned divisor = (period * TONE_ALARM_CLOCK) + 0.5f;

	return divisor;
}

void ToneAlarm::start_note(unsigned frequency)
{
	// check if circuit breaker is enabled
	if (_cbrk == CBRK_UNINIT) {
		_cbrk = circuit_breaker_enabled("CBRK_BUZZER", CBRK_BUZZER_KEY);
	}

	if (_cbrk != CBRK_OFF) { return; }

	// compute the divisor
	unsigned divisor = frequency_to_divisor(frequency);

	// calculate the timer period for the selected prescaler value
	unsigned period = (divisor / 2) - 1;

	putreg32(period, rTIMERLOADCOUNT_TONE);
	putreg32(period, rTIMERLOADCOUNT2_TONE);

	putreg32(AR_TIMERCONTROLREG_PWM_ENABLE	| \
		 AR_TIMERCONTROLREG_INTMASK_MASKED 	| \
		 AR_TIMERCONTROLREG_MODE_USERDEFINED | \
		 AR_TIMERCONTROLREG_ENABLE_ENABLE, \
		 rTIMERCONTROLREG_TONE);

	// configure the GPIO to enable timer output
	px4_arch_configgpio(GPIO_TONE_ALARM);
}

void ToneAlarm::stop_note()
{
	/* stop the current note */
	putreg32(AR_TIMERCONTROLREG_PWM_DISABLE	| \
		 AR_TIMERCONTROLREG_INTMASK_MASKED 	| \
		 AR_TIMERCONTROLREG_MODE_USERDEFINED | \
		 AR_TIMERCONTROLREG_ENABLE_ENABLE, \
		 rTIMERCONTROLREG_TONE);

	/*
	 * Make sure the GPIO is not driving the speaker.
	 */
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);
}

void ToneAlarm::next_note()
{
	if (!_should_run) {
		if (_tune_control_sub >= 0) {
			orb_unsubscribe(_tune_control_sub);
		}

		_running = false;
		return;
	}

	// subscribe to tune_control
	if (_tune_control_sub < 0) {
		_tune_control_sub = orb_subscribe(ORB_ID(tune_control));
	}

	// do we have an inter-note gap to wait for?
	if (_silence_length > 0) {
		stop_note();
		work_queue(HPWORK, &_work, (worker_t)&ToneAlarm::next_trampoline, this, USEC2TICK(_silence_length));
		_silence_length = 0;
		return;
	}

	// check for updates
	bool updated = false;
	orb_check(_tune_control_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(tune_control), _tune_control_sub, &_tune);

		if (_tunes.set_control(_tune) == 0) {
			_play_tone = true;

		} else {
			_play_tone = false;
		}
	}

	unsigned frequency = 0, duration = 0;

	if (_play_tone) {
		_play_tone = false;
		int parse_ret_val = _tunes.get_next_tune(frequency, duration, _silence_length);

		if (parse_ret_val >= 0) {
			// a frequency of 0 correspond to stop_note
			if (frequency > 0) {
				// start playing the note
				start_note(frequency);

			} else {
				stop_note();
			}

			if (parse_ret_val > 0) {
				// continue playing
				_play_tone = true;
			}
		}

	} else {
		// schedule a call with the tunes max interval
		duration = _tunes.get_maximum_update_interval();
		// stop playing the last note after the duration elapsed
		stop_note();
	}

	// and arrange a callback when the note should stop
	work_queue(HPWORK, &_work, (worker_t)&ToneAlarm::next_trampoline, this, USEC2TICK(duration));
}

void ToneAlarm::next_trampoline(void *arg)
{
	ToneAlarm *ta = (ToneAlarm *)arg;
	ta->next_note();
}

/**
 * Local functions in support of the shell command.
 */
namespace
{

ToneAlarm	*g_dev;

} // namespace

void tone_alarm_usage();

_EXT_ITCM void tone_alarm_usage()
{
	PX4_INFO("missing command, try 'start', status, 'stop'");
}

_EXT_ITCM int tone_alarm_main(int argc, char *argv[])
{

	if (argc > 1) {
		const char *argv1 = argv[1];

		if (!strcmp(argv1, "start")) {
			if (g_dev != nullptr) {
				PX4_ERR("already started");
				exit(1);
			}

			if (g_dev == nullptr) {
				g_dev = new ToneAlarm();

				if (g_dev == nullptr) {
					PX4_ERR("couldn't allocate the ToneAlarm driver");
					exit(1);
				}

				if (OK != g_dev->init()) {
					delete g_dev;
					g_dev = nullptr;
					PX4_ERR("ToneAlarm init failed");
					exit(1);
				}
			}

			exit(0);
		}

		if (!strcmp(argv1, "stop")) {
			delete g_dev;
			g_dev = nullptr;
			exit(0);
		}

		if (!strcmp(argv1, "status")) {
			g_dev->status();
			exit(0);
		}

	}

	tone_alarm_usage();
	exit(0);
}
