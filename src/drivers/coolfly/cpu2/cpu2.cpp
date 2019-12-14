/****************************************************************************
 *
 *   Copyright (C) 2012-2016 PX4 Development Team. All rights reserved.
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

#include <string.h>
#include <px4_config.h>
#include <px4_module.h>

#include <sys/types.h>
#include <systemlib/err.h>
#include <drivers/device/i2c.h>
#include <nuttx/irq.h>
#include <px4_workqueue.h>
#include <px4_getopt.h>
#include <drivers/drv_intercore.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <uORB/uORB.h>
#include <uORB/topics/h264_input_format.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/led_control.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>

#include "cpu2.h"
#include <board_config.h>


static void cycle_trampoline(void *arg);

uint8_t arsys_event_cpu0t2_append(AR_INTERCORE_EVENT *message);

int tolower(int c);

int htoi(char s[]);

static uint8_t needlogging = 0;

static struct work_s _work = {};

// static struct work_s h264_work = {};

static int h264_fd = 0;

static struct h264_input_format_s att = {0};

static orb_advert_t led_control_pub = nullptr;


class CPU2 : public device::CDev, public ModuleBase<CPU2>
{
public:
	_EXT_ITCM CPU2();

	_EXT_ITCM ~CPU2();

	_EXT_ITCM virtual int init();

	_EXT_ITCM static int custom_command(int argc, char *argv[]);

	_EXT_ITCM static int task_spawn(int argc, char *argv[]);

	_EXT_ITCM static int print_usage(const char *reason = nullptr);

	_EXT_ITCM static CPU2 *instantiate(int argc, char *argv[]);

	_EXT_ITCM void run() override;

	_EXT_ITCM void h264_cycle(void *arg);

private:
	_EXT_ITCM static void intercore_event_msg_cycle_trampoline(void *arg);

	_EXT_ITCM static void cycle_trampoline(void *arg);


	// _EXT_ITCM static void clog();

};




void CPU2::intercore_event_msg_cycle_trampoline(void *arg)
{
	uint8_t i = 0;
	volatile AR_INTERCORE_EVENT *msgPtr = (AR_INTERCORE_EVENT *)SRAM_INTERCORE_EVENT_CPU2T0_ST_STARTADDR;

	for (i = 0; i < SRAM_INTERCORE_EVENT_MAX_COUNT; i++) {
		if (msgPtr[i].isUsed == 1) {
			switch (msgPtr[i].type) {
			case SYS_EVENT_ID_CPU2_LOG:
				if (needlogging == 1) {
					PX4_INFO("%s", msgPtr[i].data);
				}

				msgPtr[i].isUsed = 2;
				break;

			default:
				break;
			}
		}
	}

	work_queue(LPWORK, &_work, (worker_t)&CPU2::intercore_event_msg_cycle_trampoline, nullptr, 5);
}


_EXT_ITCM static void initSram()
{
	memset((void *)SRAM_MAVLINK_RC_MSG_ST_ADDR, 0, SRAM_MAVLINK_RC_MSG_SIZE);

	STRU_SramBuffer *sramBuffer;

	sramBuffer = (STRU_SramBuffer *)SRAM_MAVLINK_RC_MSG_ST_ADDR;
	sramBuffer->header.buf_wr_pos = (uint32_t)sramBuffer->buf;
	sramBuffer->header.buf_rd_pos = (uint32_t)sramBuffer->buf;

	sramBuffer = (STRU_SramBuffer *)SRAM_SESSION1_TO_UART_DATA_ST_ADDR;
	sramBuffer->header.buf_wr_pos = (uint32_t)sramBuffer->buf;
	sramBuffer->header.buf_rd_pos = (uint32_t)sramBuffer->buf;

	sramBuffer = (STRU_SramBuffer *)SRAM_UART_TO_SESSION1_DATA_ST_ADDR;
	sramBuffer->header.buf_wr_pos = (uint32_t)sramBuffer->buf;
	sramBuffer->header.buf_rd_pos = (uint32_t)sramBuffer->buf;
}

_EXT_ITCM static void publish_led_control(led_control_s &led_control)
{
	led_control.timestamp = hrt_absolute_time();

	if (led_control_pub == nullptr) {
		led_control_pub = orb_advertise_queue(ORB_ID(led_control), &led_control, LED_UORB_QUEUE_LENGTH);

	} else {
		orb_publish(ORB_ID(led_control), led_control_pub, &led_control);
	}
}


_EXT_ITCM static void led(uint8_t color)
{
	led_control_s led_control = {};
	led_control.led_mask = 0xff;
	led_control.mode = led_control_s::MODE_OFF;
	led_control.priority = led_control_s::MAX_PRIORITY;
	publish_led_control(led_control);

	usleep(20 * 1000);
	
	// generate some pattern
	for (int round = 0; round <= 10; ++round) {
		for (int led = 0; led < BOARD_MAX_LEDS; ++led) {
			led_control.led_mask = 1 << led;
			led_control.mode = led_control_s::MODE_ON;
			led_control.color = color;
			publish_led_control(led_control);
			usleep(20 * 1000);
		}

		led_control.led_mask = 0xff;
		for (int i = 0; i < 3; ++i) {
			led_control.mode = led_control_s::MODE_ON;
			publish_led_control(led_control);
			usleep(20 * 1000);
			led_control.mode = led_control_s::MODE_OFF;
			publish_led_control(led_control);
			usleep(20 * 1000);
		}
	}

	usleep(500 * 1000);

	// reset
	led_control.led_mask = 0xff;
	led_control.mode = led_control_s::MODE_DISABLED;
	publish_led_control(led_control);

}

_EXT_ITCM void CPU2::h264_cycle(void *arg)
{
	if (h264_fd == 0) {
		h264_fd = orb_subscribe(ORB_ID(h264_input_format));
	}

	bool updated = false;

	if (orb_check(h264_fd, &updated) != PX4_OK) { return; }

	if (updated) {
		orb_copy(ORB_ID(h264_input_format), h264_fd, &att);

		AR_INTERCORE_EVENT msg;
		uint8_t length = 13;
		msg.data[0] = att.index;
		msg.data[1] = (att.width >> 8) & 0xff;
		msg.data[2] = att.width & 0xff;
		msg.data[3] = (att.hight >> 8) & 0xff;
		msg.data[4] = att.hight & 0xff;
		msg.data[5] = att.framerate;
		msg.data[6] = att.vic;
		msg.data[7] = att.e_h264InputSrc;

		msg.length = length;
		msg.type = SYS_EVENT_ID_H264_INPUT_FORMAT_CHANGE;

		bool needIndicator = (att.width != 0 && att.hight != 0 && att.framerate != 0);

		if (needIndicator) 
		{
			PX4_INFO("\r\n ----- Resolution: %d * %d p%d -----\r\n", att.width, att.hight, att.framerate);
			led(led_control_s::COLOR_GREEN);
		}
		else 
		{
			led(led_control_s::COLOR_CYAN);
		}

		arsys_event_cpu0t2_append(&msg);
	}
}

CPU2::CPU2(): CDev("cpu2", "/dev/cpu2")
{}


_EXT_ITCM uint8_t arsys_event_cpu0t2_append(AR_INTERCORE_EVENT *message)
{
	static uint8_t seq = 0;

	uint8_t i = 0;
	volatile AR_INTERCORE_EVENT *msgPtr = (AR_INTERCORE_EVENT *)SRAM_INTERCORE_EVENT_CPU0T2_ST_STARTADDR;

	for (i = 0; i < SRAM_INTERCORE_EVENT_MAX_COUNT; i++) {
		if (msgPtr[i].isUsed == 0) {
			msgPtr[i].length = message->length;
			msgPtr[i].type = message->type;
			memcpy((void *)msgPtr[i].data, message->data, message->length);
			msgPtr[i].isUsed = 1;
			msgPtr[i].seq = seq++;

			break;
		}
	}

	if (i == SRAM_INTERCORE_EVENT_MAX_COUNT) {
		for (i = 0; i < SRAM_INTERCORE_EVENT_MAX_COUNT; i++) {
			msgPtr[i].isUsed = 0;
		}

		arsys_event_cpu0t2_append(message);
	}

	return 0;
}


CPU2::~CPU2() {}

int CPU2::init()
{
	int ret;

	if ((ret = CDev::init()) != PX4_OK) {
		PX4_ERR("CPU2::init fail");
		return ret;
	}

	px4_flash_init();

	initSram();

	return PX4_OK;
}

int tolower(int c)
{
	if (c >= 'A' && c <= 'Z') {
		return c + 'a' - 'A';

	} else {
		return c;
	}
}

int htoi(char s[])
{
	int i;
	int n = 0;

	if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
		i = 2;

	} else {
		i = 0;
	}


	for (; (s[i] >= '0' && s[i] <= '9') || (s[i] >= 'a' && s[i] <= 'z') || (s[i] >= 'A' && s[i] <= 'Z'); ++i) {
		if (tolower(s[i]) > '9') {
			n = 16 * n + (10 + tolower(s[i]) - 'a');

		} else {
			n = 16 * n + (tolower(s[i]) - '0');
		}
	}

	return n;
}


int CPU2::task_spawn(int argc, char *argv[])
{
	// int ret;

	_task_id = px4_task_spawn_cmd("cpu2",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_ACTUATOR_OUTPUTS,
					      1800,
					      (px4_main_t)&run_trampoline,
					      nullptr);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	if (wait_until_running() < 0) {
		_task_id = -1;
		return -1;
	}

	return PX4_OK;
}

void CPU2::run()
{
	if (init() != 0) {
		PX4_ERR("init failed");
		exit_and_cleanup();
		return;
	}

	intercore_event_msg_cycle_trampoline(NULL);

	while(true) 
	{
		h264_cycle(NULL);
		
		usleep(1000 * 1000);
	}
}


_EXT_ITCM  CPU2 *CPU2::instantiate(int argc, char *argv[]) {
	return new CPU2();
}


int CPU2::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("cpu2", "driver");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("setid", "cpu2 setid ab cd ef 12 34 aa bb");
	PRINT_MODULE_USAGE_COMMAND_DESCR("log", "'cpu2 log', display  intercore cpu2 log info");
	PRINT_MODULE_USAGE_COMMAND_DESCR("blind", "'cpu2 blind', not display  intercore cpu2 log info");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return OK;
}


int CPU2::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (strcmp(verb, "setid") == 0) {
		if (argc < 8) {
			print_usage();

		} else {
			uint8_t buffer[7] = {0};

			buffer[0] = htoi(argv[1]);
			buffer[1] = htoi(argv[2]);
			buffer[2] = htoi(argv[3]);
			buffer[3] = htoi(argv[4]);
			buffer[4] = htoi(argv[5]);
			buffer[5] = htoi(argv[6]);
			buffer[6] = htoi(argv[7]);
			px4_flash_updateid(buffer, sizeof(buffer), 2);
		}

	} else if (strcmp(verb, "log") == 0) {
		needlogging = 1;

	} else if (strcmp(verb, "blind") == 0) {
		needlogging = 0;

	} else {

		print_usage();
	}

	return PX4_OK;
}



extern "C" __EXPORT int cpu2_main(int argc, char *argv[]);

_EXT_ITCM int cpu2_main(int argc, char *argv[])
{
	return CPU2::main(argc, argv);
}

