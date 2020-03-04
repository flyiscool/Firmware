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

#include "vedio_monitor.h"
#include <board_config.h>


static struct work_s _work = {};

static bool  flag_it66021a = false;

// static struct work_s h264_work = {};

class VEDIO_MONITOR : public device::CDev, public ModuleBase<VEDIO_MONITOR>
{
public:
	_EXT_ITCM VEDIO_MONITOR();

	_EXT_ITCM ~VEDIO_MONITOR();

	_EXT_ITCM virtual int init();

	_EXT_ITCM static int custom_command(int argc, char *argv[]);

	_EXT_ITCM static int task_spawn(int argc, char *argv[]);

	_EXT_ITCM static int print_usage(const char *reason = nullptr);

i	_EXT_ITCM static int print_status() override;

	_EXT_ITCM static VEDIO_MONITOR *instantiate(int argc, char *argv[]);

	_EXT_ITCM void run() override;

private:
	_EXT_ITCM static void it66021_polling(void *arg);


};


VEDIO_MONITOR::VEDIO_MONITOR(): CDev("vedio_monitor", "/dev/vedio_monitor")
{}

VEDIO_MONITOR::~VEDIO_MONITOR() {}


int VEDIO_MONITOR::init()
{
	int ret;

	if ((ret = CDev::init()) != PX4_OK) {
		PX4_ERR("VEDIO_MONITOR::init fail");
		return ret;
	}

	return PX4_OK;
}


int VEDIO_MONITOR::task_spawn(int argc, char *argv[])
{
	PX4_INFO("test task_spawn");
	_task_id = px4_task_spawn_cmd("vedio_monitor",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_SLOW_DRIVER,
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




void VEDIO_MONITOR::it66021_polling(void *arg)
{
	PX4_INFO("it66021_polling");
	usleep(1000*1000);
	work_queue(LPWORK, &_work, (worker_t)&VEDIO_MONITOR::it66021_polling, nullptr, 5);
}

void VEDIO_MONITOR::run()
{
	PX4_INFO("test run");
	if (init() != 0) {
		PX4_ERR("init failed");
		exit_and_cleanup();
		return;
	}

	

	it66021_polling(NULL);

	while(true) 
	{		
		PX4_INFO("test run");
		usleep(1000 * 1000);
	}
}


_EXT_ITCM  VEDIO_MONITOR *VEDIO_MONITOR::instantiate(int argc, char *argv[]) {
	return new VEDIO_MONITOR();
}


int VEDIO_MONITOR::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("vedio_monitor", "driver");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("setid", "vedio_monitor setid ab cd ef 12 34 aa bb");
	PRINT_MODULE_USAGE_COMMAND_DESCR("log", "'vedio_monitor log', display  intercore vedio_monitor log info");
	PRINT_MODULE_USAGE_COMMAND_DESCR("blind", "'vedio_monitor blind', not display  intercore vedio_monitor log info");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return OK;
}


void VEDIO_MONITOR::print_status()
{
	if(flag_it66021a == true)
	{
		PX4_INFO("IT66021A is running");
	}
	
}


int VEDIO_MONITOR::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (strcmp(verb, "it66021A") == 0) {

		PX4_INFO("TEST IT66021A");
		flag_it66021a = true;
	} 
	else {

		print_usage();
	}

	return PX4_OK;
}








extern "C"  int vedio_monitor_main(int argc, char *argv[]);

_EXT_ITCM int vedio_monitor_main(int argc, char *argv[])
{
	return VEDIO_MONITOR::main(argc, argv);
}

