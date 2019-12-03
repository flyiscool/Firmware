
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
#include <drivers/device/device.h>
#include <nuttx/can/can.h>
#include <board_config.h>
#include <px4_workqueue.h>
#include <nuttx/fs/fs.h>


static struct work_s _work = {};

static int fd;

class Can : public ModuleBase<Can>
{
public:
	_EXT_ITCM static int custom_command(int argc, char *argv[]);

	_EXT_ITCM static int task_spawn(int argc, char *argv[]);

	_EXT_ITCM static int print_usage(const char *reason = nullptr);

private:
	_EXT_ITCM static void cycle_trampoline(void *arg);


};
 

void Can::cycle_trampoline(void *arg)
{
	Can *dev = reinterpret_cast<Can *>(arg);

	if (dev == nullptr) {

		if ((dev = new Can()) == NULL) {
			PX4_ERR("alloc failed");
			return;
		}

		_object = dev;
	}

	work_queue(LPWORK, &_work, (worker_t)&Can::cycle_trampoline, nullptr, 10);


	// dev->intercore_event_msg_cycle_trampoline(NULL);
	// dev->h264_cycle(NULL);
}

int Can::task_spawn(int argc, char *argv[])
{
	int ret;
	if ((ret = work_queue(LPWORK, &_work, (worker_t)&Can::cycle_trampoline, nullptr, 0)) < 0) {
		return ret;
	}

	_task_id = task_id_is_work_queue;

	if ((fd = open("/dev/can0", O_RDWR)) < 0) {
		PX4_INFO("open file \n");
	}



	// if ((file = fs_getfilep(fd)) == NULL) {
	// 	PX4_INFO("get filep fail \n");
	// 	return;
	// }
		
	return PX4_OK;
}


int Can::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("Can", "driver");

	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return OK;
}

int Can::custom_command(int argc, char *argv[]) 
{
	return PX4_OK;
}

extern "C" __EXPORT int can_main(int argc, char *argv[]);

_EXT_ITCM int can_main(int argc, char *argv[])
{
	Can::main(argc, argv);

	while (1)
	{
		usleep(100);
	}

	return  OK;
}

