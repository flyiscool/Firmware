
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


static int fd = -1;

class Can : public ModuleBase<Can>
{
public:
	_EXT_ITCM static int custom_command(int argc, char *argv[]);

	_EXT_ITCM static int task_spawn(int argc, char *argv[]);

	_EXT_ITCM static int print_usage(const char *reason = nullptr);

	_EXT_ITCM void run() override;

	_EXT_ITCM static Can *instantiate(int argc, char *argv[]);

};


_EXT_ITCM Can *Can::instantiate(int argc, char *argv[]) 
{
	return new Can();
}

void Can::run()
{
	if ((fd = open("/dev/can0", O_RDWR)) < 0) {
		PX4_INFO("open file \n");
	}

	PX4_INFO("CAN RUN \r\n");


	while(1)
	{
		struct can_msg_s msgs;

		msgs.cm_hdr.ch_id = 0x01;
		msgs.cm_hdr.ch_dlc = 0x04;
		msgs.cm_hdr.ch_rtr = 0x00;

		msgs.cm_data[0] = 0x10;
		msgs.cm_data[1] = 0x11;
		msgs.cm_data[2] = 0x12;
		msgs.cm_data[3] = 0x13;

		write(fd, (uint8_t *)&msgs, sizeof(msgs));

		usleep(1000 * 1000);

	}

	PX4_INFO("CAN END \n");
}



int Can::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("can",
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
	return  OK;
}

