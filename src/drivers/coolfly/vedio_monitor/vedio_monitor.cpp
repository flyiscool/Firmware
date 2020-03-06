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

#include "it66021.h"
#include "it66021_reg.h"
#include "it66021_type.h"
#include "chip/ar_define.h"

static struct work_s _work = {};

STRU_HDMI_RX_STATUS s_st_hdmiRxStatus;

extern struct IT6602_REG_INI  IT6602_HDMI_INIT_TABLE[];

VEDIO_MONITOR *p_it66021a;
VEDIO_MONITOR *p_it66021a_edid;
VEDIO_MONITOR *p_it66021a_mhl;


VEDIO_MONITOR::VEDIO_MONITOR(const char *name, const char *devname,
			     int bus, uint16_t address, uint32_t frequency):
	I2C(name, devname, bus, address, frequency)
{
}


VEDIO_MONITOR::~VEDIO_MONITOR() {}


int VEDIO_MONITOR::init()
{
	int ret;

	if ((ret = I2C::init()) != PX4_OK) {
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


_EXT_ITCM  VEDIO_MONITOR *VEDIO_MONITOR::instantiate(int argc, char *argv[])
{
	return new VEDIO_MONITOR("vedio_monitor",
				 "/dev/vedio_monitor",
				 PX4_I2C_IT66021_A,
				 0x00,
				 100000);
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


int VEDIO_MONITOR::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (strcmp(verb, "it66021A") == 0) {
		PX4_INFO("TEST IT66021A cmd");

	} else {
		print_usage();
	}

	return PX4_OK;
}


int VEDIO_MONITOR::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		PX4_INFO("buffer is to small!");
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}

int VEDIO_MONITOR::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}


unsigned char VEDIO_MONITOR::readbyte(unsigned char address)
{
	// IT_66021_ReadByte(HdmiI2cAddr,RegAddr);
	uint8_t data[1] = {0};

	if (read(address, &data[0], 1)) {
		PX4_INFO("readbyte error. addr =  %x \n", address);
		return 0;
	}

	return data[0];
}

unsigned char VEDIO_MONITOR::writebyte(unsigned char address, unsigned char data)
{
	return write(address, &data, 1);
}







extern "C"  int vedio_monitor_main(int argc, char *argv[]);

_EXT_ITCM int vedio_monitor_main(int argc, char *argv[])
{
	return VEDIO_MONITOR::main(argc, argv);
}


///////////////////////////////////////

void VEDIO_MONITOR::it66021_polling(void *arg)
{

	if (ar_gpioread(27) == 0) {
		IT6602_Interrupt();

    	IT6602_fsm(); 

		HDMI_RX_CheckFormatStatus();

	}


	if (is_running()) {
		work_queue(LPWORK, &_work, (worker_t)&VEDIO_MONITOR::it66021_polling, nullptr, USEC2TICK(1000 * 1000));
	}
}

void VEDIO_MONITOR::run()
{

	while (!is_running()) {
		usleep(10 * 1000);
	}

	p_it66021a = new VEDIO_MONITOR("it66021a",
				       "/dev/it66021a",
				       PX4_I2C_IT66021_A,
				       IT66021A_HDMI_ADDR >> 1,
				       100000);

	ar_gpiowrite(IT66021A_RST_PIN, 0);
	usleep(100 * 1000);
	ar_gpiowrite(IT66021A_RST_PIN, 1);
	usleep(10 * 1000);

	// init i2c
	int ret = p_it66021a->init();

	if (ret != OK) {
		PX4_INFO("p_it66021a i2c init fail \r\n");
		return ;
	}

	// test 66021
	uint8_t probe_id = p_it66021a->readbyte(REG_RX_000);

	if (probe_id != 0x54) {
		PX4_INFO("can't find the 66021\r\n");
		return;
	}

	p_it66021a_edid = new VEDIO_MONITOR("it66021a_edid",
				       "/dev/it66021a_edid",
				       PX4_I2C_IT66021_A,
				       EDID_ADDR >> 1,
				       100000);


	ret = p_it66021a_edid->init();

	if (ret != OK) {
		PX4_INFO("p_it66021a_edid i2c init fail \r\n");
		return ;
	}				   

	p_it66021a_mhl = new VEDIO_MONITOR("it66021a_mhl",
				       "/dev/it66021a_mhl",
				       PX4_I2C_IT66021_A,
				       MHL_ADDR >> 1,
				       100000);

	ret = p_it66021a_mhl->init();

	if (ret != OK) {
		PX4_INFO("p_it66021a_mhl i2c init fail \r\n");
		return ;
	}		


	it66021_init();

	it66021_polling(NULL);

	while (true) {
		usleep(1000 * 1000);
	}

}

