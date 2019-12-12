/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
#include <px4_workqueue.h>
#include <px4_getopt.h>
#include "it66021.h"
#include "it66021_i2c.h"
#include "px4_log.h"
#include "it66021_reg.h"
#include "it66021_config.h"

// static IT66021 *it66021_B = nullptr;
static I2CARG IT66021_I2CARG_A = {
	.name 		= "it66021",
	.devname 	= "/dev/it66021_A",
	.bus 		= PX4_I2C_BUS_IT66021_A,
	.address	= IT6602A0_HDMI_ADDR >> 1,
					     .frequency	= IT66021_DEFAULT_BUS_SPEED
};

static I2CARG IT66021_I2CARG_A_EDID = {
	.name 		= "it66021_edid",
	.devname 	= "/dev/it66021_A_edid",
	.bus 		= PX4_I2C_BUS_IT66021_A_EDID,
	.address	= IT66021A_HDMI_ADDR_EDID >> 1,
			.frequency	= IT66021_DEFAULT_BUS_SPEED
};


static IT66021_BUS_ARG IT66021_BUS_ARG_A = {
	.it66021arg = &IT66021_I2CARG_A,
	.edidarg 	= &IT66021_I2CARG_A_EDID,
	.type 		= IT66021TYPE_A,
	.dev        = nullptr
};

static IT66021 *it66021_A 	= new IT66021(*IT66021_BUS_ARG_A.it66021arg);
static EDID *it66021_A_EDID = new EDID(*IT66021_BUS_ARG_A.edidarg);
static bool it66021_i2c_intialized = false;

/////////////////// TODO //////////////////////
unsigned char IT66021::mhlrxrd(unsigned char offset)
{
	return 0;
	// uint8_t value = 0;
	// // (*(this->ring))->read(offset, &value, 1);
	// if (it66021_A_RING->read(offset, &value, 1) != OK)
	// {
	// 	IT_INFO("ring read failure");
	// }
	// return value;
}

unsigned char IT66021::mhlrxwr(unsigned char offset, unsigned char ucdata)
{
	return 0;

	// if (it66021_A_RING->write(offset, &ucdata, 1) != OK)
	// {
	// 	IT_INFO("mhl writ offset = %02x, data = %d failure", offset, ucdata);
	// }
	// // return (*this->ring)->write(offset, &ucdata, 1);
	// return OK;
	// return IT_66021_WriteByte(MHL_ADDR, offset, ucdata);
}


//////////////////////////////////////////////////////////////////


IT66021::IT66021(I2CARG arg) : I2C(arg.name, arg.devname, arg.bus, arg.address,  arg.frequency), work{}
{
	this->s_st_hdmiRxStatus.st_configure.e_getFormatMethod = HAL_HDMI_INTERRUPT;
	this->s_st_hdmiRxStatus.st_configure.e_colorDepth = HAL_HDMI_RX_8BIT;
	this->s_st_hdmiRxStatus.st_configure.u8_hdmiToEncoderCh = 1;
	// this->devtype = dev;
	// memset(&work, 0, sizeof(work));
}

IT66021::~IT66021()
{
	IT_INFO("~deinit for it66021 \r\n");
}

int IT66021::init()
{
	// PX4_LOG("IT66021::init");
	int ret;
	ret = I2C::init();

	if (ret != OK) {
		IT_INFO("ret != OK\r\n");
		return ret;
	}

	// 100 Microseconds Timer Calibration
	usleep(100 * 1000);

	return OK;
}

int IT66021::probe()
{
	// PX4_LOG("IT66021::probe");

	uint8_t data[1] = {0};

	_retries = 4;

	uint8_t res;

	res = read(REG_RX_000, &data[0], 1);
	IT_INFO("res = %d, data = %02x", res, data[0]);


	if (res != OK || data[0] != 0x54) {
		IT_INFO("read_reg1 fail \r\n");
		return -EIO;
	}

	res = read(REG_RX_001, &data[0], 1);

	if (res != OK || data[0] != 0x49) {
		IT_INFO("read_reg2 fail \r\n");
		return -EIO;
	}

	res = read(REG_RX_002, &data[0], 1);

	if (res != OK || data[0] != 0x02) {
		IT_INFO("read_re3g fail \r\n");
		return -EIO;
	}

	res = read(REG_RX_003, &data[0], 1);

	if (res != OK || data[0] != 0x68) {
		IT_INFO("read_reg4 fail \r\n");
		return -EIO;
	}

	// if (read(REG_RX_034, &data[0], 1))
	// {
	// 	IT_INFO("read_re6g fail \r\n");
	// 	return -EIO;
	// }
	// PX4_LOG("reg = 0%02x value = 0x%02x", REG_RX_034, data[0]);

	// if (read(REG_RX_086, &data[0], 1))
	// {
	// 	IT_INFO("read_reg fail \r\n");
	// 	return -EIO;
	// }
	// PX4_LOG("reg = 0%02x value = 0x%02x", REG_RX_086, data[0]);

	// if (read(REG_RX_087, &data[0], 1))
	// {
	// 	IT_INFO("read_reg fail \r\n");
	// 	return -EIO;
	// }
	// PX4_LOG("reg = 0%02x value = 0x%02x", REG_RX_087, data[0]);

	_retries = 10;

	return OK;
}

int IT66021::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;

	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}

int IT66021::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}

SYS_STATUS IT66021::EDID_RAM_Write(unsigned char offset, unsigned char byteno, _CODE unsigned char *p_data)
{
	// EDID *edit= *this->edid;
	EDID *edit = it66021_A_EDID;

	if (edit->write(offset, p_data, byteno) == OK) {
		uint8_t value = EDID_RAM_Read(offset);

		if (value != p_data[0]) {
			IT_INFO("edit write address = 0x%02x, value = 0x%02x \n", offset, p_data[0]);
		}

		return ER_SUCCESS;
	};

	PX4_LOG("edit write address = 0x%02x, value = 0x%02x \n", offset, p_data[0]);

	// TODO：
	// IT_66021_WriteBytes(RX_I2C_EDID_MAP_ADDR, offset, byteno, p_data);
	return ER_FAIL;
}

unsigned char IT66021::EDID_RAM_Read(unsigned char offset)
{
	uint8_t value = 0;
	EDID *edit = it66021_A_EDID; //*this->edid;
	edit->read(offset, &value, 1);

	return value;
	// TODO：
	// return IT_66021_ReadByte(RX_I2C_EDID_MAP_ADDR,offset);
	// return 0;
}

unsigned char IT66021::hdmirxrd(unsigned char address)
{
	// IT_66021_ReadByte(HdmiI2cAddr,RegAddr);
	uint8_t data[1] = {0};

	if (read(address, &data[0], 1)) {
		IT_INFO("hdmirxrd %x \n", address);

		return 0;
	}

	return data[0];
}

unsigned char IT66021::hdmirxwr(unsigned char address, unsigned char data)
{
	unsigned char wres = write(address, &data, 1);
	unsigned char rdrew = hdmirxrd(address);

	if (rdrew != data) {
		IT_INFO("address = %x, write data = %02x, rddata = %02x", address, data, rdrew);
	}

	return wres;
}

unsigned char IT66021::hdmirxset(unsigned char offset, unsigned char mask, unsigned char ucdata)
{
	unsigned char temp;
	temp = hdmirxrd(offset);
	temp = (temp & ((~mask) & 0xFF)) + (mask & ucdata);
	return hdmirxwr(offset, temp);
}


void IT66021::cycle_trampoline(void *arg)
{
	IT66021 *dev = (IT66021 *)arg;
	if (ar_gpioread(27) == 0) {
		IT_INFO("\r\n------------------------------\r\n");

		dev->IT6602_Interrupt();
		dev->IT6602_fsm();
		dev->HDMI_RX_CheckFormatStatus(HAL_HDMI_RX_1, HAL_HDMI_RX_FALSE);
	}
	else if (ar_gpioread(68) == 0) { }

	work_queue(LPWORK, &dev->work, (worker_t)&IT66021::cycle_trampoline, dev, USEC2TICK(1000 * 1000));
}


int IT66021::task_spawn(int argc, char *argv[])
{
	char dev = NULL;
	bool error_flag = false;
	IT66021_BUS_ARG bus_option;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			dev = *myoptarg;
			break;
			
		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag)  {
		return -1;
	}

	if (dev == 'a') {
		bus_option = IT66021_BUS_ARG_A;
		bus_option.dev = it66021_A;
		it66021_A->edid = it66021_A_EDID;

	} else {
		return -1;
	}

	IT66021 *interface = bus_option.dev;

	if (interface->init() != OK) {
		return false;
	}

	// EDID
	interface->hdmirxset(REG_RX_0C0, 0x43, 0x40);
	interface->hdmirxset(REG_RX_087, 0xFF, bus_option.edidarg->address | 0x01);

	EDID *edid = interface->edid;

	if (edid->init() != OK) {
		return false;
	}

	it66021_A_EDID = edid;

	interface->it66021_init();

	int ret = work_queue(LPWORK, &interface->work, (worker_t)&IT66021::cycle_trampoline, interface, USEC2TICK(1000));

	if (ret < 0) {
		return ret;
	}

	_task_id = task_id_is_work_queue;
	
	return true;
}


int IT66021::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("it66021", "driver");

	PRINT_MODULE_USAGE_COMMAND("start");

	return OK;
}


int IT66021::custom_command(int argc, char *argv[])
{
	print_usage();

	return PX4_OK;
}


extern "C" __EXPORT int it66021_main(int argc, char *argv[]);
_EXT_ITCM int it66021_main(int argc, char *argv[])
{
	return IT66021::main(argc, argv);
}
