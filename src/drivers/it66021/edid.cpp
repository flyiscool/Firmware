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
#include "edid.h"
#include "px4_log.h"


EDID::EDID(I2CARG arg) : I2C(arg.name, arg.devname, arg.bus, arg.address,  arg.frequency) 
{
    PX4_INFO("EDID INIT \r\n");
}

EDID::~EDID()
{
	PX4_INFO("~deinit for it66021_EDID \r\n");
}

int EDID::init()
{
	PX4_LOG("EDID::init");

	int ret;
	ret = I2C::init();

	if (ret != OK)
	{
		PX4_INFO("ret != OK\r\n");
		return ret;
	}

	// 100 Microseconds Timer Calibration

	return OK;
}

int EDID::probe()
{
	PX4_LOG("EDID::probe");


	// default OK
	// uint8_t data[1] = {0};

	// _retries = 4;

	// if (read(0x00, &data[0], 1))
	// {
	// 	PX4_INFO("read_reg fail \r\n");
	// 	return -EIO;
	// }
	// PX4_LOG("reg = 0%02x value = 0x%02x", REG_RX_000, data[0]);

	_retries = 10;

	return OK;
}



int EDID::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1))
	{
		return -EIO;
	}
	buf[0] = address;

	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}

int EDID::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}