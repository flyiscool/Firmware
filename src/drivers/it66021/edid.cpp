
#include <string.h>
#include "edid.h"
#include "px4_log.h"

EDID::EDID(I2CARG arg) : I2C(arg.name, arg.devname, arg.bus, arg.address,  arg.frequency)
{}

EDID::~EDID()
{}

int EDID::init()
{
	int ret;
	ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	return OK;
}

int EDID::probe()
{
	return OK;
}

int EDID::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
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