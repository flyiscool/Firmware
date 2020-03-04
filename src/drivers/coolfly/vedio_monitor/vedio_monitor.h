#ifndef COOLFLY_CPU2_H_
#define COOLFLY_CPU2_H_

#include <stddef.h>
#include <stdint.h>
#include <chip/ar_config.h>



#define IT66021A_RST_PIN		25
#define IT66021A_INT_PIN		27

#define USE_IT66021_EDID_CONFIG_BIN


class VEDIO_MONITOR : public device::I2C, public ModuleBase<VEDIO_MONITOR>
{
public:
	_EXT_ITCM VEDIO_MONITOR(const char *name, const char *devname,
				int bus, uint16_t address, uint32_t frequency);

	_EXT_ITCM ~VEDIO_MONITOR();

	_EXT_ITCM virtual int init();

	_EXT_ITCM static int custom_command(int argc, char *argv[]);

	_EXT_ITCM static int task_spawn(int argc, char *argv[]);

	_EXT_ITCM static int print_usage(const char *reason = nullptr);

	_EXT_ITCM static VEDIO_MONITOR *instantiate(int argc, char *argv[]);

	_EXT_ITCM void run() override;

	_EXT_ITCM int read(unsigned address, void *data, unsigned count);
	_EXT_ITCM int write(unsigned address, void *data, unsigned count);
	_EXT_ITCM unsigned char readbyte(unsigned char address);
	_EXT_ITCM unsigned char writebyte(unsigned char address, unsigned char data);

private:
	_EXT_ITCM static void it66021_polling(void *arg);


};


#endif