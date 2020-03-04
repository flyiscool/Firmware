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

#include "it66021_reg.h"
#include "it66021_type.h"
#include "ar_define.h"

static struct work_s _work = {};

STRU_HDMI_RX_STATUS s_st_hdmiRxStatus;


VEDIO_MONITOR *p_it66021a;

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
	if (is_running()) {
		work_queue(LPWORK, &_work, (worker_t)&VEDIO_MONITOR::it66021_polling, nullptr, 1000);
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
				       0x92 >> 1,
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

	char ret = FALSE;

	it6602HPDCtrl(1, 0); // HDMI port , set HPD = 0
	IT_Delay(10); //for power sequence
	ret = IT6602_fsm_init();
	it6602HPDCtrl(1, 1);

	if (FALSE != ret) {
		mhlrxwr(MHL_RX_0A, 0xFF);
		mhlrxwr(MHL_RX_08, 0xFF);
		mhlrxwr(MHL_RX_09, 0xFF);

		hdmirxset(REG_RX_063, 0xFF, 0x3F);
		hdmirxset(REG_RX_012, 0xFF, 0xF8);

		DLOG_Info("REG_RX_012=%2x", hdmirxrd(REG_RX_012));
		DLOG_Info("REG_RX_063=%2x", hdmirxrd(REG_RX_063));
		DLOG_Info("MHL_RX_0A=%2x", mhlrxrd(MHL_RX_0A));
		DLOG_Info("MHL_RX_08=%2x", mhlrxrd(MHL_RX_08));
		DLOG_Info("MHL_RX_09=%2x", mhlrxrd(MHL_RX_09));
	}

	return ret;



	while (true) {
		usleep(1000 * 1000);
	}

}




// char it66021_init(void)
// {


//     //IT_Delay(1000); //for power sequence
// }




// void it6602_SetOutputColorDepth(unsigned char color_depth)
// {
// 	if (color_depth == 0)
// 	{
// 		HDMIRX_OUTPUT_VID_MODE = eYUV422_Sep_Sync_SDR;
// 	}
// 	else if(color_depth == 1)
// 	{
// 		HDMIRX_OUTPUT_VID_MODE = eCCIR656_Sep_Sync_SDR;
// 	}
// }



// char IT_66021_Initial(uint8_t index, uint8_t color_depth)
// {
//     it6602_SetOutputColorDepth(color_depth);

//     return it66021_init();

// }


// static uint8_t HDMI_RX_MapToDeviceIndex(ENUM_HAL_HDMI_RX e_hdmiIndex)
// {
//     return (e_hdmiIndex == HAL_HDMI_RX_0) ? 0 : 1;
// }

// HAL_RET_T HAL_HDMI_RX_Init(ENUM_HAL_HDMI_RX e_hdmiIndex,
//                            STRU_HDMI_CONFIGURE *pst_hdmiConfigure)
// {
//     s_st_hdmiRxStatus[e_hdmiIndex].u8_devEnable = 1;
//     memcpy(&(s_st_hdmiRxStatus[e_hdmiIndex].st_configure),pst_hdmiConfigure,sizeof(STRU_HDMI_CONFIGURE));

// 	// use the orb to publish to the cpu2 then notify
//     //SYS_EVENT_RegisterHandler(SYS_EVENT_ID_VIDEO_EVENT, HDMI_RxVideoHandle);
//     //SYS_EVENT_RegisterHandler(SYS_EVENT_ID_AUDIO_EVENT, HDMI_RxAudioHandle);

//     // if (s_st_hdmiRxStatus[e_hdmiIndex].st_configure.e_getFormatMethod == HAL_HDMI_INTERRUPT)
//     // {
//     //     HAL_NVIC_SetPriority(GPIO_INTR_N0_VECTOR_NUM + ((pst_hdmiConfigure->st_interruptGpio.e_interruptGpioNum)>>5),INTR_NVIC_PRIORITY_HDMI_GPIO,0);
//     //     switch (e_hdmiIndex)
//     //     {
//     //         case HAL_HDMI_RX_0:
//     //         {
//     //             HAL_GPIO_RegisterInterrupt(s_st_hdmiRxStatus[e_hdmiIndex].st_configure.st_interruptGpio.e_interruptGpioNum,
//     //                                        s_st_hdmiRxStatus[e_hdmiIndex].st_configure.st_interruptGpio.e_interruptGpioType,
//     //                                        s_st_hdmiRxStatus[e_hdmiIndex].st_configure.st_interruptGpio.e_interruptGpioPolarity, HAL_HDMI_RX_IrqHandler0);
//     //             break;
//     //         }
//     //         case HAL_HDMI_RX_1:
//     //         {
//     //              HAL_GPIO_RegisterInterrupt(s_st_hdmiRxStatus[e_hdmiIndex].st_configure.st_interruptGpio.e_interruptGpioNum,
//     //                                        s_st_hdmiRxStatus[e_hdmiIndex].st_configure.st_interruptGpio.e_interruptGpioType,
//     //                                        s_st_hdmiRxStatus[e_hdmiIndex].st_configure.st_interruptGpio.e_interruptGpioPolarity, HAL_HDMI_RX_IrqHandler1);

//     //             break;
//     //         }
//     //         default :
//     //         {
//     //             return HAL_HDMI_INPUT_COUNT;
//     //         }

//     //     }
//     // }

//     // #ifdef USE_ADV7611_EDID_CONFIG_BIN
//     //     ADV_7611_Initial(HDMI_RX_MapToDeviceIndex(e_hdmiIndex));
//     // #endif

//     #ifdef USE_IT66021_EDID_CONFIG_BIN
//     if (FALSE == IT_66021_Initial(HDMI_RX_MapToDeviceIndex(e_hdmiIndex), s_st_hdmiRxStatus[e_hdmiIndex].st_configure.e_colorDepth))
//     {
//         return HAL_HDMI_RX_ERR_INIT;
//     }
//         //SYS_EVENT_RegisterHandler(SYS_EVENT_ID_IDLE, (SYS_Event_Handler)IT6602_fsm);
//     #endif

//     // if (s_st_hdmiRxStatus[e_hdmiIndex].st_configure.e_getFormatMethod == HAL_HDMI_POLLING)
//     // {
//     //     SYS_EVENT_RegisterHandler(SYS_EVENT_ID_IDLE, e_hdmiIndex == HAL_HDMI_RX_0 ? HDMI_RX_IdleCallback0 : HDMI_RX_IdleCallback1);
//     // }

//     if (s_st_hdmiRxStatus[e_hdmiIndex].st_configure.e_getFormatMethod != HAL_HDMI_POLLING ||
//         s_st_hdmiRxStatus[e_hdmiIndex].st_configure.e_getFormatMethod != HAL_HDMI_INTERRUPT)
//     {
//         return HAL_HDMI_GET_ERR_GORMAT_METHOD;
//     }

//     return HAL_OK;
// }

