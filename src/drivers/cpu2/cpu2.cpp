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
#include <sys/types.h>
#include <systemlib/err.h>
#include <drivers/device/i2c.h>
#include <nuttx/irq.h>
#include <px4_workqueue.h>
#include <px4_getopt.h>
#include <drivers/drv_intercore.h>

#include <uORB/uORB.h>
#include <uORB/topics/h264_input_format.h>
#include <uORB/topics/input_rc.h>

#include "cpu2.h"
#include "Mavlink.h"

// static void ar_intercore_init();
static void cycle_trampoline(void *arg);
// static int drv_intercore_tim_isr(int irq, void *context, void *arg);
uint8_t arsys_event_cpu0t2_append(AR_INTERCORE_EVENT *message);
int tolower(int c);
int htoi(char s[]);

static uint8_t needlogging = 0;

// static void parseRcMavlinkMsg(uint8_t *bufferPtr, uint16_t length);

static struct work_s h264_work = {};
static int h264_fd = 0;
static struct h264_input_format_s att; 
static void h264_cycle(void *arg);


// static mavlink_message_t mavlinkMsg;

// static mavlink_status_t _status; 


class CPU2 : public device::CDev
{
public:
	_EXT_ITCM CPU2();

	_EXT_ITCM ~CPU2();

	_EXT_ITCM virtual int init();

	_EXT_ITCM int start();

	_EXT_ITCM void handler_event();
};

namespace
{
	CPU2 *g_cpu2 = nullptr;

	work_s _work = {};
}



// static int daemon_task;				/**< Handle of daemon task / thread */
_EXT_ITCM static void cycle_trampoline(void *arg)
{
	uint8_t i = 0;

    volatile AR_INTERCORE_EVENT* msgPtr = (AR_INTERCORE_EVENT*)SRAM_INTERCORE_EVENT_CPU2T0_ST_STARTADDR;

    for(i = 0; i < SRAM_INTERCORE_EVENT_MAX_COUNT; i++)
    {
        if (msgPtr[i].isUsed == 1) {
			switch (msgPtr[i].type)
			{
			case SYS_EVENT_ID_CPU2_LOG:
				if(needlogging == 1)
				{
					PX4_INFO("%s", msgPtr[i].data);
				}
				msgPtr[i].isUsed = 2;
				break;
			default:
				break;
			}
        }        
    }
	
	work_queue(LPWORK, &_work, (worker_t)&cycle_trampoline, nullptr, 5);
} 


_EXT_ITCM static void initSramForMavlink() 
{
    memset((void *)SRAM_MAVLINK_RC_MSG_ST_ADDR, 0, SRAM_MAVLINK_RC_MSG_SIZE);
	
	STRU_SramBuffer *msgBuffer = (STRU_SramBuffer*)SRAM_MAVLINK_RC_MSG_ST_ADDR;
	msgBuffer->header.buf_wr_pos = (uint32_t)msgBuffer->buf;
	msgBuffer->header.buf_rd_pos = (uint32_t)msgBuffer->buf;
}


_EXT_ITCM static void h264_cycle(void *arg)
{
    if (h264_fd == 0)
    {
        h264_fd = orb_subscribe(ORB_ID(h264_input_format));
    }

    bool updated = false;

    if (orb_check(h264_fd, &updated) != PX4_OK) { return; }

    if (updated)
    {
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

		PX4_INFO("-----------------------------------------");
        PX4_INFO("index = %d ", att.index);
        PX4_INFO("width = %d ", att.width);
        PX4_INFO("hight = %d ", att.hight);
        PX4_INFO("framerate = %d", att.framerate);
        PX4_INFO("vic = %d", att.vic);
        PX4_INFO("e_h264InputSrc = %d", att.e_h264InputSrc);
		PX4_INFO("-----------------------------------------\n");

		arsys_event_cpu0t2_append(&msg);
 
		// arsys_event_cpu0t2_try_trigger();
    }	

	(void)work_queue(LPWORK, &h264_work, (worker_t)&h264_cycle, nullptr, USEC2TICK(1000 * 1000));
}


extern "C" __EXPORT int cpu2_main(int argc, char *argv[]);


CPU2::CPU2():CDev("cpu2", "/dev/cpu2") 
{
}
 

_EXT_ITCM uint8_t arsys_event_cpu0t2_append(AR_INTERCORE_EVENT *message)
{   
	static uint8_t seq = 0;

	uint8_t i = 0;
    volatile AR_INTERCORE_EVENT* msgPtr = (AR_INTERCORE_EVENT*)SRAM_INTERCORE_EVENT_CPU0T2_ST_STARTADDR;

    for(i = 0; i < SRAM_INTERCORE_EVENT_MAX_COUNT; i++)
    {
        if (msgPtr[i].isUsed == 0) 
        {
            msgPtr[i].length = message->length;
            msgPtr[i].type = message->type;
            memcpy((void *)msgPtr[i].data, message->data, message->length);
            msgPtr[i].isUsed = 1;
            msgPtr[i].seq = seq++;

            break;
        }        
    }
   
    if (i == SRAM_INTERCORE_EVENT_MAX_COUNT) 
    {
        for(i = 0; i < SRAM_INTERCORE_EVENT_MAX_COUNT; i++)
        {
            msgPtr[i].isUsed = 0;
        }
        
        arsys_event_cpu0t2_append(message);
    }
    
    return 0;
}


CPU2::~CPU2() {}

int CPU2::init()
{
	int ret = CDev::init();

	return ret;
}


int CPU2::start()
{
	px4_flash_init();

	initSramForMavlink();

	return PX4_OK;
}
 
int tolower(int c)  
{  
    if (c >= 'A' && c <= 'Z')  
    {  
        return c + 'a' - 'A';  
    }  
    else  
    {  
        return c;  
    }  
}

int htoi(char s[])  
{  
    int i;  
    int n = 0; 

    if (s[0] == '0' && (s[1]=='x' || s[1]=='X'))  
    {  
        i = 2;  
    }  
    else  
    {  
        i = 0;  
    }


    for (; (s[i] >= '0' && s[i] <= '9') || (s[i] >= 'a' && s[i] <= 'z') || (s[i] >='A' && s[i] <= 'Z');++i)  
    {  
        if (tolower(s[i]) > '9')  
        {  
            n = 16 * n + (10 + tolower(s[i]) - 'a');  
        }  
        else  
        {  
            n = 16 * n + (tolower(s[i]) - '0');  
        }  
    }  
    return n;  
}  

void cpu2_usage();

_EXT_ITCM void cpu2_usage()
{

	PX4_INFO("missing command: try 'start' or 'setid' or 'log' or 'blind'");
	PX4_INFO("Example:");
	PX4_INFO("		using setid: 'cpu2 setid ab cd ef 12 34 aa bb' ");
	PX4_INFO("		using log: 'cpu2 log', display  intercore cpu2 log info");
	PX4_INFO("		using blind: 'cpu2 blind', not display  intercore cpu2 log info");
}

_EXT_ITCM int cpu2_main(int argc, char *argv[])
{
	const char *verb = argv[1];

	if (strcmp(verb, "start") == 0)
	{
		if (g_cpu2 != nullptr)
		{
			PX4_ERR("already started");
			return EXIT_SUCCESS;
		}

		g_cpu2 = new CPU2();
		
		if (g_cpu2 == nullptr)
		{
			PX4_ERR("couldn't allocate the CPU2 driver \n");
			return EXIT_FAILURE;
		}
		if (OK != g_cpu2->init())
		{
			delete g_cpu2;
			g_cpu2 = nullptr;
			PX4_ERR("CPU2 init failed \n");
			return EXIT_FAILURE;
		}	

		g_cpu2->start();

		
		h264_work = {};
    	h264_fd = 0;
		memset(&att, 0 , sizeof(att));

		(void)work_queue(LPWORK, &h264_work, (worker_t)&h264_cycle, nullptr, USEC2TICK(1000 * 1000));

		work_queue(LPWORK, &_work, (worker_t)&cycle_trampoline, nullptr, 5);

	} 
	else if (strcmp(verb, "setid") == 0)
	{	
		// cpu2 setid 1 2 3 4 5 6 7 
		if (argc < 9)
		{
			cpu2_usage();
		}
		else 
		{
			uint8_t buffer[7] = {0};			

			buffer[0] = htoi(argv[2]);
			buffer[1] = htoi(argv[3]);
			buffer[2] = htoi(argv[4]);
			buffer[3] = htoi(argv[5]);
			buffer[4] = htoi(argv[6]);
			buffer[5] = htoi(argv[7]);
			buffer[6] = htoi(argv[8]);
			px4_flash_updateid(buffer, sizeof(buffer), 2);

		}
	} 
	else if (strcmp(verb, "log") == 0) 
	{
		needlogging = 1;
	} 
	else if (strcmp(verb, "blind") == 0) 
	{
		needlogging = 0;
	} 
	else 
	{
		cpu2_usage();
	}
	
	return EXIT_SUCCESS;
}
