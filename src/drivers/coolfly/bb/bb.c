/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

/**
 * @file top.c
 * Tool similar to UNIX top command
 * @see http://en.wikipedia.org/wiki/Top_unix
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <px4_config.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>

#include <chip/ar_config.h>
#include <systemlib/cpuload.h>
#include <systemlib/printload.h>
#include <drivers/drv_hrt.h>
#include <px4_module.h>

typedef struct
{
    uint8_t         magic_header[2];
    uint16_t        msg_id;
    uint8_t         packet_num;
    uint8_t         packet_cur;
    uint16_t        msg_len;
    uint16_t        msg_chksum;
    uint16_t        snr_vlaue[2];           //0,1 3,2
    uint16_t        u16_afterErr;           //5,4  masoic
    uint8_t         u8_optCh;               //6 current optional channel
    uint8_t         u8_mcs;                 //7
    int16_t         sweep_energy[21*8];     //Max channel: 21
    uint16_t        ldpc_error;             //9,8 error after Harq
    uint8_t         agc_value[4];           //13 12 11 10
    uint8_t         harq_count;             //14
    uint8_t         modulation_mode;        //15
    uint8_t         e_bandwidth;            //16
    uint8_t         code_rate;              //17
    uint8_t         osd_enable;             //18
    uint8_t         IT_channel;             //19
    uint8_t         head;                   //20
    uint8_t         tail;                   //21
    uint8_t         in_debug;               //22
    uint8_t         lock_status;            //23
    uint16_t        video_width[2];         //27,26 25,24
    uint16_t        video_height[2];        //31,30 29,28
    uint8_t         frameRate[2];           //33,32
    uint8_t         encoder_bitrate[2];     //35,34
    uint8_t         rc_modulation_mode;     //36
    uint8_t         rc_code_rate;           //37
    uint8_t         encoder_status;         //38
    uint8_t         errcnt1;                //39
    uint8_t         errcnt2;                //40
    uint8_t         u8_rclock;              //41
    uint8_t         u8_nrlock;              //42
    uint8_t         sky_agc[4];             //43,44,45,46
    uint8_t         reserved0;              //47
    uint16_t        dist_zero;              //49,48
    uint16_t        dist_value;             //50,51
    uint16_t        sky_snr;                //52~53
    uint8_t         reserved[2];            //54~55
    uint32_t        sdram_buf_size[2];      //56~63
    uint8_t         find_beside_dev_finish;     //64
    uint8_t         find_beside_dev_num;    //65, sky start, search how many the same device type beside(default function is off , need app open)
} BB_STRU_WIRELESS_INFO_DISPLAY;

HAL_RET_T HAL_BB_GetInfo(BB_STRU_WIRELESS_INFO_DISPLAY **ppst_bbInfoAddr);

BB_STRU_WIRELESS_INFO_DISPLAY infoDisplay = {0};


uint16_t lockCnt = 0;
uint16_t unlockCnt = 0;
uint8_t cycleCnt = 0;

typedef uint32_t HAL_RET_T;

#define HAL_OK                                      (0)
#define HAL_TIME_OUT                                (0xFF)
#define HAL_BUSY                                    (0xFE)
#define HAL_OCCUPIED                                (0xFD)
#define HAL_NOT_INITED                              (0xFC)


HAL_RET_T HAL_BB_GetInfo(BB_STRU_WIRELESS_INFO_DISPLAY **ppst_bbInfoAddr)
{
    if (NULL != ppst_bbInfoAddr && NULL != *ppst_bbInfoAddr)
    {
        *ppst_bbInfoAddr = (BB_STRU_WIRELESS_INFO_DISPLAY *)(SRAM_BB_STATUS_SHARE_MEMORY_ST_ADDR);     
    }

    if (((*ppst_bbInfoAddr)->head != 0x00) || ((*ppst_bbInfoAddr)->tail != 0xFF))
    {
        return HAL_BUSY;
    }
    else
    {
        return HAL_OK;
    }
}


static char *modulationModeName(uint8_t mode)
{
    switch (mode)
    {
    case 0x0:
        return "BPSK";
    case 0x1:
        return "QPSK";
    case 0x2:
        return "QAM16";
    case 0x3:
        return "QAM64";
    default:
        break;
    }
    return "";
}

static void bb_print(BB_STRU_WIRELESS_INFO_DISPLAY *pst_bbInfoAddr, int fd)
{
    BB_STRU_WIRELESS_INFO_DISPLAY *info = (BB_STRU_WIRELESS_INFO_DISPLAY *)(SRAM_BB_STATUS_SHARE_MEMORY_ST_ADDR);

    // -信噪比　,滤波打印
    infoDisplay.snr_vlaue[0] = (infoDisplay.snr_vlaue[0] + info->snr_vlaue[0]) >>1;
    infoDisplay.snr_vlaue[1] = (infoDisplay.snr_vlaue[1] + info->snr_vlaue[1]) >>1;


    // 接受信号的能量，滤波打印
    infoDisplay.agc_value[0] = (infoDisplay.agc_value[0] + info->agc_value[0]) >>1;
    infoDisplay.agc_value[1] = (infoDisplay.agc_value[1] + info->agc_value[1]) >>1;
    infoDisplay.agc_value[2] = (infoDisplay.agc_value[2] + info->agc_value[2]) >>1;
    infoDisplay.agc_value[3] = (infoDisplay.agc_value[3] + info->agc_value[3]) >>1; 

    // -调制模式　BPSK QPSK 16QAM 64QAM　，直接打印
    infoDisplay.modulation_mode = info->modulation_mode;

    // 图传信道　　直接打印
    infoDisplay.IT_channel = info->IT_channel;

    //  锁定状态　　判断打印
    infoDisplay.lock_status = info->lock_status;

    // 视频分辨率　判断打印
    infoDisplay.video_width[0] = info->video_width[0];
    infoDisplay.video_width[1] = info->video_width[1];
    infoDisplay.video_height[0] = info->video_height[0];
    infoDisplay.video_height[1] = info->video_height[1];
    
    // 视频帧率   滤波打印
    infoDisplay.frameRate[0] = (infoDisplay.frameRate[0] + info->frameRate[0]) >>1;
    infoDisplay.frameRate[1] = (infoDisplay.frameRate[1] + info->frameRate[1]) >>1;

    // 视频压缩码率　　滤波打印
    infoDisplay.encoder_bitrate[0] = (infoDisplay.encoder_bitrate[0] + info->encoder_bitrate[0]) >>1;
    infoDisplay.encoder_bitrate[1] = (infoDisplay.encoder_bitrate[1] + info->encoder_bitrate[1]) >>1;

    //  rc调制模式   直接打印
    infoDisplay.rc_modulation_mode = info->rc_modulation_mode;

    // rc码率   滤波打印
    infoDisplay.rc_code_rate = (infoDisplay.rc_code_rate + info->rc_code_rate) >>1;

    // 遥控锁定  判断打印
    if (info->u8_rclock)
    {
        lockCnt++;
    } 
    else
    {
        unlockCnt++;
    }
    
    //  天空端信噪比　　滤波打印
    infoDisplay.sky_snr = (infoDisplay.sky_snr + info->sky_snr) >> 1;

    if (cycleCnt ++ <= 50)
    {
        return;
    }

    cycleCnt = 0;

	if (fd == 1) {
        // ref:http://www.termsys.demon.co.uk/vtansi.htm
		dprintf(fd, "\033[2J\033[H");
	}

    printf("\r\n------------------- BB INFO STATUS --------------------\r\n\r\n");

    printf("    %-20s    0x%x\r\n", "snr_vlaue[0]",   infoDisplay.snr_vlaue[0]);
    printf("    %-20s    0x%x\r\n", "snr_vlaue[1]",   infoDisplay.snr_vlaue[1]);
    printf("    %-20s    0x%02x\r\n", "agc_value[0]",   infoDisplay.agc_value[0]);
    printf("    %-20s    0x%02x\r\n", "agc_value[1]",   infoDisplay.agc_value[1]);
    printf("    %-20s    0x%02x\r\n", "agc_value[2]",   infoDisplay.agc_value[2]);
    printf("    %-20s    0x%02x\r\n", "agc_value[3]",   infoDisplay.agc_value[3]);
    printf("    %-20s    %s\r\n", "modulation_mode", modulationModeName(infoDisplay.modulation_mode));
    printf("    %-20s    %2d\r\n", "IT_channel",     infoDisplay.IT_channel);
    printf("    %-20s    %2d\r\n", "lock_status",    infoDisplay.lock_status);
    printf("    %-20s    %d\r\n", "video_width[0]", infoDisplay.video_width[0]);
    printf("    %-20s    %d\r\n", "video_width[1]", infoDisplay.video_width[1]);
    printf("    %-20s    %d\r\n", "video_height[0]",infoDisplay.video_height[0]);
    printf("    %-20s    %d\r\n", "video_height[1]",infoDisplay.video_height[1]);
    printf("    %-20s    %d\r\n", "frameRate[0]",   infoDisplay.frameRate[0]);
    printf("    %-20s    %d\r\n", "frameRate[1]",   infoDisplay.frameRate[1]);
    printf("    %-20s    %d\r\n", "encoder_bitrate[0]",  infoDisplay.encoder_bitrate[0]);
    printf("    %-20s    %d\r\n", "encoder_bitrate[1]",  infoDisplay.encoder_bitrate[1]);
    printf("    %-20s    %d\r\n", "rc_modulation_mode",  infoDisplay.rc_modulation_mode);
    printf("    %-20s    %d/%d\r\n", "u8_rclock",      lockCnt, unlockCnt);
    printf("    %-20s    %d\r\n", "sky_snr",        infoDisplay.sky_snr);

    printf("\r\n\r\n");

    fflush(stdout);
}

/**
 * Start the top application.
 */
__EXPORT int bb_main(int argc, char *argv[]);


// static void print_usage(void)
// {
// 	PRINT_MODULE_DESCRIPTION("Monitor running processes and their CPU, stack usage, priority and state");
// 	PRINT_MODULE_USAGE_NAME_SIMPLE("bb", "command");
// 	// PRINT_MODULE_USAGE_COMMAND_DESCR("once", "print load only once");
// }

int
bb_main(int argc, char *argv[])
{
    memset(&infoDisplay, 0, sizeof(BB_STRU_WIRELESS_INFO_DISPLAY));

	dprintf(1, "\033[2J\n");

	// print_usage();

	for (;;) {

		HAL_RET_T hal_ret;

    	BB_STRU_WIRELESS_INFO_DISPLAY *pst_bbInfoAddr;

    	hal_ret = HAL_BB_GetInfo(&pst_bbInfoAddr);

		if(hal_ret != HAL_OK)
		{
			// // PX4_INFO("hal_ret = %d \r\n", hal_ret);
			// continue;
		}

		bb_print(pst_bbInfoAddr, 1);


		/* Sleep 200 ms waiting for user input five times ~ 1s */
		for (int k = 0; k < 1; k++) {
			char c;

			struct pollfd fds;
			int ret;
			fds.fd = 0; /* stdin */
			fds.events = POLLIN;
			ret = poll(&fds, 1, 0);

			if (ret > 0) {

				ret = read(0, &c, 1);

				if (ret) {
					return 1;
				}

				switch (c) {
				case 0x03: // ctrl-c
				case 0x1b: // esc
				case 'c':
				case 'q':
					return 0;
					/* not reached */
				}
			}

			usleep(10000);
		}

	}

	return 0;
}
