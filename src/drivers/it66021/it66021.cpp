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

#include <px4_config.h>
#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>
#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <drivers/drv_intercore.h>
#include <uORB/uORB.h>
#include <uORB/topics/h264_input_format.h>

#include "ite_type.h"
#include "ite_define.h"
#include "ite_reg.h"

#include "it66021.h"

#include "px4_log.h"

#include <ar_gpio.h>




static STRU_HDMI_RX_OUTPUT_FORMAT s_st_hdmiRxSupportedOutputFormat[] = {
	{720,  480,  60},
	{1280, 720,  30},
	{1280, 720,  25},
	{1280, 720,  50},
	{1280, 720,  60},
	{1920, 1080, 25},
	{1920, 1080, 30},
//    {1920, 1080, 50},
//    {1920, 1080, 60},
};

static unsigned int s_u8Array_ARCastSupportedOutputFormat[][4] = {
	{ 720, 480,  50,  2},//4:3
	{ 720, 480,  60,  3},//16:9
	{ 720, 576,  50, 17},//4:3
	{ 720, 576,  60, 18},//16:9
	{1280, 720,  60,  4},//16:9
	{1280, 720,  50, 19},//16:9
	{1920, 1080, 25, 33}, //16:9
	{1920, 1080, 30, 34},//16:9
};



static uint32_t HDMI_RX_CheckVideoFormatSupportOrNot(uint16_t u16_width, uint16_t u16_hight, uint8_t u8_framerate)
{
	uint8_t count = sizeof(s_st_hdmiRxSupportedOutputFormat) / sizeof(s_st_hdmiRxSupportedOutputFormat[0]);

	for (uint8_t i = 0; i < count; i++) {
		if ((u16_width == s_st_hdmiRxSupportedOutputFormat[i].u16_width) &&
		    (u16_hight == s_st_hdmiRxSupportedOutputFormat[i].u16_hight) &&
		    (u8_framerate == s_st_hdmiRxSupportedOutputFormat[i].u8_framerate)) {
			return HAL_OK;
		}
	}

	return HAL_HDMI_RX_FALSE;
}


#define _CODE


//FIX_ID_016 xxxxx Support Dual Pixel Mode for IT66023 Only
#if defined(_IT66023_)
#pragma message ("defined ENABLE_IT66023")
#endif
//FIX_ID_016 xxxxx

/*****************************************************************************/
/* Local Defines    **********************************************************/
/*****************************************************************************/
//#define DISABLE_HDMI_CSC
#define Enable_Vendor_Specific_packet
//#define EN_DUAL_PIXEL_MODE	//2013-0520




//FIX_ID_033 xxxxx  //Fine-tune EQ Adjust function for HDCP receiver and repeater mode
#define VSATE_CONFIRM_SCDT_COUNT        MS_TimeOut(100)


#define SCDT_OFF_TIMEOUT              		MS_TimeOut(20)		//100 x MS_LOOP = 5000 ms = 5 sec
#define ECC_TIMEOUT              			MS_TimeOut(20)
#define DESKEW_TIMEOUT            			MS_TimeOut(20)


// Debug Mode
#define EnCBusDbgMode  	    FALSE
#define MSCCBusDbgCtrl 	    TRUE
#define DDCCBusDbgCtrl 	FALSE
#define RCLKFreqSel 	    1	//; //0: RING/2 ; 1: RING/4 ; 2: RING/8 ; 3: RING/16
#define GenPktRecType	    0x81
#define PPHDCPOpt	        TRUE	//2013-0509 MHL 1080p packet pixel mode HDCP


#ifndef IT6811B0
#define PPHDCPOpt2	TRUE	//2013-0509 MHL 1080p packet pixel mode HDCP
#else
#define PPHDCPOpt2	FALSE 	//only for it6811b0
#endif


//FIX_ID_021 xxxxx		//To use CP_100ms for CBus_100ms and CEC_100m
//FIX_ID_004 xxxxx //Add 100ms calibration for Cbus
//#ifdef _SelectExtCrystalForCbus_
#define T10usSrcSel   TRUE	//FALSE: 100ms calibration , TRUR: 27MHz Crystal(only IT6602)
//#else
//#define T10usSrcSel   FALSE	 //FALSE: 100ms calibration , TRUR: 27MHz Crystal(only IT6602)
//#endif
//FIX_ID_004 xxxxx
//FIX_ID_021 xxxxx

#define EnMSCBurstWr	TRUE
#define MSCBurstWrID	TRUE   // TRUE: from MHL5E/MHL5F
#define MSCBurstWrOpt	FALSE  // TRUE: Not write Adopter ID unsigned char o ScratchPad
#define EnPktFIFOBurst	TRUE
// DDC Option
#define EnDDCSendAbort	TRUE  // Send ABORT after segment write with EOF
//CBUS Capability
#define MHLVersion	0x20
#define PLIM	1
#define POW	1
#define DEV_TYPE_SINK	1 //06-26
#define DEV_TYPE	1
#define ADOPTER_ID_H	0x02
#define ADOPTER_ID_L	0x45
#define DEVICE_ID_H		0x68
#define DEVICE_ID_L		0x02
#define AckHigh	0xB
#define AckLow	1
// CBUS INput Option
#define EnCBusDeGlitch	TRUE


//---------------------//
//----- WatchDog -----//
//--------------------//
#define DeltaNum 	1
#define RegBurstWrTOSel	2 // 2	//0: 320ms, 1: 340ms, 2: 360ms (ATC)
#define Reg100msTOAdj	2 // 2	//00: 100ms, 01: 99ms, 10: 101ms (ATC)
#define EnMSCHwRty	FALSE
#define EnHWPathEn	FALSE
#define MSCRxUCP2Nack	TRUE



/////////////////////////////////////////
//Cbus command fire wait time
//Maxmun time for determin CBUS fail
//	CBUSWAITTIME(ms) x CBUSWAITNUM
/////////////////////////////////////////
//FIX_ID_024 xxxxx	//Fixed for RCP compliance issue
#define CBUSWAITTIME    1
#define CBUSWAITNUM     100
//FIX_ID_024	xxxxx

#define  HDCPIntKey   FALSE   //TRUE: Internal HDCP Key, FALSE: SIPROM

#define  VCLK_INV	0
#define  VCLK_DLY	0
#define  EnMultiSeg     TRUE
#define  EnIntEDID      TRUE

//Discovery
#define  CBUSFloatAdj	FALSE
#define EQFAILCNT 2



//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#define EQRETRYFAILCNT 1	// for EQ interrupt
#define RCLKVALUE 12			// for show TMDS and Pixel Clk
#define TMDSCLKVALUE 160	// for TMDS > 160 then set RS to 00, otherwise set to 3F

#define TMDSCLKVALUE_1080P 160	// for TMDS > 160 then set RS to 00, otherwise set to 3F
#define TMDSCLKVALUE_480P 35
#define TMDSCLKVALUE_MHL_ER1 90
#define JUDGE_ER1_VALUE 90

//FIX_ID_001 xxxxx


//FIX_ID_021 xxxxx		//To use CP_100ms for CBus_100ms and CEC_100m
//FIX_ID_004 xxxxx 		//Add 100ms calibration for Cbus
//#ifndef _SelectExtCrystalForCbus_
#define _RCLK_FREQ_20M  FALSE
//#endif
//FIX_ID_004 xxxxx
//FIX_ID_021 xxxxx


//FIX_ID_037 xxxxx //Allion MHL compliance issue !!!
//FIX_ID_033 xxxxx  //Fine-tune EQ Adjust function for HDCP receiver and repeater mode
//FIX_ID_005 xxxx	//Wait for video on then read MHL device capability
#define MAX_CBUS_WAITNO 		(300/MS_LOOP)		// 250ms
#define MAX_PATHEN_WAITNO 	(700/MS_LOOP)		// 700ms
#define MAX_BUSY_WAITNO 		(2500/MS_LOOP)		// 150ms
#define MAX_DISCOVERY_WAITNO 	(100/MS_LOOP)		// 100ms
//FIX_ID_005 xxxx
//FIX_ID_033 xxxxx
//FIX_ID_037 xxxxx

//FIX_ID_014 xxxx
#define MAX_TMDS_WAITNO 		(350/MS_LOOP)		// 400ms
#define MAX_HDCP_WAITNO 		(100/MS_LOOP)		// 150ms
//FIX_ID_014 xxxx
//FIX_ID_018	xxxxx	//modify 1K pull-down to 1.033K ohm HDMI Reg1C0[3:2]=2
#define RENEW_WAKEUP		(12000/MS_LOOP)
#define IGNORE_WAKEUP		(1000/MS_LOOP)
#define TOGGLE_WAKEUP		(4000/MS_LOOP)
#define CDSENSE_WAKEUP		(500/MS_LOOP)
//FIX_ID_018	xxxxx

#define DEFAULT_EQVALUE 0x1F


//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_EQ_ADJUST_
#define MaxEQIndex 3


unsigned char IT6602EQTable[] = {0xFF, 0x9F, 0x83};

//for EQ state machine handler
//#define	MAXSYNCOFF 		5
#define	MAXECCWAIT 		(10)
#define	EQSTATE_WAIT		(20)
#define	EQSTATE_START		(EQSTATE_WAIT+MAXECCWAIT)
#define	EQSTATE_LOW		(EQSTATE_WAIT+EQSTATE_START+(MAXECCWAIT*1))
#define	EQSTATE_MIDDLE	(EQSTATE_WAIT+EQSTATE_START+(MAXECCWAIT*2))
#define	EQSTATE_HIGH		(EQSTATE_WAIT+EQSTATE_START+(MAXECCWAIT*3))
#define	EQSTATE_END		(255-100)
#define	MINECCFAILCOUNT 	(MAXECCWAIT/2)
#endif



/*****************************************************************************/
/* Init, Power, and IO Structures ********************************************/
/*****************************************************************************/
////////////////////////////////////////////////////////////////////
//it6602 chip inital table
//
//
//
////////////////////////////////////////////////////////////////////
static _CODE struct IT6602_REG_INI  IT6602_HDMI_INIT_TABLE[] = {
	//port 0
	{REG_RX_00F,	0x03,	0x00},	//change bank 0
	{REG_RX_010,	0xFF,	0x08},	//[3]1: Register reset
	{REG_RX_00F,	0x03,	0x00},	//change bank 0

	{REG_RX_034,	0xFF,	MHL_ADDR + 0x01},	//I2C Slave Addresss for MHL block

	{REG_RX_010,	0xFF,	0x17},	//[4]Auto Video Reset [2]Int Reset [1]Audio Reset [0]Video Reset

	{REG_RX_011,	0xFF,	0x1F},	//Port 0�G[4]EQ Reset [3]CLKD5 Reset [2]CDR Reset [1]HDCP Reset [0]All logic Reset
	{REG_RX_018,	0xFF,	0x1F},	//Port 1�G[4]EQ Reset [3]CLKD5 Reset [2]CDR Reset [1]HDCP Reset [0]All logic Reset
	{REG_RX_012,	0xFF,	0xF8},

	{REG_RX_012,	0xFF,	0xF8},	//Port 0�G[7:3] MHL Logic reset

	{REG_RX_010,	0xFF,	0x10},	//[4]Auto Video Reset [2]Int Reset [1]Audio Reset [0]Video Reset

	{REG_RX_011,	0xFF,	0xA0},	//Port 0�G[7] Enable Auto Reset when Clock is not stable [5]Enable Auto Reset
	{REG_RX_018,	0xFF,	0xA0},	//Port 1�G[7] Enable Auto Reset when Clock is not stable [5]Enable Auto Reset

	{REG_RX_012,	0xFF,	0x00},	//Port 0�G[7:3] MHL Logic reset

	{REG_RX_00F,	0x03,	0x01},	//change bank 1	//2013-0430 Andrew suggestion
	{REG_RX_1B0,	0x03,	0x01},	// MHL Port Set HPD = 0 at Power On initial state

	//FIX_ID_037 xxxxx //Allion MHL compliance issue debug !!!
	//FIX_ID_018 xxxxx 	modify 1K pull-down to 1.033K ohm HDMI Reg1C0[3:2]=2
	//2014-0526 MHL compliance issue Debug disable ->	{REG_RX_1C0,	0x8C,	0x08},	//[7] PWSB_LV = 0	//2013-0430 Andrew suggestion
	//FIX_ID_018 xxxxx
	//FIX_ID_037 xxxxx


	{REG_RX_00F,	0x03,	0x00},	//change bank 0	//2013-0430 Andrew suggestion
	{REG_RX_017,	0xC0,	0x80},	//Port 0�G[7:6] = 10 invert Port 0 input HCLK , CLKD5I	//2013-0430 Andrew suggestion
	{REG_RX_01E,	0xC0,	0x00},	//Port 1�G[7:6] = 00 invert Port 1 input TMDS , CLKD5I	//2013-0430 Andrew suggestion

#ifdef Enable_IT6602_CEC
	{REG_RX_00E,	0xFF,	0xFF},	//for enable CEC Clock
	{REG_RX_086,	0xFF,	(CEC_ADDR | 0x01)},	//CEC chip Slave Adr
#endif

	//	{0xFE,	0x80,	0x80},	//BUS10B for FPGA

	{REG_RX_016,	0x08,	0x08},	//Port 0�G[3]1: Enable CLKD5 auto power down
	{REG_RX_01D,	0x08,	0x08},	//Port 1�G[3]1: Enable CLKD5 auto power down

//	{0x20,	0x01,	0x30},	//Port 0�GAFE control

	{REG_RX_02B,	0xFF,	0x07},	//FixTek3D
	//	{REG_RX_031,	0xFF,	0x2C},	//[7:4]Enable repeater function [3:0] SCL hold time count & Update Ri sel
	//FIX_ID_042 xxxxx //Disable HDCP 1.1 feature to avoid compilance issue from ilegal HDCP 1.1 source device
	{REG_RX_031,	0xFF,	0x09},	//[7:4]Enable repeater function [3:0] SCL hold time count & Update Ri sel
	{REG_RX_049,	0xFF,	0x09},	//[7:4]Enable repeater function [3:0] SCL hold time count & Update Ri sel
	//FIX_ID_042 xxxxx
	//20131129 move to top side->	{REG_RX_034,	0xFF,	MHL_ADDR+0x01},	//I2C Slave Addresss for MHL block
	//FIX_ID_017 xxxxx Disable IPLockChk
	//FIX_ID_001 xxxxx UseIPLock = 0 for avoid clk change
	{REG_RX_035,	0x1E,	(0x10 + (DeltaNum << 2))},	//[3:2] RCLKDeltaSel , [1] UseIPLock = 0
	{REG_RX_04B,	0x1E,	(0x10 + (DeltaNum << 2))},	//[3:2] RCLKDeltaSel , [1] UseIPLock = 0
//FIX_ID_001 xxxxx
//FIX_ID_017 xxxxx
	{REG_RX_054,	0xFF,	(1 << 4) + RCLKFreqSel},	//[1:0]RCLK frequency select
	{REG_RX_06A,	0xFF,	GenPktRecType},			//Decide which kind of packet to be fully recorded on General PKT register
	{REG_RX_074,	0xFF,	0xA0},	//[7]Enable i2s and SPDIFoutput [5]Disable false DE output
	{REG_RX_050,	0x1F,	0x11},	//[4]1: Invert output DCLK and DCLK DELAY 2 Step
//2013-0606	{REG_RX_050,	0x13,	0x00},	//[4]1: Invert output DCLK and DCLK DELAY 2 Step

//	{REG_RX_065,	0x0C,	0x00},	//[3:2]0=8bits Output color depth
//	{REG_RX_065,	0x0C,	0x04},	//[3:2]1=10bits Output color depth
	{REG_RX_065,	0x0C,	0x08},	//[3:2]2=12bits Output color depth

	{REG_RX_07A,	0x80,	0x80},	//[7]1: enable audio B Frame Swap Interupt
//	{REG_RX_02D,	0x03,	0x03},	//[1:0] 11: Enable HDMI/DVI mode over-write

	{REG_RX_085,	0x02,	0x02},	//[1]1: gating avmute in video detect module

//	{REG_RX_051,	0x80,	0x80},	//[7]1: power down color space conversion logic

#ifdef  _SUPPORT_EDID_RAM_
	{REG_RX_0C0,	0x43,	0x40},	//[0]1:Reg_P0DisableShadow
	{REG_RX_087,	0xFF,	(EDID_ADDR | 0x01)},	//[7:1] EDID RAM Slave Adr ,[0]1: Enable access EDID block
#else
	{REG_RX_0C0,	0x03,	0x03},	//[0]1:Reg_P0DisableShadow
	{REG_RX_087,	0xFF,	(0x00)},	//[7:1] EDID RAM Slave Adr ,[0]1: Enable access EDID block
#endif

	{REG_RX_071,	0x08,	0x00},	//Reg71[3] RegEnPPColMode must clear to 0 for andrew suggestion 2013-0502
//FIX_ID_030 xxxxx fixed video lost at 640x480 timing
	{REG_RX_037,	0xFF,	0xA6},	//Reg37 Reg_P0_WCLKValidNum must set to 0xA6 for andrew suggestion 2014-0403
	{REG_RX_04D,	0xFF,	0xA6},	//Reg4D Reg_P1_WCLKValidNum must set to 0xA6 for andrew suggestion 2014-0403
//FIX_ID_030 xxxxx
	{REG_RX_067,	0x80,	0x00},	//Reg67[7] disable HW CSCSel

	{REG_RX_07A, B_CTS_RES, B_CTS_RES},

//FIX_ID_037 xxxxx //Allion MHL compliance issue debug !!!
//FIX_ID_018 xxxxx 	modify 1K pull-down to 1.033K ohm HDMI Reg1C0[3:2]=2
//2014-0526 MHL compliance issue Debug disable ->	{REG_RX_1C0,	0x8C,	0x08},	//[7] PWSB_LV = 0	//2013-0430 Andrew suggestion
// Reg1C0[3:2] = 00 -> 1.08Kohm	0 %
// Reg1C0[3:2] = 01 -> 1.18Kohm	+10 %
// Reg1C0[3:2] = 10 -> 0.98Kohm	-10%
// Reg1C0[3:2] = 11 -> 0.88Kohm	-20%
//FIX_ID_018 xxxxx
#if defined(_IT6602_) || defined(_IT66023_)
	{REG_RX_077, 0x80, 0x00},
	{REG_RX_00F, 0x03, 0x01},	//change bank 1
	{REG_RX_1C0, 0x8C, 0x04},	//FIX_ID_037  2014-0527 +10% for W1070 only
	{REG_RX_00F, 0x03, 0x00},	//change bank 0
#else
	{REG_RX_077, 0x80, 0x80},	 // IT66021 Audio i2s sck and mclk is common pin
	{REG_RX_00F, 0x03, 0x01},	//change bank 1
	{REG_RX_1C0, 0x80, 0x80},
	{REG_RX_00F, 0x03, 0x00},	//change bank 0
#endif
//FIX_ID_037 xxxxx

#ifdef _HBR_I2S_
	{REG_RX_07E, B_HBRSel, 0x00},
#else
	{REG_RX_07E, B_HBRSel, B_HBRSel},
#endif

	{REG_RX_052, (B_DisVAutoMute), (B_DisVAutoMute)},				//Reg52[5] = 1 for disable Auto video MUTE
	{REG_RX_053, (B_VDGatting | B_VIOSel | B_TriVDIO | B_TriSYNC), (B_VIOSel | B_TriVDIO | B_TriSYNC)},				//Reg53[7][5] = 01    // for disable B_VDIO_GATTING

	{REG_RX_058, 0xFF, 0x33},			// Reg58 for 4Kx2K Video output Driving Strength

//	{REG_RX_059,0xFF,0xAA},			// Reg59 for Audio output Driving Strength

//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
//!!!  For Manual Adjust EQ only  !!!
#ifdef _SUPPORT_MANUAL_ADJUST_EQ_
	{REG_RX_03E, 0x20, 0x20},	// Enable OvWrRsCs
	{REG_RX_026, 0x20, 0x20},	// Enable OvWrRsCs
#endif
//FIX_ID_001 xxxxx


#ifdef _ONLY_SUPPORT_MANUAL_EQ_ADJUST_
	{REG_RX_026,	0xFF,	0x20},	//Reg26=0x00 disable Auto Trigger
	{REG_RX_03E,	0xFF,	0x20},	//Reg3E=0x00 disable Auto Trigger
#endif

	//RS initial valie
	// Dr. Liu said, reg25/reg3D should set as 0x1F for auto EQ start option.
	{REG_RX_025, 0xFF, DEFAULT_EQVALUE},
	{REG_RX_03D, 0xFF, DEFAULT_EQVALUE},
	{REG_RX_027, 0xFF, DEFAULT_EQVALUE},	// B ch
	{REG_RX_028, 0xFF, DEFAULT_EQVALUE},	// G
	{REG_RX_029, 0xFF, DEFAULT_EQVALUE},	// R
	{REG_RX_03F, 0xFF, DEFAULT_EQVALUE},
	{REG_RX_040, 0xFF, DEFAULT_EQVALUE},
	{REG_RX_041, 0xFF, DEFAULT_EQVALUE},

	{REG_RX_00F,	0x03,	0x01},	//change bank 1	//2013-0515 Andrew suggestion	for Auto EQ
	{REG_RX_1BC,	0xFF,	0x06},	//Reg1BC=0x06		//2013-0515 Andrew suggestion	for Auto EQ
//FIX_ID_020 xxxxx		//Turn off DEQ for HDMI port 1 with 20m DVI Cable
	{REG_RX_1CC,	0xFF,	0x00},	//Reg1CC=0x00		for TURN OFF DEQ
	{REG_RX_1C6,      0x07,      0x03},	// [2:0]Reg_P1_ENHYS = 03 for default enable filter to gating output
//FIX_ID_020 xxxxx

	{REG_RX_1B5,	0x03,	0x03},	//Reg1B5[1:0]='11'	for fix Korea K706 MHL pattern Generator	//2013-0515 Andrew suggestion
//FIX_ID_019	xxxxx modify ENHYS control for MHL mode
	{REG_RX_1B8,      0x80,      0x00},	// [7] Reg_HWENHYS = 0
	{REG_RX_1B6,      0x07,      0x03},	// [2:0]Reg_P0_ENHYS = 03 for default enable filter to gating output
//FIX_ID_019	xxxxx


//FIX_ID_029	xxxxx fixed Ulta-2000 HDCP fail issue at Receiver mode
	{REG_RX_128,      0xFF,      0x00},	// Clear KSV LIST
	{REG_RX_129,      0xFF,      0x00},	// Clear KSV LIST
	{REG_RX_12A,      0xFF,      0x00},	// Clear KSV LIST
	{REG_RX_12B,      0xFF,      0x00},	// Clear KSV LIST
	{REG_RX_12C,      0xFF,      0x00},	// Clear KSV LIST
//FIX_ID_029	xxxxx

	{REG_RX_00F,	0x03,	0x00},	//change bank 0	//2013-0515 Andrew suggestion	for Auto EQ

//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
// 	for Auto EQ
#ifdef _SUPPORT_AUTO_EQ_
//0704 disable ->	{REG_RX_022,	0xFF,	0x38},	//Reg22=0x38		//2013-0515 Andrew suggestion	for Auto EQ
//0704 disable ->	{REG_RX_03A,	0xFF,	0x38},	//Reg3A=0x38		//2013-0515 Andrew suggestion	for Auto EQ
	{REG_RX_022,	0xFF,	0x00},	// 07-16 Reg22=0x30	power down auto EQ
	{REG_RX_03A,	0xFF,	0x00},	// 07-16 Reg3A=0x30	power down auto EQ

#ifdef ENABLE_AUTO_TRIGGER
	{REG_RX_026,	0xFF,	0x80},	//Reg26=0x80	enable Auto Trigger
	{REG_RX_03E,	0xFF,	0x80},	//Reg3E=0x80	enable Auto Trigger
#else
	{REG_RX_026,	0xFF,	0x00},	//Reg26=0x00 disable Auto Trigger
	{REG_RX_03E,	0xFF,	0x00},	//Reg3E=0x00 disable Auto Trigger
#endif

#else
	{REG_RX_022,	0xFF,	0x00},	// 07-16 Reg22=0x30	power down auto EQ
	{REG_RX_03A,	0xFF,	0x00},	// 07-16 Reg3A=0x30	power down auto EQ

	{REG_RX_026,	0xFF,	0x00},	// 07-16 Reg26=0x00 disable Auto Trigger
	{REG_RX_03E,	0xFF,	0x00},	// 07-16 Reg3E=0x00 disable Auto Trigger

#endif

	// {REG_RX_014,0xFF,0xFF},		//for enable interrupt output Pin
	// {REG_RX_063,0xFF,0x3F},		//for enable interrupt output Pin MZY 17/5/5

	{REG_RX_073, 0x08, 0x00},		// for HDCPIntKey = false

	{REG_RX_060, 0x40, 0x00},		// disable interrupt mask for NoGenPkt_Rcv

	//FIX_ID_017 xxxxx Disable IPLockChk
	{REG_RX_02A, 0x01, 0x00},		// disable PORT 0 EnIPLockChk
	{REG_RX_042, 0x01, 0x00},		// disable PORT 1 EnIPLockChk
	//{REG_RX_035, 0x02, 0x00},		// disable PORT 0 EnIPLockChk
	//{REG_RX_04B, 0x02, 0x00},
	//FIX_ID_017 xxxxx


#if defined(_IT66023_)
//FIX_ID_016 xxxxx Support Dual Pixel Mode for IT66023 Only
	{REG_RX_08C, 0x09, 0x09},		// Reg8C[0] = 1	// SPOutMode�G//  for enable IO Mapping for Signal Pixel mode
//FIX_ID_016 xxxxx
#endif


//FIX_ID_025 xxxxx Audio lock method select for HDMI Repeater / splitter application
	{REG_RX_077, 0x0C, 0x08},		// Reg77[3:2] = 01	Audio lock method select
//FIX_ID_025 xxxxx

	{0xFF, 0xFF, 0xFF},
};


//FIX_ID_036	xxxxx

static _CODE unsigned char bCSCMtx_RGB2YUV_ITU601_16_235[] = {
	0x00,		0x80,		0x10,
	0xB2, 0x04,	0x65, 0x02,	0xE9, 0x00,
	0x93, 0x3C,	0x18, 0x04,	0x55, 0x3F,
	0x49, 0x3D,	0x9F, 0x3E,	0x18, 0x04
};

static unsigned char bCSCMtx_RGB2YUV_ITU601_0_255[] = {
	0x10,		0x80,		0x10,
	0x09, 0x04,	0x0E, 0x02,	0xC9, 0x00,
	0x0F, 0x3D,	0x84, 0x03,	0x6D, 0x3F,
	0xAB, 0x3D,	0xD1, 0x3E,	0x84, 0x03
};

static _CODE unsigned char bCSCMtx_RGB2YUV_ITU709_16_235[] = {
	0x00,		0x80,		0x10,
	0xB8, 0x05,	0xB4, 0x01,	0x94, 0x00,
	0x4A, 0x3C,	0x17, 0x04,	0x9F, 0x3F,
	0xD9, 0x3C,	0x10, 0x3F,	0x17, 0x04
};

static _CODE unsigned char bCSCMtx_RGB2YUV_ITU709_0_255[] = {
	0x10,		0x80,		0x10,
	0xEA, 0x04,	0x77, 0x01,	0x7F, 0x00,
	0xD0, 0x3C,	0x83, 0x03,	0xAD, 0x3F,
	0x4B, 0x3D,	0x32, 0x3F,	0x83, 0x03
};


// static _CODE unsigned char bCSCMtx_YUV2RGB_ITU601_16_235[] =
// {
// 	0x00,		0x00,		0x00,
// 	0x00,0x08,	0x6B,0x3A,	0x50,0x3D,
// 	0x00,0x08,	0xF5,0x0A,	0x02,0x00,
// 	0x00,0x08,	0xFD,0x3F,	0xDA,0x0D
// } ;

// static _CODE unsigned char bCSCMtx_YUV2RGB_ITU601_0_255[] =
// {
// 	0x04,		0x00,		0xA7,
// 	0x4F,0x09,	0x81,0x39,	0xDD,0x3C,
// 	0x4F,0x09,	0xC4,0x0C,	0x01,0x00,
// 	0x4F,0x09,	0xFD,0x3F,	0x1F,0x10
// } ;

// static _CODE unsigned char bCSCMtx_YUV2RGB_ITU709_16_235[] =
// {
// 	0x00,		0x00,		0x00,
// 	0x00,0x08,	0x55,0x3C,	0x88,0x3E,
// 	0x00,0x08,	0x51,0x0C,	0x00,0x00,
// 	0x00,0x08,	0x00,0x00,	0x84,0x0E
// } ;

// static _CODE unsigned char bCSCMtx_YUV2RGB_ITU709_0_255[] =
// {
// 	0x04,		0x00,		0xA7,
// 	0x4F,0x09,	0xBA,0x3B,	0x4B,0x3E,
// 	0x4F,0x09,	0x57,0x0E,	0x02,0x00,
// 	0x4F,0x09,	0xFE,0x3F,	0xE8,0x10
// } ;

//FIX_ID_027 xxxxx Support Full/Limited Range convert
//full 2 limit
// static _CODE unsigned char bCSCMtx_RGB_0_255_RGB_16_235[] =
// {
// 	0x10,		0x10,		0x00,
// 	0xe0,0x06,	0x00,0x00,	0x00,0x00,
// 	0x00,0x00,	0xe0,0x06,	0x00,0x00,
// 	0x00,0x00,	0x00,0x00,	0xe0,0x06,


// } ;

//limit 2 full
// static _CODE unsigned char bCSCMtx_RGB_16_235_RGB_0_255[] =
// {
// 	0xED,		0xED,		0x00,
// 	0x50,0x09,	0x00,0x00,	0x00,0x00,
// 	0x00,0x00,	0x50,0x09,	0x00,0x00,
// 	0x00,0x00,	0x00,0x00,	0x50,0x09,
// } ;
//FIX_ID_027 xxxxx

#ifdef  _SUPPORT_EDID_RAM_
// EDID_SELECT_TABLE
// (0) IT6602 support 4K2k
// (1) IT6602 3D
// (2) Philips monitor for 4kX2k
// (3) AOC monitor without 3D
// (4) Astro 1831 HDMI analyzer
// (5) TI PICO 343X EDID
// (6) IT6602 with 640x480p , 720x480p , 1280x720p , 1920x1080p
// (7) IT6602 with 640x480p , 720x480p , 1280x720p , 1920x1080p, 1440x480i
// (8) artosyn edid 720P60,1080P30
// (9) only 720p 60fs
#define EDID_SELECT_TABLE	(8)

static unsigned char _CODE Default_Edid_Block[256] = {

	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x06, 0x8F, 0x07, 0x11, 0x01, 0x00, 0x00, 0x00,
	0x17, 0x11, 0x01, 0x03, 0x80, 0x0C, 0x09, 0x78, 0x0A, 0x1E, 0xAC, 0x98, 0x59, 0x56, 0x85, 0x28,
	0x29, 0x52, 0x57, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E,
	0x96, 0x00, 0x81, 0x60, 0x00, 0x00, 0x00, 0x18, 0x01, 0x1D, 0x80, 0x18, 0x71, 0x1C, 0x16, 0x20,
	0x58, 0x2C, 0x25, 0x00, 0x81, 0x49, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x56,
	0x41, 0x2D, 0x31, 0x38, 0x30, 0x39, 0x41, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFD,
	0x00, 0x17, 0x3D, 0x0D, 0x2E, 0x11, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x9C,
	0x02, 0x03, 0x0F, 0x10, 0x42, 0x04, 0x22, 0x67, 0x03, 0x0C, 0x00, 0x10, 0x00, 0x88, 0x2D, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39

	// 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x05, 0xE3, 0x69, 0x23, 0x66, 0x00, 0x00, 0x00,
	// 0x0D, 0x17, 0x01, 0x03, 0x80, 0x33, 0x1D, 0x78, 0x2A, 0xE5, 0x95, 0xA6, 0x56, 0x52, 0x9D, 0x27,
	// 0x10, 0x50, 0x54, 0x00, 0x00, 0x00, 0x01, 0x01, 0x81, 0xC0, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	// 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x1D, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40, 0x58, 0x2C,
	// 0x45, 0x00, 0xFD, 0x1E, 0x11, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0xFD, 0x00, 0x32, 0x4C, 0x1E,
	// 0x53, 0x11, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x32,
	// 0x33, 0x36, 0x39, 0x4D, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFF,
	// 0x00, 0x42, 0x46, 0x44, 0x44, 0x33, 0x39, 0x41, 0x30, 0x30, 0x30, 0x31, 0x30, 0x32, 0x01, 0x76,
	// 0x02, 0x03, 0x20, 0xF1, 0x4D, 0xA2, 0x04, 0x13, 0x03, 0x01, 0x11, 0x02, 0x12, 0x3E, 0x3D, 0x3C,
	// 0x21, 0x20, 0x23, 0x09, 0x06, 0x01, 0x83, 0x01, 0x00, 0x00, 0x65, 0x03, 0x0C, 0x00, 0x10, 0x00,
	// 0x01, 0x1D, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40, 0x58, 0x2C, 0x45, 0x00, 0xFD, 0x1E, 0x11, 0x00,
	// 0x00, 0x1E, 0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0, 0x1E, 0x20, 0x6E, 0x28, 0x55, 0x00, 0xFD, 0x1E,
	// 0x11, 0x00, 0x00, 0x1E, 0x01, 0x1D, 0x00, 0xBC, 0x52, 0xD0, 0x1E, 0x20, 0xB8, 0x28, 0x55, 0x40,
	// 0xFD, 0x1E, 0x11, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E,
	// 0x96, 0x00, 0xFD, 0x1E, 0x11, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	// 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3D
};
#endif

const char *VStateStr[] = {
	"VSTATE_Off",
	"VSTATE_TerminationOff",
	"VSTATE_TerminationOn",
	"VSTATE_5VOff",
	"VSTATE_SyncWait",
	"VSTATE_SWReset",
	"VSTATE_SyncChecking",
	"VSTATE_HDCPSet",
	"VSTATE_HDCP_Reset",
	"VSTATE_ModeDetecting",
	"VSTATE_VideoOn",
	"VSTATE_ColorDetectReset",
	"VSTATE_HDMI_OFF",
	"VSTATE_Reserved",
};



const char  *VModeStateStr[] = {
	"0 eRGB444_SDR",
	"1 eYUV444_SDR",
	"2 eRGB444_DDR",
	"3 eYUV444_DDR",
	"4 eYUV422_Emb_Sync_SDR",
	"5 eYUV422_Emb_Sync_DDR",
	"6 eYUV422_Sep_Sync_SDR",
	"7 eYUV422_Sep_Sync_DDR",
	"8 eCCIR656_Emb_Sync_SDR",
	"9 eCCIR656_Emb_Sync_DDR",
	"10 eCCIR656_Sep_Sync_SDR",
	"11 eCCIR656_Sep_Sync_DDR",
	"12 eRGB444_Half_Bus",
	"13 eYUV444_Half_Bus",
	"14 eBTA1004_SDR",
	"15 eBTA1004_DDR",
};

#ifndef _ITEHDMI_
#define _ITEHDMI_
#endif


// transfer to cpu2
static struct h264_input_format_s att;
static orb_advert_t h264_input_format_topic;


static void IT_Delay(uint32_t delay);

static void IT_Delay(uint32_t delay)
{
	usleep(delay * 1000);
	// if (40 * delay > 0xffffffff)
	// {
	// 	for (uint32_t i = 0; i < 0xffffffff; i++)
	// 	{
	// 		;
	// 	}
	// }
	// else
	// {
	// 	for (uint32_t i = 0; i < 40 * delay; i++)
	// 	{
	// 		;
	// 	}
	// }
}

///////////////////////////////////////

//FIX_ID_003 xxxxx	//Add IT6602 Video Output Configure setting
// 0 eRGB444_SDR=0,
// 1	eYUV444_SDR,
// 2	eRGB444_DDR,
// 3	eYUV444_DDR,
// 4	eYUV422_Emb_Sync_SDR,
// 5	eYUV422_Emb_Sync_DDR,
// 6	eYUV422_Sep_Sync_SDR,
// 7	eYUV422_Sep_Sync_DDR,
// 8	eCCIR656_Emb_Sync_SDR,
// 9	eCCIR656_Emb_Sync_DDR,
// 10 eCCIR656_Sep_Sync_SDR,
// 11 eCCIR656_Sep_Sync_DDR,
// 12 eRGB444_Half_Bus,
// 13 eYUV444_Half_Bus,
// 14 eBTA1004_SDR,
// 15 eBTA1004_DDR
//06-27 disable --> #define HDMIRX_OUTPUT_VID_MODE (F_MODE_EN_UDFILT | F_MODE_RGB444)
// static unsigned char HDMIRX_OUTPUT_VID_MODE = (unsigned char)eYUV422_Sep_Sync_SDR;
static unsigned char HDMIRX_OUTPUT_VID_MODE = (unsigned char)eCCIR656_Sep_Sync_SDR;

//FIX_ID_003 xxxxx

/*****************************************************************************/
/* Private and Local Variables    ********************************************/
/*****************************************************************************/
#if 1

unsigned char V3D_EntryCnt = 0;
unsigned char wrburstoff, wrburstnum;
unsigned char TxWrBstSeq = 0;

//FIX_ID_013	xxxxx	//For Acer MHL Dongle MSC 3D request issue
//unsigned char  EnMSCWrBurst3D  = TRUE;
//unsigned char  EnMHL3DSupport  = FALSE;
//FIX_ID_013	xxxxx

unsigned char wakeupcnt = 0;
#define MinEQValue 0x03

#ifdef _SUPPORT_AUTO_EQ_
unsigned char ucPortAMPOverWrite[2];
unsigned char ucPortAMPValid[2];
unsigned char ucChannelB[2]; // ch0
unsigned char ucChannelG[2]; // ch1
unsigned char ucChannelR[2]; // ch2
unsigned char ucEQMode[2];
#endif

unsigned char HdmiI2cAddr = IT66021A_HDMI_ADDR;

int CurTMDSCLK;
volatile VTiming CurVTiming;
AVI_InfoFrame aviinfoframe;
int InColorMode = RGB444;  //RGB444, YCbCr422, YCbCr444
int OutColorMode = RGB444; //RGB444, YCbCr422, YCbCr444
int OutCD = OUT8B;
int VIC;

#ifdef _FIX_ID_028_
//FIX_ID_028 xxxxx //For Debug Audio error with S2
static unsigned char m_bAudioWaiting = 0;
//FIX_ID_028 xxxxx
#else

#ifdef EnableCalFs
//FIX_ID_023 xxxxx		//Fixed for Audio Channel Status Error with invalid HDMI source
static unsigned int m_u16TMDSCLK = 0;
static unsigned char m_ForceFsValue = 0;
//static unsigned long m_ROSCCLK;
static unsigned char m_AudioChannelStatusErrorCount = 0;
#define MAX_AUDIO_CHANNEL_STATUS_ERROR 4
//FIX_ID_023 xxxxx
#endif
#endif

/****************************************************************************/
/*							EDID Argument									*/
/****************************************************************************/
unsigned char VSDB_Addr;											   // for EDID RAM function

unsigned char txphyadr[2], txphyA, txphyB, txphyC, txphyD, txphylevel; // for CEC function

unsigned char rxphyadr[2][2];										   // for EDID RAM function

unsigned char rxphyA, rxphyB, rxphyC, rxphyD, rxcurport;			   // for CEC function

#ifdef FIX_ID_013
//FIX_ID_013	xxxxx	//For MSC 3D request issue
unsigned char uc3DDtd[] = {0x00};
struct PARSE3D_STR st3DParse;
MHL3D_STATE e3DReqState = MHL3D_REQ_DONE;
unsigned char SVD_LIST[16];
//unsigned char STRUCTURE_3D[16]={1,0,0,0,0,0,2,0,4,0,0,0,0,0,0,0};
//FIX_ID_013	xxxxx
#endif //FIX_ID_013

#endif


//FIX_ID_037 xxxxx //Allion MHL compliance issue !!!
unsigned char m_MHLabortID = 0x00;

// default: do nothing
// 1 test retry HPD with MSC abort command
// 2 test retry Path_EN with MSC abort command
// 3 test retry DCap Ready with MSC abort command
// 4 test retry Read DCap register with MSC abort command
#define MHL_ABORT_ID (0x00)

///////////////////////////////// hdmi /////////////////////////////

// TODO check
uint8_t IT66021::HDMI_RX_MapToDeviceIndex(ENUM_HAL_HDMI_RX e_hdmiIndex)
{
	return (e_hdmiIndex == HAL_HDMI_RX_0) ? 1 : 0;
}


uint32_t IT66021::HDMI_RX_CheckVideoFormatChangeOrNot(ENUM_HAL_HDMI_RX e_hdmiIndex,
		uint16_t u16_width,
		uint16_t u16_hight,
		uint8_t u8_framerate)
{
	STRU_HDMI_RX_OUTPUT_FORMAT videoFormat = this->s_st_hdmiRxStatus.st_videoFormat;

	if (videoFormat.u16_width != u16_width || videoFormat.u16_hight != u16_hight
	    || videoFormat.u8_framerate != u8_framerate) {
		return HAL_OK;
	}

	return HAL_HDMI_RX_FALSE;
}

uint8_t IT66021::IT_66021_GetVideoFormat(uint8_t index, uint16_t *widthPtr, uint16_t *hightPtr, uint8_t *framteratePtr,
		uint8_t *vic)
{
	if (this->it6602DEV.m_VState == VSTATE_VideoOn) {
		uint8_t i = 0;
		uint8_t array_size = sizeof(s_u8Array_ARCastSupportedOutputFormat) / sizeof(s_u8Array_ARCastSupportedOutputFormat[0]);
		IT_INFO("%d", array_size);

		for (i = 0; i < array_size; i++) {
			if (this->it6602DEV.VIC == s_u8Array_ARCastSupportedOutputFormat[i][3]) {
				*widthPtr = s_u8Array_ARCastSupportedOutputFormat[i][0];
				*hightPtr = s_u8Array_ARCastSupportedOutputFormat[i][1];
				*framteratePtr = s_u8Array_ARCastSupportedOutputFormat[i][2];
				*vic = this->it6602DEV.VIC;

				IT_INFO("111 widthPtr = %d \n", *widthPtr);
				IT_INFO("222 hightPtr = %d \n", *hightPtr);
				IT_INFO("333 framteratePtr = %d \n", *framteratePtr);
				IT_INFO("444 vic = %d \n", *vic);

				return TRUE;
			}
		}

		// uint8_t hdmi_i2c_addr = (index == 0) ? RX_I2C_HDMI_MAP_ADDR : (RX_I2C_HDMI_MAP_ADDR + 2);

		uint32_t u32_HTotal   = ((hdmirxrd(0x9D) & 0x3F) << 8) + hdmirxrd(0x9C);
		uint32_t u32_HActive  = ((hdmirxrd(0x9F) & 0x3F) << 8) + hdmirxrd(0x9E);

		uint32_t u32_VTotal   = ((hdmirxrd(0xA4) & 0x0F) << 8) + hdmirxrd(0xA3);
		uint32_t u32_VActive  = ((hdmirxrd(0xA4) & 0xF0) << 4) + hdmirxrd(0xA5);

		IT_INFO("u32_HTotal = %d \n", u32_HTotal);
		IT_INFO("u32_HActive = %d \n", u32_HActive);
		IT_INFO("u32_VTotal = %d \n", u32_VTotal);
		IT_INFO("u32_VActive = %d \n", u32_VActive);


		uint8_t u8_rddata = hdmirxrd(0x9A);
		uint32_t PCLK = (124 * 255 / u8_rddata) / 10;

		uint64_t u64_FrameRate = (uint64_t)(PCLK) * 1000 * 1000;
		u64_FrameRate /= u32_HTotal;
		u64_FrameRate /= u32_VTotal;

		IT_INFO("PCLK = %d, u64_FrameRate = %d", PCLK, u64_FrameRate);

		*widthPtr = (uint16_t)u32_HActive;
		*hightPtr = (uint16_t)u32_VActive;

		if ((u64_FrameRate > 55) || (u64_FrameRate > 65)) {
			*framteratePtr = 60;

		} else if ((u64_FrameRate > 45) || (u64_FrameRate > 55)) {
			*framteratePtr = 50;

		} else if ((u64_FrameRate > 25) || (u64_FrameRate > 35)) {
			*framteratePtr = 30;
		}

		*vic = 0xff;
		return TRUE;

	} else {
		*widthPtr = 0;
		*hightPtr = 0;
		*framteratePtr = 0;
		*vic = 0xff;
		return FALSE;
	}
}



void IT66021::HDMI_RX_CheckFormatStatus(ENUM_HAL_HDMI_RX e_hdmiIndex, uint32_t b_noDiffCheck)
{
	static uint8_t s_u8_formatNotSupportCount = 0;

	uint16_t u16_width;
	uint16_t u16_hight;
	uint8_t u8_framerate;
	uint8_t u8_vic;

	uint8_t u8_hdmiIndex = HDMI_RX_MapToDeviceIndex(e_hdmiIndex);

	IT_66021_GetVideoFormat(u8_hdmiIndex, &u16_width, &u16_hight, &u8_framerate, &u8_vic);

	if (h264_input_format_topic == NULL) {
		h264_input_format_topic = orb_advertise(ORB_ID(h264_input_format), &att);
	}

	if (HDMI_RX_CheckVideoFormatSupportOrNot(u16_width, u16_hight, u8_framerate) == HAL_OK) {
		s_u8_formatNotSupportCount = 0;

		if ((b_noDiffCheck == HAL_OK) ||
		    (HDMI_RX_CheckVideoFormatChangeOrNot(e_hdmiIndex, u16_width, u16_hight, u8_framerate) == HAL_OK)) {
			// STRU_SysEvent_H264InputFormatChangeParameter p;
			att.index = this->s_st_hdmiRxStatus.st_configure.u8_hdmiToEncoderCh;
			att.width = u16_width;
			att.hight = u16_hight;
			att.framerate = u8_framerate;
			att.vic = u8_vic;

			if (HAL_HDMI_RX_0 == e_hdmiIndex) {
				att.e_h264InputSrc = ENCODER_INPUT_SRC_HDMI_0;

			} else {
				att.e_h264InputSrc = ENCODER_INPUT_SRC_HDMI_1;
			}

			IT_INFO("index = %d, width = %d, height = %d, frame = %d, vic = %d, e_h264InputSrc = %d", \
				att.index, att.width, att.hight, att.framerate, att.vic, att.e_h264InputSrc);

			orb_publish(ORB_ID(h264_input_format), h264_input_format_topic, &att);

			this->s_st_hdmiRxStatus.st_videoFormat.u16_width    = u16_width;
			this->s_st_hdmiRxStatus.st_videoFormat.u16_hight    = u16_hight;
			this->s_st_hdmiRxStatus.st_videoFormat.u8_framerate = u8_framerate;
			this->s_st_hdmiRxStatus.st_videoFormat.u8_vic = u8_vic;
		}

	} else {
		// STRU_SysEvent_H264InputFormatChangeParameter p;
		att.index = HAL_HDMI_RX_8BIT; // this->s_st_hdmiRxStatus.st_configure.u8_hdmiToEncoderCh;
		att.width = 0;
		att.hight = 0;
		att.framerate = 0;
		att.vic = u8_vic;

		if (HAL_HDMI_RX_0 == e_hdmiIndex) {
			att.e_h264InputSrc = ENCODER_INPUT_SRC_HDMI_0;

		} else {
			att.e_h264InputSrc = ENCODER_INPUT_SRC_HDMI_1;
		}

		// SYS_EVENT_NotifyLocal(SYS_EVENT_ID_VIDEO_EVENT, (void*)&p);
		orb_publish(ORB_ID(h264_input_format), h264_input_format_topic, &att);

		this->s_st_hdmiRxStatus.st_videoFormat.u16_width    = 0;
		this->s_st_hdmiRxStatus.st_videoFormat.u16_hight    = 0;
		this->s_st_hdmiRxStatus.st_videoFormat.u8_framerate = 0;
		this->s_st_hdmiRxStatus.st_videoFormat.u8_vic = u8_vic;

	}
}

unsigned char IT66021::CheckAVMute(void)
{
	unsigned char ucAVMute;
	unsigned char ucPortSel;

	ucAVMute = hdmirxrd(REG_RX_0A8) & (B_P0_AVMUTE | B_P1_AVMUTE);
	ucPortSel = hdmirxrd(REG_RX_051) & B_PORT_SEL;

	if (((ucAVMute & B_P0_AVMUTE) && (ucPortSel == F_PORT_SEL_0)) ||
	    ((ucAVMute & B_P1_AVMUTE) && (ucPortSel == F_PORT_SEL_1))) {
		return TRUE;

	} else {
		return FALSE;
	}
}

unsigned char IT66021::CheckPlg5VPwr(unsigned char ucPortSel)
{
	if (ucPortSel != 0) {
		PX4_ERR("it66021 only support Port0 in reg51 \r\n");
		return FALSE;
	}

	unsigned char sys_state_P0 = hdmirxrd(REG_RX_P0_SYS_STATUS);

	// IT_INFO("CheckPlg5VPwr: sys_state_P0 = %d", sys_state_P0);

	return sys_state_P0 & B_P0_PWR5V_DET;
}

// ---------------------------------------------------------------------------
unsigned char IT66021::IsHDMIMode(void)
{
	unsigned char sys_state_P0;
	unsigned char sys_state_P1;
	unsigned char ucPortSel;

	// TODO: 0x0B 与 Ox51 中的 0x01
	sys_state_P0 = hdmirxrd(REG_RX_P0_SYS_STATUS) & B_P0_HDMI_MODE;
	sys_state_P1 = hdmirxrd(REG_RX_P1_SYS_STATUS) & B_P1_HDMI_MODE;
	ucPortSel = hdmirxrd(REG_RX_051) & B_PORT_SEL;

	IT_INFO("P0 = %d, p1 = %d, sel = %d", sys_state_P0, sys_state_P1, ucPortSel);

	if (((sys_state_P0 & B_P0_HDMI_MODE) && (ucPortSel == F_PORT_SEL_0)) ||
	    ((sys_state_P1 & B_P1_HDMI_MODE) && (ucPortSel == F_PORT_SEL_1))) {
		return TRUE;

	} else {
		return FALSE;
	}
}

void IT66021::GetAVIInfoFrame(struct it6602_dev_data *it6602)
{
	chgbank(2);
	it6602->ColorMode = ((hdmirxrd(REG_RX_AVI_DB1) & 0x60) >> 5);
	it6602->Colorimetry = ((hdmirxrd(REG_RX_AVI_DB2) & 0xC0) >> 6);
	it6602->ExtendedColorimetry = ((hdmirxrd(REG_RX_AVI_DB3) & 0x70) >> 4);
	it6602->RGBQuantizationRange = ((hdmirxrd(REG_RX_AVI_DB3) & 0x0C) >> 2);
	it6602->VIC = ((hdmirxrd(REG_RX_AVI_DB4) & 0x7F));
	it6602->YCCQuantizationRange = ((hdmirxrd(REG_RX_AVI_DB5) & 0xC0) >> 6);
	chgbank(0);

	//FIX_ID_027 xxxxx Support RGB limited / Full range convert
	if (it6602->RGBQuantizationRange == 0) {
		if (it6602->VIC >= 2) {
			// CE Mode
			it6602->RGBQuantizationRange = 1; // limited range

		} else {
			// IT mode
			it6602->RGBQuantizationRange = 2; // Full range
		}
	}

	IT_INFO("AVI ColorMode = %X \r\n", (int)it6602->ColorMode);
	IT_INFO("AVI Colorimetry = %X \r\n", (int)it6602->Colorimetry);
	IT_INFO("AVI ExtendedColorimetry = %X \r\n", (int)it6602->ExtendedColorimetry);
	IT_INFO("AVI RGBQuantizationRange = %X \r\n", (int)it6602->RGBQuantizationRange);
	IT_INFO("AVI VIC = %X \r\n", (int)it6602->VIC);
	IT_INFO("AVI YCCQuantizationRange = %X \r\n", (int)it6602->YCCQuantizationRange);
}

void IT66021::hdmirxbwr(unsigned char offset, unsigned char byteno, unsigned char *rddata)
{
	write(offset, rddata, byteno);
	// IT_66021_WriteBytes(HdmiI2cAddr, offset, byteno, rddata);
}

struct it6602_dev_data *IT66021::get_it6602_dev_data(void)
{
	return  &this->it6602DEV;
}

void IT66021::hdimrx_write_init(struct IT6602_REG_INI _CODE *tdata)
{
	int cnt = 0;

	while (tdata[cnt].ucAddr != 0xFF) {
		hdmirxset(tdata[cnt].ucAddr, tdata[cnt].andmask, tdata[cnt].ucValue);
		cnt++;
	}
}

/*****************************************************************************/
/* HDMIRX functions    *******************************************************/
/*****************************************************************************/

void IT66021::chgbank(int bank)
{
	switch (bank) {
	case 0:
		hdmirxset(0x0F, 0x03, 0x00);
		break;

	case 1:
		hdmirxset(0x0F, 0x03, 0x01);
		break;

	case 2:
		hdmirxset(0x0F, 0x03, 0x02);
		break;

	case 3:
		hdmirxset(0x0F, 0x03, 0x03);
		break;

	default:
		break;
	}
}

unsigned char IT66021::CheckSCDT(struct it6602_dev_data *it6602)
{
	unsigned char ucPortSel;
	unsigned char sys_state_P0;

	ucPortSel = hdmirxrd(REG_RX_051) & B_PORT_SEL;
	sys_state_P0 = hdmirxrd(REG_RX_P0_SYS_STATUS);

	IT_INFO("CheckSCDT: SEL = %d, po = %02x, curr = %d", ucPortSel, sys_state_P0, it6602->m_ucCurrentHDMIPort);

	if (ucPortSel == it6602->m_ucCurrentHDMIPort) {

		if (sys_state_P0 & B_P0_SCDT) {
			// it6602->m_ucSCDTOffCount=0;
			return TRUE;

		} else {
			//SCDT off
			return FALSE;
		}
	}

	return FALSE;
}

void IT66021::WaitingForSCDT(struct it6602_dev_data *it6602)
{
	unsigned char sys_state_P0;
	unsigned char ucPortSel;
	//	unsigned char ucTMDSClk ;

	sys_state_P0 = hdmirxrd(REG_RX_P0_SYS_STATUS) & (B_P0_SCDT | B_P0_PWR5V_DET | B_P0_RXCK_VALID);
	ucPortSel = hdmirxrd(REG_RX_051) & B_PORT_SEL;

	IT_INFO("WaitingForSCDT sys_state_P0 = %02x, ucPortSel = %02x", sys_state_P0, ucPortSel);

	if (sys_state_P0 & B_P0_SCDT) {
		IT6602SwitchVideoState(it6602, VSTATE_SyncChecking); //2013-0520
		return;

	} else {
#ifdef _SUPPORT_EQ_ADJUST_

		if (it6602->EQPort[ucPortSel].f_manualEQadjust == TRUE) { // ignore SCDT off when manual EQ adjust !!!
			IT_INFO("[WaitingForSCDT]: f_manualEQadjust = TRUE \n");
			return;
		}

#endif


		if (ucPortSel == F_PORT_SEL_0) {

			if ((sys_state_P0 & (B_P0_PWR5V_DET | B_P0_RXCK_VALID)) == (B_P0_PWR5V_DET | B_P0_RXCK_VALID)) {
				it6602->m_ucSCDTOffCount++;
				IT_INFO(" SCDT off count = %X ", (int)it6602->m_ucSCDTOffCount);
				IT_INFO(" sys_state_P0 = %X", (int)hdmirxrd(REG_RX_P0_SYS_STATUS));
			}
		}

		if ((it6602->m_ucSCDTOffCount) > SCDT_OFF_TIMEOUT) {
			it6602->m_ucSCDTOffCount = 0;
			IT_INFO(" WaitingForSCDT( ) CDR reset !!! \r\n");
			hdmirx_ECCTimingOut(ucPortSel);

#ifdef _SUPPORT_AUTO_EQ_
			//xxxxx
			DisableOverWriteRS(ucPortSel);
			TMDSCheck(ucPortSel);
//xxxxx
#endif
		}
	}
}

unsigned char IT66021::CLKCheck(unsigned char ucPortSel)
{
	unsigned char sys_state;

	if (ucPortSel == F_PORT_SEL_1) {
		sys_state = hdmirxrd(REG_RX_P1_SYS_STATUS) & (B_P1_RXCK_VALID);

	} else {
		sys_state = hdmirxrd(REG_RX_P0_SYS_STATUS) & (B_P0_RXCK_VALID);
	}

	IT_INFO("CLKCheck: %d", sys_state);

	if (sys_state == B_P0_RXCK_VALID) {
		return TRUE;

	} else {
		return FALSE;
	}
}

//FIX_ID_009 xxxxx	//verify interrupt event with reg51[0] select port
unsigned char IT66021::IT6602_IsSelectedPort(unsigned char ucPortSel)
{
	unsigned char ucCurrentPort;

	// struct it6602_dev_data *it6602data = get_it6602_dev_data();

	ucCurrentPort = hdmirxrd(REG_RX_051) & B_PORT_SEL;

	if (ucCurrentPort == ucPortSel) {
		return TRUE;

	} else {
		return FALSE;
	}
}

void IT66021::IT6602_VideoOutputConfigure_Init(struct it6602_dev_data *it6602, Video_Output_Configure eVidOutConfig)
{
	it6602->m_VidOutConfigMode = eVidOutConfig;

	switch (eVidOutConfig) {
	case eRGB444_SDR:
		it6602->m_bOutputVideoMode = F_MODE_RGB444;
		it6602->m_bOutputVideoMode = F_MODE_RGB444 | F_MODE_0_255;
		it6602->m_VidOutDataTrgger = eSDR;
		it6602->m_VidOutSyncMode = eSepSync;
		break;

	case eYUV444_SDR:
		it6602->m_bOutputVideoMode = F_MODE_YUV444;
		it6602->m_VidOutDataTrgger = eSDR;
		it6602->m_VidOutSyncMode = eSepSync;
		break;

	case eRGB444_DDR:
		it6602->m_bOutputVideoMode = F_MODE_RGB444;
		it6602->m_VidOutDataTrgger = eHalfPCLKDDR;
		it6602->m_VidOutSyncMode = eSepSync;
		break;

	case eYUV444_DDR:
		it6602->m_bOutputVideoMode = F_MODE_YUV444;
		it6602->m_VidOutDataTrgger = eHalfPCLKDDR;
		it6602->m_VidOutSyncMode = eSepSync;
		break;

	case eYUV422_Emb_Sync_SDR:
		it6602->m_bOutputVideoMode = F_MODE_YUV422;
		it6602->m_VidOutDataTrgger = eSDR;
		it6602->m_VidOutSyncMode = eEmbSync;
		break;

	case eYUV422_Emb_Sync_DDR:
		it6602->m_bOutputVideoMode = F_MODE_YUV422;
		it6602->m_VidOutDataTrgger = eHalfPCLKDDR;
		it6602->m_VidOutSyncMode = eEmbSync;
		break;

	case eYUV422_Sep_Sync_SDR:
		it6602->m_bOutputVideoMode = F_MODE_YUV422;
		it6602->m_VidOutDataTrgger = eSDR;
		it6602->m_VidOutSyncMode = eSepSync;
		break;

	case eYUV422_Sep_Sync_DDR:
		it6602->m_bOutputVideoMode = F_MODE_YUV422;
		it6602->m_VidOutDataTrgger = eHalfPCLKDDR;
		it6602->m_VidOutSyncMode = eSepSync;
		break;

	case eCCIR656_Emb_Sync_SDR:
		it6602->m_bOutputVideoMode = F_MODE_YUV422;
		it6602->m_VidOutDataTrgger = eSDR;
		it6602->m_VidOutSyncMode = eCCIR656EmbSync;
		break;

	case eCCIR656_Emb_Sync_DDR:
		it6602->m_bOutputVideoMode = F_MODE_YUV422;
		it6602->m_VidOutDataTrgger = eHalfPCLKDDR;
		it6602->m_VidOutSyncMode = eCCIR656EmbSync;
		break;

	case eCCIR656_Sep_Sync_SDR:
		it6602->m_bOutputVideoMode = F_MODE_YUV422;
		it6602->m_VidOutDataTrgger = eSDR;
		it6602->m_VidOutSyncMode = eCCIR656SepSync;
		break;

	case eCCIR656_Sep_Sync_DDR:
		it6602->m_bOutputVideoMode = F_MODE_YUV422;
		it6602->m_VidOutDataTrgger = eHalfPCLKDDR;
		it6602->m_VidOutSyncMode = eCCIR656SepSync;
		break;

	case eRGB444_Half_Bus:
		it6602->m_bOutputVideoMode = F_MODE_RGB444;
		it6602->m_VidOutDataTrgger = eHalfBusDDR;
		it6602->m_VidOutSyncMode = eSepSync;
		break;

	case eYUV444_Half_Bus:
		it6602->m_bOutputVideoMode = F_MODE_YUV444;
		it6602->m_VidOutDataTrgger = eHalfBusDDR;
		it6602->m_VidOutSyncMode = eSepSync;
		break;

	case eBTA1004_SDR: //BTA1004_SDR_Emb_Sync
		it6602->m_bOutputVideoMode = F_MODE_YUV422;
		it6602->m_VidOutDataTrgger = eSDR_BTA1004;
		it6602->m_VidOutSyncMode = eEmbSync;
		break;

	case eBTA1004_DDR: //BTA1004_DDR_Emb_Sync
		it6602->m_bOutputVideoMode = F_MODE_YUV422;
		it6602->m_VidOutDataTrgger = eDDR_BTA1004; // eHalfPCLKDDR
		it6602->m_VidOutSyncMode = eEmbSync;
		break;

	default:
		break;
	}
}

void IT66021::hdmirx_Var_init(struct it6602_dev_data *it6602)
{
	it6602->m_ucSCDTOffCount = 0;
	it6602->m_ucEccCount_P0 = 0;
	it6602->m_ucDeskew_P0 = 0;

	it6602->m_VState = VSTATE_Off;
	it6602->m_AState = ASTATE_AudioOff;
	it6602->m_RxHDCPState = RxHDCP_PwrOff;

	it6602->m_SWResetTimeOut = 0;
	it6602->m_VideoCountingTimer = 0;
	it6602->m_AudioCountingTimer = 0;

	it6602->m_bVideoOnCountFlag = FALSE;

	it6602->m_MuteAutoOff = FALSE;
	it6602->m_bUpHDMIMode = FALSE;
	it6602->m_bUpHDCPMode = FALSE;
	it6602->m_NewAVIInfoFrameF = FALSE;
	it6602->m_NewAUDInfoFrameF = FALSE;
	it6602->m_HDCPRepeater = FALSE;

	IT6602_VideoOutputConfigure_Init(it6602, (Video_Output_Configure)HDMIRX_OUTPUT_VID_MODE);

	it6602->m_bRxAVmute = FALSE;

#ifdef _SUPPORT_EQ_ADJUST_
	it6602->EQPort[0].ucEQState = 0xFF;
	it6602->EQPort[0].ucAuthR0 = 0;
	it6602->EQPort[0].ucECCvalue = 0;
	it6602->EQPort[0].ucECCfailCount = 0;
	it6602->EQPort[0].ucPkt_Err = 0; //Pkt_Err
	it6602->EQPort[0].ucPortID = F_PORT_SEL_0;

	it6602->EQPort[0].f_manualEQadjust = FALSE;

#endif

#ifdef _SUPPORT_AUTO_EQ_

	ucPortAMPOverWrite[0] = 0; //2013-0801
	ucPortAMPValid[0] = 0;
	ucChannelB[0] = 0;
	ucChannelG[0] = 0;
	ucChannelR[0] = 0;
#endif

	//FIX_ID_005 xxxxx 	//Add Cbus Event Handler
	it6602->CBusIntEvent = 0;
	it6602->CBusSeqNo = 0;
	it6602->CBusWaitNo = 0x00;
	//FIX_ID_005 xxxxx

	//FIX_ID_005 xxxxx 	//Add Cbus Event Handler
	it6602->HDMIIntEvent = 0;
	it6602->HDMIWaitNo[0] = 0;
	it6602->HDMIWaitNo[1] = 0;
	//FIX_ID_005 xxxxx

#ifdef _IT6607_GeNPacket_Usage_
	asdasdasda
	it6602->m_PollingPacket = 0;
	it6602->m_PacketState = 0;
	it6602->m_ACPState = 0;
	it6602->m_GamutPacketRequest = FALSE;
	it6602->m_GeneralRecPackType = 0x00;
#endif
	it6602->m_ucCurrentHDMIPort = 0xff;

	//FIX_ID_034 xxxxx //Add MHL HPD Control by it6602HPDCtrl( )
	it6602->m_DiscoveryDone = 0;
	//FIX_ID_034 xxxxx

	//FIX_ID_037 xxxxx //Allion MHL compliance issue !!!
	//xxxxx 2014-0529 //Manual Content On/Off
	it6602->m_RAP_ContentOff = 0;
	it6602->m_HDCP_ContentOff = 0;
	//xxxxx 2014-0529
	//FIX_ID_037 xxxxx
}

void IT66021::IT6602_Rst(struct it6602_dev_data *it6602)
{
	hdmirx_Var_init(it6602);
	hdimrx_write_init(IT6602_HDMI_INIT_TABLE);
}

//=========================================================================//
char IT66021::IT6602_fsm_init(void)
{
	struct it6602_dev_data *it6602data = get_it6602_dev_data();

	IT6602_Rst(it6602data);

	EDIDRAMInitial(&Default_Edid_Block[0]);

	hdmirxset(REG_RX_0C0, 0x20, 0x20);

	IT_Delay(1);

	hdmirxset(REG_RX_0C0, 0x20, 0x00);

	it6602PortSelect(0);

	return TRUE;
}

// ---------------------------------------------------------------------------
void IT66021::SetVideoInputFormatWithInfoFrame(struct it6602_dev_data *it6602)
{
	unsigned char i;

	chgbank(2);
	i = hdmirxrd(REG_RX_215); //REG_RX_AVI_DB1
	chgbank(0);
	it6602->m_bInputVideoMode &= ~F_MODE_CLRMOD_MASK;

	switch ((i >> O_AVI_COLOR_MODE) & M_AVI_COLOR_MASK) {
	case B_AVI_COLOR_YUV444:
		IT_INFO("input YUV444 mode ");
		it6602->m_bInputVideoMode |= F_MODE_YUV444;
		break;

	case B_AVI_COLOR_YUV422:
		IT_INFO("input YUV422 mode ");
		it6602->m_bInputVideoMode |= F_MODE_YUV422;
		break;

	case B_AVI_COLOR_RGB24:
		IT_INFO("input RGB24 mode ");
		it6602->m_bInputVideoMode |= F_MODE_RGB24;
		break;

	default:
		return;
	}

	IT_INFO("SetVideoInputFormatWithInfoFrame - RegAE=%X it6602->m_bInputVideoMode=%X\n", (int)i,
		(int)it6602->m_bInputVideoMode);
	i = hdmirxrd(REG_RX_IN_CSC_CTRL);
	i &= ~B_IN_FORCE_COLOR_MODE;
	hdmirxwr(REG_RX_IN_CSC_CTRL, i);
}

// ---------------------------------------------------------------------------
void IT66021::SetColorimetryByInfoFrame(struct it6602_dev_data *it6602)
{
	unsigned char i;

	chgbank(2);
	i = hdmirxrd(REG_RX_216); //REG_RX_AVI_DB2
	chgbank(0);

	i &= M_AVI_CLRMET_MASK << O_AVI_CLRMET;

	if (i == (B_AVI_CLRMET_ITU601 << O_AVI_CLRMET)) {
		it6602->m_bInputVideoMode &= ~F_MODE_ITU709;

	} else if (i == (B_AVI_CLRMET_ITU709 << O_AVI_CLRMET)) {
		it6602->m_bInputVideoMode |= F_MODE_ITU709;
	}
}

void IT66021::SetCSCBYPASS(struct it6602_dev_data *it6602)
{
	it6602->m_bOutputVideoMode = it6602->m_bInputVideoMode;

	switch (it6602->m_bInputVideoMode & F_MODE_CLRMOD_MASK) {
	case F_MODE_RGB24:
		hdmirxset(REG_RX_OUT_CSC_CTRL, (M_OUTPUT_COLOR_MASK), B_OUTPUT_RGB24);
#ifdef _SUPPORT_RBSWAP_
		hdmirxset(REG_RX_064, 0x18, 0x08);
#endif
		break;

	case F_MODE_YUV422:
		hdmirxset(REG_RX_OUT_CSC_CTRL, (M_OUTPUT_COLOR_MASK), B_OUTPUT_YUV422);
#ifdef _SUPPORT_RBSWAP_
		hdmirxset(REG_RX_064, 0x18, 0x10);
#endif
		break;

	case F_MODE_YUV444:
		hdmirxset(REG_RX_OUT_CSC_CTRL, (M_OUTPUT_COLOR_MASK), B_OUTPUT_YUV444);
#ifdef _SUPPORT_RBSWAP_
		hdmirxset(REG_RX_064, 0x18, 0x08);
#endif
		break;
	}
}

// ---------------------------------------------------------------------------
void IT66021::SetColorSpaceConvert(struct it6602_dev_data *it6602)
{
	unsigned char csc = 0;
	//    unsigned char uc ;
	unsigned char filter = 0; // filter is for Video CTRL DN_FREE_GO, EN_DITHER, and ENUDFILT

#ifdef DISABLE_HDMI_CSC
	IT_INFO("ITEHDMI - HDMI Color Space Convert is disabled \r\n");

	csc = B_CSC_BYPASS;
	it6602->m_bOutputVideoMode = it6602->m_bInputVideoMode;

#else
	IT_INFO("\n!!! SetColorSpaceConvert( ) !!!\n");

#ifdef _AVOID_REDUNDANCE_CSC_

	if ((it6602->m_Backup_OutputVideoMode == it6602->m_bOutputVideoMode)
	    && (it6602->m_Backup_InputVideoMode == it6602->m_bInputVideoMode)) {
		IT_INFO("I/P and O/P color without change , No need to setup CSC convert again \n");
		return;
	}

#endif

	//IT_INFO("Input mode is YUV444 ");
	switch (it6602->m_bOutputVideoMode & F_MODE_CLRMOD_MASK) {
#if defined(SUPPORT_OUTPUTYUV444)

	case F_MODE_YUV444:
		IT_INFO("Output mode is YUV444\n");

		switch (it6602->m_bInputVideoMode & F_MODE_CLRMOD_MASK) {
		case F_MODE_YUV444:
			IT_INFO("Input mode is YUV444\n");
			csc = B_CSC_BYPASS;
			break;

		case F_MODE_YUV422:
			IT_INFO("Input mode is YUV422\n");
			csc = B_CSC_BYPASS;

			if (it6602->m_bOutputVideoMode & F_MODE_EN_UDFILT) { // RGB24 to YUV422 need up/dn filter.
				filter |= B_RX_EN_UDFILTER;
			}

			if (it6602->m_bOutputVideoMode & F_MODE_EN_DITHER) { // RGB24 to YUV422 need up/dn filter.
				filter |= B_RX_EN_UDFILTER | B_RX_DNFREE_GO;
			}

			break;

		case F_MODE_RGB24:
			IT_INFO("Input mode is RGB444\n");
			csc = B_CSC_RGB2YUV;
			break;
		}

		break;
#endif

#if defined(SUPPORT_OUTPUTYUV422)

	case F_MODE_YUV422:
		switch (it6602->m_bInputVideoMode & F_MODE_CLRMOD_MASK) {
		case F_MODE_YUV444:
			IT_INFO("Input mode is YUV444\n");

			if (it6602->m_bOutputVideoMode & F_MODE_EN_UDFILT) {
				filter |= B_RX_EN_UDFILTER;
			}

			csc = B_CSC_BYPASS;
			break;

		case F_MODE_YUV422:
			IT_INFO("Input mode is YUV422\n");
			csc = B_CSC_BYPASS;

			if (it6602->m_bOutputVideoMode & F_MODE_EN_DITHER) { // RGB24 to YUV422 need up/dn filter.
				filter |= B_RX_EN_UDFILTER | B_RX_DNFREE_GO;
			}

			break;

		case F_MODE_RGB24:
			IT_INFO("Input mode is RGB444\n");

			if (it6602->m_bOutputVideoMode & F_MODE_EN_UDFILT) { // RGB24 to YUV422 need up/dn filter.
				filter |= B_RX_EN_UDFILTER;
			}

			csc = B_CSC_RGB2YUV;
			break;
		}

		break;
#endif

#if defined(SUPPORT_OUTPUTRGB)

	case F_MODE_RGB24:
		IT_INFO("Output mode is RGB24\n");

		switch (it6602->m_bInputVideoMode & F_MODE_CLRMOD_MASK) {
		case F_MODE_YUV444:
			IT_INFO("Input mode is YUV444\n");
			csc = B_CSC_YUV2RGB;
			break;

		case F_MODE_YUV422:
			IT_INFO("Input mode is YUV422\n");
			csc = B_CSC_YUV2RGB;

			if (it6602->m_bOutputVideoMode & F_MODE_EN_UDFILT) { // RGB24 to YUV422 need up/dn filter.
				filter |= B_RX_EN_UDFILTER;
			}

			if (it6602->m_bOutputVideoMode & F_MODE_EN_DITHER) { // RGB24 to YUV422 need up/dn filter.
				filter |= B_RX_EN_UDFILTER | B_RX_DNFREE_GO;
			}

			break;

		case F_MODE_RGB24:
			IT_INFO("Input mode is RGB444\n");
			csc = B_CSC_BYPASS;
			break;
		}

		break;
#endif
	}

#if defined(SUPPORT_OUTPUTYUV)

	// set the CSC associated registers
	if (csc == B_CSC_RGB2YUV) {
		// IT_INFO("CSC = RGB2YUV ");
		//FIX_ID_039 xxxxx fix image flick when enable RGB limited / Full range convert
		//default to turn off CSC offset
		hdmirxset(REG_RX_067, 0x78, 0x00);
		hdmirxwr(REG_RX_068, 0x00);
		IT_INFO(" Clear Reg67 and Reg68 ... \r\n");

		//FIX_ID_039 xxxxx
		if (it6602->m_bInputVideoMode & F_MODE_ITU709) {
			IT_INFO("ITU709 ");

			if (it6602->m_bInputVideoMode & F_MODE_16_235) {
				IT_INFO(" 16-235\n");
				chgbank(1); //for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_RGB2YUV_ITU709_16_235), &bCSCMtx_RGB2YUV_ITU709_16_235[0]);

			} else {
				IT_INFO(" 0-255\n");
				chgbank(1); //for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_RGB2YUV_ITU709_0_255), &bCSCMtx_RGB2YUV_ITU709_0_255[0]);
			}

		} else {
			IT_INFO("ITU601 ");

			if (it6602->m_bInputVideoMode & F_MODE_16_235) {
				chgbank(1); //for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_RGB2YUV_ITU601_16_235), &bCSCMtx_RGB2YUV_ITU601_16_235[0]);
				IT_INFO(" 16-235\n");

			} else {
				chgbank(1); //for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_RGB2YUV_ITU601_0_255), &bCSCMtx_RGB2YUV_ITU601_0_255[0]);
				IT_INFO(" 0-255\n");
			}
		}
	}

#endif

#if defined(SUPPORT_OUTPUTRGB)

	if (csc == B_CSC_YUV2RGB) {
		IT_INFO("CSC = YUV2RGB ");
		//FIX_ID_039 xxxxx fix image flick when enable RGB limited / Full range convert
		//default to turn off CSC offset
		hdmirxset(REG_RX_067, 0x78, 0x00);
		hdmirxwr(REG_RX_068, 0x00);
		IT_INFO(" Clear Reg67 and Reg68 ... \r\n");

		//FIX_ID_039 xxxxx
		if (it6602->m_bInputVideoMode & F_MODE_ITU709) {
			IT_INFO("ITU709 ");

			if (it6602->m_bOutputVideoMode & F_MODE_16_235) {
				IT_INFO("16-235\n");
				chgbank(1); //for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_YUV2RGB_ITU709_16_235), &bCSCMtx_YUV2RGB_ITU709_16_235[0]);

			} else {
				IT_INFO("0-255\n");
				chgbank(1); //for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_YUV2RGB_ITU709_0_255), &bCSCMtx_YUV2RGB_ITU709_0_255[0]);
			}

		} else {
			IT_INFO("ITU601 ");

			if (it6602->m_bOutputVideoMode & F_MODE_16_235) {
				IT_INFO("16-235\n");
				chgbank(1); //for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_YUV2RGB_ITU601_16_235), &bCSCMtx_YUV2RGB_ITU601_16_235[0]);

			} else {
				IT_INFO("0-255\n");
				chgbank(1); //for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_YUV2RGB_ITU601_0_255), &bCSCMtx_YUV2RGB_ITU601_0_255[0]);
			}
		}
	}

	//FIX_ID_027 xxxxx Support Full/Limited Range convert
	if (csc == B_CSC_BYPASS) {

		if ((it6602->m_bInputVideoMode & F_MODE_CLRMOD_MASK) == F_MODE_RGB24) {
			if (it6602->RGBQuantizationRange == 1) { // Limited range from HDMI source
				if ((it6602->m_bOutputVideoMode & F_MODE_16_235) != F_MODE_16_235) { // Full range to back-end device
					// RedText;
					IT_INFO(" bCSCMtx_RGB_16_235_RGB_0_255 \r\n");
					// printf("pccmd w 65 02 90;\r\n");
					// printf("pccmd w 67 78 90;\r\n");
					// printf("pccmd w 68 ED 90;\r\n");
					// WhileText;
					csc = B_CSC_RGB2YUV;
					chgbank(1); //for CSC setting Reg170 ~ Reg184 !!!!
					hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_RGB_16_235_RGB_0_255), &bCSCMtx_RGB_16_235_RGB_0_255[0]);
					chgbank(0);
					//hdmirxset(REG_RX_065,0x03,0x02);	// B_CSC_RGB2YUV
					hdmirxset(REG_RX_067, 0x78, 0x78);
					hdmirxwr(REG_RX_068, 0xED);
				}

			} else if (it6602->RGBQuantizationRange == 2) { //Full range from HDMI source
				if ((it6602->m_bOutputVideoMode & F_MODE_16_235) == F_MODE_16_235) { // Limited range to back-end device
					// RedText;
					IT_INFO(" bCSCMtx_RGB_0_255_RGB_16_235 \r\n");
					// printf("pccmd w 65 02 90;\r\n");
					// printf("pccmd w 67 40 90;\r\n");
					// printf("pccmd w 68 10 90;\r\n");
					// WhileText;
					csc = B_CSC_RGB2YUV;
					chgbank(1); //for CSC setting Reg170 ~ Reg184 !!!!
					hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_RGB_0_255_RGB_16_235), &bCSCMtx_RGB_0_255_RGB_16_235[0]);
					chgbank(0);
					//hdmirxset(REG_RX_065,0x03,0x02);	// B_CSC_RGB2YUV
					hdmirxset(REG_RX_067, 0x78, 0x40);
					hdmirxwr(REG_RX_068, 0x10);
				}
			}
		}
	}

	//FIX_ID_027 xxxxx

#endif // SUPPORT_OUTPUTRGB

#endif //end of DISABLE_HDMI_CSC

	chgbank(0);
	hdmirxset(REG_RX_OUT_CSC_CTRL, (M_CSC_SEL_MASK), csc);

	// set output Up/Down Filter, Dither control
	hdmirxset(REG_RX_VIDEO_CTRL1, (B_RX_DNFREE_GO | B_RX_EN_DITHER | B_RX_EN_UDFILTER), filter);

	//FIX_ID_039 xxxxx fix image flick when enable RGB limited / Full range convert
	if (csc == B_CSC_BYPASS) {
		//default to turn off CSC offset
		hdmirxset(REG_RX_067, 0x78, 0x00);
		hdmirxwr(REG_RX_068, 0x00);
	}

#ifdef _AVOID_REDUNDANCE_CSC_
	it6602->m_Backup_OutputVideoMode = it6602->m_bOutputVideoMode;
	it6602->m_Backup_InputVideoMode = it6602->m_bInputVideoMode;
#endif
	//FIX_ID_039 xxxxx
}

// ---------------------------------------------------------------------------
void IT66021::SetNewInfoVideoOutput(struct it6602_dev_data *it6602)
{

	IT_INFO("SetNewInfoVideoOutput() \n");

	SetVideoInputFormatWithInfoFrame(it6602);
	SetColorimetryByInfoFrame(it6602);
	SetColorSpaceConvert(it6602);

	SetVideoOutputColorFormat(it6602); //2013-0502
}

// ---------------------------------------------------------------------------
void IT66021::SetVideoInputFormatWithoutInfoFrame(struct it6602_dev_data *it6602, unsigned char bInMode)
{
	unsigned char i;

	i = hdmirxrd(REG_RX_IN_CSC_CTRL);
	i |= B_IN_FORCE_COLOR_MODE;

	i &= (~M_INPUT_COLOR_MASK);
	it6602->m_bInputVideoMode &= ~F_MODE_CLRMOD_MASK;

	switch (bInMode) {
	case F_MODE_YUV444:
		i |= B_INPUT_YUV444;
		it6602->m_bInputVideoMode |= F_MODE_YUV444;
		break;

	case F_MODE_YUV422:
		i |= B_INPUT_YUV422;
		it6602->m_bInputVideoMode |= F_MODE_YUV422;
		break;

	case F_MODE_RGB24:
		i |= B_INPUT_RGB24;
		it6602->m_bInputVideoMode |= F_MODE_RGB24;
		break;

	default:
		return;
	}

	hdmirxwr(REG_RX_IN_CSC_CTRL, i);
}

// ---------------------------------------------------------------------------
void IT66021::SetColorimetryByMode(struct it6602_dev_data *it6602)
{
	unsigned char RxClkXCNT;
	RxClkXCNT = hdmirxrd(REG_RX_PIXCLK_SPEED);

	IT_INFO(" SetColorimetryByMode() REG_RX_PIXCLK_SPEED=%X \n", (int)RxClkXCNT);

	it6602->m_bInputVideoMode &= ~F_MODE_ITU709;

	if (RxClkXCNT < 0x34) {

		it6602->m_bInputVideoMode |= F_MODE_ITU709;

	} else {

		it6602->m_bInputVideoMode &= ~F_MODE_ITU709;
	}
}

// ---------------------------------------------------------------------------
void IT66021::SetDVIVideoOutput(struct it6602_dev_data *it6602)
{
	IT_INFO("SetDVIVideoOutput() \n");

	SetVideoInputFormatWithoutInfoFrame(it6602, F_MODE_RGB24);
	SetColorimetryByMode(it6602);
	SetColorSpaceConvert(it6602);

	SetVideoOutputColorFormat(it6602); //2013-0502
}

//FIX_ID_003 xxxxx	//Add IT6602 Video Output Configure setting
void IT66021::IT6602_VideoOutputModeSet(struct it6602_dev_data *it6602)
{
	unsigned char ucReg51;
	unsigned char ucReg65;

	IT_INFO("IT6602_VideoOutputModeSet() \r\n");

	IT_INFO("+++ %s", VModeStateStr[(unsigned char)it6602->m_VidOutConfigMode]);

	ucReg51 = hdmirxrd(REG_RX_051) & 0x9B; // Reg51 [6] Half PCLK DDR , [5] Half Bus DDR , [2] CCIR656 mode
	ucReg65 = hdmirxrd(REG_RX_065) &
		  0x0F; // Reg65 [7] BTA1004Fmt , [6] SyncEmb , [5:4] output color 0x00 RGB, 0x10 YUV422, 0x20 YUV444

	switch ((it6602->m_bOutputVideoMode & F_MODE_CLRMOD_MASK)) {
	case F_MODE_RGB444:
		ucReg65 |= (B_OUTPUT_RGB24); // 0x00 B_OUTPUT_RGB24
#ifdef _SUPPORT_RBSWAP_
		hdmirxset(REG_RX_064, 0x18, 0x08);
#endif
		break;

	case F_MODE_YUV422:
		ucReg65 |= (B_OUTPUT_YUV422); // 0x10 B_OUTPUT_YUV422
#ifdef _SUPPORT_RBSWAP_
		hdmirxset(REG_RX_064, 0x18, 0x10);
#endif
		break;

	case F_MODE_YUV444:
		ucReg65 |= (B_OUTPUT_YUV444); // 0x20 B_OUTPUT_YUV444
#ifdef _SUPPORT_RBSWAP_
		hdmirxset(REG_RX_064, 0x18, 0x10);
#endif
		break;
	}

	switch (it6602->m_VidOutDataTrgger) {
	case eSDR:
		break;

	case eHalfPCLKDDR:
		ucReg51 |= (B_HALF_PCLKC); // 0x40 half PCLK
		break;

	case eHalfBusDDR:
		ucReg51 |= (B_OUT_DDR); // 0x20 half bus
		break;

	case eSDR_BTA1004:
		ucReg65 |= (B_BTA1004Fmt | B_SyncEmb); // 0x80 BTA1004 + 0x40 SyncEmb
		break;

	case eDDR_BTA1004:
		ucReg51 |= (B_HALF_PCLKC);			   // 0x40 half PCLK
		ucReg65 |= (B_BTA1004Fmt | B_SyncEmb); // 0x80 BTA1004 + 0x40 SyncEmb
		break;
	}

	switch (it6602->m_VidOutSyncMode) {
	case eSepSync:
		break;

	case eEmbSync:
		ucReg65 |= (B_SyncEmb); // 0x40 SyncEmb
		break;

	case eCCIR656SepSync:
		ucReg51 |= (B_CCIR656); // 0x04 CCIR656
		break;

	case eCCIR656EmbSync:
		ucReg51 |= (B_CCIR656); // 0x04 CCIR656
		ucReg65 |= (B_SyncEmb); // 0x40 SyncEmb
		break;
	}

	IT_INFO("Reg51 = %X ", (int)ucReg51);
	IT_INFO("Reg65 = %X\r\n", (int)ucReg65);

	hdmirxwr(REG_RX_051, ucReg51);
	hdmirxwr(REG_RX_065, ucReg65);
}

void IT66021::IT6602VideoOutputConfigure(struct it6602_dev_data *it6602)
{
	it6602->m_bUpHDMIMode = IsHDMIMode();

	if (it6602->m_bUpHDMIMode == FALSE) {
		SetDVIVideoOutput(it6602);

	} else {
		GetAVIInfoFrame(it6602);
		SetNewInfoVideoOutput(it6602);
	}

	it6602->m_NewAVIInfoFrameF = FALSE;

	it6602->GCP_CD = ((hdmirxrd(0x99) & 0xF0) >> 4);

	switch (it6602->GCP_CD) {
	case 5:
		IT_INFO("\n Output ColorDepth = 30 bits per pixel\r\n");
		hdmirxset(0x65, 0x0C, 0x04);
		break;

	case 6:
		IT_INFO("\n Output ColorDepth = 36 bits per pixel\r\n");
		hdmirxset(0x65, 0x0C, 0x08);
		break;

	default:
		IT_INFO("\n Output ColorDepth = 24 bits per pixel\r\n");
		hdmirxset(0x65, 0x0C, 0x00);
		break;
	}

	// Configure TTL Video Output mode
	IT6602_VideoOutputModeSet(it6602);
}

// ---------------------------------------------------------------------------
void IT66021::SetVideoOutputColorFormat(struct it6602_dev_data *it6602)
{
	switch (it6602->m_bOutputVideoMode & F_MODE_CLRMOD_MASK) {
	case F_MODE_RGB24:
		hdmirxset(REG_RX_OUT_CSC_CTRL, (M_OUTPUT_COLOR_MASK), B_OUTPUT_RGB24);
#ifdef _SUPPORT_RBSWAP_
		hdmirxset(REG_RX_064, 0x18, 0x08);
#endif

		break;

	case F_MODE_YUV422:
		hdmirxset(REG_RX_OUT_CSC_CTRL, (M_OUTPUT_COLOR_MASK), B_OUTPUT_YUV422);
#ifdef _SUPPORT_RBSWAP_
		hdmirxset(REG_RX_064, 0x18, 0x10);
#endif
		break;

	case F_MODE_YUV444:
		hdmirxset(REG_RX_OUT_CSC_CTRL, (M_OUTPUT_COLOR_MASK), B_OUTPUT_YUV444);
#ifdef _SUPPORT_RBSWAP_
		hdmirxset(REG_RX_064, 0x18, 0x08);
#endif
		break;
	}
}

void IT66021::it6602PortSelect(unsigned char ucPortSel)
{
	struct it6602_dev_data *it6602data = get_it6602_dev_data();

	// The IT66021 must to set ‘0’ on HDMI Reg51[0]
	// Reg51[0]: RegMainPortSel Main Port selector 0: Port 0 HDMI
	hdmirxset(REG_RX_051, B_PORT_SEL, F_PORT_SEL_0);

	if (it6602data->m_ucCurrentHDMIPort != ucPortSel) {
		IT6602SwitchVideoState(it6602data, VSTATE_SyncWait);
		it6602data->m_ucCurrentHDMIPort = ucPortSel;
		IT_INFO("it6602PortSelect = %X \r\n", (int)ucPortSel);
	}
}

void IT66021::hdmirx_ECCTimingOut(unsigned char ucport)
{
	IT_INFO("CDR reset for hdmirx_ECCTimingOut()  \r\n");

	if (ucport == F_PORT_SEL_0) {
		it6602HPDCtrl(0, 0); // MHL port , set HPD = 0

		hdmirxset(REG_RX_011, (B_P0_DCLKRST | B_P0_CDRRST | B_P0_HDCPRST | B_P0_SWRST),
			  (B_P0_DCLKRST | B_P0_CDRRST | B_P0_HDCPRST | B_P0_SWRST));
		IT_Delay(300);
		hdmirxset(REG_RX_011, (B_P0_DCLKRST | B_P0_CDRRST | B_P0_HDCPRST | B_P0_SWRST), 0x00);
		//set port 0 HPD=1
		it6602HPDCtrl(0, 1); // MHL port , set HPD = 1

	} else {
		//set port 1 HPD=0
		it6602HPDCtrl(1, 0); // HDMI port , set HPD = 0

		hdmirxset(REG_RX_018, (B_P1_DCLKRST | B_P1_CDRRST | B_P1_HDCPRST | B_P1_SWRST),
			  (B_P1_DCLKRST | B_P1_CDRRST | B_P1_HDCPRST | B_P1_SWRST));
		IT_Delay(300);
		hdmirxset(REG_RX_018, (B_P1_DCLKRST | B_P1_CDRRST | B_P1_HDCPRST | B_P1_SWRST), 0x00);
		//set port 1 HPD=1
		it6602HPDCtrl(1, 1); // HDMI port , set HPD = 1
	}
}

// ***************************************************************************
// Video function
// ***************************************************************************
void IT66021::IT6602_AFE_Rst(void)
{
	unsigned char Reg51h;
	struct it6602_dev_data *it6602data = get_it6602_dev_data(); //2013-0814

	chgbank(0);
	Reg51h = hdmirxrd(0x51);

	if (Reg51h & 0x01) {
		IT_INFO("=== port 1 IT6602_AFE_Rst() === \r\n");
		hdmirxset(REG_RX_018, 0x01, 0x01);
		IT_Delay(1);
		hdmirxset(REG_RX_018, 0x01, 0x00);
#ifdef _SUPPORT_AUTO_EQ_
		DisableOverWriteRS(1); //2013-1129
#endif

	} else {
		IT_INFO("=== port 0 IT6602_AFE_Rst() === \r\n");
		hdmirxset(REG_RX_011, 0x01, 0x01);
		IT_Delay(1);
		hdmirxset(REG_RX_011, 0x01, 0x00);
#ifdef _SUPPORT_AUTO_EQ_
		DisableOverWriteRS(0); //2013-1129 for MHL unplug detected
#endif
	}

	it6602data->m_ucSCDTOffCount = 0;
}

void IT66021::IT6602_HDCP_ContentOff(unsigned char ucPort, unsigned char bOff)
{
	struct it6602_dev_data *it6602data = get_it6602_dev_data(); //2013-0814

	if (IT6602_IsSelectedPort(ucPort) == FALSE) {
		return;
	}

	if (bOff != 0) {
		IT6602_ManualVideoTristate(1);
		it6602data->m_HDCP_ContentOff = 1;
		IT_INFO("+++++++++++ HDCP Content Off   +++++++++++++++++\n");

	} else {
		if (it6602data->m_VState == VSTATE_VideoOn) {
			if (it6602data->m_HDCP_ContentOff == 1) {
				IT6602_ManualVideoTristate(0);
				IT_INFO("+++++++++++ HDCP Content On   +++++++++++++++++\n");
			}
		}

		it6602data->m_HDCP_ContentOff = 0;
	}
}

void IT66021::IT6602_RAPContentOff(unsigned char bOff)
{
	struct it6602_dev_data *it6602data = get_it6602_dev_data(); //2013-0814

	if (IT6602_IsSelectedPort(0) == FALSE) {
		return;
	}

	if (bOff != 0) {
		IT6602_ManualVideoTristate(1);
		it6602data->m_RAP_ContentOff = 1;
		IT_INFO("+++++++++++ RAP Content Off   +++++++++++++++++\n");

		IT6602AudioOutputEnable(0);

	} else {
		if (it6602data->m_VState == VSTATE_VideoOn) {
			if (it6602data->m_RAP_ContentOff == 1) {
				IT6602_ManualVideoTristate(0);
				IT_INFO("+++++++++++ RAP Content On   +++++++++++++++++\n");

#ifndef _FIX_ID_028_
				IT6602SwitchAudioState(it6602data, ASTATE_RequestAudio);
#endif
			}
		}

		it6602data->m_RAP_ContentOff = 0;
	}
}

void IT66021::IT6602_SetVideoMute(struct it6602_dev_data *it6602, unsigned char bMute)
{

	if (bMute) {
		//******** AV Mute -> ON ********//
		hdmirxset(REG_RX_053, (B_VDGatting | B_VIOSel),
			  (B_VDGatting | B_VIOSel)); //Reg53[7][5] = 11    // for enable B_VDIO_GATTING and VIO_SEL
		hdmirxset(REG_RX_052, (B_DisVAutoMute), (B_DisVAutoMute));				   //Reg52[5] = 1 for disable Auto video MUTE
		hdmirxset(REG_RX_053, (B_TriVDIO), (0x00));								   //Reg53[2:0] = 000;         // 0 for enable video io data output

		IT_INFO("+++++++++++ IT6602_SetVideoMute -> On +++++++++++++++++\n");

	} else {
		if (it6602->m_VState == VSTATE_VideoOn) {
			//******** AV Mute -> OFF ********//
			hdmirxset(REG_RX_053, (B_TriSYNC), (0x00)); //Reg53[0] = 0;                 // for enable video sync
			hdmirxset(REG_RX_053, (B_TriVDIO), (0x00)); //Reg53[3:1] = 000;         // 0 for enable video io data output

			if (CheckAVMute() == TRUE) {
				hdmirxset(REG_RX_052, (B_DisVAutoMute), (B_DisVAutoMute)); //Reg52[5] = 1 for disable Auto video MUTE

			} else {
				hdmirxset(REG_RX_053, (B_TriVDIO), (B_TriVDIO)); //Reg53[2:0] = 111;         // 1 for enable tri-state of video io data
				hdmirxset(REG_RX_053, (B_TriVDIO), (0x00));		 //Reg53[2:0] = 000;         // 0 for enable video io data output

				hdmirxset(REG_RX_053, (B_VDGatting | B_VIOSel),
					  (B_VDGatting | B_VIOSel)); //Reg53[7][5] = 11    // for enable B_VDIO_GATTING and VIO_SEL
				hdmirxset(REG_RX_053, (B_VDGatting | B_VIOSel), (B_VIOSel));			   //Reg53[7][5] = 01    // for disable B_VDIO_GATTING

				IT_INFO("+++++++++++  IT6602_SetVideoMute -> Off +++++++++++++++++\n");
			}
		}
	}
}


void IT66021::IT6602VideoOutputEnable(unsigned char bEnableOutput)
{
	if (bEnableOutput) {
		hdmirxset(REG_RX_053, (B_TriSYNC | B_TriVDIO), (0x00));
		IT_INFO("---------------- IT6602VideoOutputEnable -> On ----------------\n");

	} else {
		hdmirxset(REG_RX_053, (B_TriSYNC | B_TriVDIO), (B_TriSYNC | B_TriVDIO));
		IT_INFO("---------------- IT6602VideoOutputEnable -> Off ----------------\n");
	}

	// TODO:
	// 2019-12-11 我感觉上面代码逻辑错误了，因此改为如下形式了
	// if (!bEnableOutput) {
	// 	hdmirxset(REG_RX_053, (B_TriSYNC | B_TriVDIO), (0x00));
	// 	IT_INFO("---------------- IT6602VideoOutputEnable -> OFF ----------------\n");

	// } else {
	// 	hdmirxset(REG_RX_053, (B_TriSYNC | B_TriVDIO), (B_TriSYNC | B_TriVDIO));
	// 	IT_INFO("---------------- IT6602VideoOutputEnable -> ON ----------------\n");
	// }
}

void IT66021::IT6602SwitchVideoState(struct it6602_dev_data *it6602, Video_State_Type eNewVState)
{
	if (it6602->m_VState == eNewVState) {
		return;
	}

	IT_INFO("@@@ %s\n", VStateStr[(unsigned char)eNewVState]);

	it6602->m_VState = eNewVState;

	switch (it6602->m_VState) {
	case VSTATE_SWReset: {
			IT6602VideoOutputEnable(FALSE);
			//FIX_ID_039 xxxxx fix image flick when enable RGB limited / Full range convert
#ifdef _AVOID_REDUNDANCE_CSC_
			it6602->m_Backup_OutputVideoMode = 0xFF;
			it6602->m_Backup_InputVideoMode = 0xFF;
#endif
			//FIX_ID_039 xxxxx				IT6602_AFE_Rst();
#ifdef Enable_IT6602_CEC
			//xxxxx FIX_ID_022		//Fixed for CEC capacitor issue
			IT6602_ResetCEC();
			//xxxxx
#endif
		}
		break;

	case VSTATE_SyncWait: {
			// 1. SCDT off interrupt
			// 2. VideoMode_Chg interrupt
			IT6602VideoOutputEnable(FALSE);
			//FIX_ID_039 xxxxx fix image flick when enable RGB limited / Full range convert
#ifdef _AVOID_REDUNDANCE_CSC_
			it6602->m_Backup_OutputVideoMode = 0xFF;
			it6602->m_Backup_InputVideoMode = 0xFF;
#endif
			//FIX_ID_039 xxxxx
			it6602->m_NewAVIInfoFrameF = FALSE;
			it6602->m_ucDeskew_P0 = 0;
			//it6602->m_ucSCDTOffCount=0;

#ifdef Enable_Vendor_Specific_packet
			it6602->f_de3dframe_hdmi = FALSE;
			hdmirxwr(REG_RX_06A, 0x82);
#endif
		}
		break;

	case VSTATE_SyncChecking: {
			// 1. SCDT on interrupt
			//AssignVideoVirtualTime(VSATE_CONFIRM_SCDT_COUNT);
			//AssignVideoTimerTimeout(VSATE_CONFIRM_SCDT_COUNT);

			it6602->m_VideoCountingTimer = VSATE_CONFIRM_SCDT_COUNT;

#ifdef Enable_Vendor_Specific_packet
			hdmirxwr(REG_RX_06A, 0x82);
#endif
		}
		break;

	case VSTATE_VideoOn: {
			IT6602VideoOutputConfigure(it6602);
			IT6602VideoOutputEnable(TRUE);
			IT6602SwitchAudioState(it6602, ASTATE_RequestAudio);

			get_vid_info();
			show_vid_info();

			// TODO：DLOG_Output
			// DLOG_Output(1000);
			hdmirxwr(0x84, 0x8F); //2011/06/17 xxxxx, for enable Rx Chip count

#ifdef Enable_Vendor_Specific_packet
			hdmirxwr(REG_RX_06A, 0x81);
#endif

			//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
			//	#ifdef _SUPPORT_EQ_ADJUST_
			//	HDMIStartEQDetect(&(it6602->EQPort[it6602->m_ucCurrentHDMIPort]));
			//	#endif
			//FIX_ID_001 xxxxx

			//xxxxx 2013-0812 @@@@@
			it6602->m_ucSCDTOffCount = 0;
			//xxxxx 2013-0812

#ifdef _SUPPORT_HDCP_REPEATER_
#ifdef _PSEUDO_HDCP_REPEATER_TEST_

			// TX_BSTATUS = 0x102;
			if (m_RxHDCPstatus == 2) {
				ITEHDMI_RxHDCP2ndAuthenticationRequest(TX_KSVList, TX_BKSV, TX_BSTATUS);
			}

#endif
#endif
			break;
		}

	default:
		break;
	}
}

void IT66021::get_vid_info(void)
{
#if 1
	int HSyncPol, VSyncPol, InterLaced;
	int HTotal, HActive, HFP, HSYNCW;
	int VTotal, VActive, VFP, VSYNCW;
	//	int rddata;
	//	int i;
	//	unsigned long PCLK, sump;
	unsigned int ucTMDSClk = 0; //, sumt;
	unsigned char ucPortSel = 0;
	unsigned char rddata = 0;
	unsigned char ucClk;
	int PCLK; //, sump;

	rddata = hdmirxrd(0x9A);
	PCLK = (124 * 255 / rddata) / 10;

	ucPortSel = hdmirxrd(REG_RX_051) & B_PORT_SEL;
	rddata = hdmirxrd(0x90);

	if (ucPortSel == F_PORT_SEL_1) {

		IT_INFO("Reg51[0] = 1 Active Port HDMI \r\n");
		ucClk = hdmirxrd(REG_RX_092);

		if (ucClk != 0) {

			if (rddata & 0x04) {
				ucTMDSClk = 2 * RCLKVALUE * 256 / ucClk;

			} else if (rddata & 0x08) {
				ucTMDSClk = 4 * RCLKVALUE * 256 / ucClk;

			} else {
				ucTMDSClk = RCLKVALUE * 256 / ucClk;
			}

			IT_INFO(" Port 1 TMDS CLK  = %d \r\n", (int)ucTMDSClk);
		}

		//IT_INFO(" HDMI Reg92  = %X \r\n",(int) hdmirxrd(0x92));
		//IT_INFO(" HDMI Reg38  = %X \r\n",(int) hdmirxrd(0x38));

	} else {
		IT_INFO("Reg51[0] = 0 Active Port MHL \r\n");
		ucClk = hdmirxrd(REG_RX_091);

		if (ucClk != 0) {
			if (rddata & 0x01) {
				ucTMDSClk = 2 * RCLKVALUE * 256 / ucClk;

			} else if (rddata & 0x02) {
				ucTMDSClk = 4 * RCLKVALUE * 256 / ucClk;

			} else {
				ucTMDSClk = RCLKVALUE * 256 / ucClk;
			}

			IT_INFO("Port 0 TMDS CLK  = %d \r\n", (int)ucTMDSClk);
		}

		//IT_INFO(" HDMI Reg91  = %X \r\n",(int) hdmirxrd(0x91));
		//IT_INFO(" HDMI Reg20  = %X \r\n",(int) hdmirxrd(0x20));
	}

	InterLaced = (hdmirxrd(0x99) & 0x02) >> 1;

	HTotal = ((hdmirxrd(0x9D) & 0x3F) << 8) + hdmirxrd(0x9C);
	HActive = ((hdmirxrd(0x9F) & 0x3F) << 8) + hdmirxrd(0x9E);
	HFP = ((hdmirxrd(0xA1) & 0xF0) << 4) + hdmirxrd(0xA2);
	HSYNCW = ((hdmirxrd(0xA1) & 0x01) << 8) + hdmirxrd(0xA0);
	HSyncPol = hdmirxrd(0xA8) & 0x04 >> 2;

	VTotal = ((hdmirxrd(0xA4) & 0x0F) << 8) + hdmirxrd(0xA3);
	VActive = ((hdmirxrd(0xA4) & 0xF0) << 4) + hdmirxrd(0xA5);
	VFP = hdmirxrd(0xA7) & 0x3F;
	VSYNCW = hdmirxrd(0xA6) & 0x1F;
	VSyncPol = (hdmirxrd(0xA8) & 0x08) >> 3;

	//	CurVTiming.TMDSCLK     = (int)TMDSCLK;
	CurTMDSCLK = (int)ucTMDSClk;
	CurVTiming.PCLK = (int)PCLK;
	CurVTiming.HActive = HActive;
	CurVTiming.HTotal = HTotal;
	CurVTiming.HFrontPorch = HFP;
	CurVTiming.HSyncWidth = HSYNCW;
	CurVTiming.HBackPorch = HTotal - HActive - HFP - HSYNCW;
	CurVTiming.VActive = VActive;
	CurVTiming.VTotal = VTotal;
	CurVTiming.VFrontPorch = VFP;
	CurVTiming.VSyncWidth = VSYNCW;
	CurVTiming.VBackPorch = VTotal - VActive - VFP - VSYNCW;
	CurVTiming.ScanMode = (InterLaced) & 0x01;
	CurVTiming.VPolarity = (VSyncPol) & 0x01;
	CurVTiming.HPolarity = (HSyncPol) & 0x01;
#endif
}

void IT66021::show_vid_info(void)
{
#if 1
	int InBPC, InBPP;
	int MHL_Mode;
	int MHL_CLK_Mode;
	int GCP_CD = CD8BIT; //24 bits per pixel

	unsigned long FrameRate;

	GCP_CD = ((hdmirxrd(0x99) & 0xF0) >> 4);

	switch (GCP_CD) {
	case 5:
		IT_INFO("I/P ColorDepth = 30 bits per pixel \r\n");
		InBPC = 10;
		hdmirxset(0x65, 0x0C, 0x04);
		OutCD = OUT10B;
		break;

	case 6:
		IT_INFO("I/P ColorDepth = 36 bits per pixel \r\n");
		InBPC = 12;
		hdmirxset(0x65, 0x0C, 0x08);
		OutCD = OUT12B;
		break;

	default:
		IT_INFO("I/P ColorDepth = 24 bits per pixel \r\n");
		InBPC = 8;
		hdmirxset(0x65, 0x0C, 0x00);
		OutCD = OUT8B;
		break;
	}

	switch (OutCD) {
	case 1:
		IT_INFO("O/P ColorDepth = 30 bits per pixel \r\n");
		break;

	case 2:
		IT_INFO("O/P ColorDepth = 36 bits per pixel \r\n");
		break;

	default:
		IT_INFO("O/P ColorDepth = 24 bits per pixel \r\n");
		break;
	}

	chgbank(2);
	InColorMode = (hdmirxrd(0x15) & 0x60) >> 5;
	chgbank(0);

	if (InColorMode == 1) {
		//YCbCr422
		InBPP = InBPC * 2;

	} else {
		InBPP = InBPC * 3;
	}

	IT_INFO("InBPP = %d", InBPP);

	switch (InColorMode) {
	case 0:
		IT_INFO("Input Color Mode = RGB444 \n");
		//		 hdmirxset(0xAE, 0x01, 0x01);
		//		 defaultrgb();
		break;

	case 1:
		IT_INFO("Input Color Mode = YCbCr422\n");
		//		 hdmirxset(0xAE, 0x01, 0x00);
		//		 yuv422torgb();
		break;

	case 2:
		IT_INFO("Input Color Mode = YCbCr444\n");
		//		 hdmirxset(0xAE, 0x01, 0x00);
		//		 yuv444torgb();
		break;

	default:
		IT_INFO("Input Color Mode = Reserved !!!\n");
		break;
	}

	OutColorMode = (hdmirxrd(0x65) & 0x30) >> 4;

	switch (OutColorMode) {
	case 0:
		IT_INFO("Output Color Mode = RGB444\n");
		//		 hdmirxset(0x65, 0x30, 0x00);
		break;

	case 1:
		IT_INFO("Output Color Mode = YCbCr422\n");
		//		 hdmirxset(0x65, 0x30, 0x10);
		break;

	case 2:
		IT_INFO("Output Color Mode = YCbCr444\n");
		//		 hdmirxset(0x65, 0x30, 0x20);
		break;

	default:
		IT_INFO("Output Color Mode = Reserved !!!\n");
		break;
	}

	//    IT_INFO("Video Input Timing: %s\n", s_VMTable[VIC].format);
	//    IT_INFO("TMDSCLK = %3.3fMHz\n", (unsigned long)(CurTMDSCLK)/1000);
	//    IT_INFO("PCLK = %3.3fMHz\n", (unsigned long)(CurVTiming.PCLK)/1000);

	IT_INFO("HFrontPorch = %d\n", CurVTiming.HFrontPorch);
	IT_INFO("HSyncWidth = %d\n", CurVTiming.HSyncWidth);
	IT_INFO("HBackPorch = %d\n", CurVTiming.HBackPorch);
	IT_INFO("VFrontPorch = %d\n", CurVTiming.VFrontPorch);
	IT_INFO("VSyncWidth = %d\n", CurVTiming.VSyncWidth);
	IT_INFO("VBackPorch = %d\n", CurVTiming.VBackPorch);

	FrameRate = (unsigned long)(CurVTiming.PCLK) * 1000 * 1000;
	FrameRate /= CurVTiming.HTotal;
	FrameRate /= CurVTiming.VTotal;
	IT_INFO("FrameRate = %ld Hz\n", FrameRate);

	if (CurVTiming.ScanMode == 0) {
		IT_INFO("ScanMode = Progressive\n");

	} else {
		IT_INFO("ScanMode = InterLaced\n");
	}

	if (CurVTiming.VPolarity == 1) {
		IT_INFO("VSyncPol = Positive\n");

	} else {
		IT_INFO("VSyncPol = Negative\n");
	}

	if (CurVTiming.HPolarity == 1) {
		IT_INFO("HSyncPol = Positive\n");

	} else {
		IT_INFO("HSyncPol = Negative\n");
	}

	if (((hdmirxrd(0x51) & 0x01))) {
		IT_INFO("Port= 1 ,Reg18=%X ,", (int)hdmirxrd(REG_RX_018));
		IT_INFO("Reg38=%X, ", (int)hdmirxrd(REG_RX_038));
		IT_INFO("Reg3E=%X, ", (int)hdmirxrd(REG_RX_03E));
		IT_INFO("Reg3F=%X, ", (int)hdmirxrd(REG_RX_03F));
		IT_INFO("Reg40=%X \r\n", (int)hdmirxrd(REG_RX_040));
		IT_INFO("Reg41=%X \r\n", (int)hdmirxrd(REG_RX_041));
		chgbank(1);
		IT_INFO("Rec_B_CS=%X  ", (int)(hdmirxrd(REG_RX_1DD) & 0x80) >> 7);
		IT_INFO("Rec_G_CS=%X  ", (int)(hdmirxrd(REG_RX_1DE) & 0x80) >> 7);
		IT_INFO("Rec_R_CS=%X  \n", (int)(hdmirxrd(REG_RX_1DF) & 0x80) >> 7);
		IT_INFO("Rec_B_RS=%X  ", (int)(hdmirxrd(REG_RX_1DD) & 0x7F));
		IT_INFO("Rec_G_RS=%X  ", (int)(hdmirxrd(REG_RX_1DE) & 0x7F));
		IT_INFO("Rec_R_RS=%X  \n", (int)(hdmirxrd(REG_RX_1DF) & 0x7F));
		IT_INFO(" Reg1C1  = %X , Reg1C2  = %X\r\n", (int)hdmirxrd(REG_RX_1C1), (int)hdmirxrd(REG_RX_1C2));
		chgbank(0);

	} else {

		IT_INFO("Port= 0 ,Reg11=%X ,", (int)hdmirxrd(REG_RX_011));
		IT_INFO("Reg20=%X, ", (int)hdmirxrd(REG_RX_020));
		IT_INFO("Reg26=%X, ", (int)hdmirxrd(REG_RX_026));
		IT_INFO("Reg27=%X, ", (int)hdmirxrd(REG_RX_027));
		IT_INFO("Reg28=%X, ", (int)hdmirxrd(REG_RX_028));
		IT_INFO("Reg29=%X \r\n", (int)hdmirxrd(REG_RX_029));
		chgbank(1);
		IT_INFO("Rec_B_CS=%X  ", (int)(hdmirxrd(REG_RX_1D5) & 0x80) >> 7);
		IT_INFO("Rec_G_CS=%X  ", (int)(hdmirxrd(REG_RX_1D6) & 0x80) >> 7);
		IT_INFO("Rec_R_CS=%X  \n", (int)(hdmirxrd(REG_RX_1D7) & 0x80) >> 7);

		IT_INFO("Rec_B_RS=%X  ", (int)(hdmirxrd(REG_RX_1D5) & 0x7F));
		IT_INFO("Rec_G_RS=%X  ", (int)(hdmirxrd(REG_RX_1D6) & 0x7F));
		IT_INFO("Rec_R_RS=%X  \n", (int)(hdmirxrd(REG_RX_1D7) & 0x7F));
		IT_INFO("REG_RX_1B1 = %X ,  REG_RX_1B2 = %X\r\n", (int)hdmirxrd(REG_RX_1B1), (int)hdmirxrd(REG_RX_1B2));

		chgbank(0);
	}

	IT_INFO("TMDSCLK = %d MHz\n", (int)(CurTMDSCLK));
	IT_INFO("PCLK = %d MHz\n", (int)(CurVTiming.PCLK));
	IT_INFO("HActive = %d\n", CurVTiming.HActive);
	IT_INFO("VActive = %d\n", CurVTiming.VActive);
	IT_INFO("HTotal = %d\n", CurVTiming.HTotal);
	IT_INFO("VTotal = %d\n", CurVTiming.VTotal);

//FIX_ID_036	xxxxx //Enable MHL Function for IT68XX
#ifdef _ENABLE_IT68XX_MHL_FUNCTION_
	MHL_Mode = ((hdmirxrd(0x0A) & 0x40) >> 6);
	MHL_CLK_Mode = ((mhlrxrd(0xB1) & 0x07));
#else
	MHL_Mode = 0;
#endif

	//FIX_ID_036	xxxxx
	if (MHL_Mode) {
		if (MHL_CLK_Mode == 0x02) {
			IT_INFO("BUS MODE : MHL PackPixel Mode\n");

		} else {
			IT_INFO("BUS MODE : MHL 24 bits Mode\n");

		}
	}

	if (IsHDMIMode()) {
		IT_INFO("HDMI/DVI Mode : HDMI \n");

	} else {
		IT_INFO("HDMI/DVI Mode : DVI \n");
	}

#endif
}


// ---------------------------------------------------------------------------
void IT66021::IT6602VideoHandler(struct it6602_dev_data *it6602)
{
	IT_Delay(500);

	if (it6602->m_VideoCountingTimer > MS_LOOP) {
		it6602->m_VideoCountingTimer -= MS_LOOP;

	} else {
		it6602->m_VideoCountingTimer = 0;
	}

	IT_INFO("it6602->m_VideoCountingTimer = %d", it6602->m_VideoCountingTimer);
	IT_INFO("it6602->m_VState = %s\n", VStateStr[(unsigned char)it6602->m_VState]);

	switch (it6602->m_VState) {
	case VSTATE_SyncWait: {
			WaitingForSCDT(it6602);
			break;
		}

	case VSTATE_SyncChecking: {
			if (it6602->m_VideoCountingTimer == 0) {
				IT6602SwitchVideoState(it6602, VSTATE_VideoOn);
			}

			break;
		}

	case VSTATE_VideoOn: {
#ifdef _SUPPORT_HDCP_REPEATER_
#ifdef _PSEUDO_HDCP_REPEATER_TEST_

			// TX_BSTATUS = 0x102;
			if (m_RxHDCPstatus == 2) {
				ITEHDMI_RxHDCP2ndAuthenticationRequest(TX_KSVList, TX_BKSV, TX_BSTATUS);
			}

#endif
#endif

			if (it6602->m_NewAVIInfoFrameF == TRUE) {
				if (it6602->m_RxHDCPState != RxHDCP_ModeCheck) {
					IT6602VideoOutputConfigure(it6602);
					it6602->m_NewAVIInfoFrameF = FALSE;
				}
			}

			if (hdmirxrd(REG_RX_053) & B_VDGatting) {
				//if(IT6602_IsSelectedPort(0)
				{
					if ((it6602->m_RAP_ContentOff == 0) && (it6602->m_HDCP_ContentOff == 0)) {
						if (CheckAVMute() == FALSE) {
							IT6602_SetVideoMute(it6602, OFF);
						}
					}
				}
			}

#ifdef _FIX_ID_028_

			if (hdmirxrd(REG_RX_0AA) & 0x80) {
				//FIX_ID_037 xxxxx //Allion MHL compliance issue !!!
				if (it6602->m_RAP_ContentOff == 0) { //xxxxx 2014-0529 //Manual Content On/Off
					if (it6602->m_AState != ASTATE_AudioOn) {
						it6602->m_AudioCountingTimer = AUDIO_READY_TIMEOUT;
						it6602->m_AState = ASTATE_AudioOn;
						m_bAudioWaiting = TRUE;

					} else {

						if (it6602->m_AudioCountingTimer > MS_LOOP) {
							it6602->m_AudioCountingTimer -= MS_LOOP;

						} else {
							it6602->m_AudioCountingTimer = 0;

							if (m_bAudioWaiting == TRUE) {
								IT6602AudioOutputEnable(TRUE);
							}
						}
					}
				} //xxxxx 2014-0529

				//FIX_ID_037 xxxxx

			} else {
				if (it6602->m_AState == ASTATE_AudioOn) {
					IT6602AudioOutputEnable(FALSE);
				}
			}

#endif
			break;
		}

	default:
		break;
	}
}

// ***************************************************************************
// Interrupt function
// ***************************************************************************
void IT66021::hdmirx_INT_5V_Pwr_Chg(struct it6602_dev_data *it6602, unsigned char ucport)
{
	unsigned char ucPortSel = hdmirxrd(REG_RX_051) & B_PORT_SEL;

	if (ucport != 0 || ucPortSel != 0) {
		PX4_ERR("it66021 Only support port 0, some error!!");
		return;
	}

	if (CheckPlg5VPwr(ucport) == TRUE) {
		IT_INFO("#### Power 5V ON ####\r\n");
		IT6602SwitchVideoState(it6602, VSTATE_SyncWait);
		it6602HPDCtrl(ucport, 1); // set ucport's HPD = 1

	} else {
		IT_INFO("#### Power 5V OFF ####\r\n");
		IT6602SwitchVideoState(it6602, VSTATE_SWReset);
		it6602HPDCtrl(ucport, 0); // clear ucport's HPD = 0
	}
}
// ---------------------------------------------------------------------------
void IT66021::hdmirx_INT_P0_ECC(struct it6602_dev_data *it6602)
{
	// struct it6602_dev_data *it6602data = get_it6602_dev_data();

	if ((it6602->m_ucEccCount_P0++) > ECC_TIMEOUT) {

#ifdef _SUPPORT_EQ_ADJUST_

		if (it6602->EQPort[F_PORT_SEL_0].f_manualEQadjust == TRUE) { // ignore ECC interrupt when manual EQ adjust !!!
			return;
		}

#endif

		it6602->m_ucEccCount_P0 = 0;

		IT_INFO("CDR reset for Port0 ECC_TIMEOUT !!!\r\n");

		hdmirx_ECCTimingOut(F_PORT_SEL_0);
	}
}



void IT66021::hdmirx_INT_P0_Deskew(struct it6602_dev_data *it6602)
{
	IT_INFO("m_ucDeskew_P0 = %d", it6602->m_ucDeskew_P0);

	if ((it6602->m_ucDeskew_P0++) > DESKEW_TIMEOUT) {
#ifdef _SUPPORT_EQ_ADJUST_

		if (it6602->EQPort[F_PORT_SEL_0].f_manualEQadjust == TRUE) { // ignore ECC interrupt when manual EQ adjust !!!
			return;
		}

#endif
		it6602->m_ucDeskew_P0 = 0;

		IT_INFO("CDR reset for Port0 DESKEW_TIMEOUT !!!\r\n");

		if (hdmirxrd(REG_RX_020) == 0x00) {
			hdmirxwr(REG_RX_020, 0x3F);        // Dr. Liu suggestion to 0x00

		} else {
			hdmirxwr(REG_RX_020, 0x00);        // Dr. Liu suggestion to 0x3F
		}
	}
}


void IT66021::hdmirx_INT_HDMIMode_Chg(struct it6602_dev_data *it6602, unsigned char ucport)
{
	unsigned char ucPortSel;
	ucPortSel = hdmirxrd(REG_RX_051) & B_PORT_SEL;
	IT_INFO("hdmirx_INT_HDMIMode_Chg = %d", ucPortSel);

	if (ucPortSel != ucport) {
		return;
	}

	if (IsHDMIMode()) {
		if (it6602->m_VState == VSTATE_VideoOn) {
			IT6602SwitchAudioState(it6602, ASTATE_RequestAudio);
		}

		it6602->m_bUpHDMIMode = TRUE;
		IT_INFO("#### HDMI/DVI Mode : HDMI ####\r\n");

	} else {
		IT6602SwitchAudioState(it6602, ASTATE_AudioOff);
		it6602->m_NewAVIInfoFrameF = FALSE;

		if (it6602->m_VState == VSTATE_VideoOn) {
			SetDVIVideoOutput(it6602);
		}

		it6602->m_bUpHDMIMode = FALSE;
		IT_INFO("#### HDMI/DVI Mode : DVI ####\r\n");
	}
}

void IT66021::hdmirx_INT_SCDT_Chg(struct it6602_dev_data *it6602)
{
	if (CheckSCDT(it6602) == TRUE) {
		IT_INFO("#### SCDT ON ####\r\n");
		IT6602SwitchVideoState(it6602, VSTATE_SyncChecking);

	} else {
		IT_INFO("#### SCDT OFF ####\r\n");
		IT6602SwitchVideoState(it6602, VSTATE_SyncWait);
		IT6602SwitchAudioState(it6602, ASTATE_AudioOff);
	}
}

#ifdef _SUPPORT_AUTO_EQ_
void IT66021::hdmirx_INT_EQ_FAIL(struct it6602_dev_data *it6602, unsigned char ucPortSel)
{
	if (ucPortSel > F_PORT_SEL_1) {
		return;
	}

#ifdef _SUPPORT_EQ_ADJUST_

	if (it6602->EQPort[ucPortSel].f_manualEQadjust == FALSE) // ignore EQ fail interrupt when manual EQ adjust !!!
#endif
	{
		if (CheckPlg5VPwr(ucPortSel)) {

			//07-08
			if (ucPortSel == 0) {
				if ((it6602->HDMIIntEvent & (B_PORT0_TMDSEvent))) {
					IT_INFO("#### hdmirx_INT_EQ_FAIL not yet !!! ####\r\n");

					//FIX_ID_033 xxxxx  //Fine-tune EQ Adjust function for HDCP receiver and repeater mode
					if ((it6602->HDMIIntEvent & (B_PORT0_Waiting)) == 0) {
						hdmirxwr(REG_RX_022, 0x00); // power down auto EQ
						it6602->HDMIIntEvent |= (B_PORT0_Waiting);
						it6602->HDMIIntEvent |= (B_PORT0_TMDSEvent);
						it6602->HDMIWaitNo[0] = MAX_TMDS_WAITNO;

					} else if ((it6602->HDMIIntEvent & (B_PORT0_TMDSEvent))) {
						it6602->HDMIIntEvent |= (B_PORT0_Waiting);
						it6602->HDMIWaitNo[0] += MAX_HDCP_WAITNO;
					}

					return;
				}

				if (hdmirxrd(REG_RX_P0_SYS_STATUS) & (B_P0_MHL_MODE)) {
					if ((ucPortAMPValid[0] & 0x03) != 0x03) {
						AmpValidCheck(ucPortSel);
					}

				} else {
					if ((ucPortAMPValid[ucPortSel] & 0x3F) != 0x3F) {
						AmpValidCheck(ucPortSel);
					}
				}

			} else {
				if ((it6602->HDMIIntEvent & (B_PORT1_TMDSEvent))) {
					IT_INFO("#### hdmirx_INT_EQ_FAIL not yet !!! ####\r\n");

					if ((it6602->HDMIIntEvent & (B_PORT1_Waiting)) == 0) {
						hdmirxwr(REG_RX_03A, 0x00); // power down auto EQ
						it6602->HDMIIntEvent |= (B_PORT1_Waiting);
						it6602->HDMIIntEvent |= (B_PORT1_TMDSEvent);
						it6602->HDMIWaitNo[1] = MAX_TMDS_WAITNO;

					} else if ((it6602->HDMIIntEvent & (B_PORT1_TMDSEvent))) {
						it6602->HDMIIntEvent |= (B_PORT1_Waiting);
						it6602->HDMIWaitNo[1] += MAX_HDCP_WAITNO;
					}

					return;
				}

				if ((ucPortAMPValid[ucPortSel] & 0x3F) != 0x3F) {
					AmpValidCheck(ucPortSel);
				}
			}

			{
				if (ucPortSel == 0) {
					if (hdmirxrd(REG_RX_P0_SYS_STATUS) & (B_P0_MHL_MODE)) {
						if ((ucPortAMPValid[0] & 0x03) != 0x03) {
							TogglePolarity(ucPortSel);
						}

					} else {
						if ((ucPortAMPValid[ucPortSel] & 0x3F) != 0x3F) {
							TogglePolarity(ucPortSel);
						}
					}

				} else {
					if ((ucPortAMPValid[ucPortSel] & 0x3F) != 0x3F) {
						TogglePolarity(ucPortSel);
					}
				}
			}
		}
	}
}
#endif

#ifdef _SUPPORT_EDID_RAM_
/*****************************************************************************/
/* EDID RAM  functions    *******************************************************/
/*****************************************************************************/

unsigned char IT66021::UpdateEDIDRAM(unsigned char *pEDID, unsigned char BlockNUM)
{
	unsigned char i, offset, sum = 0;

	if (BlockNUM == 0x02) {
		offset = 0x00 + 128 * 0x01;

	} else {
		offset = 0x00 + 128 * BlockNUM;
	}

	for (i = 0; i < 0x7F; i++) {
		EDID_RAM_Write(offset, 1, pEDID + offset);
		sum += *(pEDID + offset);
		offset++;
	}

	sum = 0x00 - sum;

	return sum;
}

void IT66021::EnableEDIDupdata(void)
{
	IT_INFO("EnableEDIDupdata() \n");

	it6602HPDCtrl(0, 0); // HDMI/MHL port 0, set HPD = 0
	it6602HPDCtrl(1, 0); // HDMI port 1, set HPD = 0
}

void IT66021::DisableEDIDupdata(void)
{
	IT_INFO("DisableEDIDupdata() \n");
}

//static void EDIDRAMInitial(_CODE unsigned char *pIT6602EDID)
void IT66021::EDIDRAMInitial(unsigned char *pIT6602EDID)
{
	unsigned char Block0_CheckSum;
	unsigned char Block1_CheckSum;
	unsigned char u8_VSDB_Addr;
	unsigned char BlockNo;

	u8_VSDB_Addr = 0;

	EnableEDIDupdata();

	for (BlockNo = 0; BlockNo < 2; BlockNo++) {
		if (BlockNo == 0) {
			Block0_CheckSum =  UpdateEDIDRAM(pIT6602EDID, 0);
			hdmirxwr(REG_RX_0C4, Block0_CheckSum);		//Port 0 Bank 0 CheckSum
			hdmirxwr(REG_RX_0C8, Block0_CheckSum);		//Port 1 Bank 0 CheckSum

		} else {
			Block1_CheckSum =  UpdateEDIDRAM(pIT6602EDID, 1);
			u8_VSDB_Addr = Find_Phyaddress_Location(pIT6602EDID, 1);
			PhyAdrSet();

			if (u8_VSDB_Addr != 0) {
				UpdateEDIDReg(u8_VSDB_Addr, pIT6602EDID[u8_VSDB_Addr], pIT6602EDID[u8_VSDB_Addr + 1], Block1_CheckSum);
			}
		}
	}

	DisableEDIDupdata();
}

//static unsigned char Find_Phyaddress_Location(_CODE unsigned char *pEDID,unsigned char Block_Number)
unsigned char IT66021:: Find_Phyaddress_Location(unsigned char *pEDID, unsigned char Block_Number)
{
	unsigned char AddStart;
	unsigned char tag, count;
	unsigned char offset, End;
	unsigned char u8_VSDB_Addr;

#ifdef FIX_ID_013_
	//FIX_ID_013	xxxxx	//For MSC 3D request issue
	unsigned char u8_3DPresent_Addr;
	unsigned char ucTemp;
	struct PARSE3D_STR *pstParse3D = get_EDID_VSDB_3Ddata();
	//FIX_ID_013	xxxxx
#endif //FIX_ID_013

	if (Block_Number == 0x02) {
		AddStart = 0x00 + 128 * 0x01;

	} else {
		AddStart = 0x00 + 128 * Block_Number;
	}

	if ((*(pEDID + AddStart)) != 0x2 || (*(pEDID + AddStart + 1)) != 0x3) {
		return 0;
	}

	End = (*(pEDID + AddStart + 2));
	u8_VSDB_Addr = 0;

#ifdef FIX_ID_013_
	//FIX_ID_013	xxxxx	//For MSC 3D request issue
	// initial value then check with SVD and VSDB block to find the SVD of 3D support timing
	pstParse3D->bVSDBspport3D = 0x00;
	pstParse3D->ucVicCnt = 0;
	//FIX_ID_013	xxxxx
#endif //FIX_ID_013

	for (offset = (AddStart + 0x04); offset < (AddStart + End);) {
		tag = (*(pEDID + offset)) >> 5;
		count = (*(pEDID + offset)) & 0x1f;

		// IT_INFO("offset = %X , Tag = %X , count =%X \n", (int) offset, (int)  tag, (int) count);

		offset++;

		if (tag == 0x03) {	// HDMI VSDB Block of EDID
			// IT_INFO("HDMI VSDB Block address = %X\n", (int)  offset);

			if ((*(pEDID + offset)) == 0x03 &&
			    (*(pEDID + offset + 1)) == 0x0C &&
			    (*(pEDID + offset + 2)) == 0x0) {
				u8_VSDB_Addr = offset + 3;
				txphyadr[0] = (*(pEDID + offset + 3));
				txphyadr[1] = (*(pEDID + offset + 4));
				// IT_INFO("txphyadr[0] = %X\n", (int)  txphyadr[0]);
				// IT_INFO("txphyadr[1] = %X\n", (int)  txphyadr[1]);

#ifdef FIX_ID_013_
				//FIX_ID_013	xxxxx	//For MSC 3D request issue

				if (count < 7) {	// no 3D support !!!
					return u8_VSDB_Addr;
				}

				u8_3DPresent_Addr = offset + 7;

				ucTemp = *(pEDID + offset + 7);

				if (ucTemp & 0x80) {			// Video and Audio Latency present
					u8_3DPresent_Addr += 2;
				}

				if (ucTemp & 0x40) {		// Interlaced Video and Audio Latency present
					u8_3DPresent_Addr += 2;
				}

				if (ucTemp & 0x20) {			// HDMI additional video format present
					u8_3DPresent_Addr++;
				}

				pstParse3D->uc3DEdidStart = u8_3DPresent_Addr;

				pstParse3D->uc3DBlock = Block_Number;

				pstParse3D->bVSDBspport3D = 0x01;		// for identify the HDMI VSDB 3D support
				//FIX_ID_013	xxxxx
#endif //FIX_ID_013
				return u8_VSDB_Addr;
			}
		}

#ifdef FIX_ID_013_
//FIX_ID_013	xxxxx	//For MSC 3D request issue

		if (tag == 0x02) {	// Short Video Descriptor of EDID
			//#ifdef printf_EDID
			IT_INFO("Short Video Descriptor Address = %X, VIC count = %X \r\n", (int)  offset, (int) count);
			//#endif

			// get the SVD size
			pstParse3D->ucVicCnt = count;

			for (ucTemp = 0; ucTemp < count; ucTemp++) {
				u8_3DPresent_Addr = (*(pEDID + offset + ucTemp)) & 0x7F;
				SVD_LIST[ucTemp] = u8_3DPresent_Addr;
				IT_INFO("SVD[%X] = %X\n", ucTemp, u8_3DPresent_Addr);
			}
		}

//FIX_ID_013	xxxxx
#endif //FIX_ID_013

		offset = offset + count;
	}

	return 0;
}

void IT66021::UpdateEDIDReg(unsigned char u8_VSDB_Addr, unsigned char CEC_AB, unsigned char CEC_CD,
			    unsigned char Block1_CheckSum)
{

	unsigned char  A_Addr_AB, A_Addr_CD, A_Block1_CheckSum;
	unsigned char  B_Addr_AB, B_Addr_CD, B_Block1_CheckSum;

	A_Addr_AB = rxphyadr[0][0];
	A_Addr_CD = rxphyadr[0][1];

	B_Addr_AB = rxphyadr[1][0];
	B_Addr_CD = rxphyadr[1][1];


	A_Block1_CheckSum = (Block1_CheckSum + CEC_AB + CEC_CD - A_Addr_AB - A_Addr_CD) % 0x100;
	B_Block1_CheckSum = (Block1_CheckSum + CEC_AB + CEC_CD - B_Addr_AB - B_Addr_CD) % 0x100;


	hdmirxwr(REG_RX_0C1, u8_VSDB_Addr);			//VSDB Start Address
	hdmirxwr(REG_RX_0C2, A_Addr_AB);					//Port 0 AB
	hdmirxwr(REG_RX_0C3, A_Addr_CD);				//Port 0 CD
	hdmirxwr(REG_RX_0C5, A_Block1_CheckSum);		//Port 0 Bank 1 CheckSum

	hdmirxwr(REG_RX_0C6, B_Addr_AB);					//Port 1 AB
	hdmirxwr(REG_RX_0C7, B_Addr_CD);				//Port 1 CD
	hdmirxwr(REG_RX_0C9, B_Block1_CheckSum);		//Port 1 Bank 1 CheckSum
}

void IT66021::PhyAdrSet(void)
{
	{
		rxphyadr[0][0] = 0x10;
		rxphyadr[0][1] = 0x00;
		rxphyadr[1][0] = 0x20;
		rxphyadr[1][1] = 0x00;
	}
}

#endif

void IT66021::IT6602HDMIInterruptHandler(struct it6602_dev_data *it6602)
{
	volatile unsigned char Reg05h;

	// it66021 have no reg06, although it can be read
	// volatile unsigned char Reg06h;

	volatile unsigned char Reg07h;
	volatile unsigned char Reg08h;
	volatile unsigned char Reg09h;
	volatile unsigned char Reg0Ah;
	volatile unsigned char RegD0h;

	chgbank(0);

	Reg05h = hdmirxrd(REG_RX_005);
	Reg07h = hdmirxrd(REG_RX_007);
	Reg08h = hdmirxrd(REG_RX_008);
	Reg09h = hdmirxrd(REG_RX_009);

	Reg0Ah = hdmirxrd(REG_RX_P0_SYS_STATUS);
	//	Reg0Bh = hdmirxrd(REG_RX_P1_SYS_STATUS);
	RegD0h = hdmirxrd(REG_RX_0D0);

	hdmirxwr(REG_RX_005, Reg05h);
	hdmirxwr(REG_RX_007, Reg07h);
	hdmirxwr(REG_RX_008, Reg08h);
	hdmirxwr(REG_RX_009, Reg09h);
	hdmirxwr(REG_RX_0D0, RegD0h & 0x0F);

	IT_INFO("---------ite interrupt------------\r\n");
	IT_INFO("Reg05 = %X", (int)Reg05h);
	IT_INFO("Reg07 = %X", (int)Reg07h);
	IT_INFO("Reg08 = %X", (int)Reg08h);
	IT_INFO("Reg09 = %X", (int)Reg09h);
	IT_INFO("Reg0A = %X", (int)Reg0Ah);
	IT_INFO("RegD0 = %X", (int)RegD0h);

	if (Reg05h != 0x00) {
		IT_INFO("Reg05 = 0x%x \r\n", (int)Reg05h);

		if (Reg05h & 0x80) {
			IT_INFO("#### Port 0 HDCP Off Detected ###\r\n");
			it6602->m_ucEccCount_P0 = 0;
		}

		if (Reg05h & 0x40) {
			IT_INFO("#### Port 0 ECC Error %X ####\r\n", (int)(it6602->m_ucEccCount_P0));
			//	HDMICheckErrorCount(&(it6602->EQPort[F_PORT_SEL_0]));	//07-04 for port 0
			hdmirx_INT_P0_ECC(it6602);
		}

		if (Reg05h & 0x20) {

			IT_INFO("#### Port 0 HDMI/DVI Mode change ####\r\n");

			if (CLKCheck(0)) {
				hdmirx_INT_HDMIMode_Chg(it6602, 0);
			}
		}

		if (Reg05h & 0x08) {
			IT_INFO("#### Port 0 HDCP Authentication Start ####\r\n");
			it6602->m_ucEccCount_P0 = 0;

#ifdef _SUPPORT_AUTO_EQ_

			if (ucPortAMPOverWrite[F_PORT_SEL_0] == 0) {
				if ((it6602->HDMIIntEvent & (B_PORT0_Waiting)) == 0) {
					hdmirxwr(REG_RX_022, 0x00); // power down auto EQ

					it6602->HDMIIntEvent |= (B_PORT0_Waiting);
					it6602->HDMIIntEvent |= (B_PORT0_TMDSEvent);
					it6602->HDMIWaitNo[0] = MAX_TMDS_WAITNO;

				} else if ((it6602->HDMIIntEvent & (B_PORT0_TMDSEvent))) {
					it6602->HDMIIntEvent |= (B_PORT0_Waiting);
					it6602->HDMIWaitNo[0] += MAX_HDCP_WAITNO;
				}

			} else {
				if ((it6602->HDMIIntEvent & (B_PORT0_TMDSEvent))) {
					it6602->HDMIIntEvent |= (B_PORT0_Waiting);
					it6602->HDMIWaitNo[0] += MAX_HDCP_WAITNO;
				}
			}

#endif

			if ((Reg0Ah & 0x40)) {
				it6602->CBusIntEvent |= (B_MSC_Waiting);
				it6602->CBusWaitNo = MAX_CBUS_WAITNO;
			}
		}

		if (Reg05h & 0x10) {

			IT_INFO("#### Port 0 HDCP Authentication Done ####\r\n");

			if ((Reg0Ah & 0x40)) {
				it6602->CBusIntEvent |= (B_MSC_Waiting);
				it6602->CBusWaitNo = MAX_CBUS_WAITNO;
			}

#ifdef _SUPPORT_AUTO_EQ_

			if (ucPortAMPOverWrite[0] == 0) { // 2013-0801
				it6602->HDMIIntEvent &= ~(B_PORT0_Waiting);
				it6602->HDMIWaitNo[0] = 0;
				it6602->HDMIIntEvent |= B_PORT0_TMDSEvent;
				//return;
			}

#endif
		}

		if (Reg05h & 0x04) {
			IT_INFO("#### Port 0 Input Clock Change Detect ####\r\n");
		}


		if (Reg05h & 0x02) {
			it6602->m_ucEccCount_P0 = 0;
			it6602->m_ucDeskew_P0 = 0;

			IT_INFO("#### Port 0 Rx CKOn Detect ####\r\n");

			if (CLKCheck(F_PORT_SEL_0)) {
#ifdef _SUPPORT_AUTO_EQ_
				TMDSCheck(F_PORT_SEL_0);
#else
				//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_EQ_ADJUST_
				HDMIStartEQDetect(&(it6602->EQPort[F_PORT_SEL_0]));
#endif
				//FIX_ID_001 xxxxx
#endif
			}
		}

		if (Reg05h & 0x01) {
			IT_INFO("#### Port 0 Power 5V change ####\n");
			hdmirx_INT_5V_Pwr_Chg(it6602, 0);
		}
	}

	if (Reg07h != 0x00) {

		IT_INFO("Reg07 = 0x%x \r\n", (int)Reg07h);

		if (Reg07h & 0x80) {
			IT_INFO("#### Audio FIFO Error ####\r\n");
			aud_fiforst();
		}

		if (Reg07h & 0x40) {
			IT_INFO("#### Audio Auto Mute ####\r\n");
		}

		if (Reg07h & 0x20) {
			IT_INFO("#### Packet Left Mute ####\r\n");
			IT6602_SetVideoMute(it6602, OFF);
		}

		if (Reg07h & 0x10) {
			IT_INFO("#### Set Mute Packet Received ####\r\n");

			IT6602_SetVideoMute(it6602, ON);
		}

		if (Reg07h & 0x08) {
			IT_INFO("#### Timer Counter Tntterrupt ####\r\n");
		}

		if (Reg07h & 0x04) {
			IT_INFO("#### Video Mode Changed ####\r\n");
		}

		if (Reg07h & 0x02) {
			hdmirx_INT_SCDT_Chg(it6602);
		}

		if (Reg07h & 0x01) {
			if ((Reg0Ah & 0x40) >> 6) {
				IT_INFO("#### Port 0 Bus Mode : MHL ####\r\n");

				//FIX_ID_002 xxxxx 	Check IT6602 chip version Identify for TogglePolarity and Port 1 Deskew
				if (HdmiI2cAddr == IT66021A_HDMI_ADDR) {
					chgbank(1);
					hdmirxset(REG_RX_1B6, 0x07, 0x00);
					//FIX_ID_007 xxxxx 	//for debug IT6681  HDCP issue
					hdmirxset(REG_RX_1B1, 0x20, 0x20); //Reg1b1[5] = 1 for enable over-write
					hdmirxset(REG_RX_1B2, 0x07, 0x01); // default 0x04 , change to 0x01
					IT_INFO(" Port 0 Bus Mode Reg1B1  = %X ,Reg1B2  = %X\r\n", (int)hdmirxrd(REG_RX_1B1), (int)hdmirxrd(REG_RX_1B2));
					//FIX_ID_007 xxxxx
					chgbank(0);
				}

			} else {
				IT_INFO("#### Port 0 Bus Mode : HDMI ####\r\n");

				//FIX_ID_002 xxxxx 	Check IT6602 chip version Identify for TogglePolarity and Port 1 Deskew
				if (HdmiI2cAddr == IT66021A_HDMI_ADDR) {
					chgbank(1);
					hdmirxset(REG_RX_1B6, 0x07, 0x03);
					////FIX_ID_007 xxxxx 	//for debug IT6681  HDCP issue
					hdmirxset(REG_RX_1B1, 0x20, 0x00); //Reg1b1[5] = 0 for disable over-write
					hdmirxset(REG_RX_1B2, 0x07, 0x04); // default 0x04 , change to 0x01
					IT_INFO(" Port 0 Bus Mode Reg1B1  = %X ,Reg1B2  = %X\r\n", (int)hdmirxrd(REG_RX_1B1), (int)hdmirxrd(REG_RX_1B2));
					////FIX_ID_007 xxxxx
					chgbank(0);
				}

				//FIX_ID_002 xxxxx
			}
		};
	}

	if (Reg08h != 0x00) {
		//IT_INFO("Reg08h = %X",(int) Reg08h);
		if (Reg08h & 0x80) {
			//			 IT_INFO("#### No General Packet 2 Received ####\n");
		}

		if (Reg08h & 0x40) {
			//			 IT_INFO("#### No General Packet Received ####\n");
		}

		if (Reg08h & 0x20) {
			IT_INFO("#### No Audio InfoFrame Received ####\r\n");
		}

		if (Reg08h & 0x10) {
			IT_INFO("#### No AVI InfoFrame Received ####\r\n");
		}

		if (Reg08h & 0x08) {
			IT_INFO("#### CD Detect ####\r\n");
		}

		if (Reg08h & 0x04) {
			//			 IT_INFO("#### Gen Pkt Detect ####\n");
			IT_INFO("#### 3D InfoFrame Detect ####\r\n");

#ifdef Enable_Vendor_Specific_packet

			if (it6602->f_de3dframe_hdmi == FALSE) {
				it6602->f_de3dframe_hdmi = IT6602_DE3DFrame(TRUE);
			}

#endif
		}

		if (Reg08h & 0x02) {
			IT_INFO("#### ISRC2 Detect ####\r\n");
		}

		if (Reg08h & 0x01) {
			IT_INFO("#### ISRC1 Detect ####\r\n");
		}
	}

	if (Reg09h != 0x00) {
		//IT_INFO("Reg09h = %X",(int) Reg09h);
		if (Reg09h & 0x80) {
			IT_INFO("#### H2V Buffer Skew Fail ####\r\n");
		}

		if (Reg09h & 0x20) {
			hdmirxwr(0x09, 0x20);
			IT_INFO("#### Port 0 Deskew Error ####\r\n");
			hdmirx_INT_P0_Deskew(it6602);
		}

		if (Reg09h & 0x10) {
			IT_INFO("#### New Audio Packet Received ####\r\n");
		}

		if (Reg09h & 0x08) {
			IT_INFO("#### New ACP Packet Received ####\r\n");
		}

		if (Reg09h & 0x04) {
			IT_INFO("#### New SPD Packet Received ####\r\n");
		}

		if (Reg09h & 0x02) {
			IT_INFO("#### New MPEG InfoFrame Received ####\r\n");
		}

		if (Reg09h & 0x01) {
			IT_INFO("#### New AVI InfoFrame Received ####\r\n");
			//IT6602VideoOutputConfigure();
			it6602->m_NewAVIInfoFrameF = TRUE;
		}
	}

	if (RegD0h != 0x00) {
		if (RegD0h & 0x10) {

			hdmirxwr(0xD0, 0x30);
			RegD0h &= 0x30;
			IT_INFO("#### Port 0 EQ done interrupt ####\r\n");

#ifdef _SUPPORT_AUTO_EQ_
			AmpValidCheck(0); //2013-0801
#endif

#ifdef _SUPPORT_EQ_ADJUST_
			HDMIStartEQDetect(&(it6602->EQPort[F_PORT_SEL_0]));
#endif
		}

		if (RegD0h & 0x40) {

			hdmirxwr(0xD0, 0xC0);
			RegD0h &= 0xC0;
			IT_INFO("#### Port 1 EQ done interrupt ####\r\n");

#ifdef _SUPPORT_AUTO_EQ_
			AmpValidCheck(1); //2013-0801
#endif
		}

		if (RegD0h & 0x20) {
			hdmirxwr(0xD0, 0x20);
			IT_INFO("#### Port 0 EQ Fail Interrupt ####\r\n");
#ifdef _SUPPORT_AUTO_EQ_
			hdmirx_INT_EQ_FAIL(it6602, F_PORT_SEL_0);
#endif
		}

		if (RegD0h & 0x80) {

			hdmirxwr(0xD0, 0x80);
			IT_INFO("#### Port 1 EQ Fail Interrupt ####\r\n");
			//	HDMICheckErrorCount(&(it6602->EQPort[F_PORT_SEL_1]));	//07-04 for port 0
			//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_AUTO_EQ_
			hdmirx_INT_EQ_FAIL(it6602, F_PORT_SEL_1);
#endif
		}
	}
}

#if Enable_IR
void IT66021::it6602AutoPortSelect(struct it6602_dev_data *it6602)
{
	if (SEL_PORT_1 == 1) {
		if (it6602->m_ucCurrentHDMIPort != 0) {
			it6602PortSelect(0);
		}

	} else {
		if (it6602->m_ucCurrentHDMIPort == 0) {
			it6602PortSelect(1);
		}
	}
}
#endif

#ifdef Enable_Vendor_Specific_packet

#define HDMI_3DFORMAT_PRESENT 0x40
#define HDMI_3DFORMAT_OFF 0x00
#define FRAME_PACKING 0x00
#define TOP_AND_BOTTOM 0x60
#define SIDE_BY_SIDE 0x80

SET_DE3D_FRAME t_3d_syncgen[] = {
	//640x480      //524   //559   //514   //526
	{0x01, 0x020C, 0x022F, 0x0202, 0x020E, 480}, // 60Hz
	//480p      //524   //560   //515   //530
	{0x02, 0x020C, 0x0230, 0x0203, 0x0212, 480}, // 60Hz
	{0x03, 0x020C, 0x0230, 0x0203, 0x0212, 480}, // 60Hz
	//576p      //624   //668   //619   //629
	{0x11, 0x0270, 0x029C, 0x026B, 0x0275, 576}, // 50Hz
	{0x12, 0x0270, 0x029C, 0x026B, 0x0275, 576}, // 50Hz
	//720p      //749   //774   //744   //754
	{0x3c, 0x02ED, 0x0306, 0x02E8, 0x02F2, 720}, // 24Hz
	{0x3d, 0x02ED, 0x0306, 0x02E8, 0x02F2, 720}, // 25Hz
	{0x3e, 0x02ED, 0x0306, 0x02E8, 0x02F2, 720}, // 30Hz
	{0x13, 0x02ED, 0x0306, 0x02E8, 0x02F2, 720}, // 50Hz
	{0x04, 0x02ED, 0x0306, 0x02E8, 0x02F2, 720}, // 60Hz

	//disable -> 1080i     //1124   //1165   //1120   //1129
	//disable ->     {0x05      ,0x0464  ,0x048D  ,0x0460  ,0x0469,	540}, // 50Hz
	//disable ->     {0x14      ,0x0464  ,0x048D  ,0x0460  ,0x0469,	540}, // 60Hz
	//disable -> 1080i     //1124   //1165   //1120   //1129
	//disable ->     {0x20      ,0x0464  ,0x048D  ,0x0460  ,0x0469,	540}, // 24Hz
	//disable ->     {0x22      ,0x0464  ,0x048D  ,0x0460  ,0x0469,	540}, // 30Hz
	//disable ->     {0x1f      ,0x0464  ,0x048D  ,0x0460  ,0x0469,	540}, // 50Hz
	//disable ->     {0x10      ,0x0464  ,0x048D  ,0x0460  ,0x0469,	540}, // 60Hz

	//1080p    //1124   //1165   //1120   //1129
	{0x20, 0x0464, 0x048D, 0x0460, 0x0469, 1080}, // 24Hz
	{0x21, 0x0464, 0x048D, 0x0460, 0x0469, 1080}, // 25Hz
	{0x22, 0x0464, 0x048D, 0x0460, 0x0469, 1080}, // 30Hz

	//default
	{0xFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000}
};
//Reg_PGVTotal	749		// 0x2ED
//Reg_PGVActst 	774		// 0x306
//Reg_PGVActEd	744		// 0x2E8
//Reg_PGVsyncEd	754		// 0x2F2
#define Reg_PGVTotal_19D 0x9D  //[11:4]	0x19D[7:0]
#define Reg_PGVTotal_19C 0x9C  //[3:0]		0x19C[7:4]
#define Reg_PGVActSt_192 0x92  //[7:0]		0x192[7:0]
#define Reg_PGVActSt_193 0x93  //[11:8]	0x193[3:0]
#define Reg_PGVActEd_193 0x93  //[3:0]		0x193[7:4]
#define Reg_PGVActEd_194 0x94  //[11:4]	0x194[7:0]
#define Reg_PGVSyncEd_19F 0x9F //[3:0]		0x19F[7:4]
#define Reg_PGVSyncSt_19F 0x9F //[11:8]	0x19F[3:0]
#define Reg_PGVSyncSt_19E 0x9E //[7:0]		0x19E[7:0]

#define Reg_PG3DRSt_18F 0x8F   //[7:0]		0x190[11:8] 0x18F[7:0]
#define Reg_PG3DRStEd_190 0x90 //[7:0]		0x191[3:0] 0x18F[11:8]
#define Reg_PG3DREd_191 0x91   //[11:4]		0x191[11:4] 0x190[3:0]

#define REG_RX_066_4_DE3DFrame 0x66 //[4] 1: 3D frame-packet mode to sequence mode
#define REG_RX_085_5_En3DROut 0x85  //[5] 1: Enable 3DR output
//
//pccmd w 0f 01 94
//pccmd w 8f 86 94
//pccmd w 90 41 94
//pccmd w 91 47 94
//pccmd w 92 06 94
//pccmd w 93 83 94
//pccmd w 94 2E 94
//pccmd w 9c d0 94
//pccmd w 9d 2e 94
//pccmd w 9f 22 94
//pccmd w 0f 00 94
//pccmd w 66 58 94

void IT66021::Dump3DReg(void)
{
	ushort i, j;
	BYTE ucData;

	IT_INFO("\r\n       ");

	for (j = 0; j < 16; j++) {
		IT_INFO(" %02X", (int)j);

		if ((j == 3) || (j == 7) || (j == 11)) {
			IT_INFO(" :");
		}
	}

	IT_INFO("\r\n");

	chgbank(1);

	for (i = 0x80; i < 0xa0; i += 16) {
		IT_INFO("[%03X]  ", i);

		for (j = 0; j < 16; j++) {
			ucData = hdmirxrd((BYTE)((i + j) & 0xFF));
			IT_INFO(" %02X", (int)ucData);

			if ((j == 3) || (j == 7) || (j == 11)) {
				IT_INFO(" :");
			}
		}

		IT_INFO("\r\n");
	}

	IT_INFO("\n        =====================================================\r\n");

	chgbank(0);
}

unsigned char IT66021::IT6602_DE3DFrame(unsigned char ena_de3d)
{
	unsigned char i, uc;
	unsigned int v_total;
	unsigned int v_act_start;
	unsigned int v_act_end;
	unsigned int v_sync_end;
	unsigned int v_act_bspace = 0;
	unsigned int v_2d_Vtotal;
	// unsigned int HActive;
	unsigned int LR_3D_Start;
	unsigned int LR_3D_End;

#ifdef DEBUG_MODE
//dbmsg_trace(DBM_DPATH,"ITEHDMI - HDMI_DE3DFrame \r\n");
#endif

	struct it6602_dev_data *it6602data = get_it6602_dev_data();

	if (ena_de3d == TRUE) {

		chgbank(2);
		uc = hdmirxrd(REG_RX_224);
		chgbank(0);

		if (uc == 0x81) { // 3D InfoFrame Packet Type is valid

			chgbank(2);
			it6602data->s_Current3DFr.VIC = hdmirxrd(REG_RX_218); //AVI INFO PB4
			it6602data->s_Current3DFr.HB0 = hdmirxrd(REG_RX_224); // General Packet Header Byte 0
			it6602data->s_Current3DFr.HB1 = hdmirxrd(REG_RX_225);
			it6602data->s_Current3DFr.HB2 = hdmirxrd(REG_RX_226);
			it6602data->s_Current3DFr.PB0 = hdmirxrd(REG_RX_227); // General Packet Data Byte 0
			it6602data->s_Current3DFr.PB1 = hdmirxrd(REG_RX_228);
			it6602data->s_Current3DFr.PB2 = hdmirxrd(REG_RX_229);
			it6602data->s_Current3DFr.PB3 = hdmirxrd(REG_RX_22A);
			it6602data->s_Current3DFr.PB4 = hdmirxrd(REG_RX_22B);
			it6602data->s_Current3DFr.PB5 = hdmirxrd(REG_RX_22C);
			it6602data->s_Current3DFr.PB6 = hdmirxrd(REG_RX_22D);
			it6602data->s_Current3DFr.PB7 = hdmirxrd(REG_RX_22E);
			chgbank(0);

			//#ifdef DEBUG_MODE_3D
			IT_INFO("\r\nIT653x - HDMI_DumpDE3DFrameInfo: \r\n");
			IT_INFO("        HDMI VIC = 0x%X \r\n", it6602data->s_Current3DFr.VIC);
			IT_INFO("        Record HDMI vender specific inforframe HB0 = 0x%X \r\n", (int)it6602data->s_Current3DFr.HB0);
			IT_INFO("        Record HDMI vender specific inforframe HB1 = 0x%X \r\n", (int)it6602data->s_Current3DFr.HB1);
			IT_INFO("        Record HDMI vender specific inforframe HB2 = 0x%X \r\n", (int)it6602data->s_Current3DFr.HB2);
			IT_INFO("        Record HDMI vender specific inforframe PB0 = 0x%X \r\n", (int)it6602data->s_Current3DFr.PB0);
			IT_INFO("        Record HDMI vender specific inforframe PB1 = 0x%X \r\n", (int)it6602data->s_Current3DFr.PB1);
			IT_INFO("        Record HDMI vender specific inforframe PB2 = 0x%X \r\n", (int)it6602data->s_Current3DFr.PB2);
			IT_INFO("        Record HDMI vender specific inforframe PB3 = 0x%X \r\n", (int)it6602data->s_Current3DFr.PB3);
			IT_INFO("        Record HDMI vender specific inforframe PB4 = 0x%X \r\n", (int)it6602data->s_Current3DFr.PB4);
			IT_INFO("        Record HDMI vender specific inforframe PB5 = 0x%X \r\n", (int)it6602data->s_Current3DFr.PB5);
			IT_INFO("        Record HDMI vender specific inforframe PB6 = 0x%X \r\n", (int)it6602data->s_Current3DFr.PB6);
			IT_INFO("        Record HDMI vender specific inforframe PB7 = 0x%X \r\n", (int)it6602data->s_Current3DFr.PB7);
			//#endif

			/******************************  3D integration  *************************************/

			it6602data->de3dframe_config.LR_Reference = 2;   // Source of the 3D L/R reference.
			it6602data->de3dframe_config.FrameDominance = 0; // Left or Right Eye is first in L/R image pair.
			it6602data->de3dframe_config.LR_Encoding = 1;	// Type of 3D L/R encoding
			it6602data->de3dframe_config.TB_Reference = 2;   // Top/Bottom reference for vertically sub-sampled sources
			it6602data->de3dframe_config.OE_Reference = 2;   // Odd/Even reference for horizontally sub-sampled sources

			it6602data->de3dframe_config.NumActiveBlankLines =
				0;		// Number of lines separating vertically packed L/R data to be removed (cropped)before being displayed
			it6602data->de3dframe_config.NumberOfEncodedLines =
				0;		// Number of encoded lines in one L/R eye frame of the display data to be blanked out with "Blanking Color".
			it6602data->de3dframe_config.LeftEncodedLineLocation =
				-1;  // Active line number of 1st encoded line in one Left eye frame of the display data (-1=unknown).
			it6602data->de3dframe_config.RightEncodedLineLocation =
				-1; // Active line number of 1st encoded line in one Right eye frame of the display data (-1=unknown).
			it6602data->de3dframe_config.BlankingColor =
				7;				// Color to use when blanking (or masking off) any embedded L/R encoding

			if (((it6602data->s_Current3DFr.PB4 & 0xE0) == HDMI_3DFORMAT_PRESENT)
			    && ((it6602data->s_Current3DFr.PB5 & 0xF0) == FRAME_PACKING)) {
				i = 0;

				while (t_3d_syncgen[i].Vic != 0xFF) {
					if (t_3d_syncgen[i].Vic == it6602data->s_Current3DFr.VIC) {
						break;
					}

					i++;
				}

				v_total = t_3d_syncgen[i].V_total;
				v_act_start = t_3d_syncgen[i].V_act_start;
				v_act_end = t_3d_syncgen[i].V_act_end;
				v_sync_end = t_3d_syncgen[i].V_sync_end;
				v_2d_Vtotal = t_3d_syncgen[i].V_2D_active_total;
				chgbank(1);
				hdmirxset(Reg_PGVTotal_19D, 0xFF, (unsigned char)((v_total & 0xFF0) >> 4));		//pccmd w 9d 2e
				hdmirxset(Reg_PGVTotal_19C, 0xF0, (unsigned char)((v_total & 0x00F) << 4));		//pccmd w 9c d0
				hdmirxset(Reg_PGVActSt_192, 0xFF, (unsigned char)((v_act_start & 0x0FF)));		//pccmd w 92 06
				hdmirxset(Reg_PGVActSt_193, 0x0F, (unsigned char)((v_act_start & 0xF00) >> 8)); //pccmd w 93 83
				hdmirxset(Reg_PGVActEd_193, 0xF0, (unsigned char)((v_act_end & 0x00F) << 4));   //pccmd w 93 83
				hdmirxset(Reg_PGVActEd_194, 0xFF, (unsigned char)((v_act_end & 0xFF0) >> 4));   //pccmd w 94 2E
				hdmirxset(Reg_PGVSyncEd_19F, 0xF0, (unsigned char)((v_sync_end & 0x00F) << 4)); //pccmd w 9f 22

#if 1
				LR_3D_Start = (v_act_start - (v_2d_Vtotal / 2));
				LR_3D_End = (v_act_start + (v_2d_Vtotal / 2));
#else
				LR_3D_Start = ((v_total / 2));
				LR_3D_End = (LR_3D_Start * 3);
#endif

				hdmirxset(Reg_PG3DRSt_18F, 0xFF, (unsigned char)((LR_3D_Start & 0x0FF)));
				hdmirxset(Reg_PG3DRStEd_190, 0x0F, (unsigned char)((LR_3D_Start & 0xF00) >> 8));
				hdmirxset(Reg_PG3DRStEd_190, 0xF0, (unsigned char)((LR_3D_End & 0x00F) << 4));
				hdmirxset(Reg_PG3DREd_191, 0xFF, (unsigned char)((LR_3D_End & 0xFF0) >> 4));

				IT_INFO("\nv_total = %X or %d \r\n", (int)(v_total), (int)(v_total));
				IT_INFO("Reg_PGVTotal_19D = %X \r\n", (int)(hdmirxrd(Reg_PGVTotal_19D)));
				IT_INFO("Reg_PGVTotal_19C = %X \r\n", (int)(hdmirxrd(Reg_PGVTotal_19C)));
				IT_INFO("\nv_act_start = %X or %d \r\n", (int)(v_act_start), (int)(v_act_start));
				IT_INFO("Reg_PGVActSt_192 = %X \r\n", (int)(hdmirxrd(Reg_PGVActSt_192)));
				IT_INFO("Reg_PGVActSt_193 = %X \r\n", (int)(hdmirxrd(Reg_PGVActSt_193)));
				IT_INFO("\nv_act_end = %X or %d \r\n", (int)(v_act_end), (int)(v_act_end));
				IT_INFO("Reg_PGVActEd_193 = %X \r\n", (int)(hdmirxrd(Reg_PGVActEd_193)));
				IT_INFO("Reg_PGVActEd_194 = %X \r\n", (int)(hdmirxrd(Reg_PGVActEd_194)));
				IT_INFO("\nv_sync_end = %X or %d \r\n", (int)(v_sync_end), (int)(v_sync_end));
				IT_INFO("Reg_PGVSyncEd_19F = %X \r\n", (int)(hdmirxrd(Reg_PGVSyncEd_19F)));

				IT_INFO("LR_3D_Start = %X or %d  \r\n", (int)(LR_3D_Start), (int)(LR_3D_Start));
				IT_INFO("Reg_PG3DRSt_18F = %X \r\n", (int)(hdmirxrd(Reg_PG3DRSt_18F)));
				IT_INFO("Reg_PG3DRStEd_190 = %X \r\n", (int)(hdmirxrd(Reg_PG3DRStEd_190)));
				IT_INFO("Reg_PG3DREd_191 = %X \r\n", (int)(hdmirxrd(Reg_PG3DREd_191)));
				IT_INFO("LR_3D_End = %X or %d  \r\n", (int)(LR_3D_End), (int)(LR_3D_End));

				IT_INFO("\n\nv_total = %X or %d \r\n", (int)(v_total), (int)(v_total));
				IT_INFO("v_act_start = %X or %d \r\n", (int)(v_act_start), (int)(v_act_start));
				IT_INFO("v_act_end = %X or %d \r\n", (int)(v_act_end), (int)(v_act_end));
				IT_INFO("v_sync_end = %X or %d \r\n", (int)(v_sync_end), (int)(v_sync_end));
				IT_INFO("LR_3D_Start = %X or %d  \r\n", (int)(LR_3D_Start), (int)(LR_3D_Start));
				IT_INFO("LR_3D_End = %X or %d  \r\n", (int)(LR_3D_End), (int)(LR_3D_End));

				chgbank(0);
				hdmirxset(REG_RX_066_4_DE3DFrame, 0x10, 0x10); // Reg66[4] = 1 for enable 3D FP2FS
				hdmirxset(REG_RX_085_5_En3DROut, 0x20, 0x20);  // Reg85[5] = 1 for enable 3DR output

				Dump3DReg();

				// enable output
				// HActive = ((hdmirxrd(0x9F) & 0x3F) << 8) + hdmirxrd(0x9E);
				//ChangePicoResolution(HActive,v_2d_Vtotal);
				v_act_bspace = v_act_start - v_act_end;
			}

			if (((it6602data->s_Current3DFr.PB4 & 0xE0) == HDMI_3DFORMAT_PRESENT) && (!it6602data->DE3DFormat_HDMIFlag)) {
				it6602data->DE3DFormat_HDMIFlag = TRUE;
			}

			if (((it6602data->s_Current3DFr.PB4 & 0xE0) == HDMI_3DFORMAT_PRESENT) && (it6602data->DE3DFormat_HDMIFlag)) {
				if (((it6602data->s_Current3DFr.PB5 & 0xF0) == FRAME_PACKING) && (!it6602data->FramePacking_Flag)) {
					it6602data->FramePacking_Flag = TRUE;
					it6602data->TopAndBottom_Flag = FALSE;
					it6602data->SideBySide_Flag = FALSE;
					it6602data->oldVIC = 0;
				}

				if (((it6602data->s_Current3DFr.PB5 & 0xF0) == FRAME_PACKING) && (it6602data->FramePacking_Flag)) {
					it6602data->newVIC = it6602data->s_Current3DFr.VIC;

					if (it6602data->newVIC != it6602data->oldVIC) {
						if ((it6602data->s_Current3DFr.VIC == 0x3c) || (it6602data->s_Current3DFr.VIC == 0x3e)
						    || (it6602data->s_Current3DFr.VIC == 0x13) ||
						    (it6602data->s_Current3DFr.VIC == 0x04) || (it6602data->s_Current3DFr.VIC == 0x20)
						    || (it6602data->s_Current3DFr.VIC == 0x22))
							//(it6602data->s_Current3DFr.VIC == 0x05) ||(it6602data->s_Current3DFr.VIC == 0x14) // 1080i@50&60Hz not supported for frame packing
						{
							it6602data->de3dframe_config.NumActiveBlankLines = (unsigned char)v_act_bspace;
							it6602data->de3dframe_config.Format = VERT_PACKED_FULL; // Type of 3D source format is FRAME_PACKING(VERT_PACKED_FULL)

#ifdef DEBUG_MODE_3D
							dbmsg_trace(DBM_3D, "ITEHDMI - HDMI_3DFORMAT is FRAME_PACKING \r\n");
#else
							IT_INFO("ITEHDMI - HDMI_3DFORMAT is FRAME_PACKING \r\n");

#endif

						} else {
							it6602data->de3dframe_config.Format = 6; // Type of 3D source format is UNDEFINED_FORMAT

#ifdef DEBUG_MODE_3D
							dbmsg_trace(DBM_3D, "ITEHDMI - HDMI_3DFORMAT is UNDEFINED_FORMAT \r\n");
#endif
						}

#ifdef DEBUG_MODE_3D
						dbmsg_trace(DBM_3D, "ITEHDMI - HDMI_3DFORMAT is FRAME_PACKING call detect3D_Port_3D_On( ) \r\n");
#endif
						//detect3D_Port_3D_On(&it6602data->de3dframe_config);  //ralph
						//HDMI_DumpDE3DFrameInfo(&it6602data->s_Current3DFr);
						it6602data->oldVIC = it6602data->newVIC;
					}
				}

				if (((it6602data->s_Current3DFr.PB5 & 0xF0) == TOP_AND_BOTTOM) && (!it6602data->TopAndBottom_Flag)) {
					if ((it6602data->s_Current3DFr.VIC == 0x3c) || (it6602data->s_Current3DFr.VIC == 0x3e)
					    || (it6602data->s_Current3DFr.VIC == 0x13) || (it6602data->s_Current3DFr.VIC == 0x04)
					    || (it6602data->s_Current3DFr.VIC == 0x05) ||
					    (it6602data->s_Current3DFr.VIC == 0x14) || (it6602data->s_Current3DFr.VIC == 0x20)
					    || (it6602data->s_Current3DFr.VIC == 0x22) || (it6602data->s_Current3DFr.VIC == 0x1f)
					    || (it6602data->s_Current3DFr.VIC == 0x10)) {
						it6602data->de3dframe_config.Format = VERT_PACKED_HALF; // Type of 3D source format is TOP_AND_BOTTOM(VERT_PACKED_HALF)

#ifdef DEBUG_MODE_3D
						dbmsg_trace(DBM_3D, "ITEHDMI - HDMI_3DFORMAT is TOP_AND_BOTTOM \r\n");
#else
						IT_INFO("ITEHDMI - HDMI_3DFORMAT is TOP_AND_BOTTOM \r\n");
#endif

					} else {
						it6602data->de3dframe_config.Format = 6; // Type of 3D source format is UNDEFINED_FORMAT

#ifdef DEBUG_MODE_3D
						dbmsg_trace(DBM_3D, "ITEHDMI - HDMI_3DFORMAT is UNDEFINED_FORMAT \r\n");
#endif
					}

					//detect3D_Port_3D_On(&it6602data->de3dframe_config);  //ralph
					//HDMI_DumpDE3DFrameInfo(&it6602data->s_Current3DFr);

					it6602data->FramePacking_Flag = FALSE;
					it6602data->TopAndBottom_Flag = TRUE;
					it6602data->SideBySide_Flag = FALSE;
				}

				if (((it6602data->s_Current3DFr.PB5 & 0xF0) == SIDE_BY_SIDE) && (!it6602data->SideBySide_Flag)) {
					if ((it6602data->s_Current3DFr.VIC == 0x3c) || (it6602data->s_Current3DFr.VIC == 0x3e)
					    || (it6602data->s_Current3DFr.VIC == 0x13) || (it6602data->s_Current3DFr.VIC == 0x04)
					    || (it6602data->s_Current3DFr.VIC == 0x05) ||
					    (it6602data->s_Current3DFr.VIC == 0x14) || (it6602data->s_Current3DFr.VIC == 0x20)
					    || (it6602data->s_Current3DFr.VIC == 0x22) || (it6602data->s_Current3DFr.VIC == 0x1f)
					    || (it6602data->s_Current3DFr.VIC == 0x10)) {
						it6602data->de3dframe_config.Format = HORIZ_PACKED_HALF; // Type of 3D source format is SIDE_BY_SIDE(HORIZ_PACKED_HALF)

#ifdef DEBUG_MODE_3D
						dbmsg_trace(DBM_3D, "ITEHDMI - HDMI_3DFORMAT is SIDE_BY_SIDE \r\n");
#else
						IT_INFO("ITEHDMI - HDMI_3DFORMAT is SIDE_BY_SIDE \r\n");
#endif

					} else {
						it6602data->de3dframe_config.Format = 6; // Type of 3D source format is UNDEFINED_FORMAT

#ifdef DEBUG_MODE_3D
						dbmsg_trace(DBM_3D, "ITEHDMI - HDMI_3DFORMAT is UNDEFINED_FORMAT \r\n");
#endif
					}

					//detect3D_Port_3D_On(&it6602data->de3dframe_config);  //ralph
					//HDMI_DumpDE3DFrameInfo(&it6602data->s_Current3DFr);

					it6602data->FramePacking_Flag = FALSE;
					it6602data->TopAndBottom_Flag = FALSE;
					it6602data->SideBySide_Flag = TRUE;
				}

#ifdef DEBUG_MODE_3D
				dbmsg_trace(DBM_3D, "\r\nITEHDMI - HDMI_3D_SourceConfiguration: \r\n");
				dbmsg_ftrace(DBM_3D, "        Format                   = %X \r\n", (int)it6602data->de3dframe_config.Format);
				dbmsg_ftrace(DBM_3D, "        LR_Reference             = %X \r\n", (int)it6602data->de3dframe_config.LR_Reference);
				dbmsg_ftrace(DBM_3D, "        FrameDominance           = %X \r\n", (int)it6602data->de3dframe_config.FrameDominance);
				dbmsg_ftrace(DBM_3D, "        LR_Encoding              = %X \r\n", (int)it6602data->de3dframe_config.LR_Encoding);
				dbmsg_ftrace(DBM_3D, "        TB_Reference             = %X \r\n", (int)it6602data->de3dframe_config.TB_Reference);
				dbmsg_ftrace(DBM_3D, "        OE_Reference             = %X \r\n", (int)it6602data->de3dframe_config.OE_Reference);
				dbmsg_ftrace(DBM_3D, "        NumActiveBlankLines      = %X \r\n",
					     (int)it6602data->de3dframe_config.NumActiveBlankLines);
				dbmsg_ftrace(DBM_3D, "        NumberOfEncodedLines     = %X \r\n",
					     (int)it6602data->de3dframe_config.NumberOfEncodedLines);
				dbmsg_ftrace(DBM_3D, "        LeftEncodedLineLocation  = %X \r\n",
					     (int)it6602data->de3dframe_config.LeftEncodedLineLocation);
				dbmsg_ftrace(DBM_3D, "        RightEncodedLineLocation = %X \r\n",
					     (int)it6602data->de3dframe_config.RightEncodedLineLocation);
				dbmsg_ftrace(DBM_3D, "        BlankingColor            = %X \r\n", (int)it6602data->de3dframe_config.BlankingColor);

#else
				IT_INFO("\r\nITEHDMI - HDMI_3D_SourceConfiguration: \r\n");
				IT_INFO("        Format                   = %X \r\n", (int)it6602data->de3dframe_config.Format);
				IT_INFO("        LR_Reference             = %X \r\n", (int)it6602data->de3dframe_config.LR_Reference);
				IT_INFO("        FrameDominance           = %X \r\n", (int)it6602data->de3dframe_config.FrameDominance);
				IT_INFO("        LR_Encoding              = %X \r\n", (int)it6602data->de3dframe_config.LR_Encoding);
				IT_INFO("        TB_Reference             = %X \r\n", (int)it6602data->de3dframe_config.TB_Reference);
				IT_INFO("        OE_Reference             = %X \r\n", (int)it6602data->de3dframe_config.OE_Reference);
				IT_INFO("        NumActiveBlankLines      = %X \r\n", (int)it6602data->de3dframe_config.NumActiveBlankLines);
				IT_INFO("        NumberOfEncodedLines     = %X \r\n", (int)it6602data->de3dframe_config.NumberOfEncodedLines);
				IT_INFO("        LeftEncodedLineLocation  = %X \r\n", (int)it6602data->de3dframe_config.LeftEncodedLineLocation);
				IT_INFO("        RightEncodedLineLocation = %X \r\n", (int)it6602data->de3dframe_config.RightEncodedLineLocation);
				IT_INFO("        BlankingColor            = %X \r\n", (int)it6602data->de3dframe_config.BlankingColor);
#endif

				return TRUE;
			}
		}

		if (it6602data->DE3DFormat_HDMIFlag) { // 3D InfoFrame Packet Type is not valid
#ifdef DEBUG_MODE_3D
			dbmsg_trace(DBM_3D, "ITEHDMI - HDMI_3DFORMAT is OFF \r\n");
#endif

			//ralph
			//detect3D_Port_3D_Off();
			//mbSend( detect3DMbxID, D3DMSG_STATE_PORT_2D, -1, 0, FALSE, 0);
			//dbmsg_ftrace( DBM_3D, "detect3D_Port_3D_Off: Current state=%s\r\n", detect3DStateStringTable[detect3DState]);

			it6602data->DE3DFormat_HDMIFlag = FALSE;
			it6602data->FramePacking_Flag = FALSE;
			it6602data->TopAndBottom_Flag = FALSE;
			it6602data->SideBySide_Flag = FALSE;
		}

		/******************************  3D integration  *************************************/

	} else {

		//it6602data->f_de3dframe_hdmi = FALSE;
		hdmirxwr(REG_RX_06A, 0x82);
		hdmirxset(REG_RX_066_4_DE3DFrame, 0x10, 0x00); // Reg66[4] = 0 for disable 3D FP2FS
		hdmirxset(REG_RX_085_5_En3DROut, 0x20, 0x00);  // Reg85[5] = 0 for disable 3DR output
	}

	return FALSE;
}
#endif

void IT66021::it6602HPDCtrl(unsigned char ucport, unsigned char ucEnable)
{
	if (ucport != 0) {
		PX4_ERR("it66021 only support ucport in it6602HPDCtrl \n");
		return;
	}

	if (ucEnable == 0) {
		// Disable HDMI DDC Bus to access ITEHDMI EDID RAM
		//hdmirxset(REG_RX_0C0, 0x01, 0x01);                            // HDMI RegC0[1:0]=11 for disable HDMI DDC bus to access EDID RAM
		IT_INFO("[it6602HPDCtrl]: Port 0 HPD HDMI\r\n");
		chgbank(1);
		hdmirxset(REG_RX_1B0, 0x03, 0x01); //clear port 0 HPD=1 for EDID update
		chgbank(0);

	} else {
		if ((hdmirxrd(REG_RX_P0_SYS_STATUS) & B_P0_PWR5V_DET)) {
			// Enable HDMI DDC bus to access ITEHDMI EDID RAM
			//hdmirxset(REG_RX_0C0, 0x01, 0x00);                        // HDMI RegC0[1:0]=00 for enable HDMI DDC bus to access EDID RAM
			IT_INFO("[it6602HPDCtrl] Port 0 HPD HDMI 11111 \r\n");
			chgbank(1);
			hdmirxset(REG_RX_1B0, 0x03, 0x03); //set port 0 HPD=1
			chgbank(0);
		}
	}
}

char IT66021::it66021_init(void)
{
	char ret = false;

	it6602HPDCtrl(0, 0); // HDMI port , set HPD = 0

	usleep(200 * 1000);

	ret = IT6602_fsm_init();

	it6602HPDCtrl(0, 1);

	if (ret == TRUE) {
		hdmirxset(REG_RX_063, 0xFF, 0x3F);
		hdmirxset(REG_RX_012, 0xFF, 0xF8);

		IT_INFO("REG_RX_012=%2x", hdmirxrd(REG_RX_012));
		IT_INFO("REG_RX_063=%2x", hdmirxrd(REG_RX_063));
	}


	uint32_t u32_HTotal   = ((hdmirxrd(0x9D) & 0x3F) << 8) + hdmirxrd(0x9C);
	uint32_t u32_HActive  = ((hdmirxrd(0x9F) & 0x3F) << 8) + hdmirxrd(0x9E);

	uint32_t u32_VTotal   = ((hdmirxrd(0xA4) & 0x0F) << 8) + hdmirxrd(0xA3);
	uint32_t u32_VActive  = ((hdmirxrd(0xA4) & 0xF0) << 4) + hdmirxrd(0xA5);

	PX4_INFO("init u32_HTotal = %d \n", u32_HTotal);
	PX4_INFO("init u32_HActive = %d \n", u32_HActive);
	PX4_INFO("init u32_VTotal = %d \n", u32_VTotal);
	PX4_INFO("init u32_VActive = %d \n", u32_VActive);

	return ret;
}

//FIX_ID_037 xxxxx //Allion MHL compliance issue !!!
//xxxxx 2014-0529 //HDCP Content On/Off
void IT66021::IT6602_ManualVideoTristate(unsigned char bOff)
{
	if (bOff) {
		hdmirxset(REG_RX_053, (B_VDGatting | B_VIOSel),
			  (B_VDGatting | B_VIOSel)); //Reg53[7][5] = 11    // for enable B_VDIO_GATTING and VIO_SEL
		hdmirxset(REG_RX_052, (B_DisVAutoMute), (B_DisVAutoMute));				   //Reg52[5] = 1 for disable Auto video MUTE
		hdmirxset(REG_RX_053, (B_TriVDIO), (0x00));								   //Reg53[2:0] = 000;         // 0 for enable video io data output
		IT_INFO("+++++++++++ Manual Video / Audio off  +++++++++++++++++\n");

	} else {
		hdmirxset(REG_RX_053, (B_TriSYNC), (0x00));								   //Reg53[0] = 0;                 // for enable video sync
		hdmirxset(REG_RX_053, (B_TriVDIO), (0x00));								   //Reg53[3:1] = 000;         // 0 for enable video io data output
		hdmirxset(REG_RX_053, (B_TriVDIO), (
				  B_TriVDIO));						   //Reg53[2:0] = 111;         // 1 for enable tri-state of video io data
		hdmirxset(REG_RX_053, (B_TriVDIO), (0x00));								   //Reg53[2:0] = 000;         // 0 for enable video io data output
		hdmirxset(REG_RX_053, (B_VDGatting | B_VIOSel),
			  (B_VDGatting | B_VIOSel)); //Reg53[7][5] = 11    // for enable B_VDIO_GATTING and VIO_SEL
		hdmirxset(REG_RX_053, (B_VDGatting | B_VIOSel), (B_VIOSel));			   //Reg53[7][5] = 01    // for disable B_VDIO_GATTING
		IT_INFO("+++++++++++ Manual Video on  +++++++++++++++++\n");
	}
}

//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_EQ_ADJUST_

void IT66021::HDMIStartEQDetect(struct it6602_eq_data *ucEQPort)
{
	unsigned char ucPortSel;

	if (ucEQPort->ucPortID == F_PORT_SEL_0) {
		// for MHL mode , there are no need to adjust EQ for long cable.
		if (hdmirxrd(REG_RX_P0_SYS_STATUS) & (B_P0_MHL_MODE)) {
			return;
		}
	}

	if (ucEQPort->ucEQState == 0xFF) {
		ucPortSel = hdmirxrd(REG_RX_051) & B_PORT_SEL;

		if (ucPortSel == ucEQPort->ucPortID) {
			HDMISwitchEQstate(ucEQPort, 0); // for SCDT off state

		} else {
			HDMISwitchEQstate(ucEQPort, EQSTATE_WAIT + 1); //for SCDT on state
		}

		ucEQPort->f_manualEQadjust = TRUE;
		HDMIAdjustEQ(ucEQPort);

		ucEQPort->ErrorCount[0] = MAXECCWAIT;
		ucEQPort->ErrorCount[1] = MAXECCWAIT;
		ucEQPort->ErrorCount[2] = MAXECCWAIT;
	}
}

void IT66021::HDMISetEQValue(struct it6602_eq_data *ucEQPort, unsigned char ucIndex)
{
	if (ucIndex < MaxEQIndex) {
		if (ucEQPort->ucPortID == F_PORT_SEL_0) {
#ifdef _SUPPORT_AUTO_EQ_
			ucEQMode[F_PORT_SEL_0] = 1; // 1 for Manual Mode
#endif
			hdmirxset(REG_RX_026, 0x20, 0x20); //07-04 add for adjust EQ
			hdmirxwr(REG_RX_027, IT6602EQTable[ucIndex]);
			IT_INFO("Port=%X ,ucIndex = %X ,HDMISetEQValue Reg027 = %X \r\n", (int)ucEQPort->ucPortID, (int)ucIndex,
				(int)hdmirxrd(REG_RX_027));

		} else {
#ifdef _SUPPORT_AUTO_EQ_
			ucEQMode[F_PORT_SEL_1] = 1; // 1 for Manual Mode
#endif
			hdmirxset(REG_RX_03E, 0x20, 0x20); //07-04 add for adjust EQ
			hdmirxwr(REG_RX_03F, IT6602EQTable[ucIndex]);
			IT_INFO("Port=%X ,ucIndex = %X ,HDMISetEQValue Reg03F = %X \r\n", (int)ucEQPort->ucPortID, (int)ucIndex,
				(int)hdmirxrd(REG_RX_03F));
		}
	}
}

void IT66021::HDMISwitchEQstate(struct it6602_eq_data *ucEQPort, unsigned char state)
{
	ucEQPort->ucEQState = state;
	IT_INFO("!!! Port=%X ,HDMISwitchEQstate %X \r\n", (int)ucEQPort->ucPortID, (int)ucEQPort->ucEQState);

	switch (ucEQPort->ucEQState) {
	case EQSTATE_START:
		HDMISetEQValue(ucEQPort, 0);
		break;

	case EQSTATE_LOW:
		HDMISetEQValue(ucEQPort, 1);
		break;

	case EQSTATE_MIDDLE:
		HDMISetEQValue(ucEQPort, 2);
		break;

	case EQSTATE_HIGH:
		HDMISetEQValue(ucEQPort, 3);
		break;

	default:
		IT6602_HDCP_ContentOff(ucEQPort->ucPortID, 0);

		HDMISetEQValue(ucEQPort, 0xff); //dont care
		break;
	}

	ucEQPort->ucPkt_Err = 0;
	ucEQPort->ucECCvalue = 0;
	ucEQPort->ucECCfailCount = 0;
}

void IT66021::HDMICheckSCDTon(struct it6602_eq_data *ucEQPort)
{
	unsigned char ucResult = 0;
	unsigned char Receive_Err;
	unsigned char ucStatus;
	unsigned char ucCurrentPort;
	unsigned char ucHDCP;

	ucCurrentPort = hdmirxrd(REG_RX_051) & B_PORT_SEL;

	if (ucEQPort->ucPortID != ucCurrentPort) {
		return;
	}

	if (ucEQPort->ucPortID == F_PORT_SEL_1) {
		ucStatus = hdmirxrd(REG_RX_P1_SYS_STATUS);
		// !!! check ECC error register  !!!
		Receive_Err = hdmirxrd(REG_RX_0B7);
		hdmirxwr(REG_RX_0B7, Receive_Err);

		ucHDCP = hdmirxrd(REG_RX_095);

	} else {
		ucStatus = hdmirxrd(REG_RX_P0_SYS_STATUS);
		// !!! check ECC error register  !!!
		Receive_Err = hdmirxrd(REG_RX_0B2);
		hdmirxwr(REG_RX_0B2, Receive_Err);

		ucHDCP = hdmirxrd(REG_RX_093);
	}

	if ((ucStatus & (B_P0_SCDT | B_P0_PWR5V_DET | B_P0_RXCK_VALID)) == (B_P0_PWR5V_DET | B_P0_RXCK_VALID)) {
		ucEQPort->ucECCfailCount++;
	}

	IT_INFO("Port=%d, CheckSCDTon=%d, Receive_Err=%X, ucECCfailCount=%X, SCDT=%X, HDCP=%X \r\n",
		(int)ucEQPort->ucPortID, (int)ucEQPort->ucEQState, (int)Receive_Err, (int)ucEQPort->ucECCfailCount, (int)ucStatus,
		(int)ucHDCP);

	if ((Receive_Err & 0xC0) != 0x00) {
		ucEQPort->ucECCvalue++;

		IT6602_HDCP_ContentOff(ucEQPort->ucPortID, 1);

		if (ucEQPort->ucECCvalue > ((MINECCFAILCOUNT / 2))) {
			ucEQPort->ucECCvalue = 0;
			IT_INFO("HDMICheckSCDTon() for ECC / Deskew issue !!!");

			if (ucEQPort->ucPortID == F_PORT_SEL_1) {
				if (hdmirxrd(REG_RX_038) == 0x00) {
					hdmirxwr(REG_RX_038, 0x3F); // Dr. Liu suggestion to 0x00
					IT_INFO("Port 1 Reg38=%X !!!\n", (int)hdmirxrd(REG_RX_038));
				}

			} else {
				if (hdmirxrd(REG_RX_020) == 0x00) {
					hdmirxwr(REG_RX_020, 0x3F); // Dr. Liu suggestion to 0x00
					IT_INFO("Port 0 Reg20=%X !!!\n", (int)hdmirxrd(REG_RX_020));
				}
			}
		}
	}

	if (ucEQPort->ucEQState == EQSTATE_WAIT - 1) {
		IT_INFO("Port=%d, CheckSCDTon=%d, Receive_Err=%X, ucECCfailCount=%X, SCDT=%X, HDCP=%X \r\n",
			(int)ucEQPort->ucPortID, (int)ucEQPort->ucEQState, (int)Receive_Err, (int)ucEQPort->ucECCfailCount, (int)ucStatus,
			(int)ucHDCP);

		if ((Receive_Err & 0xC0) == 0xC0) {
			IT_INFO("HDMICheckSCDTon() CDR reset for Port %d ECC_TIMEOUT !!!\n", ucCurrentPort);
			hdmirx_ECCTimingOut(ucCurrentPort);

			HDMISwitchEQstate(ucEQPort, EQSTATE_END);
			return;
		}

#ifdef _SUPPORT_AUTO_EQ_

		if ((ucEQPort->ucECCfailCount) == 0) {
			if (ucEQPort->ucPortID == F_PORT_SEL_1) {
				if (ucEQMode[F_PORT_SEL_1] == 0) { // verfiy Auto EQ Value wehn auto EQ finish

					if (((ucChannelB[F_PORT_SEL_1] & 0x7F) < 0x0F) ||
					    ((ucChannelG[F_PORT_SEL_1] & 0x7F) < 0x0F) ||
					    ((ucChannelR[F_PORT_SEL_1] & 0x7F) < 0x0F)) {
						ucResult = 1; // 1 for EQ start
					}
				}

			} else {
				if (ucEQMode[F_PORT_SEL_0] == 0) { // verfiy Auto EQ Value when auto EQ finish
					if (hdmirxrd(REG_RX_P0_SYS_STATUS) & (B_P0_MHL_MODE)) {
						if ((ucChannelB[F_PORT_SEL_0] & 0x7F) < 0x0F) {
							ucResult = 1; // 1 for EQ start
						}

					} else {
						if (((ucChannelB[F_PORT_SEL_0] & 0x7F) < 0x0F) ||
						    ((ucChannelG[F_PORT_SEL_0] & 0x7F) < 0x0F) ||
						    ((ucChannelR[F_PORT_SEL_0] & 0x7F) < 0x0F))

						{
							ucResult = 1; // 1 for EQ start
						}
					}
				}
			}

			if (ucResult == 0) { // no need to do manual EQ adjust when SCDT always On !!!
				HDMISwitchEQstate(ucEQPort, EQSTATE_END);
				return;
			}
		}

#endif
		HDMISwitchEQstate(ucEQPort, EQSTATE_WAIT);
	}
}

void IT66021::HDMIPollingErrorCount(struct it6602_eq_data *ucEQPort)
{
	unsigned char Receive_Err;
	unsigned char Video_Err;
	unsigned char Code_Err;
	unsigned char Pkt_Err;
	unsigned char CrtErr;
	unsigned char ucHDCP;
	unsigned char ucStatus;

	unsigned char ucCurrentPort;
	ucCurrentPort = hdmirxrd(REG_RX_051) & B_PORT_SEL;

	if (ucEQPort->ucPortID == F_PORT_SEL_1) {
		ucStatus = hdmirxrd(REG_RX_P1_SYS_STATUS);
		// !!! check ECC error register  !!!
		Receive_Err = hdmirxrd(REG_RX_0B7);
		Video_Err = hdmirxrd(REG_RX_0B8) & 0xE0;
		Code_Err = hdmirxrd(REG_RX_0B9);
		Pkt_Err = hdmirxrd(REG_RX_0BA);
		CrtErr = hdmirxrd(REG_RX_0BB);

		hdmirxwr(REG_RX_0B7, Receive_Err);
		hdmirxwr(REG_RX_0B8, Video_Err);
		hdmirxwr(REG_RX_0B9, Code_Err);
		hdmirxwr(REG_RX_0BA, Pkt_Err);
		hdmirxwr(REG_RX_0BB, CrtErr);

		ucHDCP = hdmirxrd(REG_RX_095);

	} else {
		ucStatus = hdmirxrd(REG_RX_P0_SYS_STATUS);
		// !!! check ECC error register  !!!
		Receive_Err = hdmirxrd(REG_RX_0B2);
		Video_Err = hdmirxrd(REG_RX_0B3) & 0xE0;
		Code_Err = hdmirxrd(REG_RX_0B4);
		Pkt_Err = hdmirxrd(REG_RX_0B5);
		CrtErr = hdmirxrd(REG_RX_0B6);

		hdmirxwr(REG_RX_0B2, Receive_Err);
		hdmirxwr(REG_RX_0B3, Video_Err);
		hdmirxwr(REG_RX_0B4, Code_Err);
		hdmirxwr(REG_RX_0B5, Pkt_Err);
		hdmirxwr(REG_RX_0B6, CrtErr);

		ucHDCP = hdmirxrd(REG_RX_093);
	}

	if (ucCurrentPort == ucEQPort->ucPortID) {
		if ((ucStatus & B_P0_SCDT) == 0x00) {
			Receive_Err = 0xFF;
			ucEQPort->ucECCfailCount |= 0x80;
		}
	}

	IT_INFO("Port=%d ,EQState2No=%d, Receive_Err=%X, HDCP=%X \r\n",
		(int)ucEQPort->ucPortID, (int)ucEQPort->ucEQState, (int)Receive_Err, (int)ucHDCP);

#if 1

	if (Pkt_Err == 0xFF || Code_Err == 0xFF) {
		ucEQPort->ucPkt_Err++; // judge whether CDR reset

	} else {
		ucEQPort->ucPkt_Err = 0;
	}

	if (ucEQPort->ucPkt_Err > (MINECCFAILCOUNT - 2)) {

		if (ucEQPort->ucEQState > EQSTATE_START) {

			IT_INFO("1111111111111111111111111111111111111111111111111111111111111111111111111\r\n");

			if (ucEQPort->ucPortID == F_PORT_SEL_1) {
				Code_Err = hdmirxrd(REG_RX_0B9);
				hdmirxwr(REG_RX_0B9, Code_Err);

				if (Code_Err == 0xFF) {
					if (hdmirxrd(REG_RX_038) == 0x00) {
						hdmirxwr(REG_RX_038, 0x3F);        // Dr. Liu suggestion to 0x00

					} else {
						hdmirxwr(REG_RX_038, 0x00);        // Dr. Liu suggestion to 0x3F
					}

					IT_INFO("Port 1 Reg38=%X !!!\n", (int)hdmirxrd(REG_RX_038));
				}

			} else {
				Code_Err = hdmirxrd(REG_RX_0B4);
				hdmirxwr(REG_RX_0B4, Code_Err);

				if (Code_Err == 0xFF) {
					if (hdmirxrd(REG_RX_020) == 0x00) {
						hdmirxwr(REG_RX_020, 0x3F);        // Dr. Liu suggestion to 0x00

					} else {
						hdmirxwr(REG_RX_020, 0x00);        // Dr. Liu suggestion to 0x3F
					}

					IT_INFO("Port 0 Reg20=%X !!!\n", (int)hdmirxrd(REG_RX_020));
				}
			}

			IT_INFO("1111111111111111111111111111111111111111111111111111111111111111111111111\r\n");

			if (ucEQPort->ucPortID == F_PORT_SEL_0) {

				hdmirxset(REG_RX_011, (B_P0_DCLKRST | B_P0_CDRRST), (B_P0_DCLKRST | B_P0_CDRRST /*|B_P0_SWRST*/));
				hdmirxset(REG_RX_011, (B_P0_DCLKRST | B_P0_CDRRST), 0x00);
				IT_INFO(" HDMIPollingErrorCount( ) Port 0 CDR reset !!!!!!!!!!!!!!!!!! \r\n");

			} else {
				hdmirxset(REG_RX_018, (B_P1_DCLKRST | B_P1_CDRRST), (B_P1_DCLKRST | B_P1_CDRRST /*|B_P1_SWRST*/));
				hdmirxset(REG_RX_018, (B_P1_DCLKRST | B_P1_CDRRST), 0x00);
				IT_INFO(" HDMIPollingErrorCount( ) Port 1 CDR reset !!!!!!!!!!!!!!!!!! \r\n");
			}
		}

		ucEQPort->ucPkt_Err = 0;
		ucEQPort->ucECCfailCount |= 0x40;
		ucEQPort->ucECCfailCount &= 0xF0;
	}

#endif

	if (Receive_Err != 0) {
		IT_INFO("Video_Err = %X \r\n", (int)Video_Err);
		IT_INFO("Code_Err = %X \r\n", (int)Code_Err);
		IT_INFO("Pkt_Err = %X \r\n", (int)Pkt_Err);
		IT_INFO("CrtErr = %X \r\n", (int)CrtErr);

		ucEQPort->ucECCvalue++;
		ucEQPort->ucECCfailCount++;

	} else {
		ucEQPort->ucECCfailCount = 0;
	}

#if 1

	if ((ucEQPort->ucECCfailCount & 0x7F) < (0x40)) { // before CDR reset , dont care pkt_error and code_error

		if (Pkt_Err == 0xFF || Code_Err == 0xFF) {
			return;
		}
	}

#endif

	if ((ucEQPort->ucECCfailCount & 0x0F) > (MINECCFAILCOUNT - 2)) {

		ucEQPort->ucECCvalue = MAXECCWAIT;

		ucCurrentPort = hdmirxrd(REG_RX_051) & B_PORT_SEL;

		if (ucEQPort->ucPortID == F_PORT_SEL_1) {
			ucStatus = hdmirxrd(REG_RX_P1_SYS_STATUS);

		} else {
			ucStatus = hdmirxrd(REG_RX_P0_SYS_STATUS);
		}

		if (ucCurrentPort == ucEQPort->ucPortID) {
			if (((ucStatus & B_P0_SCDT) == 0x00) || ((ucEQPort->ucECCfailCount & 0x80) != 0x00)) {
				ucEQPort->ucECCvalue = MAXECCWAIT | 0x80; // 0x80 for Identify SCDT off with Ecc error
			}
		}

		StoreEccCount(ucEQPort); // abnormal judge ucECCvalue mode

		if (ucEQPort->ucEQState < EQSTATE_START) {
			HDMISwitchEQstate(ucEQPort, EQSTATE_START);

		} else if (ucEQPort->ucEQState < EQSTATE_LOW) {
			HDMISwitchEQstate(ucEQPort, EQSTATE_LOW);

		} else if (ucEQPort->ucEQState < EQSTATE_MIDDLE) {
			HDMISwitchEQstate(ucEQPort, EQSTATE_MIDDLE);

		} else if (ucEQPort->ucEQState <= EQSTATE_HIGH) {
			HDMISwitchEQstate(ucEQPort, EQSTATE_HIGH);
		}
	}
}

void IT66021::HDMIJudgeECCvalue(struct it6602_eq_data *ucEQPort)
{
	IT_INFO("!!! HDMI Judge ECCvalue( ) %X!!! \r\n", (int)ucEQPort->ucECCvalue);
	StoreEccCount(ucEQPort); // normal judge ucECCvalue mode

	if ((ucEQPort->ucECCvalue) > (MAXECCWAIT / 2)) {
		if (ucEQPort->ucEQState == EQSTATE_START) {
			HDMISwitchEQstate(ucEQPort, EQSTATE_START);

		} else if (ucEQPort->ucEQState == EQSTATE_LOW) {
			HDMISwitchEQstate(ucEQPort, EQSTATE_LOW);

		} else if (ucEQPort->ucEQState == EQSTATE_MIDDLE) {
			HDMISwitchEQstate(ucEQPort, EQSTATE_MIDDLE);

		} else if (ucEQPort->ucEQState == EQSTATE_HIGH) {
			HDMISwitchEQstate(ucEQPort, EQSTATE_HIGH);
		}

	} else {
		HDMISwitchEQstate(ucEQPort, EQSTATE_END); // quit EQadjust( )
	}

	ucEQPort->ucPkt_Err = 0;
	ucEQPort->ucECCvalue = 0;
	ucEQPort->ucECCfailCount = 0;
}

void IT66021::HDMIAdjustEQ(struct it6602_eq_data *ucEQPort)
{
	unsigned char ucCurrentPort;
	ucCurrentPort = hdmirxrd(REG_RX_051) & B_PORT_SEL;

	switch (ucEQPort->ucEQState) {
	case EQSTATE_WAIT:
		break;

	case EQSTATE_START:
	case EQSTATE_LOW:
	case EQSTATE_MIDDLE:
		HDMIJudgeECCvalue(ucEQPort);
		break;

	case EQSTATE_HIGH:
		HDMIJudgeECCvalue(ucEQPort);
		ucEQPort->ucEQState = EQSTATE_END;
		break;

	case EQSTATE_HIGH + 1:
	case EQSTATE_END + 1:

		ucEQPort->f_manualEQadjust = FALSE;
		ucEQPort->ucEQState = 0xFF;

		if (ucEQPort->ucPortID == ucCurrentPort) {
			IT6602VideoCountClr();
		}

		break;

	case 0xff:
		IT_INFO("====================== f_manualEQadjust = FALSE ====================== \r\n");

	default:
		break;
	}

	if (ucEQPort->ucEQState != 0xFF) {
		if (ucEQPort->ucEQState < EQSTATE_WAIT) { //20120410
			HDMICheckSCDTon(ucEQPort);

		} else if (ucEQPort->ucEQState < EQSTATE_HIGH) {
			HDMIPollingErrorCount(ucEQPort);
		}

		//		else
		//			HDMICheckErrorCount(ucEQPort);
		ucEQPort->ucEQState++;

	} else {
		ucEQPort->f_manualEQadjust = FALSE;
	}
}

//FIX_ID_010 xxxxx 	//Add JudgeBestEQ to avoid wrong EQ setting
void IT66021::StoreEccCount(struct it6602_eq_data *ucEQPort)
{
	IT_INFO("StoreEccCount() ucEQPort->ucECCvalue = %02X \r\n", (int)ucEQPort->ucECCvalue);

	if (ucEQPort->ucEQState <= EQSTATE_LOW) {
		ucEQPort->ErrorCount[0] = ucEQPort->ucECCvalue;

	} else if (ucEQPort->ucEQState <= EQSTATE_MIDDLE) {
		ucEQPort->ErrorCount[1] = ucEQPort->ucECCvalue;

	} else if (ucEQPort->ucEQState <= EQSTATE_HIGH) {
		ucEQPort->ErrorCount[2] = ucEQPort->ucECCvalue;
		JudgeBestEQ(ucEQPort);
	}
}

void IT66021::JudgeBestEQ(struct it6602_eq_data *ucEQPort)
{
	unsigned char i, j, Result;

	j = 0;
	Result = ucEQPort->ErrorCount[0];

	for (i = 1; i < MaxEQIndex; i++) {
		if (Result >= ucEQPort->ErrorCount[i]) {
			Result = ucEQPort->ErrorCount[i];
			j = i;
		}
	}

	IT_INFO(" Best IT6602EQTable ErrorCount[%X]=%X !!! IT6602EQTable Value=%X !!!\n", (int)j, (int)Result,
		(int)IT6602EQTable[j]);

	if (ucEQPort->ucPortID == F_PORT_SEL_0) {
#ifdef _SUPPORT_AUTO_EQ_

		if ((hdmirxrd(REG_RX_027) & 0x80) == 0) {
			OverWriteAmpValue2EQ(ucEQPort->ucPortID);

		} else
#endif
		{
			hdmirxset(REG_RX_026, 0x20, 0x20); //07-04 add for adjust EQ
			hdmirxwr(REG_RX_027, IT6602EQTable[j]);
			IT_INFO("Port=%X ,ucIndex = %X ,JudgeBestEQ Reg027 = %X \r\n", (int)ucEQPort->ucPortID, (int)j,
				(int)hdmirxrd(REG_RX_027));
		}

	} else {
#ifdef _SUPPORT_AUTO_EQ_

		if ((hdmirxrd(REG_RX_03F) & 0x80) == 0) {
			OverWriteAmpValue2EQ(ucEQPort->ucPortID);

		} else
#endif
		{
			hdmirxset(REG_RX_03E, 0x20, 0x20); //07-04 add for adjust EQ
			hdmirxwr(REG_RX_03F, IT6602EQTable[j]);
			IT_INFO("Port=%X ,ucIndex = %X ,JudgeBestEQ Reg03F = %X \r\n", (int)ucEQPort->ucPortID, (int)j,
				(int)hdmirxrd(REG_RX_03F));
		}
	}
}

void IT66021::IT6602VideoCountClr(void)
{
	struct it6602_dev_data *it6602data = get_it6602_dev_data();
	it6602data->m_VideoCountingTimer = 1;
}
#endif

//---------------------------------------------------------------------------------------------------
//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_AUTO_EQ_
void IT66021::DisableOverWriteRS(unsigned char ucPortSel)
{
#ifdef _ENABLE_IT68XX_MHL_FUNCTION_
	unsigned char uc;
#endif

	struct it6602_dev_data *it6602data = get_it6602_dev_data();
#ifdef _SUPPORT_AUTO_EQ_
	ucPortAMPOverWrite[F_PORT_SEL_0] = 0; //2013-0801
	ucPortAMPValid[F_PORT_SEL_0] = 0;

	ucEQMode[F_PORT_SEL_0] = 0;		   // 0 for Auto Mode
	hdmirxset(REG_RX_022, 0xFF, 0x00); // 07-16 Reg22=0x30	power down auto EQ
	hdmirxset(REG_RX_026, 0x20, 0x00); //Manually set RS Value
	IT_INFO(" ############# DisableOverWriteRS( ) port 0 ###############\n");
#endif

#ifdef _SUPPORT_EQ_ADJUST_
	it6602data->EQPort[0].f_manualEQadjust = FALSE;
	it6602data->EQPort[F_PORT_SEL_0].ucEQState = 0xFF;
#endif
	it6602data->m_ucDeskew_P0 = 0;
	it6602data->m_ucEccCount_P0 = 0;

	it6602data->HDMIIntEvent &= 0xF0;
	;
	it6602data->HDMIWaitNo[F_PORT_SEL_0] = 0;

#ifdef _ENABLE_IT68XX_MHL_FUNCTION_
	wakeupcnt = 0; //07-23
	it6602data->CBusIntEvent = 0;
	it6602data->CBusSeqNo = 0;
	it6602data->CBusWaitNo = 0x00;

	chgbank(1);
	hdmirxset(REG_RX_1B8, 0x80, 0x00); // [7] Reg_HWENHYS = 0
	hdmirxset(REG_RX_1B6, 0x07,
		  0x03); // [2:0]Reg_P0_ENHYS = 03  [2:0]Reg_P0_ENHYS = 03 for default enable filter to gating output
	chgbank(0);
	uc = mhlrxrd(0x05);
	mhlrxwr(0x05, uc);

#if 1
	it6602data->m_bRCPTimeOut = FALSE;
	it6602data->m_bRCPError = FALSE;
#endif

	it6602data->m_DiscoveryDone = 0;

	mhlrxset(MHL_RX_2B, 0x04, 0x00); // MHL2B[2] 0 for disable HW wake up fail machanism
	m_MHLabortID = 0;				 // test MSC Abort command only !!!
	chgbank(1);
	hdmirxset(REG_RX_1C0, 0x8C, 0x04); //FIX_ID_037  2014-0527 +10% for W1070 only
	IT_INFO("Reset 1k pull down to +10 percent for W1070 only \r\n");
	chgbank(0);

	it6602data->m_RAP_ContentOff = 0;
	it6602data->m_HDCP_ContentOff = 0;
#endif
}

void IT66021::AmpValidCheck(unsigned char ucPortSel)
{
	struct it6602_dev_data *it6602data = get_it6602_dev_data();

#ifdef _SUPPORT_AUTO_EQ_
	unsigned char uc;

	if (ucPortSel == F_PORT_SEL_1) {
		chgbank(1);
		uc = hdmirxrd(REG_RX_1D8);
		IT_INFO(" ############# AmpValidCheck( ) port 1 ###############\n");
		IT_INFO(" ############# Reg1D8 = %X ###############\n", (int)uc);
		IT_INFO(" ############# Reg1DC = %X ###############\n", (int)hdmirxrd(REG_RX_1DC));

		if ((uc & 0x03) == 0x03) {
			ucChannelB[1] = hdmirxrd(REG_RX_1DD);
			ucPortAMPValid[1] |= 0x03;
			IT_INFO(" ############# B AMP VALID port 1 Reg1DD = 0x%X  ###############\n", (int)ucChannelB[1]);
		}

		if ((uc & 0x0C) == 0x0C) {
			ucChannelG[1] = hdmirxrd(REG_RX_1DE);
			ucPortAMPValid[1] |= 0x0C;
			IT_INFO(" ############# G AMP VALID port 1 Reg1DD = 0x%X  ###############\n", (int)ucChannelG[1]);
		}

		if ((uc & 0x30) == 0x30) {
			ucChannelR[1] = hdmirxrd(REG_RX_1DF);
			ucPortAMPValid[1] |= 0x30;
			IT_INFO(" ############# R AMP VALID port 1 Reg1DD = 0x%X  ###############\n", (int)ucChannelR[1]);
		}

		chgbank(0);

		if ((ucPortAMPValid[1] & 0x3F) == 0x3F) {
			OverWriteAmpValue2EQ(F_PORT_SEL_1);
			//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_EQ_ADJUST_
			HDMIStartEQDetect(&(it6602data->EQPort[F_PORT_SEL_1]));
#endif
		}

	} else {
		chgbank(1);
		uc = hdmirxrd(REG_RX_1D0);
		IT_INFO(" ############# AmpValidCheck( ) port 0 ###############\n");
		IT_INFO(" ############# REG_RX_1D0 = %X ###############\n", (int)uc);
		IT_INFO(" ############# Reg1D4 = %X ###############\n", (int)hdmirxrd(REG_RX_1D4));

		if ((uc & 0x03) == 0x03) {
			ucChannelB[0] = hdmirxrd(REG_RX_1D5);
			ucPortAMPValid[0] |= 0x03;
			IT_INFO(" ############# B AMP VALID port 0 Reg1D5 = 0x%X  ###############\n", (int)ucChannelB[0]);
		}

		if ((uc & 0x0C) == 0x0C) {
			ucChannelG[0] = hdmirxrd(REG_RX_1D6);
			ucPortAMPValid[0] |= 0x0C;
			IT_INFO(" ############# G AMP VALID port 0 Reg1D6 = 0x%X  ###############\n", (int)ucChannelG[0]);
		}

		if ((uc & 0x30) == 0x30) {
			ucChannelR[0] = hdmirxrd(REG_RX_1D7);
			ucPortAMPValid[0] |= 0x30;
			IT_INFO(" ############# R AMP VALID port 0 Reg1D7 = 0x%X  ###############\n", (int)ucChannelR[0]);
		}

		chgbank(0);

		if (hdmirxrd(REG_RX_P0_SYS_STATUS) & (B_P0_MHL_MODE)) {
			if ((ucPortAMPValid[0] & 0x03) == 0x03) {

				OverWriteAmpValue2EQ(F_PORT_SEL_0);
#ifdef _SUPPORT_EQ_ADJUST_
				HDMIStartEQDetect(&(it6602data->EQPort[F_PORT_SEL_0]));
#endif
			}

		} else {
			if ((ucPortAMPValid[0] & 0x3F) == 0x3F) {
				OverWriteAmpValue2EQ(F_PORT_SEL_0);

#ifdef _SUPPORT_EQ_ADJUST_
				HDMIStartEQDetect(&(it6602data->EQPort[F_PORT_SEL_0]));
#endif
			}
		}
	}

#endif
}

void IT66021::TogglePolarity(unsigned char ucPortSel)
{
#ifdef _SUPPORT_AUTO_EQ_
	unsigned char ucPortSelCurrent;
	ucPortSelCurrent = hdmirxrd(REG_RX_051) & B_PORT_SEL;

#ifdef _ONLY_SUPPORT_MANUAL_EQ_ADJUST_
	return;
#endif

	if (HdmiI2cAddr == IT66021A_HDMI_ADDR) {
		if (ucPortSelCurrent != ucPortSel) {
			return;
		}
	}

	if (ucPortSel == F_PORT_SEL_1) {
		IT_INFO(" ############# TogglePolarity Port 1###############\n");
		chgbank(1);

		hdmirxset(REG_RX_1C5, 0x10, 0x00);

		//FIX_ID_002 xxxxx 	Check IT6602 chip version Identify for TogglePolarity and Port 1 Deskew
		if (HdmiI2cAddr == IT66021A_HDMI_ADDR) {
			//xxxxx only for IT6602A0 Version
			if ((hdmirxrd(REG_RX_1B9) & 0x80) >> 7) {
				hdmirxset(REG_RX_1B9, 0x80, 0x00); // Change Polarity

			} else {
				hdmirxset(REG_RX_1B9, 0x80, 0x80); // Change Polarity
			}

		} else {
			if ((hdmirxrd(REG_RX_1C9) & 0x80) >> 7) {
				hdmirxset(REG_RX_1C9, 0x80, 0x00); // Change Polarity

			} else {
				hdmirxset(REG_RX_1C9, 0x80, 0x80); // Change Polarity
			}
		}

		hdmirxset(REG_RX_1C5, 0x10, 0x10);
		chgbank(0);

		IT_INFO(" ############# TogglePolarity Trigger Port 1 EQ ###############\n");
		hdmirxset(REG_RX_03A, 0xFF, 0x38);
		hdmirxset(REG_RX_03A, 0x04, 0x04);
		hdmirxset(REG_RX_03A, 0x04, 0x00);

	} else {
		IT_INFO(" ############# TogglePolarity Port 0###############\n");
		chgbank(1);
		hdmirxset(REG_RX_1B5, 0x10, 0x00);

		if ((hdmirxrd(REG_RX_1B9) & 0x80) >> 7) {
			hdmirxset(REG_RX_1B9, 0x80, 0x00); // Change Polarity

		} else {
			hdmirxset(REG_RX_1B9, 0x80, 0x80); // Change Polarity
		}

		hdmirxset(REG_RX_1B5, 0x10, 0x10);
		chgbank(0);

		IT_INFO(" ############# TogglePolarity Trigger Port 0 EQ ###############\n");

		hdmirxset(REG_RX_022, 0xFF, 0x38); //07-04
		hdmirxset(REG_RX_022, 0x04, 0x04);
		hdmirxset(REG_RX_022, 0x04, 0x00);
	}

#endif
}

void IT66021::TMDSCheck(unsigned char ucPortSel)
{
	if (ucPortSel != 0) {
		PX4_ERR("[TMDSCheck]: it66021 only support port 0 !! \n");
		return;
	}

#ifdef _SUPPORT_AUTO_EQ_
	unsigned int ucTMDSClk = 0;
	unsigned char rddata;
	unsigned char ucClk;

	struct it6602_dev_data *it6602data = get_it6602_dev_data();

	IT_INFO("TMDSCheck() !!!\n");

	IT_INFO(" HDMI Reg90  = %X ,Reg91  = %X\r\n", (int)hdmirxrd(0x90), (int)hdmirxrd(0x91));
	ucClk = hdmirxrd(REG_RX_091);
	rddata = hdmirxrd(0x90);

	if (ucClk != 0) {
		if (rddata & 0x01) {
			ucTMDSClk = 2 * RCLKVALUE * 256 / ucClk;

		} else if (rddata & 0x02) {
			ucTMDSClk = 4 * RCLKVALUE * 256 / ucClk;

		} else {
			ucTMDSClk = RCLKVALUE * 256 / ucClk;
		}

		IT_INFO(" Port 0 TMDS CLK  = %X \r\n", (int)ucTMDSClk);
	}

	// TODO 不一样地方, sdk 确认的是 IT66021A_HDMI_ADDR = 0x94 实际上不会有这个情况
	// if (HdmiI2cAddr == IT66021A_HDMI_ADDR)
	// {
	// 	if (hdmirxrd(REG_RX_P0_SYS_STATUS) & (B_P0_MHL_MODE))
	// 	{
	// 		chgbank(1);
	// 		hdmirxset(REG_RX_1B1, 0x20, 0x20); //Reg1b1[5] = 1 for enable over-write
	// 		hdmirxset(REG_RX_1B2, 0x07, 0x01); // default 0x04 , change to 0x01
	// 		IT_INFO(" HDMI Reg1B1  = %X ,Reg1B2  = %X\r\n", (int)hdmirxrd(REG_RX_1B1), (int)hdmirxrd(REG_RX_1B2));
	// 		chgbank(0);
	// 	}
	// 	else
	// 	{
	// 		chgbank(1);
	// 		hdmirxset(REG_RX_1B1, 0x20, 0x00); //Reg1b1[5] = 0 for disable over-write
	// 		hdmirxset(REG_RX_1B2, 0x07, 0x04); // default 0x04
	// 		IT_INFO(" HDMI Reg1B1  = %X ,Reg1B2  = %X\r\n", (int)hdmirxrd(REG_RX_1B1), (int)hdmirxrd(REG_RX_1B2));
	// 		chgbank(0);
	// 	}
	// }

	// TODO 不一样地方, sdk 确认的是 IT66021A_HDMI_ADDR = 0x94 实际上不会有这个情况
	// if (HdmiI2cAddr == IT66021A_HDMI_ADDR)
	// {
	// 	if (ucTMDSClk < TMDSCLKVALUE_480P || ucTMDSClk > TMDSCLKVALUE_1080P)
	// 		hdmirxwr(REG_RX_020, 0x00); // Dr. Liu suggestion to 0x00
	// 	else
	// 		hdmirxwr(REG_RX_020, 0x3F); // Dr. Liu suggestion to 0x3F
	// }

	IT_INFO(" HDMI Reg020  = %X  ucPortAMPOverWrite[0] = %d\r\n", (int)hdmirxrd(REG_RX_020), ucPortAMPOverWrite[0]);

	//FIX_ID_019	xxxxx modify ENHYS control for MHL mode
	if (hdmirxrd(REG_RX_P0_SYS_STATUS) & (B_P0_MHL_MODE)) {
		IT_INFO("hdmirxrd(REG_RX_P0_SYS_STATUS) = %d", hdmirxrd(REG_RX_P0_SYS_STATUS));

		chgbank(1);
		hdmirxset(REG_RX_1B8, 0x80, 0x00); // [7] Reg_HWENHYS = 0
		hdmirxset(REG_RX_1B6, 0x07, 0x00); // [2:0]Reg_P0_ENHYS = 00 for MHL mode only  [2:0]Reg_P0_ENHYS = 00 for disable ENHYS
		chgbank(0);
	}

	if (ucPortAMPOverWrite[0] == 0 || 1) { // 2013-0801
		//FIX_ID_001 xxxxx check State of AutoEQ
		chgbank(1);
		rddata = hdmirxrd(REG_RX_1D4);
		chgbank(0);

		//FIX_ID_032 xxxxx	//Support HDCP Repeater function for HDMI Tx device
#ifdef _ONLY_SUPPORT_MANUAL_EQ_ADJUST_
#ifndef _SUPPORT_HDCP_REPEATER_
#ifdef _SUPPORT_EQ_ADJUST_
		HDMIStartEQDetect(&(it6602data->EQPort[F_PORT_SEL_0]));
#endif
#endif
#else

		if (rddata == 0) {
			IT_INFO(" ############# Trigger Port 0 EQ ###############\n");
			hdmirxset(REG_RX_022, 0xFF, 0x38); //07-04
			hdmirxset(REG_RX_022, 0x04, 0x04);
			hdmirxset(REG_RX_022, 0x04, 0x00);
		}

#endif

		it6602data->HDMIIntEvent &= ~(B_PORT0_TMDSEvent | B_PORT0_Waiting | B_PORT0_TimingChgEvent);

	} else {
		IT_INFO(" ############# B_PORT0_TimingChgEvent###############\n");
		it6602data->HDMIIntEvent |= (B_PORT0_Waiting);
		it6602data->HDMIIntEvent |= (B_PORT0_TimingChgEvent);
		it6602data->HDMIWaitNo[0] = MAX_TMDS_WAITNO;
	}

#endif
}

void IT66021::OverWriteAmpValue2EQ(unsigned char ucPortSel)
{
	struct it6602_dev_data *it6602data = get_it6602_dev_data();

	IT_INFO(" 111111111111111111 OverWriteAmpValue2EQ 111111111111111111111111111111\r\n");
	IT_INFO(" 111111111111111111 OverWriteAmpValue2EQ 111111111111111111111111111111\r\n");
	IT_INFO(" 111111111111111111 OverWriteAmpValue2EQ 111111111111111111111111111111\r\n");
	IT_INFO(" 111111111111111111 OverWriteAmpValue2EQ 111111111111111111111111111111\r\n");
	IT_INFO(" 111111111111111111 OverWriteAmpValue2EQ 111111111111111111111111111111\r\n");
	IT_INFO(" 111111111111111111 OverWriteAmpValue2EQ 111111111111111111111111111111\r\n");

	if (hdmirxrd(REG_RX_P0_SYS_STATUS) & (B_P0_MHL_MODE)) {
		if ((ucPortAMPValid[F_PORT_SEL_0] & 0x03) == 0x03) {
			ucPortAMPOverWrite[F_PORT_SEL_0] = 1; //2013-0801
			ucEQMode[F_PORT_SEL_0] = 0;			  // 0 for Auto Mode
			IT_INFO("#### REG_RX_026 = 0x%X ####\r\n", (int)hdmirxrd(REG_RX_026));

			hdmirxset(REG_RX_026, 0x20, 0x20); //Manually set RS Value
			IT_INFO("#### REG_RX_026 = 0x%X ####\r\n", (int)hdmirxrd(REG_RX_026));

			if ((ucChannelB[F_PORT_SEL_0]) < MinEQValue) {
				hdmirxwr(REG_RX_027, (MinEQValue));
				hdmirxwr(REG_RX_028, (MinEQValue)); //07-08 using B channal to over-write G and R channel
				hdmirxwr(REG_RX_029, (MinEQValue));

			} else {
				hdmirxwr(REG_RX_027, (ucChannelB[F_PORT_SEL_0] & 0x7F));
				hdmirxwr(REG_RX_028, (ucChannelB[F_PORT_SEL_0] & 0x7F)); //07-08 using B channal to over-write G and R channel
				hdmirxwr(REG_RX_029, (ucChannelB[F_PORT_SEL_0] & 0x7F));
			}

			IT_INFO(" ############# Over-Write port 0 MHL EQ###############\r\n");
			IT_INFO(" ############# B port 0 REG_RX_027 = 0x%X  ###############\r\n", (int)hdmirxrd(REG_RX_027));
			IT_INFO(" ############# G port 0 REG_RX_028 = 0x%X  ###############\r\n", (int)hdmirxrd(REG_RX_028));
			IT_INFO(" ############# R port 0 REG_RX_029 = 0x%X  ###############\r\n", (int)hdmirxrd(REG_RX_029));

			hdmirxwr(REG_RX_022, 0x00); // power down auto EQ
			hdmirxwr(0xD0, 0x30);
			//FIX_ID_033 xxxxx
		}

	} else {
		if ((ucPortAMPValid[F_PORT_SEL_0] & 0x3F) == 0x3F) {
			ucPortAMPOverWrite[F_PORT_SEL_0] = 1; //2013-0801
			ucEQMode[F_PORT_SEL_0] = 0;			  // 0 for Auto Mode
			IT_INFO("#### REG_RX_026 = 0x%X ####\r\n", (int)hdmirxrd(REG_RX_026));
			hdmirxset(REG_RX_026, 0x20, 0x20); //Manually set RS Value
			IT_INFO("#### REG_RX_026 = 0x%X ####\r\n", (int)hdmirxrd(REG_RX_026));

			if (ucChannelB[F_PORT_SEL_0] < MinEQValue) {
				hdmirxwr(REG_RX_027, MinEQValue);

			} else {
				hdmirxwr(REG_RX_027, (ucChannelB[F_PORT_SEL_0] & 0x7F));
			}

			if (ucChannelG[F_PORT_SEL_0] < MinEQValue) {
				hdmirxwr(REG_RX_028, MinEQValue);

			} else {
				hdmirxwr(REG_RX_028, (ucChannelG[F_PORT_SEL_0] & 0x7F));
			}

			if (ucChannelR[F_PORT_SEL_0] < MinEQValue) {
				hdmirxwr(REG_RX_029, MinEQValue);

			} else {
				hdmirxwr(REG_RX_029, (ucChannelR[F_PORT_SEL_0] & 0x7F));
			}

			//if Auto EQ done  interrupt then clear HDMI Event !!!
			it6602data->HDMIIntEvent &= ~(B_PORT0_TMDSEvent | B_PORT0_Waiting | B_PORT0_TimingChgEvent);

			IT_INFO(" ############# Over-Write port 0 EQ###############\r\n");
			IT_INFO(" ############# B port 0 REG_RX_027 = 0x%X  ###############\r\n", (int)hdmirxrd(REG_RX_027));
			IT_INFO(" ############# G port 0 REG_RX_028 = 0x%X  ###############\r\n", (int)hdmirxrd(REG_RX_028));
			IT_INFO(" ############# R port 0 REG_RX_029 = 0x%X  ###############\r\n", (int)hdmirxrd(REG_RX_029));

			hdmirxwr(REG_RX_022, 0x00); // power down auto EQ
			hdmirxwr(0xD0, 0x30);
			//FIX_ID_033 xxxxx
		}
	}
}


//FIX_ID_014 xxxxx
void IT66021::IT6602HDMIEventManager(struct it6602_dev_data *it6602)
{
	if (it6602->HDMIIntEvent != 0) {
		if ((it6602->HDMIIntEvent & B_PORT0_Waiting) == B_PORT0_Waiting) {
			if (it6602->HDMIWaitNo[0] == 0) {
				it6602->HDMIIntEvent &= ~(B_PORT0_Waiting);
				IT_INFO("B_PORT0_Waiting  OK ...\n");

			} else {
				it6602->HDMIWaitNo[0]--;
				IT_INFO("B_PORT0_Waiting  %X ...Event=%X ...Reg93=%X \n",
					(int)it6602->HDMIWaitNo[0], (int)it6602->HDMIIntEvent, (int)hdmirxrd(0x93));
			}

		} else {
			if ((it6602->HDMIIntEvent & B_PORT0_TMDSEvent) == B_PORT0_TMDSEvent) {
				if (CLKCheck(F_PORT_SEL_0)) {
					IT_INFO("TMDSEvent &&&&& Port 0 Rx CKOn Detect &&&&&\r\n");
#ifdef _SUPPORT_AUTO_EQ_
					TMDSCheck(F_PORT_SEL_0);
#else
//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_EQ_ADJUST_
					HDMIStartEQDetect(&(it6602->EQPort[F_PORT_SEL_0]));
#endif
//FIX_ID_001 xxxxx
#endif
					it6602->HDMIIntEvent &= ~(B_PORT0_TMDSEvent); // finish MSC
				}

			} else if ((it6602->HDMIIntEvent & B_PORT0_TimingChgEvent) == B_PORT0_TimingChgEvent) {
				if (CLKCheck(F_PORT_SEL_0)) {
					IT_INFO("TimingChgEvent &&&&& Port 0 Rx CKOn Detect &&&&&\r\n");
//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_EQ_ADJUST_
					HDMIStartEQDetect(&(it6602->EQPort[F_PORT_SEL_0]));
#endif
					//FIX_ID_001 xxxxx

					it6602->HDMIIntEvent &= ~(B_PORT0_TimingChgEvent); // finish MSC
				}
			}
		}

		if ((it6602->HDMIIntEvent & B_PORT1_Waiting) == B_PORT1_Waiting) {
			if (it6602->HDMIWaitNo[1] == 0) {
				it6602->HDMIIntEvent &= ~(B_PORT1_Waiting);
				IT_INFO("B_PORT1_Waiting  OK ...\n");

			} else {
				it6602->HDMIWaitNo[1]--;
				IT_INFO("B_PORT1_Waiting  %X ...\n", (int)it6602->HDMIWaitNo[1]);
			}

		} else {
			if ((it6602->HDMIIntEvent & B_PORT1_TMDSEvent) == B_PORT1_TMDSEvent) {
				if (CLKCheck(F_PORT_SEL_1)) {
					IT_INFO("TMDSEvent &&&&& Port 1 Rx CKOn Detect &&&&&\r\n");
#ifdef _SUPPORT_AUTO_EQ_
					TMDSCheck(F_PORT_SEL_1);
#else
//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_EQ_ADJUST_
					HDMIStartEQDetect(&(it6602->EQPort[F_PORT_SEL_1]));
#endif
//FIX_ID_001 xxxxx
#endif
					it6602->HDMIIntEvent &= ~(B_PORT1_TMDSEvent); // finish MSC
				}

			} else if ((it6602->HDMIIntEvent & B_PORT1_TimingChgEvent) == B_PORT1_TimingChgEvent) {
				if (CLKCheck(F_PORT_SEL_1)) {
					IT_INFO("TimingChgEvent &&&&& Port 1 Rx CKOn Detect &&&&&\r\n");
#ifdef _SUPPORT_EQ_ADJUST_
					HDMIStartEQDetect(&(it6602->EQPort[F_PORT_SEL_1]));
#endif
					it6602->HDMIIntEvent &= ~(B_PORT1_TimingChgEvent); // finish MSC
				}
			}
		}
	}
}


#endif

/*****************************************************************************/
/* Driver State Machine Process **********************************************/
/*****************************************************************************/
#ifdef _ITEHDMI_

//FIX_ID_036	xxxxx

void IT66021::IT6602_Interrupt(void)
{
	struct it6602_dev_data *it6602data = get_it6602_dev_data();
	IT6602HDMIInterruptHandler(it6602data);
}

void IT66021::IT6602_fsm(void)
{
	struct it6602_dev_data *it6602data = get_it6602_dev_data();

	IT6602VideoHandler(it6602data);
#ifndef _FIX_ID_028_
	IT6602AudioHandler(it6602data);
#endif

#ifdef _SUPPORT_EQ_ADJUST_

	if (it6602data->EQPort[F_PORT_SEL_0].f_manualEQadjust == TRUE) {
		IT_INFO("[IT6602_fsm]: f_manualEQadjust == TRUE \n");
		HDMIAdjustEQ(&(it6602data->EQPort[F_PORT_SEL_0]));        // for port 0
	}

#endif

	IT6602HDMIEventManager(it6602data);
}

#endif

void  IT66021::Dump_ITEHDMIReg(void)//max7088
{
	ushort	i, j ;
	BYTE ucData ;
	IT_INFO("\r\n       ") ;

	for (j = 0 ; j < 16 ; j++) {
		IT_INFO(" %02X", (int) j) ;

		if ((j == 3) || (j == 7) || (j == 11)) {
			IT_INFO(" :") ;
		}
	}

	IT_INFO("\n        =====================================================\r\n") ;

	chgbank(0);

	for (i = 0 ; i < 0x100 ; i += 16) {
		IT_INFO("[%03X]  ", i) ;

		for (j = 0 ; j < 16 ; j++) {
			ucData = hdmirxrd((BYTE)((i + j) & 0xFF)) ;
			IT_INFO(" %02X", (int) ucData) ;

			if ((j == 3) || (j == 7) || (j == 11)) {
				IT_INFO(" :") ;
			}
		}

		IT_INFO("\r\n") ;

		if ((i % 0x40) == 0x30) {
			IT_INFO("\n        =====================================================\r\n") ;
		}
	}

	chgbank(1);

	for (i = 0xb0 ; i < 0xd0 ; i += 16) {
		IT_INFO("[%03X]  ", i + 0x100) ;

		for (j = 0 ; j < 16 ; j++) {
			ucData = hdmirxrd((BYTE)((i + j) & 0xFF)) ;
			IT_INFO(" %02X", (int) ucData) ;

			if ((j == 3) || (j == 7) || (j == 11)) {
				IT_INFO(" :") ;
			}
		}

		IT_INFO("\r\n") ;
	}

	chgbank(0);
}