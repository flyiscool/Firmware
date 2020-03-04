
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

#include "it66021_type.h"
#include "it66021_define.h"
#include "it66021_reg.h"

#include "it66021.h"

#include "px4_log.h"

#include <ar_gpio.h>

#include "vedio_monitor.h"

extern VEDIO_MONITOR *p_it66021a;
struct it6602_dev_data it66021_A;

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
};



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




static unsigned char hdmirxrd(unsigned char RegAddr)
{
	return p_it66021a->readbyte(RegAddr);
}

static unsigned char hdmirxwr(unsigned char RegAddr, unsigned char DataIn)
{
	return p_it66021a->writebyte(RegAddr, DataIn);
}

static unsigned char  hdmirxset(unsigned char  offset, unsigned char  mask, unsigned char  ucdata)
{
	unsigned char temp;
	temp = hdmirxrd(offset);
	temp = (temp & ((~mask) & 0xFF)) + (mask & ucdata);
	return hdmirxwr(offset, temp);
}

static void chgbank(int bank)
{
	switch (bank) {
	case 0 :
		hdmirxset(0x0F, 0x03, 0x00);
		break;

	case 1 :
		hdmirxset(0x0F, 0x03, 0x01);
		break;

	case 2 :
		hdmirxset(0x0F, 0x03, 0x02);
		break;

	case 3:
		hdmirxset(0x0F, 0x03, 0x03);
		break;

	default :
		break;
	}
}


static void hdmirx_Var_init(struct it6602_dev_data *it6602)
{
	it6602->m_ucSCDTOffCount = 0;
	it6602->m_ucEccCount_P0 = 0;
	it6602->m_ucEccCount_P1 = 0;
	it6602->m_ucDeskew_P0 = 0;
	it6602->m_ucDeskew_P1 = 0;

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

	IT6602_VideoOutputConfigure_Init(it6602, eCCIR656_Sep_Sync_SDR);

	it6602->m_bRxAVmute = FALSE;

#ifdef _SUPPORT_EQ_ADJUST_
	it6602->EQPort[0].ucEQState = 0xFF;
	it6602->EQPort[0].ucAuthR0 = 0;
	it6602->EQPort[0].ucECCvalue = 0;
	it6602->EQPort[0].ucECCfailCount = 0;
	it6602->EQPort[0].ucPkt_Err = 0;	//Pkt_Err
	it6602->EQPort[0].ucPortID = F_PORT_SEL_0;

	it6602->EQPort[1].ucEQState = 0xFF;
	it6602->EQPort[1].ucAuthR0 = 0;
	it6602->EQPort[1].ucECCvalue = 0;
	it6602->EQPort[1].ucECCfailCount = 0;
	it6602->EQPort[1].ucPkt_Err = 0;
	it6602->EQPort[1].ucPortID = F_PORT_SEL_1;

	it6602->EQPort[0].f_manualEQadjust = FALSE;
	it6602->EQPort[1].f_manualEQadjust = FALSE;

#endif

#ifdef _SUPPORT_AUTO_EQ_

	ucPortAMPOverWrite[1] = 0;	//2013-0801
	ucPortAMPValid[1] = 0;
	ucChannelB[1] = 0;
	ucChannelG[1] = 0;
	ucChannelR[1] = 0;

	ucPortAMPOverWrite[0] = 0;	//2013-0801
	ucPortAMPValid[0] = 0;
	ucChannelB[0] = 0;
	ucChannelG[0] = 0;
	ucChannelR[0] = 0;
#endif

	it6602->CBusIntEvent = 0;
	it6602->CBusSeqNo = 0;
	it6602->CBusWaitNo = 0x00;

	it6602->HDMIIntEvent = 0;
	it6602->HDMIWaitNo[0] = 0;
	it6602->HDMIWaitNo[1] = 0;

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


static void IT6602_Rst(struct it6602_dev_data *it6602)
{
	hdmirx_Var_init(it6602);
	hdimrx_write_init(IT6602_HDMI_INIT_TABLE);
}


char IT6602_fsm_init(void)
{
	IT6602_Rst(&it66021_A);

	EDIDRAMInitial(&Default_Edid_Block[0]);

	//FIX_ID_041 xxxxx Add EDID reset
	// fo IT6803 EDID fail issue
	hdmirxset(REG_RX_0C0, 0x20, 0x20);	//xxxxx 2014-0731 [5] 1 for  reset edid
	IT_Delay(1);
	hdmirxset(REG_RX_0C0, 0x20, 0x00);
	// fo IT6803 EDID fail issue
	//FIX_ID_041 xxxxx

	it6602PortSelect(0);	// select port 0
	return TRUE;
}



static void IT6602VideoOutputEnable(unsigned char bEnableOutput)
{
	if (bEnableOutput) {
		hdmirxset(REG_RX_053, (B_TriSYNC | B_TriVDIO), (0x00)); // enable output
		PX4_INFO("---------------- IT6602VideoOutputEnable -> On ----------------\n");

	} else {
		hdmirxset(REG_RX_053, (B_TriSYNC | B_TriVDIO), (B_TriSYNC | B_TriVDIO)); // disable output
		PX4_INFO("---------------- IT6602VideoOutputEnable -> Off ----------------\n");
	}
}

static unsigned char IsHDMIMode(void)
{
	unsigned char sys_state_P0;
	unsigned char sys_state_P1;
	unsigned char ucPortSel;

	sys_state_P0 = hdmirxrd(REG_RX_P0_SYS_STATUS) & B_P0_HDMI_MODE;
	sys_state_P1 = hdmirxrd(REG_RX_P1_SYS_STATUS) & B_P1_HDMI_MODE;
	ucPortSel = hdmirxrd(REG_RX_051) & B_PORT_SEL;

	if (((sys_state_P0 & B_P0_HDMI_MODE) && (ucPortSel == F_PORT_SEL_0)) ||
	    ((sys_state_P1 & B_P1_HDMI_MODE) && (ucPortSel == F_PORT_SEL_1))) {
		return TRUE;

	} else {
		return FALSE;
	}

}

static void SetVideoInputFormatWithoutInfoFrame(struct it6602_dev_data *it6602, unsigned char bInMode)
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

static void SetColorimetryByMode(struct it6602_dev_data *it6602)
{
	unsigned char  RxClkXCNT;
	RxClkXCNT = hdmirxrd(REG_RX_PIXCLK_SPEED);

	PX4_INFO(" SetColorimetryByMode() REG_RX_PIXCLK_SPEED=%X \n", (int) RxClkXCNT);

	it6602->m_bInputVideoMode &= ~F_MODE_ITU709;

	if (RxClkXCNT < 0x34) {
		it6602->m_bInputVideoMode |= F_MODE_ITU709;

	} else {
		it6602->m_bInputVideoMode &= ~F_MODE_ITU709;
	}

}

static void SetColorSpaceConvert(struct it6602_dev_data *it6602)
{
	unsigned char csc ;
	//    unsigned char uc ;
	unsigned char filter = 0 ; // filter is for Video CTRL DN_FREE_GO, EN_DITHER, and ENUDFILT

#ifdef DISABLE_HDMI_CSC
	DLOG_INFO("ITEHDMI - HDMI Color Space Convert is disabled \r\n");

	csc = B_CSC_BYPASS ;
	it6602->m_bOutputVideoMode = it6602->m_bInputVideoMode;

#else
	DLOG_INFO("\n!!! SetColorSpaceConvert( ) !!!\n");

//FIX_ID_039 xxxxx fix image flick when enable RGB limited / Full range convert
//xxxxx FIX_ID_039 disable --> //FIX_ID_027 xxxxx Support Full/Limited Range convert
//xxxxx FIX_ID_039 disable --> 	//default to turn off CSC offset
//xxxxx FIX_ID_039 disable --> 	hdmirxset(REG_RX_067,0x78,0x00);
//xxxxx FIX_ID_039 disable --> 	hdmirxwr(REG_RX_068,0x00);
//xxxxx FIX_ID_039 disable --> //FIX_ID_027 xxxxx
//FIX_ID_039 xxxxx

//FIX_ID_039 xxxxx fix image flick when enable RGB limited / Full range convert
#ifdef _AVOID_REDUNDANCE_CSC_

	if ((it6602->m_Backup_OutputVideoMode == it6602->m_bOutputVideoMode)
	    && (it6602->m_Backup_InputVideoMode == it6602->m_bInputVideoMode)) {
		DLOG_INFO("I/P and O/P color without change , No need to setup CSC convert again \n");
		return;
	}

#endif
//FIX_ID_039 xxxxx

	//DLOG_INFO("Input mode is YUV444 ");
	switch (it6602->m_bOutputVideoMode & F_MODE_CLRMOD_MASK) {
#if defined(SUPPORT_OUTPUTYUV444)

	case F_MODE_YUV444:
		DLOG_INFO("Output mode is YUV444\n");

		switch (it6602->m_bInputVideoMode & F_MODE_CLRMOD_MASK) {
		case F_MODE_YUV444:
			DLOG_INFO("Input mode is YUV444\n");
			csc = B_CSC_BYPASS ;
			break ;

		case F_MODE_YUV422:
			DLOG_INFO("Input mode is YUV422\n");
			csc = B_CSC_BYPASS ;

			if (it6602->m_bOutputVideoMode & F_MODE_EN_UDFILT) { // RGB24 to YUV422 need up/dn filter.
				filter |= B_RX_EN_UDFILTER ;
			}

			if (it6602->m_bOutputVideoMode & F_MODE_EN_DITHER) { // RGB24 to YUV422 need up/dn filter.
				filter |= B_RX_EN_UDFILTER | B_RX_DNFREE_GO ;
			}

			break ;

		case F_MODE_RGB24:
			DLOG_INFO("Input mode is RGB444\n");
			csc = B_CSC_RGB2YUV ;
			break ;
		}

		break ;
#endif

#if defined(SUPPORT_OUTPUTYUV422)

	case F_MODE_YUV422:
		switch (it6602->m_bInputVideoMode & F_MODE_CLRMOD_MASK) {
		case F_MODE_YUV444:
			DLOG_INFO("Input mode is YUV444\n");

			if (it6602->m_bOutputVideoMode & F_MODE_EN_UDFILT) {
				filter |= B_RX_EN_UDFILTER ;
			}

			csc = B_CSC_BYPASS ;
			break ;

		case F_MODE_YUV422:
			DLOG_INFO("Input mode is YUV422\n");
			csc = B_CSC_BYPASS ;

			// if output is YUV422 and 16 bit or 565, then the dither is possible when
			// the input is YUV422 with 24bit input, however, the dither should be selected
			// by customer, thus the requirement should set in ROM, no need to check
			// the register value .
			if (it6602->m_bOutputVideoMode & F_MODE_EN_DITHER) { // RGB24 to YUV422 need up/dn filter.
				filter |= B_RX_EN_UDFILTER | B_RX_DNFREE_GO ;
			}

			break ;

		case F_MODE_RGB24:
			DLOG_INFO("Input mode is RGB444\n");

			if (it6602->m_bOutputVideoMode & F_MODE_EN_UDFILT) { // RGB24 to YUV422 need up/dn filter.
				filter |= B_RX_EN_UDFILTER ;
			}

			csc = B_CSC_RGB2YUV ;
			break ;
		}

		break ;
#endif

#if defined(SUPPORT_OUTPUTRGB)

	case F_MODE_RGB24:
		DLOG_INFO("Output mode is RGB24\n");

		switch (it6602->m_bInputVideoMode & F_MODE_CLRMOD_MASK) {
		case F_MODE_YUV444:
			DLOG_INFO("Input mode is YUV444\n");
			csc = B_CSC_YUV2RGB ;
			break ;

		case F_MODE_YUV422:
			DLOG_INFO("Input mode is YUV422\n");
			csc = B_CSC_YUV2RGB ;

			if (it6602->m_bOutputVideoMode & F_MODE_EN_UDFILT) { // RGB24 to YUV422 need up/dn filter.
				filter |= B_RX_EN_UDFILTER ;
			}

			if (it6602->m_bOutputVideoMode & F_MODE_EN_DITHER) { // RGB24 to YUV422 need up/dn filter.
				filter |= B_RX_EN_UDFILTER | B_RX_DNFREE_GO ;
			}

			break ;

		case F_MODE_RGB24:
			DLOG_INFO("Input mode is RGB444\n");
			csc = B_CSC_BYPASS ;
			break ;
		}

		break ;
#endif
	}


#if defined(SUPPORT_OUTPUTYUV)

	// set the CSC associated registers
	if (csc == B_CSC_RGB2YUV) {
		// DLOG_INFO("CSC = RGB2YUV ");
		//FIX_ID_039 xxxxx fix image flick when enable RGB limited / Full range convert
		//default to turn off CSC offset
		hdmirxset(REG_RX_067, 0x78, 0x00);
		hdmirxwr(REG_RX_068, 0x00);
		DLOG_INFO(" Clear Reg67 and Reg68 ... \r\n");

		//FIX_ID_039 xxxxx
		if (it6602->m_bInputVideoMode & F_MODE_ITU709) {
			DLOG_INFO("ITU709 ");

			if (it6602->m_bInputVideoMode & F_MODE_16_235) {
				DLOG_INFO(" 16-235\n");
				chgbank(1);	//for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_RGB2YUV_ITU709_16_235), &bCSCMtx_RGB2YUV_ITU709_16_235[0]);

			} else {
				DLOG_INFO(" 0-255\n");
				chgbank(1);	//for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_RGB2YUV_ITU709_0_255), &bCSCMtx_RGB2YUV_ITU709_0_255[0]);
			}

		} else {
			DLOG_INFO("ITU601 ");

			if (it6602->m_bInputVideoMode & F_MODE_16_235) {
				chgbank(1);	//for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_RGB2YUV_ITU601_16_235), &bCSCMtx_RGB2YUV_ITU601_16_235[0]);
				DLOG_INFO(" 16-235\n");

			} else {
				chgbank(1);	//for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_RGB2YUV_ITU601_0_255), &bCSCMtx_RGB2YUV_ITU601_0_255[0]);
				DLOG_INFO(" 0-255\n");
			}
		}
	}

#endif
#if defined(SUPPORT_OUTPUTRGB)

	if (csc == B_CSC_YUV2RGB) {
		DLOG_INFO("CSC = YUV2RGB ");
		//FIX_ID_039 xxxxx fix image flick when enable RGB limited / Full range convert
		//default to turn off CSC offset
		hdmirxset(REG_RX_067, 0x78, 0x00);
		hdmirxwr(REG_RX_068, 0x00);
		DLOG_INFO(" Clear Reg67 and Reg68 ... \r\n");

		//FIX_ID_039 xxxxx
		if (it6602->m_bInputVideoMode & F_MODE_ITU709) {
			DLOG_INFO("ITU709 ");

			if (it6602->m_bOutputVideoMode & F_MODE_16_235) {
				DLOG_INFO("16-235\n");
				chgbank(1);	//for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_YUV2RGB_ITU709_16_235), &bCSCMtx_YUV2RGB_ITU709_16_235[0]);

			} else {
				DLOG_INFO("0-255\n");
				chgbank(1);	//for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_YUV2RGB_ITU709_0_255), &bCSCMtx_YUV2RGB_ITU709_0_255[0]);
			}

		} else {
			DLOG_INFO("ITU601 ");

			if (it6602->m_bOutputVideoMode & F_MODE_16_235) {
				DLOG_INFO("16-235\n");
				chgbank(1);	//for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_YUV2RGB_ITU601_16_235), &bCSCMtx_YUV2RGB_ITU601_16_235[0]);

			} else {
				DLOG_INFO("0-255\n");
				chgbank(1);	//for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_YUV2RGB_ITU601_0_255), &bCSCMtx_YUV2RGB_ITU601_0_255[0]);

			}
		}

	}

//FIX_ID_027 xxxxx Support Full/Limited Range convert
	if (csc == B_CSC_BYPASS) {

		if ((it6602->m_bInputVideoMode & F_MODE_CLRMOD_MASK) == F_MODE_RGB24) {
			if (it6602->RGBQuantizationRange == 1) {	// Limited range from HDMI source
				if ((it6602->m_bOutputVideoMode & F_MODE_16_235) != F_MODE_16_235) {	// Full range to back-end device
					// RedText;
					DLOG_INFO(" bCSCMtx_RGB_16_235_RGB_0_255 \r\n");
					// printf("pccmd w 65 02 90;\r\n");
					// printf("pccmd w 67 78 90;\r\n");
					// printf("pccmd w 68 ED 90;\r\n");
					// WhileText;
					csc = B_CSC_RGB2YUV;
					chgbank(1);	//for CSC setting Reg170 ~ Reg184 !!!!
					hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_RGB_16_235_RGB_0_255), &bCSCMtx_RGB_16_235_RGB_0_255[0]);
					chgbank(0);
					//hdmirxset(REG_RX_065,0x03,0x02);	// B_CSC_RGB2YUV
					hdmirxset(REG_RX_067, 0x78, 0x78);
					hdmirxwr(REG_RX_068, 0xED);
				}

			} else if (it6602->RGBQuantizationRange == 2) { //Full range from HDMI source
				if ((it6602->m_bOutputVideoMode & F_MODE_16_235) == F_MODE_16_235) {	// Limited range to back-end device
					// RedText;
					DLOG_INFO(" bCSCMtx_RGB_0_255_RGB_16_235 \r\n");
					// printf("pccmd w 65 02 90;\r\n");
					// printf("pccmd w 67 40 90;\r\n");
					// printf("pccmd w 68 10 90;\r\n");
					// WhileText;
					csc = B_CSC_RGB2YUV;
					chgbank(1);	//for CSC setting Reg170 ~ Reg184 !!!!
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

#endif	//end of DISABLE_HDMI_CSC

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




static void SetVideoOutputColorFormat(struct it6602_dev_data *it6602)
{
	switch (it6602->m_bOutputVideoMode & F_MODE_CLRMOD_MASK) {
	case F_MODE_RGB24 :
		hdmirxset(REG_RX_OUT_CSC_CTRL, (M_OUTPUT_COLOR_MASK), B_OUTPUT_RGB24);
#ifdef _SUPPORT_RBSWAP_
		hdmirxset(REG_RX_064, 0x18, 0x08);
#endif

		break;

	case F_MODE_YUV422 :
		hdmirxset(REG_RX_OUT_CSC_CTRL, (M_OUTPUT_COLOR_MASK), B_OUTPUT_YUV422);
#ifdef _SUPPORT_RBSWAP_
		hdmirxset(REG_RX_064, 0x18, 0x10);
#endif
		break;

	case F_MODE_YUV444 :
		hdmirxset(REG_RX_OUT_CSC_CTRL, (M_OUTPUT_COLOR_MASK), B_OUTPUT_YUV444);
#ifdef _SUPPORT_RBSWAP_
		hdmirxset(REG_RX_064, 0x18, 0x08);
#endif
		break;
	}
}

static void SetDVIVideoOutput(struct it6602_dev_data *it6602)
{
	PX4_INFO("SetDVIVideoOutput() \n");

	SetVideoInputFormatWithoutInfoFrame(it6602, F_MODE_RGB24);
	SetColorimetryByMode(it6602);
	SetColorSpaceConvert(it6602);

	SetVideoOutputColorFormat(it6602);	//2013-0502
}

// ---------------------------------------------------------------------------
static void GetAVIInfoFrame(struct it6602_dev_data *it6602)
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
			it6602->RGBQuantizationRange = 1 ; // limited range

		} else {
			// IT mode
			it6602->RGBQuantizationRange = 2 ; // Full range
		}
	}

//FIX_ID_027	 xxxxx

	DLOG_Info("AVI ColorMode = %X \r\n", (int) it6602->ColorMode);
	DLOG_Info("AVI Colorimetry = %X \r\n", (int) it6602->Colorimetry);
	DLOG_Info("AVI ExtendedColorimetry = %X \r\n", (int) it6602->ExtendedColorimetry);
	DLOG_Info("AVI RGBQuantizationRange = %X \r\n", (int) it6602->RGBQuantizationRange);
	DLOG_Info("AVI VIC = %X \r\n", (int) it6602->VIC);
	DLOG_Info("AVI YCCQuantizationRange = %X \r\n", (int) it6602->YCCQuantizationRange);
}

// ---------------------------------------------------------------------------
static void SetNewInfoVideoOutput(struct it6602_dev_data *it6602)
{

	DLOG_INFO("SetNewInfoVideoOutput() \n");

	SetVideoInputFormatWithInfoFrame(it6602);
	SetColorimetryByInfoFrame(it6602);
	SetColorSpaceConvert(it6602);

	SetVideoOutputColorFormat(it6602);	//2013-0502

}




//FIX_ID_003 xxxxx	//Add IT6602 Video Output Configure setting
static void IT6602_VideoOutputModeSet(struct it6602_dev_data *it6602)
{
	unsigned char ucReg51;
	unsigned char ucReg65;

	DLOG_INFO("IT6602_VideoOutputModeSet() \r\n");

	DLOG_INFO("+++ %s", VModeStateStr[(unsigned char)it6602->m_VidOutConfigMode]);

	ucReg51 = hdmirxrd(REG_RX_051) & 0x9B;	// Reg51 [6] Half PCLK DDR , [5] Half Bus DDR , [2] CCIR656 mode

	// Reg65 [7] BTA1004Fmt , [6] SyncEmb ,[5:4] output color 0x00 RGB, 0x10 YUV422, 0x20 YUV444
	ucReg65 = hdmirxrd(REG_RX_065) & 0x0F;

	switch ((it6602->m_bOutputVideoMode & F_MODE_CLRMOD_MASK)) {
	case F_MODE_RGB444:
		ucReg65 |= (B_OUTPUT_RGB24);		// 0x00 B_OUTPUT_RGB24
#ifdef _SUPPORT_RBSWAP_
		hdmirxset(REG_RX_064, 0x18, 0x08);
#endif
		break;

	case F_MODE_YUV422:
		ucReg65 |= (B_OUTPUT_YUV422);		// 0x10 B_OUTPUT_YUV422
#ifdef _SUPPORT_RBSWAP_
		hdmirxset(REG_RX_064, 0x18, 0x10);
#endif
		break;

	case F_MODE_YUV444:
		ucReg65 |= (B_OUTPUT_YUV444);		// 0x20 B_OUTPUT_YUV444
#ifdef _SUPPORT_RBSWAP_
		hdmirxset(REG_RX_064, 0x18, 0x10);
#endif
		break;
	}


	switch (it6602->m_VidOutDataTrgger) {
	case eSDR:
		break;

	case eHalfPCLKDDR:
		ucReg51 |= (B_HALF_PCLKC);			// 0x40 half PCLK
		break;

	case eHalfBusDDR:
		ucReg51 |= (B_OUT_DDR);				// 0x20 half bus
		break;

	case eSDR_BTA1004:
		ucReg65 |= (B_BTA1004Fmt | B_SyncEmb) ;	// 0x80 BTA1004 + 0x40 SyncEmb
		break;

	case eDDR_BTA1004:
		ucReg51 |= (B_HALF_PCLKC);			// 0x40 half PCLK
		ucReg65 |= (B_BTA1004Fmt | B_SyncEmb) ;	// 0x80 BTA1004 + 0x40 SyncEmb
		break;

	}

	switch (it6602->m_VidOutSyncMode) {
	case eSepSync:
		break;

	case eEmbSync:
		ucReg65 |= (B_SyncEmb) ;	// 0x40 SyncEmb
		break;

	case eCCIR656SepSync:
		ucReg51 |= (B_CCIR656);	// 0x04 CCIR656
		break;

	case eCCIR656EmbSync:
		ucReg51 |= (B_CCIR656);	// 0x04 CCIR656
		ucReg65 |= (B_SyncEmb) ;	// 0x40 SyncEmb
		break;
	}

	DLOG_INFO("Reg51 = %X ", (int) ucReg51);
	DLOG_INFO("Reg65 = %X\r\n", (int) ucReg65);

	hdmirxwr(REG_RX_051, ucReg51);
	hdmirxwr(REG_RX_065, ucReg65);

}
//FIX_ID_003 xxxxx



static void IT6602VideoOutputConfigure(struct it6602_dev_data *it6602)
{

	// Configure Output color space convert

	//06-27 disable -->	#ifndef DISABLE_HDMI_CSC
	//06-27 disable --> 	it6602->m_bOutputVideoMode = HDMIRX_OUTPUT_VID_MODE ;
	//06-27 disable -->	#endif

	it6602->m_bUpHDMIMode = IsHDMIMode();

	if (it6602->m_bUpHDMIMode == FALSE) {
		SetDVIVideoOutput(it6602);

	} else {
//FIX_ID_027 xxxxx		//Support RGB limited / Full range convert
		GetAVIInfoFrame(it6602);
		SetNewInfoVideoOutput(it6602);
//FIX_ID_027 xxxxx
	}

	it6602->m_NewAVIInfoFrameF = FALSE;

	// Configure Output Color Depth

	it6602->GCP_CD = ((hdmirxrd(0x99) & 0xF0) >> 4);

	switch (it6602->GCP_CD) {
	case 5 :
		DLOG_INFO("\n Output ColorDepth = 30 bits per pixel\r\n");
		hdmirxset(0x65, 0x0C, 0x04);
		break;

	case 6 :
		DLOG_INFO("\n Output ColorDepth = 36 bits per pixel\r\n");
		hdmirxset(0x65, 0x0C, 0x08);
		break;

	default :
		DLOG_INFO("\n Output ColorDepth = 24 bits per pixel\r\n");
		hdmirxset(0x65, 0x0C, 0x00);
		break;
	}

	// Configure TTL Video Output mode
	IT6602_VideoOutputModeSet(it6602);

}




static void IT6602SwitchVideoState(struct it6602_dev_data *it6602, Video_State_Type eNewVState)
{
	if (it6602->m_VState == eNewVState) {
		return;
	}


	PX4_INFO("+++ %s\n", VStateStr[(unsigned char)eNewVState]);

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
			it6602->m_ucDeskew_P1 = 0;
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
			DLOG_Output(1000);
			hdmirxwr(0x84, 0x8F);	//2011/06/17 xxxxx, for enable Rx Chip count

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

//FIX_ID_033 xxxxx  //Fine-tune EQ Adjust function for HDCP receiver and repeater mode
#ifdef _SUPPORT_HDCP_REPEATER_
#ifdef _PSEUDO_HDCP_REPEATER_TEST_

			// TX_BSTATUS = 0x102;
			if (m_RxHDCPstatus == 2) {
				ITEHDMI_RxHDCP2ndAuthenticationRequest(TX_KSVList, TX_BKSV, TX_BSTATUS);
			}

#endif
#endif
//FIX_ID_033 xxxxx

		}
		break;
	}

}



void it6602PortSelect(unsigned char ucPortSel)
{
	struct it6602_dev_data *it6602data = &it66021_A;

	hdmirxset(REG_RX_051, B_PORT_SEL, F_PORT_SEL_0); //select port 0
	it6602data->m_ucCurrentHDMIPort = F_PORT_SEL_0;

	if (it6602data->m_ucCurrentHDMIPort != ucPortSel) {
		IT6602SwitchVideoState(it6602data, VSTATE_SyncWait);
		it6602data->m_ucCurrentHDMIPort = ucPortSel;
	}

}


void it6602HPDCtrl(unsigned char ucport, unsigned char ucEnable)
{
	if (ucport == 0) {
		if (ucEnable == 0) {
			chgbank(1);
			hdmirxset(REG_RX_1B0, 0x03, 0x01); //clear port 0 HPD=1 for EDID update
			chgbank(0);

		} else {
			if ((hdmirxrd(REG_RX_P0_SYS_STATUS) & B_P0_PWR5V_DET)) {
				chgbank(1);
				hdmirxset(REG_RX_1B0, 0x03, 0x03); //set port 0 HPD=1
				chgbank(0);
			}
		}
	}

}


char it66021_init(void)
{
	char ret = FALSE;

	it6602HPDCtrl(0, 0); // HDMI port , set HPD = 0

	usleep(200 * 1000);

	ret = IT6602_fsm_init();

	it6602HPDCtrl(0, 1);

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
	//IT_Delay(1000); //for power sequence
}
