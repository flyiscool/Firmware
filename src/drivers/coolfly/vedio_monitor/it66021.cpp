
/*****************************************************************************/
/* Header Files Included *****************************************************/
/*****************************************************************************/


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

#include "it66021.h"
#include "it66021_type.h"
#include "it66021_define.h"
#include "it66021_reg.h"

#include "px4_log.h"

#include <ar_gpio.h>

#include "vedio_monitor.h"

extern VEDIO_MONITOR *p_it66021a;
extern VEDIO_MONITOR *p_it66021a_edid;
extern VEDIO_MONITOR *p_it66021a_mhl;
struct it6602_dev_data it66021_A;
STRU_HDMI_RX_OUTPUT_FORMAT ite_a;

// transfer to cpu2
static struct h264_input_format_s att;
static orb_advert_t h264_input_format_topic;


static STRU_HDMI_RX_OUTPUT_FORMAT s_st_hdmiRxSupportedOutputFormat[] = {
	{720,  480,  60},
	{1280, 720,  30},
	{1280, 720,  25},
	{1280, 720,  50},
	{1280, 720,  60},
	{1920, 1080, 25},
	{1920, 1080, 30},
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


#define _CODE

#ifndef _ITEHDMI_
#define _ITEHDMI_
#endif


#define Enable_Vendor_Specific_packet

#define MS_TimeOut(x) (x+1)

#define VSATE_CONFIRM_SCDT_COUNT	MS_TimeOut(100)

#define AUDIO_READY_TIMEOUT   MS_TimeOut(200)

#define AUDIO_MONITOR_TIMEOUT 	MS_TimeOut(150)

#define SCDT_OFF_TIMEOUT  MS_TimeOut(20)		//100 x MS_LOOP = 5000 ms = 5 sec
#define ECC_TIMEOUT       MS_TimeOut(20)
#define DESKEW_TIMEOUT    MS_TimeOut(20)

// Debug Mode
#define EnCBusDbgMode  	FALSE
#define MSCCBusDbgCtrl 	TRUE
#define DDCCBusDbgCtrl  FALSE
#define RCLKFreqSel 	1	//; //0: RING/2 ; 1: RING/4 ; 2: RING/8 ; 3: RING/16
#define GenPktRecType	0x81
#define PPHDCPOpt		TRUE	//2013-0509 MHL 1080p packet pixel mode HDCP

#define PPHDCPOpt2		TRUE	//2013-0509 MHL 1080p packet pixel mode HDCP

#define T10usSrcSel   	TRUE	//FALSE: 100ms calibration , TRUR: 27MHz Crystal(only IT6602)

#define EnMSCBurstWr	TRUE
#define MSCBurstWrID	TRUE   // TRUE: from MHL5E/MHL5F
#define MSCBurstWrOpt	FALSE  // TRUE: Not write Adopter ID unsigned char o ScratchPad
#define EnPktFIFOBurst	TRUE
// DDC Option
#define EnDDCSendAbort	TRUE  // Send ABORT after segment write with EOF
//CBUS Capability
#define MHLVersion		0x20
#define PLIM			1
#define POW				1
#define DEV_TYPE_SINK	1 //06-26
#define DEV_TYPE		1
#define ADOPTER_ID_H	0x02
#define ADOPTER_ID_L	0x45
#define DEVICE_ID_H		0x68
#define DEVICE_ID_L		0x02
#define AckHigh			0xB
#define AckLow			1

// CBUS INput Option
#define EnCBusDeGlitch	TRUE
//---------------------//
//----- WatchDog -----//
//--------------------//
#define DeltaNum 		1
#define RegBurstWrTOSel	2 // 2	//0: 320ms, 1: 340ms, 2: 360ms (ATC)
#define Reg100msTOAdj	2 // 2	//00: 100ms, 01: 99ms, 10: 101ms (ATC)
#define EnMSCHwRty		FALSE
#define EnHWPathEn		FALSE
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



#define HDCPIntKey   	FALSE   //TRUE: Internal HDCP Key, FALSE: SIPROM

#define VCLK_INV		0
#define VCLK_DLY		0
#define EnMultiSeg   	TRUE
#define EnIntEDID    	TRUE

//Discovery
#define CBUSFloatAdj	FALSE
#define EQFAILCNT 		2



//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#define EQRETRYFAILCNT 	1	// for EQ interrupt
#define RCLKVALUE 		12			// for show TMDS and Pixel Clk
#define TMDSCLKVALUE 	160	// for TMDS > 160 then set RS to 00, otherwise set to 3F

#define TMDSCLKVALUE_1080P 	160	// for TMDS > 160 then set RS to 00, otherwise set to 3F
#define TMDSCLKVALUE_480P 	35
#define TMDSCLKVALUE_MHL_ER1 90
#define JUDGE_ER1_VALUE 	90

#define _RCLK_FREQ_20M  FALSE


#define MAX_CBUS_WAITNO 		(300/MS_LOOP)		// 250ms
#define MAX_PATHEN_WAITNO 		(700/MS_LOOP)		// 700ms
#define MAX_BUSY_WAITNO 		(2500/MS_LOOP)		// 150ms
#define MAX_DISCOVERY_WAITNO 	(100/MS_LOOP)		// 100ms

#define MAX_TMDS_WAITNO 		(350/MS_LOOP)		// 400ms
#define MAX_HDCP_WAITNO 		(100/MS_LOOP)		// 150ms

#define RENEW_WAKEUP		(12000/MS_LOOP)
#define IGNORE_WAKEUP		(1000/MS_LOOP)
#define TOGGLE_WAKEUP		(4000/MS_LOOP)
#define CDSENSE_WAKEUP		(500/MS_LOOP)

#define DEFAULT_EQVALUE		0x1F

/*****************************************************************************/
/* Private and Local Variables    ********************************************/
/*****************************************************************************/
#if 1

struct it6602_dev_data it6602DEV;

unsigned char  V3D_EntryCnt = 0;
unsigned char  wrburstoff, wrburstnum;
unsigned char  TxWrBstSeq = 0;

//FIX_ID_013	xxxxx	//For Acer MHL Dongle MSC 3D request issue
//unsigned char  EnMSCWrBurst3D  = TRUE;
//unsigned char  EnMHL3DSupport  = FALSE;
//FIX_ID_013	xxxxx

unsigned char  wakeupcnt = 0;
#define MinEQValue	0x03

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

#ifdef _SUPPORT_AUTO_EQ_
unsigned char ucPortAMPOverWrite[2];
unsigned char ucPortAMPValid[2];
unsigned char ucChannelB[2];	// ch0
unsigned char ucChannelG[2];	// ch1
unsigned char ucChannelR[2];	// ch2
unsigned char ucEQMode[2];
#endif

//FIX_ID_035 xxxxx //For MTK6592 HDMI to SII MHL TX compliance issue
//xxxxx 2014-0508 disable -> unsigned char ucEqRetryCnt[2];
//FIX_ID_035 xxxxx
//FIX_ID_001 xxxxx



//#ifdef _IT6602_
//unsigned char   DeviceID = IT6602_CHIP;
//#else
//unsigned char   DeviceID = IT66021_CHIP;
//#endif

//for debug video format only
int CurTMDSCLK;
volatile VTiming CurVTiming;
AVI_InfoFrame aviinfoframe;
//int GCP_CD       = CD8BIT; //24 bits per pixel
int InColorMode  = RGB444; //RGB444, YCbCr422, YCbCr444
int OutColorMode = RGB444; //RGB444, YCbCr422, YCbCr444
int OutCD        = OUT8B;
int VIC;


/****************************************************************************/
/*							EDID Argument									*/
/****************************************************************************/
unsigned char  VSDB_Addr;// for EDID RAM function
unsigned char  txphyadr[2], txphyA, txphyB, txphyC, txphyD, txphylevel;	// for CEC function
unsigned char  rxphyadr[2][2];// for EDID RAM function
unsigned char  rxphyA, rxphyB, rxphyC, rxphyD, rxcurport;	// for CEC function


#endif

/*****************************************************************************/
/* Init, Power, and IO Structures ********************************************/
/*****************************************************************************/
////////////////////////////////////////////////////////////////////
//it6602 chip inital table
////////////////////////////////////////////////////////////////////
_CODE struct IT6602_REG_INI  IT6602_HDMI_INIT_TABLE[] = {
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

	{REG_RX_00F,	0x03,	0x00},	//change bank 0	//2013-0430 Andrew suggestion
	{REG_RX_017,	0xC0,	0x80},	//Port 0�G[7:6] = 10 invert Port 0 input HCLK , CLKD5I	//2013-0430 Andrew suggestion
	{REG_RX_01E,	0xC0,	0x00},	//Port 1�G[7:6] = 00 invert Port 1 input TMDS , CLKD5I	//2013-0430 Andrew suggestion

	{REG_RX_016,	0x08,	0x08},	//Port 0�G[3]1: Enable CLKD5 auto power down
	{REG_RX_01D,	0x08,	0x08},	//Port 1�G[3]1: Enable CLKD5 auto power down

	{REG_RX_02B,	0xFF,	0x07},	//FixTek3D
//	{REG_RX_031,	0xFF,	0x2C},	//[7:4]Enable repeater function [3:0] SCL hold time count & Update Ri sel

	{REG_RX_031,	0xFF,	0x09},	//[7:4]Enable repeater function [3:0] SCL hold time count & Update Ri sel
	{REG_RX_049,	0xFF,	0x09},	//[7:4]Enable repeater function [3:0] SCL hold time count & Update Ri sel

	{REG_RX_035,	0x1E,	(0x10 + (DeltaNum << 2))},	//[3:2] RCLKDeltaSel , [1] UseIPLock = 0
	{REG_RX_04B,	0x1E,	(0x10 + (DeltaNum << 2))},	//[3:2] RCLKDeltaSel , [1] UseIPLock = 0

	{REG_RX_054,	0xFF,	(1 << 4) + RCLKFreqSel},	//[1:0]RCLK frequency select
	{REG_RX_06A,	0xFF,	GenPktRecType},			//Decide which kind of packet to be fully recorded on General PKT register
	{REG_RX_074,	0xFF,	0xA0},	//[7]Enable i2s and SPDIFoutput [5]Disable false DE output
	{REG_RX_050,	0x1F,	0x12},	//[4]1: Invert output DCLK and DCLK DELAY 2 Step
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


	{REG_RX_077, 0x80, 0x80},	 // IT66021 Audio i2s sck and mclk is common pin
	{REG_RX_00F, 0x03, 0x01},	//change bank 1
	{REG_RX_1C0, 0x80, 0x80},
	{REG_RX_00F, 0x03, 0x00},	//change bank 0


#ifdef _HBR_I2S_
	{REG_RX_07E, B_HBRSel, 0x00},
#else
	{REG_RX_07E, B_HBRSel, B_HBRSel},
#endif

	{REG_RX_052, (B_DisVAutoMute), (B_DisVAutoMute)},				//Reg52[5] = 1 for disable Auto video MUTE
	{REG_RX_053, (B_VDGatting | B_VIOSel | B_TriVDIO | B_TriSYNC), (B_VIOSel | B_TriVDIO | B_TriSYNC)},				//Reg53[7][5] = 01    // for disable B_VDIO_GATTING

	{REG_RX_058, 0xFF, 0x33},			// Reg58 for 4Kx2K Video output Driving Strength


#ifdef _SUPPORT_MANUAL_ADJUST_EQ_
	{REG_RX_03E, 0x20, 0x20},	// Enable OvWrRsCs
	{REG_RX_026, 0x20, 0x20},	// Enable OvWrRsCs
#endif


#ifdef _ONLY_SUPPORT_MANUAL_EQ_ADJUST_
	{REG_RX_026,	0xFF,	0x20},	//Reg26=0x00 disable Auto Trigger
	{REG_RX_03E,	0xFF,	0x20},	//Reg3E=0x00 disable Auto Trigger
#endif

//RS initial valie
// 2013/06/06 added by jau-chih.tseng@ite.com.tw
// Dr. Liu said, reg25/reg3D should set as 0x1F for auto EQ start option.
	{REG_RX_025, 0xFF, DEFAULT_EQVALUE},
	{REG_RX_03D, 0xFF, DEFAULT_EQVALUE},
//~jau-chih.tseng@ite.com.tw
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
//FIX_ID_001 xxxxx


//	{REG_RX_014,0xFF,0xFF},		//for enable interrupt output Pin
//	{REG_RX_063,0xFF,0x3F},		//for enable interrupt output Pin MZY 17/5/5
	{REG_RX_073, 0x08, 0x00},		// for HDCPIntKey = false

	{REG_RX_060, 0x40, 0x00},		// disable interrupt mask for NoGenPkt_Rcv

	{REG_RX_02A, 0x01, 0x00},		// disable PORT 0 EnIPLockChk
	{REG_RX_042, 0x01, 0x00},		// disable PORT 1 EnIPLockChk
	//{REG_RX_035, 0x02, 0x00},		// disable PORT 0 EnIPLockChk
	//{REG_RX_04B, 0x02, 0x00},

	{REG_RX_077, 0x0C, 0x08},		// Reg77[3:2] = 01	Audio lock method select
	{0xFF, 0xFF, 0xFF},
};

//FIX_ID_036	xxxxx

static _CODE unsigned char bCSCMtx_RGB2YUV_ITU601_16_235[] = {
	0x00,		0x80,		0x10,
	0xB2, 0x04,	0x65, 0x02,	0xE9, 0x00,
	0x93, 0x3C,	0x18, 0x04,	0x55, 0x3F,
	0x49, 0x3D,	0x9F, 0x3E,	0x18, 0x04
};

static _CODE unsigned char bCSCMtx_RGB2YUV_ITU601_0_255[] = {
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


// static _CODE unsigned char bCSCMtx_YUV2RGB_ITU601_16_235[] = {
// 	0x00,		0x00,		0x00,
// 	0x00, 0x08,	0x6B, 0x3A,	0x50, 0x3D,
// 	0x00, 0x08,	0xF5, 0x0A,	0x02, 0x00,
// 	0x00, 0x08,	0xFD, 0x3F,	0xDA, 0x0D
// } ;

// static _CODE unsigned char bCSCMtx_YUV2RGB_ITU601_0_255[] = {
// 	0x04,		0x00,		0xA7,
// 	0x4F, 0x09,	0x81, 0x39,	0xDD, 0x3C,
// 	0x4F, 0x09,	0xC4, 0x0C,	0x01, 0x00,
// 	0x4F, 0x09,	0xFD, 0x3F,	0x1F, 0x10
// } ;

// static _CODE unsigned char bCSCMtx_YUV2RGB_ITU709_16_235[] = {
// 	0x00,		0x00,		0x00,
// 	0x00, 0x08,	0x55, 0x3C,	0x88, 0x3E,
// 	0x00, 0x08,	0x51, 0x0C,	0x00, 0x00,
// 	0x00, 0x08,	0x00, 0x00,	0x84, 0x0E
// } ;

// static _CODE unsigned char bCSCMtx_YUV2RGB_ITU709_0_255[] = {
// 	0x04,		0x00,		0xA7,
// 	0x4F, 0x09,	0xBA, 0x3B,	0x4B, 0x3E,
// 	0x4F, 0x09,	0x57, 0x0E,	0x02, 0x00,
// 	0x4F, 0x09,	0xFE, 0x3F,	0xE8, 0x10
// } ;

//FIX_ID_027 xxxxx Support Full/Limited Range convert
//full 2 limit
// static _CODE unsigned char bCSCMtx_RGB_0_255_RGB_16_235[] = {
// 	0x10,		0x10,		0x00,
// 	0xe0, 0x06,	0x00, 0x00,	0x00, 0x00,
// 	0x00, 0x00,	0xe0, 0x06,	0x00, 0x00,
// 	0x00, 0x00,	0x00, 0x00,	0xe0, 0x06,


// } ;

//limit 2 full
// static _CODE unsigned char bCSCMtx_RGB_16_235_RGB_0_255[] = {
// 	0xED,		0xED,		0x00,
// 	0x50, 0x09,	0x00, 0x00,	0x00, 0x00,
// 	0x00, 0x00,	0x50, 0x09,	0x00, 0x00,
// 	0x00, 0x00,	0x00, 0x00,	0x50, 0x09,
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

unsigned char _CODE Default_Edid_Block[256] = {
#if (EDID_SELECT_TABLE == 0)
// 0 for IT6602 4K2k  EDID
// 0       1       2       3       4        5       6       7       8       9       A        B       C      D       E       F
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x26, 0x85, 0x02, 0x66, 0x01, 0x01, 0x01, 0x01,	// 00h: 8 ~B  Vendor ID
	0x21, 0x17, 0x01, 0x03, 0x80, 0x55, 0x30, 0x78, 0x2A, 0x63, 0xBD, 0xA1, 0x54, 0x52, 0x9E, 0x26,	// 10h:
	0x0C, 0x47, 0x4A, 0x20, 0x08, 0x00, 0x81, 0x80, 0xD1, 0xC0, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,	// 20h:
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x04, 0x74, 0x00, 0x30, 0xF2, 0x70, 0x5A, 0x80, 0xB0, 0x58,	// 30h: 36 ~ 47 (4Kx2K)
	0x8A, 0x00, 0xA2, 0x0B, 0x32, 0x00, 0x00, 0x1E, 0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0, 0x1E, 0x20,	// 40h: 48 ~ 59 (720p60Hz)
	0x6E, 0x28, 0x55, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0xFD, 0x00, 0x18,
	0x4C, 0x1E, 0x53, 0x1E, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFC,
	0x00, 0x49, 0x54, 0x45, 0x36, 0x38, 0x30, 0x32, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x8B,
	0x02, 0x03, 0x25, 0xF1, 0x43, 0x84, 0x10, 0x03, 0x23, 0x09, 0x07, 0x07, 0x83, 0x01, 0x00, 0x00,
	0xE2, 0x00, 0x0F, 0xE3, 0x05, 0x03, 0x01, 0x6D, 0x03, 0x0C, 0x00, 0x10, 0x00, 0x38, 0x3C, 0x20,	// 0x38 for support 36bit , 0x3C for support 300MHz TMDS
	0x00, 0x60, 0x03, 0x02, 0x01, 0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0, 0x1E, 0x20, 0x6E, 0x28, 0x55,
	0x00, 0xA0, 0x5A, 0x00, 0x00, 0x00, 0x1E, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10,
	0x3E, 0x96, 0x00, 0xA0, 0x5A, 0x00, 0x00, 0x00, 0x18, 0xF3, 0x39, 0x80, 0x18, 0x71, 0x38, 0x2D,
	0x40, 0x58, 0x2C, 0x45, 0x00, 0xE0, 0x0E, 0x11, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15

#elif (EDID_SELECT_TABLE == 1)
// 1 for IT6602 3D EDID
// 0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x26, 0x85, 0x02, 0x66, 0x01, 0x00, 0x00, 0x00,
	0x0C, 0x14, 0x01, 0x03, 0x80, 0x1C, 0x15, 0x78, 0x0A, 0x1E, 0xAC, 0x98, 0x59, 0x56, 0x85, 0x28,
	0x29, 0x52, 0x57, 0x20, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E,
	0x96, 0x00, 0xFA, 0xBE, 0x00, 0x00, 0x00, 0x18, 0xD5, 0x09, 0x80, 0xA0, 0x20, 0xE0, 0x2D, 0x10,
	0x10, 0x60, 0xA2, 0x00, 0xFA, 0xBE, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x49,
	0x54, 0x45, 0x36, 0x38, 0x30, 0x32, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFD,
	0x00, 0x17, 0x3D, 0x0D, 0x2E, 0x11, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x1B,

	0x02, 0x03, 0x30, 0xF1, 0x43, 0x84, 0x10, 0x03, 0x23, 0x09, 0x07, 0x07, 0x83,
	0x01, 0x00, 0x00, 0xE2, 0x00, 0x0F, 0xE3, 0x05, 0x03, 0x01, 0x78, 0x03, 0x0C, 0x00, 0x10, 0x00,
	0x88, 0x2D, 0x20, 0xC0, 0x0E, 0x01, 0x00, 0x00, 0x12, 0x18, 0x20, 0x28, 0x20, 0x38, 0x20, 0x58,
	0x20, 0x68, 0x20, 0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0, 0x1E, 0x20, 0x6E, 0x28, 0x55, 0x00, 0xA0,
	0x5A, 0x00, 0x00, 0x00, 0x1E,
	0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10,
	0x3E, 0x96, 0x00, 0xA0, 0x5A, 0x00, 0x00, 0x00, 0x18, 0xF3, 0x39, 0x80, 0x18, 0x71, 0x38, 0x2D,
	0x40, 0x58, 0x2C, 0x45, 0x00, 0xE0, 0x0E, 0x11, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6B


#elif (EDID_SELECT_TABLE == 2)
//Philips Monitor 4kX2K EDID for test MHL 3D request function
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x41, 0x0C, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01,
	0x21, 0x17, 0x01, 0x03, 0x80, 0x55, 0x30, 0x78, 0x2A, 0x63, 0xBD, 0xA1, 0x54, 0x52, 0x9E, 0x26,
	0x0C, 0x47, 0x4A, 0xA3, 0x08, 0x00, 0x81, 0x80, 0xD1, 0xC0, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x04, 0x74, 0x00, 0x30, 0xF2, 0x70, 0x5A, 0x80, 0xB0, 0x58,
	0x8A, 0x00, 0xA2, 0x0B, 0x32, 0x00, 0x00, 0x1E, 0x1B, 0x21, 0x50, 0xA0, 0x51, 0x00, 0x1E, 0x30,
	0x48, 0x88, 0x35, 0x00, 0xA2, 0x0B, 0x32, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x00, 0xFD, 0x00, 0x18,
	0x4C, 0x1E, 0x53, 0x1E, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFC,
	0x00, 0x50, 0x68, 0x69, 0x6C, 0x69, 0x70, 0x73, 0x20, 0x54, 0x56, 0x0A, 0x20, 0x20, 0x01, 0xDF,

	0x02, 0x03, 0x35, 0xF1, 0x4C, 0x90, 0x1F, 0x21, 0x20, 0x05, 0x14, 0x04, 0x13, 0x03, 0x07, 0x12,
	0x16, 0x26, 0x09, 0x07, 0x03, 0x11, 0x06, 0x00, 0x83, 0x01, 0x00, 0x00, 0x78, 0x03, 0x0C, 0x00,
	0x30, 0x00, 0x00, 0x3C, 0x20, 0xC8, 0x8A, 0x01, 0x03, 0x02, 0x04, 0x01, 0x41, 0x00, 0xCF, 0x66,
	0x68, 0x10, 0x56, 0x58, 0x80, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E, 0x96,
	0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xA0, 0x14, 0x51, 0xF0, 0x16, 0x00, 0x26,
	0x7C, 0x43, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x98, 0x8C, 0x0A, 0xD0, 0x90, 0x20, 0x40, 0x31,
	0x20, 0x0C, 0x40, 0x55, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xA0, 0x20, 0x51,
	0x20, 0x18, 0x10, 0x18, 0x7E, 0x23, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x98, 0x00, 0x00, 0x24,

#elif (EDID_SELECT_TABLE == 3)
//AOC monitor EDID (without 3D)
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x05, 0xE3, 0x69, 0x23, 0x66, 0x00, 0x00, 0x00,
	0x0D, 0x17, 0x01, 0x03, 0x80, 0x33, 0x1D, 0x78, 0x2A, 0xE5, 0x95, 0xA6, 0x56, 0x52, 0x9D, 0x27,
	0x10, 0x50, 0x54, 0xBF, 0xEF, 0x00, 0xD1, 0xC0, 0xB3, 0x00, 0x95, 0x00, 0x81, 0x80, 0x81, 0x40,
	0x81, 0xC0, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3A, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40, 0x58, 0x2C,
	0x45, 0x00, 0xFD, 0x1E, 0x11, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0xFD, 0x00, 0x32, 0x4C, 0x1E,
	0x53, 0x11, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x32,
	0x33, 0x36, 0x39, 0x4D, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFF,
	0x00, 0x42, 0x46, 0x44, 0x44, 0x33, 0x39, 0x41, 0x30, 0x30, 0x30, 0x31, 0x30, 0x32, 0x01, 0x19,

	0x02, 0x03, 0x1F, 0xF1, 0x4C, 0x10, 0x1F, 0x05, 0x14, 0x04, 0x13, 0x03, 0x12, 0x02, 0x11, 0x01,
	0x22, 0x23, 0x09, 0x07, 0x07, 0x83, 0x01, 0x00, 0x00, 0x65, 0x03, 0x0C, 0x00, 0x10, 0x00, 0x8C,
	0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E, 0x96, 0x00, 0xFD, 0x1E, 0x11, 0x00, 0x00,
	0x18, 0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0, 0x1E, 0x20, 0x6E, 0x28, 0x55, 0x00, 0xFD, 0x1E, 0x11,
	0x00, 0x00, 0x1E, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E, 0x96, 0x00, 0xFD,
	0x1E, 0x11, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0, 0x90, 0x20, 0x40, 0x31, 0x20, 0x0C, 0x40, 0x55,
	0x00, 0xFD, 0x1E, 0x11, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x59,

#elif (EDID_SELECT_TABLE == 4)
// Astro 1831 EDID
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x06, 0x8F, 0x12, 0xB0, 0x01, 0x00, 0x00, 0x00,
	0x0C, 0x14, 0x01, 0x03, 0x80, 0x1C, 0x15, 0x78, 0x0A, 0x1E, 0xAC, 0x98, 0x59, 0x56, 0x85, 0x28,
	0x29, 0x52, 0x57, 0x20, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E,
	0x96, 0x00, 0xFA, 0xBE, 0x00, 0x00, 0x00, 0x18, 0xD5, 0x09, 0x80, 0xA0, 0x20, 0xE0, 0x2D, 0x10,
	0x10, 0x60, 0xA2, 0x00, 0xFA, 0xBE, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x56,
	0x41, 0x2D, 0x31, 0x38, 0x33, 0x31, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFD,
	0x00, 0x17, 0x3D, 0x0D, 0x2E, 0x11, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0xFA,

	0x02, 0x03, 0x33, 0xF1, 0x46, 0x90, 0x04, 0x05, 0x03, 0x20, 0x22, 0x23, 0x09, 0x07, 0x07, 0x83,
	0x01, 0x00, 0x00, 0xE2, 0x00, 0x0F, 0xE3, 0x05, 0x03, 0x01, 0x78, 0x03, 0x0C, 0x00, 0x10, 0x00,
	0x88, 0x2D, 0x20, 0xC0, 0x0E, 0x01, 0x00, 0x00, 0x12, 0x18, 0x20, 0x28, 0x20, 0x38, 0x20, 0x58,
	0x20, 0x68, 0x20, 0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0, 0x1E, 0x20, 0x6E, 0x28, 0x55, 0x00, 0xA0,
	0x5A, 0x00, 0x00, 0x00, 0x1E, 0x01, 0x1D, 0x80, 0x18, 0x71, 0x1C, 0x16, 0x20, 0x58, 0x2C, 0x25,
	0x00, 0xA0, 0x5A, 0x00, 0x00, 0x00, 0x9E, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10,
	0x3E, 0x96, 0x00, 0xA0, 0x5A, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF4

#elif (EDID_SELECT_TABLE == 5)
// TI PICO 343X EDID(864x480) + IT66021 MONITOR (720P) + gamma 2.6
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,   // address 0x00
	0x11, 0x90, 0x50, 0x50, 0x01, 0x00, 0x00, 0x00,   //
	0x08, 0x13, 0x01, 0x03, 0x80, 0x34, 0x20, 0xA0,   // address 0x10
	0x2A, 0xEE, 0x95, 0xA3, 0x54, 0x4C, 0x99, 0x26,   //
	0x0F, 0x50, 0x54, 0x21, 0x08, 0x00, 0x01, 0x01,   // address 0x20
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,   //
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x1D,   // address 0x30
	0x00, 0x72, 0x51, 0xD0, 0x1E, 0x20, 0x6E, 0x28,   //
	0x55, 0x00, 0x00, 0xD0, 0x52, 0x00, 0x00, 0x1E,   // address 0x40
	0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10,   //
	0x10, 0x3E, 0x96, 0x00, 0xD0, 0xE0, 0x21, 0x00,   // address 0x50
	0x00, 0x1E, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x49,   //
	0x54, 0x45, 0x36, 0x38, 0x30, 0x31, 0x20, 0x0A,   // address 0x60
	0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFD,   //
	0x00, 0x30, 0x7A, 0x1F, 0x41, 0x0F, 0x00, 0x0A,   // address 0x70
	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x95,   //
	0x02, 0x03, 0x1B, 0x41, 0x83, 0x01, 0x00, 0x00,   // address 0x80
	0x65, 0x03, 0x0C, 0x00, 0x10, 0x00, 0x48, 0x84,   //
	0x02, 0x03, 0x11, 0x12, 0x13, 0x2A, 0x30, 0x23,   // address 0x90
	0x09, 0x07, 0x07, 0x16, 0x0D, 0x60, 0x6A, 0x30,   //
	0xE0, 0x5F, 0x10, 0x04, 0x28, 0xFF, 0x07, 0x60,   // address 0xA0
	0xE0, 0x31, 0x00, 0x00, 0x1E, 0x44, 0x0C, 0x56,   //
	0x36, 0x30, 0xE0, 0x60, 0x10, 0xEA, 0x28, 0x0F,   // address 0xB0
	0xC3, 0x56, 0xE0, 0x31, 0x00, 0x00, 0x1E, 0x47,   //
	0x09, 0x80, 0x30, 0x20, 0xE0, 0x5F, 0x10, 0xE4,   // address 0xC0
	0x28, 0xFF, 0xCF, 0x80, 0xE0, 0x21, 0x00, 0x00,   //
	0x3E, 0x9E, 0x20, 0x00, 0x90, 0x51, 0x20, 0x1F,   // address 0xD0
	0x30, 0x48, 0x80, 0x36, 0x00, 0x00, 0x20, 0x53,   //
	0x00, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00,   // address 0xE0
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   //
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // address 0xF0
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x69,   //

#elif (EDID_SELECT_TABLE == 6)
// IT6602 with 640x480p , 720x480p , 1280x720p , 1920x1080p
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,   // address 0x00
	0x26, 0x85, 0x02, 0x66, 0x21, 0x60, 0x00, 0x00,   //
	0x00, 0x17, 0x01, 0x03, 0x80, 0x73, 0x41, 0x78,   // address 0x10
	0x2A, 0x7C, 0x11, 0x9E, 0x59, 0x47, 0x9B, 0x27,   //
	0x10, 0x50, 0x54, 0x00, 0x00, 0x00, 0x01, 0x01,   // address 0x20
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,   //
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3A,   // address 0x30
	0x80, 0x18, 0x71, 0x38, 0x2D, 0x40, 0x58, 0x2C,   //
	0x45, 0x00, 0x10, 0x09, 0x00, 0x00, 0x00, 0x1E,   // address 0x40
	0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10,   //
	0x10, 0x3E, 0x96, 0x00, 0x04, 0x03, 0x00, 0x00,   // address 0x50
	0x00, 0x18, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x49,   //
	0x54, 0x45, 0x36, 0x38, 0x30, 0x32, 0x0A, 0x20,   // address 0x60
	0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFD,   //
	0x00, 0x30, 0x7A, 0x0F, 0x50, 0x10, 0x00, 0x0A,   // address 0x70
	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0xF3,   //
	0x02, 0x03, 0x19, 0x72, 0x46, 0x90, 0x04, 0x13,   // address 0x80
	0x01, 0x02, 0x03, 0x23, 0x09, 0x07, 0x07, 0x83,   //
	0x01, 0x00, 0x00, 0x65, 0x03, 0x0C, 0x00, 0x10,   // address 0x90
	0x00, 0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0, 0x1E,   //
	0x20, 0x6E, 0x28, 0x55, 0x00, 0x10, 0x09, 0x00,   // address 0xA0
	0x00, 0x00, 0x1E, 0xD6, 0x09, 0x80, 0xA0, 0x20,   //
	0xE0, 0x2D, 0x10, 0x10, 0x60, 0xA2, 0x00, 0x04,   // address 0xB0
	0x03, 0x00, 0x00, 0x00, 0x18, 0x8C, 0x0A, 0xD0,   //
	0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3E, 0x96,   // address 0xC0
	0x00, 0x10, 0x09, 0x00, 0x00, 0x00, 0x18, 0x00,   //
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // address 0xD0
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   //
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // address 0xE0
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   //
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // address 0xF0
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7B,   //
#elif (EDID_SELECT_TABLE == 7)
// IT6602 with 640x480p , 720x480p , 1280x720p , 1920x1080p , 1440x480i
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,   // address 0x00
	0x26, 0x85, 0x02, 0x68, 0x01, 0x68, 0x00, 0x00,   //
	0x00, 0x17, 0x01, 0x03, 0x80, 0x73, 0x41, 0x78,   // address 0x10
	0x2A, 0x7C, 0x11, 0x9E, 0x59, 0x47, 0x9B, 0x27,   //
	0x10, 0x50, 0x54, 0x00, 0x00, 0x00, 0x01, 0x01,   // address 0x20
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,   //
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3A,   // address 0x30
	0x80, 0x18, 0x71, 0x38, 0x2D, 0x40, 0x58, 0x2C,   //
	0x45, 0x00, 0x10, 0x09, 0x00, 0x00, 0x00, 0x1E,   // address 0x40
	0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10,   //
	0x10, 0x3E, 0x96, 0x00, 0x04, 0x03, 0x00, 0x00,   // address 0x50
	0x00, 0x18, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x49,   //
	0x54, 0x45, 0x36, 0x38, 0x30, 0x32, 0x0A, 0x20,   // address 0x60
	0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFD,   //
	0x00, 0x30, 0x7A, 0x0F, 0x50, 0x10, 0x00, 0x0A,   // address 0x70
	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0xF3,   //
	0x02, 0x03, 0x1B, 0x72, 0x48, 0x90, 0x04, 0x13,   // address 0x80
	0x01, 0x02, 0x03, 0x06, 0x07, 0x23, 0x09, 0x07,   //
	0x07, 0x83, 0x01, 0x00, 0x00, 0x65, 0x03, 0x0C,   // address 0x90
	0x00, 0x10, 0x00, 0x01, 0x1D, 0x00, 0x72, 0x51,   //
	0xD0, 0x1E, 0x20, 0x6E, 0x28, 0x55, 0x00, 0x10,   // address 0xA0
	0x09, 0x00, 0x00, 0x00, 0x1E, 0xD6, 0x09, 0x80,   //
	0xA0, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x60, 0xA2,   // address 0xB0
	0x00, 0x04, 0x03, 0x00, 0x00, 0x00, 0x18, 0x8C,   //
	0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10,   // address 0xC0
	0x3E, 0x96, 0x00, 0x10, 0x09, 0x00, 0x00, 0x00,   //
	0x18, 0x8C, 0x0A, 0xA0, 0x14, 0x51, 0xF0, 0x16,   // address 0xD0
	0x00, 0x26, 0x7C, 0x43, 0x00, 0xD0, 0xE0, 0x21,   //
	0x00, 0x00, 0x98, 0x00, 0x00, 0x00, 0x00, 0x00,   // address 0xE0
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   //
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // address 0xF0
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7B,   //
#elif (EDID_SELECT_TABLE == 8)
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
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39,
#elif (EDID_SELECT_TABLE == 9)
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x05, 0xE3, 0x79, 0x24, 0x18, 0x05, 0x00, 0x00,
	0x2B, 0x19, 0x01, 0x03, 0x80, 0x34, 0x1D, 0x78, 0x2A, 0xEE, 0xD5, 0xA5, 0x55, 0x48, 0x9B, 0x26,
	0x12, 0x50, 0x54, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x81, 0xC0, 0x01, 0x01, 0x01, 0x01, 0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0, 0x1E, 0x20, 0x6E, 0x28,
	0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0xFD, 0x00, 0x32, 0x4C, 0x1E,
	0x53, 0x11, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x32,
	0x34, 0x37, 0x39, 0x57, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFF,
	0x00, 0x4A, 0x4A, 0x52, 0x46, 0x41, 0x4A, 0x41, 0x30, 0x30, 0x31, 0x33, 0x30, 0x34, 0x01, 0xEC,
	0x02, 0x03, 0x0C, 0x30, 0x41, 0x04, 0x65, 0x03, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x72,
	0x51, 0xD0, 0x1E, 0x20, 0x6E, 0x28, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2B,
#endif
};
#endif


_CODE char const *VStateStr[] = {
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
	"VSTATE_Reserved"
};

_CODE char const *AStateStr[] = {
	"ASTATE_AudioOff",
	"ASTATE_RequestAudio",
	"ASTATE_ResetAudio",
	"ASTATE_WaitForReady",
	"ASTATE_AudioOn",
	"ASTATE_Reserved"
};


_CODE char const *VModeStateStr[] = {
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



unsigned char	m_MHLabortID = 0x00;

#define MHL_ABORT_ID	(0x00)


/*****************************************************************************/
/*  Function Prototypes    **************************************************/
/*****************************************************************************/

/* ITEHDMI IO Functions   ***********************************************************/
static SYS_STATUS EDID_RAM_Write(unsigned char offset, unsigned char byteno, _CODE unsigned char *p_data);
static unsigned char EDID_RAM_Read(unsigned char offset);
static unsigned char hdmirxrd(unsigned char RegAddr);
static unsigned char hdmirxwr(unsigned char RegAddr, unsigned char DataIn);
static unsigned char  hdmirxset(unsigned char  offset, unsigned char  mask, unsigned char  ucdata);
static void hdmirxbwr(unsigned char offset, unsigned char byteno, _CODE unsigned char *rddata);


/* ITEHDMI Configuration and Initialization ***********************************/
struct it6602_dev_data *get_it6602_dev_data(void);


static void IT6602_VideoOutputConfigure_Init(struct it6602_dev_data *it6602, Video_Output_Configure eVidOutConfig);


static void hdmirx_Var_init(struct it6602_dev_data *it6602);
static void IT6602_Rst(struct it6602_dev_data *it6602);

static char IT6602_Identify_Chip(void);

/* HDMI RX functions   *********************************************************/
static void chgbank(int bank);
static unsigned char CheckSCDT(struct it6602_dev_data *it6602);
static void WaitingForSCDT(struct it6602_dev_data *it6602);
static unsigned char CLKCheck(unsigned char ucPortSel);

static unsigned char  IT6602_IsSelectedPort(unsigned char ucPortSel);


//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_EQ_ADJUST_
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
static void HDMIStartEQDetect(struct it6602_eq_data *ucEQPort);
static void HDMISetEQValue(struct it6602_eq_data *ucEQPort, unsigned char ucIndex);
static void HDMISwitchEQstate(struct it6602_eq_data *ucEQPort, unsigned char state);
static void HDMICheckSCDTon(struct it6602_eq_data *ucEQPort);
static void HDMIPollingErrorCount(struct it6602_eq_data *ucEQPort);
static void HDMIJudgeECCvalue(struct it6602_eq_data *ucEQPort);
static void HDMIAdjustEQ(struct it6602_eq_data *ucEQPort);

void JudgeBestEQ(struct it6602_eq_data *ucEQPort);
static void StoreEccCount(struct it6602_eq_data *ucEQPort);

static void IT6602VideoCountClr(void);
//-------------------------------------------------------------------------------------------------------
#endif

#ifdef _SUPPORT_AUTO_EQ_
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
static void DisableOverWriteRS(unsigned char ucPortSel);
static void AmpValidCheck(unsigned char ucPortSel);
static void TogglePolarity(unsigned char ucPortSel);
static void TMDSCheck(unsigned char ucPortSel);
static void OverWriteAmpValue2EQ(unsigned char ucPortSel);
//-------------------------------------------------------------------------------------------------------
#endif

static unsigned char CheckAVMute(void);

static unsigned char CheckPlg5VPwr(unsigned char ucPortSel);

static unsigned char IsHDMIMode(void);
static void GetAVIInfoFrame(struct it6602_dev_data *it6602);
static void SetVideoInputFormatWithInfoFrame(struct it6602_dev_data *it6602);
static void SetColorimetryByInfoFrame(struct it6602_dev_data *it6602);
static void SetCSCBYPASS(struct it6602_dev_data *it6602);
static void SetColorSpaceConvert(struct it6602_dev_data *it6602);
static void SetNewInfoVideoOutput(struct it6602_dev_data *it6602);
static void SetVideoInputFormatWithoutInfoFrame(struct it6602_dev_data *it6602, unsigned char bInMode);
static void SetColorimetryByMode(struct it6602_dev_data *it6602);
static void SetDVIVideoOutput(struct it6602_dev_data *it6602);

static void IT6602_VideoOutputModeSet(struct it6602_dev_data *it6602);

static void IT6602VideoOutputConfigure(struct it6602_dev_data *it6602);
static void SetVideoOutputColorFormat(struct it6602_dev_data *it6602);
//void it6602PortSelect(unsigned char ucPortSel);

static void hdmirx_ECCTimingOut(unsigned char ucport);

/* HDMI Audio function    *********************************************************/
static void aud_fiforst(void);
static void IT6602AudioOutputEnable(unsigned char bEnable);
static void hdmirx_ResetAudio(void);
static void hdmirx_SetHWMuteClrMode(void);
static void hdmirx_SetHWMuteClr(void);
static void hdmirx_ClearHWMuteClr(void);
static void getHDMIRXInputAudio(AUDIO_CAPS *pAudioCaps);
static void IT6602SwitchAudioState(struct it6602_dev_data *it6602, Audio_State_Type state);

static void IT6602AudioHandler(struct it6602_dev_data *it6602);

/* HDMI Video function    *********************************************************/
static void IT6602_AFE_Rst(void);
static void IT6602_HDCP_ContentOff(unsigned char ucPort, unsigned char bOff);
static void IT6602_RAPContentOff(unsigned char bOff);
static void IT6602_SetVideoMute(struct it6602_dev_data *it6602, unsigned char bMute);
static void IT6602VideoOutputEnable(unsigned char bEnableOutput);
static void IT6602VideoCountClr(void);
static void IT6602SwitchVideoState(struct it6602_dev_data *it6602, Video_State_Type  eNewVState);
static void IT6602VideoHandler(struct it6602_dev_data *it6602);


/* HDMI Interrupt function    *********************************************************/
static void hdmirx_INT_5V_Pwr_Chg(struct it6602_dev_data *it6602, unsigned char ucport);
static void hdmirx_INT_P0_ECC(struct it6602_dev_data *it6602);
static void hdmirx_INT_P1_ECC(struct it6602_dev_data *it6602);
static void hdmirx_INT_P0_Deskew(struct it6602_dev_data *it6602);
static void hdmirx_INT_P1_Deskew(struct it6602_dev_data *it6602);
static void hdmirx_INT_HDMIMode_Chg(struct it6602_dev_data *it6602, unsigned char ucport);
static void hdmirx_INT_SCDT_Chg(struct it6602_dev_data *it6602);

#ifdef _SUPPORT_AUTO_EQ_
static void hdmirx_INT_EQ_FAIL(struct it6602_dev_data *it6602, unsigned char ucPortSel);
#endif


/* EDID RAM  functions    *******************************************************/
#ifdef _SUPPORT_EDID_RAM_
//static unsigned char UpdateEDIDRAM(_CODE unsigned char *pEDID,unsigned char BlockNUM);
static unsigned char UpdateEDIDRAM(unsigned char *pEDID, unsigned char BlockNUM);
static void EnableEDIDupdata(void);
static void DisableEDIDupdata(void);
//static void EDIDRAMInitial(_CODE unsigned char *pIT6602EDID);
static void EDIDRAMInitial(unsigned char *pIT6602EDID);
//static unsigned char Find_Phyaddress_Location(_CODE unsigned char *pEDID,unsigned char Block_Number);
static unsigned char Find_Phyaddress_Location(unsigned char *pEDID, unsigned char Block_Number);
static void UpdateEDIDReg(unsigned char u8_VSDB_Addr, unsigned char CEC_AB, unsigned char CEC_CD,
			  unsigned char Block1_CheckSum);
static void PhyAdrSet(void);
#endif

static void IT6602HDMIInterruptHandler(struct it6602_dev_data *it6602);

#ifndef Enable_IR
static void it6602AutoPortSelect(struct it6602_dev_data *it6602);
#endif

#ifdef Enable_Vendor_Specific_packet
static void Dump3DReg(void);
static unsigned char IT6602_DE3DFrame(unsigned char ena_de3d);
#endif

/*****************************************************************************/
/*  IO Functions   ***********************************************************/
/*****************************************************************************/

static SYS_STATUS EDID_RAM_Write(unsigned char offset, unsigned char byteno, _CODE unsigned char *p_data)
{
	p_it66021a_edid->write(offset, p_data, byteno);
	return ER_SUCCESS;
}

static unsigned char EDID_RAM_Read(unsigned char offset)
{
	return p_it66021a_edid->readbyte(offset);
}


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
	unsigned char  temp;
	temp = hdmirxrd(offset);
	temp = (temp & ((~mask) & 0xFF)) + (mask & ucdata);
	return hdmirxwr(offset, temp);
}

static void hdmirxbwr(unsigned char offset, unsigned char byteno, _CODE unsigned char *wrdata)
{
	p_it66021a->write(offset, wrdata, byteno);
}

static unsigned char mhlrxrd(unsigned char offset)
{
	return p_it66021a_mhl->readbyte(offset);
}

static unsigned char mhlrxwr(unsigned char offset, unsigned char ucdata)
{
	return p_it66021a_mhl->writebyte(offset, ucdata);
}


/*****************************************************************************/
/* ITEHDMI Configuration and Initialization ***********************************/
/*****************************************************************************/
#ifdef _ITEHDMI_
struct it6602_dev_data *get_it6602_dev_data(void)
{
	return &it66021_A;
}


void hdimrx_write_init(struct IT6602_REG_INI _CODE *tdata)
{
	int cnt = 0;

	while (tdata[cnt].ucAddr != 0xFF) {
		hdmirxset(tdata[cnt].ucAddr, tdata[cnt].andmask, tdata[cnt].ucValue);
		cnt++;
	}
}


static void IT6602_VideoOutputConfigure_Init(struct it6602_dev_data *it6602, Video_Output_Configure eVidOutConfig)
{
	it6602->m_VidOutConfigMode = eVidOutConfig;

	switch (eVidOutConfig) {
	case eRGB444_SDR:
		it6602->m_bOutputVideoMode = F_MODE_RGB444;

		it6602->m_bOutputVideoMode = F_MODE_RGB444 | F_MODE_0_255;    // ITEHDMI output RGB Full Range

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

	case eBTA1004_SDR:	//BTA1004_SDR_Emb_Sync
		it6602->m_bOutputVideoMode = F_MODE_YUV422;
		it6602->m_VidOutDataTrgger = eSDR_BTA1004;
		it6602->m_VidOutSyncMode = eEmbSync;
		break;

	case eBTA1004_DDR:  //BTA1004_DDR_Emb_Sync
		it6602->m_bOutputVideoMode = F_MODE_YUV422;
		it6602->m_VidOutDataTrgger = eDDR_BTA1004;		// eHalfPCLKDDR
		it6602->m_VidOutSyncMode = eEmbSync;
		break;

	case eVOMreserve:
		break;
	}
}

////////////////////////////////////////////////////////////////////
//int hdmirx_Var_init( void )
////////////////////////////////////////////////////////////////////
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

	it6602->m_DiscoveryDone = 0;

	it6602->m_RAP_ContentOff = 0;
	it6602->m_HDCP_ContentOff = 0;
}



////////////////////////////////////////////////////////////////////
//void hdmitx_rst( void )
////////////////////////////////////////////////////////////////////
static void IT6602_Rst(struct it6602_dev_data *it6602)
{
	hdmirx_Var_init(it6602);
	hdimrx_write_init(IT6602_HDMI_INIT_TABLE);
}


//=========================================================================//
char IT6602_fsm_init(void)
{
	struct it6602_dev_data *it6602data = get_it6602_dev_data();

	IT6602_Rst(it6602data);
	
	EDIDRAMInitial(&Default_Edid_Block[0]);
	
	// fo IT6803 EDID fail issue
	hdmirxset(REG_RX_0C0, 0x20, 0x20);	//xxxxx 2014-0731 [5] 1 for  reset edid
	usleep(1000);
	hdmirxset(REG_RX_0C0, 0x20, 0x00);

	it6602PortSelect(0);	// select port 0
	return TRUE;
}

#endif


/*****************************************************************************/
/* HDMIRX functions    *******************************************************/
/*****************************************************************************/
#ifdef _ITEHDMI_

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


static unsigned char CheckSCDT(struct it6602_dev_data *it6602)
{
	unsigned char ucPortSel;
	unsigned char sys_state_P0;

	ucPortSel = hdmirxrd(REG_RX_051) & B_PORT_SEL;
	sys_state_P0 = hdmirxrd(REG_RX_P0_SYS_STATUS);

	IT_INFO("CheckSCDT: SEL = %d, po = %02x, curr = %d", ucPortSel, sys_state_P0, it6602->m_ucCurrentHDMIPort);

	if (ucPortSel == it6602->m_ucCurrentHDMIPort) {

		if (sys_state_P0 & B_P0_SCDT) {
			//SCDT on
			//it6602->m_ucSCDTOffCount=0;
			return TRUE;

		} else {
			//SCDT off
			return FALSE;
		}

	}

	return FALSE;
}


static void WaitingForSCDT(struct it6602_dev_data *it6602)
{
	unsigned char sys_state_P0;
	unsigned char sys_state_P1;
	unsigned char ucPortSel;
//	unsigned char ucTMDSClk ;

	sys_state_P0 = hdmirxrd(REG_RX_P0_SYS_STATUS) & (B_P0_SCDT | B_P0_PWR5V_DET | B_P0_RXCK_VALID);
	sys_state_P1 = hdmirxrd(REG_RX_P1_SYS_STATUS) & (B_P1_SCDT | B_P1_PWR5V_DET | B_P1_RXCK_VALID);
	ucPortSel = hdmirxrd(REG_RX_051) & B_PORT_SEL;


	IT_INFO("WaitingForSCDT sys_state_P0 = %02x, ucPortSel = %02x", sys_state_P0, ucPortSel);

	if (sys_state_P0 & B_P0_SCDT) {
		IT6602SwitchVideoState(it6602, VSTATE_SyncChecking);	//2013-0520
		return;

	} else {
#ifdef _SUPPORT_EQ_ADJUST_

		if (it6602->EQPort[ucPortSel].f_manualEQadjust == TRUE) {	// ignore SCDT off when manual EQ adjust !!!
			IT_INFO("[WaitingForSCDT]: f_manualEQadjust = TRUE \n");
			return;
		}

#endif

		if (ucPortSel == F_PORT_SEL_0) {

			if ((sys_state_P0 & (B_P0_PWR5V_DET | B_P0_RXCK_VALID)) == (B_P0_PWR5V_DET | B_P0_RXCK_VALID)) {
				it6602->m_ucSCDTOffCount++;
				IT_INFO(" SCDT off count = %X  ", (int)it6602->m_ucSCDTOffCount);
				IT_INFO(" sys_state_P0 = %X  ", (int)hdmirxrd(REG_RX_P0_SYS_STATUS));

			}

		} else {
			if ((sys_state_P1 & (B_P1_PWR5V_DET | B_P1_RXCK_VALID)) == (B_P1_PWR5V_DET | B_P1_RXCK_VALID)) {
				it6602->m_ucSCDTOffCount++;
				IT_INFO(" SCDT off count = %X  ", (int)it6602->m_ucSCDTOffCount);
				IT_INFO(" sys_state_P1 = %X  ", (int)hdmirxrd(REG_RX_P1_SYS_STATUS));

			}
		}

		if ((it6602->m_ucSCDTOffCount) > SCDT_OFF_TIMEOUT) {
			it6602->m_ucSCDTOffCount = 0;
			IT_INFO(" WaitingForSCDT( ) CDR reset !!!  ");
			hdmirx_ECCTimingOut(ucPortSel);

#ifdef _SUPPORT_AUTO_EQ_

			DisableOverWriteRS(ucPortSel);
			TMDSCheck(ucPortSel);

#endif
		}

	}
}

static unsigned char CLKCheck(unsigned char ucPortSel)
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


//---------------------------------------------------------------------------------------------------
//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_AUTO_EQ_
static void DisableOverWriteRS(unsigned char ucPortSel)
{

	struct it6602_dev_data *it6602data = get_it6602_dev_data();

	if (ucPortSel == F_PORT_SEL_1) {
#ifdef _SUPPORT_AUTO_EQ_
		ucPortAMPOverWrite[F_PORT_SEL_1] = 0;	//2013-0801
		ucPortAMPValid[F_PORT_SEL_1] = 0;
//FIX_ID_035 xxxxx //For MTK6592 HDMI to SII MHL TX compliance issue
//xxxxx 2014-0508 disable -> 		ucEqRetryCnt[F_PORT_SEL_1]=0;
//FIX_ID_035 xxxxx
		ucEQMode[F_PORT_SEL_1] = 0; // 0 for Auto Mode
		hdmirxset(REG_RX_03A, 0xFF, 0x00);	// 07-16 Reg3A=0x30	power down auto EQ
		hdmirxset(REG_RX_03E, 0x20, 0x00);		//Manually set RS Value
		IT_INFO(" ############# DisableOverWriteRS( ) port 1 ###############\n");
#endif
#ifdef _SUPPORT_EQ_ADJUST_
		it6602data->EQPort[1].f_manualEQadjust = FALSE;
		it6602data->EQPort[F_PORT_SEL_1].ucEQState = 0xFF;
#endif
		it6602data->m_ucDeskew_P1 = 0;
		it6602data->m_ucEccCount_P1 = 0;


		//FIX_ID_014 xxxxx
		it6602data->HDMIIntEvent &= 0x0F;;
		it6602data->HDMIWaitNo[F_PORT_SEL_1] = 0;
		//FIX_ID_014 xxxxx

	} else {
#ifdef _SUPPORT_AUTO_EQ_
		ucPortAMPOverWrite[F_PORT_SEL_0] = 0;	//2013-0801
		ucPortAMPValid[F_PORT_SEL_0] = 0;
//FIX_ID_035 xxxxx //For MTK6592 HDMI to SII MHL TX compliance issue
//xxxxx 2014-0508 disable ->		ucEqRetryCnt[F_PORT_SEL_0]=0;
//FIX_ID_035 xxxxx
		ucEQMode[F_PORT_SEL_0] = 0; // 0 for Auto Mode
		hdmirxset(REG_RX_022, 0xFF, 0x00);	// 07-16 Reg22=0x30	power down auto EQ
		hdmirxset(REG_RX_026, 0x20, 0x00);		//Manually set RS Value
		IT_INFO(" ############# DisableOverWriteRS( ) port 0 ###############\n");
#endif

#ifdef _SUPPORT_EQ_ADJUST_
		it6602data->EQPort[0].f_manualEQadjust = FALSE;
		it6602data->EQPort[F_PORT_SEL_0].ucEQState = 0xFF;
#endif
		it6602data->m_ucDeskew_P0 = 0;
		it6602data->m_ucEccCount_P0 = 0;

		//FIX_ID_014 xxxxx
		it6602data->HDMIIntEvent &= 0xF0;;
		it6602data->HDMIWaitNo[F_PORT_SEL_0] = 0;
		//FIX_ID_014 xxxxx

	}

}

static void AmpValidCheck(unsigned char ucPortSel)
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

//FIX_ID_010 xxxxx 	//Add JudgeBestEQ to avoid wrong EQ setting
		if ((ucPortAMPValid[1] & 0x3F) == 0x3F) {
			OverWriteAmpValue2EQ(F_PORT_SEL_1);

			//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_EQ_ADJUST_
			HDMIStartEQDetect(&(it6602data->EQPort[F_PORT_SEL_1]));
#endif
			//FIX_ID_001 xxxxx
		}

//FIX_ID_010 xxxxx

	} else {
		chgbank(1);
		uc = hdmirxrd(REG_RX_1D0);
		IT_INFO(" ############# AmpValidCheck( ) port 0 ###############\n");
		IT_INFO(" ############# REG_RX_1D0 = %X ###############\n", (int) uc);
		IT_INFO(" ############# Reg1D4 = %X ###############\n", (int) hdmirxrd(REG_RX_1D4));

		if ((uc & 0x03) == 0x03) {
			ucChannelB[0] = hdmirxrd(REG_RX_1D5);
			ucPortAMPValid[0] |= 0x03;
			IT_INFO(" ############# B AMP VALID port 0 Reg1D5 = 0x%X  ###############\n", (int) ucChannelB[0]);
		}

		if ((uc & 0x0C) == 0x0C) {
			ucChannelG[0] = hdmirxrd(REG_RX_1D6);
			ucPortAMPValid[0] |= 0x0C;
			IT_INFO(" ############# G AMP VALID port 0 Reg1D6 = 0x%X  ###############\n", (int) ucChannelG[0]);
		}

		if ((uc & 0x30) == 0x30) {
			ucChannelR[0] = hdmirxrd(REG_RX_1D7);
			ucPortAMPValid[0] |= 0x30;
			IT_INFO(" ############# R AMP VALID port 0 Reg1D7 = 0x%X  ###############\n", (int) ucChannelR[0]);
		}

		chgbank(0);

		//07-08
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
static void TogglePolarity(unsigned char ucPortSel)
{
#ifdef _SUPPORT_AUTO_EQ_
	unsigned char ucPortSelCurrent;
	ucPortSelCurrent = hdmirxrd(REG_RX_051) & B_PORT_SEL;

#ifdef _ONLY_SUPPORT_MANUAL_EQ_ADJUST_
	return;
#endif

	if (ucPortSelCurrent != ucPortSel) {
		return;
	}

	if (ucPortSel == F_PORT_SEL_1) {
		IT_INFO(" ############# TogglePolarity Port 1###############\n");
		chgbank(1);

		hdmirxset(REG_RX_1C5, 0x10, 0x00);

		if ((hdmirxrd(REG_RX_1B9) & 0x80) >> 7) {
			hdmirxset(REG_RX_1B9, 0x80, 0x00);	// Change Polarity

		} else {
			hdmirxset(REG_RX_1B9, 0x80, 0x80);	// Change Polarity
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

		//xxxxx only for IT6602A0 Version
		if ((hdmirxrd(REG_RX_1B9) & 0x80) >> 7) {
			hdmirxset(REG_RX_1B9, 0x80, 0x00);	// Change Polarity

		} else {
			hdmirxset(REG_RX_1B9, 0x80, 0x80);	// Change Polarity
		}

		//xxxxx

		hdmirxset(REG_RX_1B5, 0x10, 0x10);
		chgbank(0);


		IT_INFO(" ############# TogglePolarity Trigger Port 0 EQ ###############\n");

		hdmirxset(REG_RX_022, 0xFF, 0x38);	//07-04
		hdmirxset(REG_RX_022, 0x04, 0x04);
		hdmirxset(REG_RX_022, 0x04, 0x00);
	}

#endif
}

static void TMDSCheck(unsigned char ucPortSel)
{
	if (ucPortSel != 0) {
		PX4_ERR("[TMDSCheck]: it66021 only support port 0 !! \n");
		return;
	}

#ifdef _SUPPORT_AUTO_EQ_
	unsigned int ucTMDSClk ;
	unsigned char rddata ;
	unsigned char ucClk ;

	struct it6602_dev_data *it6602data = get_it6602_dev_data();

	IT_INFO("TMDSCheck() !!!\n");

	IT_INFO(" HDMI Reg90  = %X ,Reg91  = %X ", (int) hdmirxrd(0x90), (int) hdmirxrd(0x91));
	ucClk = hdmirxrd(REG_RX_091) ;
	rddata = hdmirxrd(0x90);


	if (ucClk != 0) {
		if (rddata & 0x01) {
			ucTMDSClk = 2 * RCLKVALUE * 256 / ucClk;

		} else if (rddata & 0x02) {
			ucTMDSClk = 4 * RCLKVALUE * 256 / ucClk;

		} else {
			ucTMDSClk = RCLKVALUE * 256 / ucClk;
		}

		IT_INFO(" Port 0 TMDS CLK  = %X  ", (int) ucTMDSClk);

		UNUSED(ucTMDSClk);
	}

	IT_INFO(" HDMI Reg020  = %X  ", (int) hdmirxrd(REG_RX_020));


	IT_INFO(" HDMI Reg020  = %X  ucPortAMPOverWrite[0] = %d\r\n", (int)hdmirxrd(REG_RX_020), ucPortAMPOverWrite[0]);


	if (hdmirxrd(REG_RX_P0_SYS_STATUS) & (B_P0_MHL_MODE)) {

		IT_INFO("hdmirxrd(REG_RX_P0_SYS_STATUS) = %d", hdmirxrd(REG_RX_P0_SYS_STATUS));
		chgbank(1);
		hdmirxset(REG_RX_1B8, 0x80, 0x00);	// [7] Reg_HWENHYS = 0
		hdmirxset(REG_RX_1B6, 0x07, 0x00);	// [2:0]Reg_P0_ENHYS = 00 for MHL mode only  [2:0]Reg_P0_ENHYS = 00 for disable ENHYS
		chgbank(0);
	}


	if (ucPortAMPOverWrite[0] == 0 || 1) {	// 2013-0801
		chgbank(1);
		rddata = hdmirxrd(REG_RX_1D4);
		chgbank(0);

		if (rddata == 0)
		{
			IT_INFO(" ############# Trigger Port 0 EQ ###############\n");
			hdmirxset(REG_RX_022, 0xFF, 0x38);	//07-04
			hdmirxset(REG_RX_022, 0x04, 0x04);
			hdmirxset(REG_RX_022, 0x04, 0x00);
		}

		// if Authentication start interrupt with CKon interrupt then do TMDSCheck() at first.
		it6602data->HDMIIntEvent &= ~(B_PORT0_TMDSEvent | B_PORT0_Waiting | B_PORT0_TimingChgEvent);

	} else {
		IT_INFO(" ############# B_PORT0_TimingChgEvent###############\n");
		it6602data->HDMIIntEvent |= (B_PORT0_Waiting);
		it6602data->HDMIIntEvent |= (B_PORT0_TimingChgEvent);
		it6602data->HDMIWaitNo[0] = MAX_TMDS_WAITNO;
	}

#endif
}




static void OverWriteAmpValue2EQ(unsigned char ucPortSel)
{
	struct it6602_dev_data *it6602data = get_it6602_dev_data();

	IT_INFO(" 111111111111111111 OverWriteAmpValue2EQ 111111111111111111111111111111 ");
	IT_INFO(" 111111111111111111 OverWriteAmpValue2EQ 111111111111111111111111111111 ");
	IT_INFO(" 111111111111111111 OverWriteAmpValue2EQ 111111111111111111111111111111 ");
	IT_INFO(" 111111111111111111 OverWriteAmpValue2EQ 111111111111111111111111111111 ");
	IT_INFO(" 111111111111111111 OverWriteAmpValue2EQ 111111111111111111111111111111 ");
	IT_INFO(" 111111111111111111 OverWriteAmpValue2EQ 111111111111111111111111111111 ");


	if (ucPortSel == F_PORT_SEL_1) {
		if ((ucPortAMPValid[1] & 0x3F) == 0x3F) {
			ucPortAMPOverWrite[F_PORT_SEL_1] = 1;	//2013-0801
			ucEQMode[F_PORT_SEL_1] = 0; // 0 for Auto Mode
			IT_INFO("#### REG_RX_03E = 0x%X #### ", (int) hdmirxrd(REG_RX_03E));
			hdmirxset(REG_RX_03E, 0x20, 0x20);	//Manually set RS Value
			IT_INFO("#### REG_RX_03E = 0x%X #### ", (int) hdmirxrd(REG_RX_03E));

			if (ucChannelB[F_PORT_SEL_1]  < MinEQValue) {
				hdmirxwr(REG_RX_03F, MinEQValue);

			} else {
				hdmirxwr(REG_RX_03F, (ucChannelB[F_PORT_SEL_1] & 0x7F));
			}

			if (ucChannelG[F_PORT_SEL_1]  < MinEQValue) {
				hdmirxwr(REG_RX_040, MinEQValue);

			} else {
				hdmirxwr(REG_RX_040, (ucChannelG[F_PORT_SEL_1] & 0x7F));
			}

			if (ucChannelR[F_PORT_SEL_1]  < MinEQValue) {
				hdmirxwr(REG_RX_041, MinEQValue);

			} else {
				hdmirxwr(REG_RX_041, (ucChannelR[F_PORT_SEL_1] & 0x7F));
			}

			//if Auto EQ done  interrupt then clear HDMI Event !!!
			it6602data->HDMIIntEvent &= ~(B_PORT1_TMDSEvent | B_PORT1_Waiting | B_PORT1_TimingChgEvent);

			IT_INFO(" ############# Over-Write port 1 EQ############### ");
			IT_INFO(" ############# B port 1 Reg03F = 0x%X  ############### ", (int) hdmirxrd(REG_RX_03F));
			IT_INFO(" ############# G port 1 Reg040 = 0x%X  ############### ", (int) hdmirxrd(REG_RX_040));
			IT_INFO(" ############# R port 1 Reg041 = 0x%X  ############### ", (int) hdmirxrd(REG_RX_041));

			hdmirxwr(REG_RX_03A, 0x00);	// power down auto EQ
			hdmirxwr(0xD0, 0xC0);

		}

	} else {

		//07-08
		if (hdmirxrd(REG_RX_P0_SYS_STATUS) & (B_P0_MHL_MODE)) {
			if ((ucPortAMPValid[F_PORT_SEL_0] & 0x03) == 0x03) {
				ucPortAMPOverWrite[F_PORT_SEL_0] = 1;	//2013-0801
				ucEQMode[F_PORT_SEL_0] = 0; // 0 for Auto Mode
				IT_INFO("#### REG_RX_026 = 0x%X #### ", (int) hdmirxrd(REG_RX_026));

				hdmirxset(REG_RX_026, 0x20, 0x20);	//Manually set RS Value
				IT_INFO("#### REG_RX_026 = 0x%X #### ", (int) hdmirxrd(REG_RX_026));

				if ((ucChannelB[F_PORT_SEL_0]) < MinEQValue) {
					hdmirxwr(REG_RX_027, (MinEQValue));
					hdmirxwr(REG_RX_028, (MinEQValue));	//07-08 using B channal to over-write G and R channel
					hdmirxwr(REG_RX_029, (MinEQValue));

				} else {
					hdmirxwr(REG_RX_027, (ucChannelB[F_PORT_SEL_0] & 0x7F));
					hdmirxwr(REG_RX_028, (ucChannelB[F_PORT_SEL_0] & 0x7F));	//07-08 using B channal to over-write G and R channel
					hdmirxwr(REG_RX_029, (ucChannelB[F_PORT_SEL_0] & 0x7F));
				}

				IT_INFO(" ############# Over-Write port 0 MHL EQ############### ");
				IT_INFO(" ############# B port 0 REG_RX_027 = 0x%X  ############### ", (int) hdmirxrd(REG_RX_027));
				IT_INFO(" ############# G port 0 REG_RX_028 = 0x%X  ############### ", (int) hdmirxrd(REG_RX_028));
				IT_INFO(" ############# R port 0 REG_RX_029 = 0x%X  ############### ", (int) hdmirxrd(REG_RX_029));

				hdmirxwr(REG_RX_022, 0x00);	// power down auto EQ
				hdmirxwr(0xD0, 0x30);

			}

		} else {
			if ((ucPortAMPValid[F_PORT_SEL_0] & 0x3F) == 0x3F) {
				ucPortAMPOverWrite[F_PORT_SEL_0] = 1;	//2013-0801
				ucEQMode[F_PORT_SEL_0] = 0; // 0 for Auto Mode
				IT_INFO("#### REG_RX_026 = 0x%X #### ", (int) hdmirxrd(REG_RX_026));
				hdmirxset(REG_RX_026, 0x20, 0x20);	//Manually set RS Value
				IT_INFO("#### REG_RX_026 = 0x%X #### ", (int) hdmirxrd(REG_RX_026));

				if (ucChannelB[F_PORT_SEL_0]  < MinEQValue) {
					hdmirxwr(REG_RX_027, MinEQValue);

				} else {
					hdmirxwr(REG_RX_027, (ucChannelB[F_PORT_SEL_0] & 0x7F));
				}

				if (ucChannelG[F_PORT_SEL_0]  < MinEQValue) {
					hdmirxwr(REG_RX_028, MinEQValue);

				} else {
					hdmirxwr(REG_RX_028, (ucChannelG[F_PORT_SEL_0] & 0x7F));
				}

				if (ucChannelR[F_PORT_SEL_0]  < MinEQValue) {
					hdmirxwr(REG_RX_029, MinEQValue);

				} else {
					hdmirxwr(REG_RX_029, (ucChannelR[F_PORT_SEL_0] & 0x7F));
				}

				//if Auto EQ done  interrupt then clear HDMI Event !!!
				it6602data->HDMIIntEvent &= ~(B_PORT0_TMDSEvent | B_PORT0_Waiting | B_PORT0_TimingChgEvent);

				IT_INFO(" ############# Over-Write port 0 EQ############### ");
				IT_INFO(" ############# B port 0 REG_RX_027 = 0x%X  ############### ", (int) hdmirxrd(REG_RX_027));
				IT_INFO(" ############# G port 0 REG_RX_028 = 0x%X  ############### ", (int) hdmirxrd(REG_RX_028));
				IT_INFO(" ############# R port 0 REG_RX_029 = 0x%X  ############### ", (int) hdmirxrd(REG_RX_029));

				//hdmirxset(REG_RX_022, 0xFF, 0x00);	//07-08 [3] power down
				hdmirxwr(REG_RX_022, 0x00);	// power down auto EQ
				hdmirxwr(0xD0, 0x30);

			}
		}

	}
}
//-------------------------------------------------------------------------------------------------------
#endif



//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_EQ_ADJUST_

static void HDMIStartEQDetect(struct it6602_eq_data *ucEQPort)
/*
 * This is the HDMIRX Start EQ Detect
 * @param it6602_eq_data
 * @return void
 */
{
	unsigned char ucPortSel;


//FIX_ID_035 xxxxx //For MTK6592 HDMI to SII MHL TX compliance issue
	if (ucEQPort->ucPortID == F_PORT_SEL_0) {
		if (hdmirxrd(REG_RX_P0_SYS_STATUS) & (B_P0_MHL_MODE)) {
			// for MHL mode , there are no need to adjust EQ for long cable.
			return;
		}
	}

//FIX_ID_035 xxxxx

	if (ucEQPort->ucEQState == 0xFF) {
		ucPortSel = hdmirxrd(REG_RX_051) & B_PORT_SEL;

		if (ucPortSel == ucEQPort->ucPortID) {
			HDMISwitchEQstate(ucEQPort, 0);	// for SCDT off state

		} else {
			HDMISwitchEQstate(ucEQPort, EQSTATE_WAIT + 1); 	//for SCDT on state
		}

		ucEQPort->f_manualEQadjust = TRUE;
		HDMIAdjustEQ(ucEQPort);

//FIX_ID_010 xxxxx 	//Add JudgeBestEQ to avoid wrong EQ setting
		ucEQPort->ErrorCount[0] = MAXECCWAIT;
		ucEQPort->ErrorCount[1] = MAXECCWAIT;
		ucEQPort->ErrorCount[2] = MAXECCWAIT;
//FIX_ID_010 xxxxx
	}
}

static void HDMISetEQValue(struct it6602_eq_data *ucEQPort, unsigned char ucIndex)
/*
 * This is the HDMIRX Set Manual EQ value
 * @param it6602_eq_data
 * @return void
 */
{
	if (ucIndex < MaxEQIndex) {
		if (ucEQPort->ucPortID == F_PORT_SEL_0) {
#ifdef _SUPPORT_AUTO_EQ_
			ucEQMode[F_PORT_SEL_0] = 1; // 1 for Manual Mode
#endif
			hdmirxset(REG_RX_026, 0x20, 0x20);	//07-04 add for adjust EQ
			hdmirxwr(REG_RX_027, IT6602EQTable[ucIndex]);
			IT_INFO("Port=%X ,ucIndex = %X ,HDMISetEQValue Reg027 = %X  ", (int) ucEQPort->ucPortID, (int) ucIndex,
				(int) hdmirxrd(REG_RX_027));

		} else {
#ifdef _SUPPORT_AUTO_EQ_
			ucEQMode[F_PORT_SEL_1] = 1; // 1 for Manual Mode
#endif
			hdmirxset(REG_RX_03E, 0x20, 0x20);	//07-04 add for adjust EQ
			hdmirxwr(REG_RX_03F, IT6602EQTable[ucIndex]);
			IT_INFO("Port=%X ,ucIndex = %X ,HDMISetEQValue Reg03F = %X  ", (int) ucEQPort->ucPortID, (int) ucIndex,
				(int) hdmirxrd(REG_RX_03F));
		}

	}

}


static void HDMISwitchEQstate(struct it6602_eq_data *ucEQPort, unsigned char state)
/*
 * This is the HDMIRX Switch EQ State
 * @param it6602_eq_data
 * @return void
 */
{

	ucEQPort->ucEQState = state;

	IT_INFO("!!! Port=%X ,HDMISwitchEQstate %X  ", (int) ucEQPort->ucPortID, (int) ucEQPort->ucEQState);

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
		//FIX_ID_037 xxxxx //Allion MHL compliance issue !!!
		//xxxxx 2014-0529 //HDCP Content On/Off
		IT6602_HDCP_ContentOff(ucEQPort->ucPortID, 0);
		//xxxxx 2014-0529
		//FIX_ID_037 xxxxx

		HDMISetEQValue(ucEQPort, 0xff);	//dont care
		break;

	}

	// !!! re-start the error count !!!

	ucEQPort->ucPkt_Err = 0;
	ucEQPort->ucECCvalue = 0;
	ucEQPort->ucECCfailCount = 0;

}

static void HDMICheckSCDTon(struct it6602_eq_data *ucEQPort)
/*
 * This is the HDMIRX SCDT on
 * @param it6602_eq_data
 * @return void
 */
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

	IT_INFO("1Port=%d, CheckSCDTon=%d, Receive_Err=%X, ucECCfailCount=%X, SCDT=%X, HDCP=%X  ",
		(int) ucEQPort->ucPortID, (int) ucEQPort->ucEQState, (int) Receive_Err, (int)ucEQPort->ucECCfailCount, (int) ucStatus,
		(int) ucHDCP);

//FIX_ID_033 xxxxx  //Fine-tune EQ Adjust function for HDCP receiver and repeater mode
	if ((Receive_Err & 0xC0) != 0x00) {
		ucEQPort->ucECCvalue++;

		//FIX_ID_037 xxxxx //Allion MHL compliance issue !!!
		//xxxxx 2014-0529 //Manual Content On/Off
		IT6602_HDCP_ContentOff(ucEQPort->ucPortID, 1);
		//xxxxx 2014-0529
		//FIX_ID_037 xxxxx

		//xxxxx 2014-0421
		//if((Receive_Err & 0xC0) == 0xC0)
		if (ucEQPort->ucECCvalue > ((MINECCFAILCOUNT / 2)))
			//xxxxx 2014-0421
		{
			ucEQPort->ucECCvalue = 0;
			IT_INFO("HDMICheckSCDTon() for ECC / Deskew issue !!!");

			if (ucEQPort->ucPortID == F_PORT_SEL_1) {
				if (hdmirxrd(REG_RX_038) == 0x00) {
					hdmirxwr(REG_RX_038, 0x3F);        // Dr. Liu suggestion to 0x00
				}

				//else
				//	hdmirxwr(REG_RX_038,0x00);	// Dr. Liu suggestion to 0x3F

				IT_INFO("Port 1 Reg38=%X !!!\n", (int) hdmirxrd(REG_RX_038));

			} else {
				if (hdmirxrd(REG_RX_020) == 0x00) {
					hdmirxwr(REG_RX_020, 0x3F);	// Dr. Liu suggestion to 0x00
					//else
					//	hdmirxwr(REG_RX_020,0x00);	// Dr. Liu suggestion to 0x3F

					IT_INFO("Port 0 Reg20=%X !!!\n", (int) hdmirxrd(REG_RX_020));
				}
			}
		}
	}

//FIX_ID_033 xxxxx

	if (ucEQPort->ucEQState == EQSTATE_WAIT - 1) {

		//FIX_ID_033 xxxxx  //Fine-tune EQ Adjust function for HDCP receiver and repeater mode
		IT_INFO("Port=%d, CheckSCDTon=%d, Receive_Err=%X, ucECCfailCount=%X, SCDT=%X, HDCP=%X  ",
			(int) ucEQPort->ucPortID, (int) ucEQPort->ucEQState, (int) Receive_Err, (int)ucEQPort->ucECCfailCount, (int) ucStatus,
			(int) ucHDCP);

		if ((Receive_Err & 0xC0) == 0xC0) {
			IT_INFO("HDMICheckSCDTon() CDR reset for Port %d ECC_TIMEOUT !!!\n", ucCurrentPort);
			hdmirx_ECCTimingOut(ucCurrentPort);

			HDMISwitchEQstate(ucEQPort, EQSTATE_END);
			return;
		}

		//FIX_ID_033 xxxxx

#ifdef _SUPPORT_AUTO_EQ_

		if ((ucEQPort->ucECCfailCount) == 0) {


			if (ucEQPort->ucPortID == F_PORT_SEL_1) {
				if (ucEQMode[F_PORT_SEL_1] == 0) {	// verfiy Auto EQ Value wehn auto EQ finish

					if (((ucChannelB[F_PORT_SEL_1] & 0x7F) < 0x0F) ||
					    ((ucChannelG[F_PORT_SEL_1] & 0x7F) < 0x0F) ||
					    ((ucChannelR[F_PORT_SEL_1] & 0x7F) < 0x0F))

					{
						ucResult	= 1;	// 1 for EQ start
					}

				}

			} else {
				if (ucEQMode[F_PORT_SEL_0] == 0) {	// verfiy Auto EQ Value when auto EQ finish
					if (hdmirxrd(REG_RX_P0_SYS_STATUS) & (B_P0_MHL_MODE)) {
						if ((ucChannelB[F_PORT_SEL_0] & 0x7F) < 0x0F) {
							ucResult	= 1;	// 1 for EQ start
						}

					} else {
						if (((ucChannelB[F_PORT_SEL_0] & 0x7F) < 0x0F) ||
						    ((ucChannelG[F_PORT_SEL_0] & 0x7F) < 0x0F) ||
						    ((ucChannelR[F_PORT_SEL_0] & 0x7F) < 0x0F))

						{
							ucResult	= 1;	// 1 for EQ start
						}

					}
				}
			}

			if (ucResult == 0) {	// no need to do manual EQ adjust when SCDT always On !!!
				HDMISwitchEQstate(ucEQPort, EQSTATE_END);
				return;
			}

		}

#endif

		HDMISwitchEQstate(ucEQPort, EQSTATE_WAIT);
	}


	UNUSED(ucHDCP);
}

static void HDMIPollingErrorCount(struct it6602_eq_data *ucEQPort)
/*
 * This is the HDMIPollingErrorCount
 * @param
 * @return void
 */
{
	unsigned char Receive_Err;
	unsigned char Video_Err;
	unsigned char Code_Err;
	unsigned char Pkt_Err;
	unsigned char CrtErr;
	unsigned char ucHDCP;
	unsigned char ucStatus;

	unsigned char ucCurrentPort;
	ucCurrentPort = hdmirxrd(REG_RX_051)&B_PORT_SEL;



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

	//xxxxx 2013-0903
	if (ucCurrentPort == ucEQPort->ucPortID) {
		if ((ucStatus & B_P0_SCDT) == 0x00) {
			Receive_Err = 0xFF;

			//xxxxx 2013-0812  ++++
			ucEQPort->ucECCfailCount |= 0x80;
			//xxxxx 2013-0812

		}
	}

	//xxxxx 2013-0903

	IT_INFO("Port=%d ,EQState2No=%d, Receive_Err=%X, HDCP=%X  ",
		(int) ucEQPort->ucPortID, (int) ucEQPort->ucEQState, (int) Receive_Err, (int) ucHDCP);

#if 1

//FIX_ID_007 xxxxx 	//07-18 xxxxx for ATC 8-7 Jitter Tolerance
	if (Pkt_Err == 0xFF || Code_Err == 0xFF) {
		ucEQPort->ucPkt_Err++;	// judge whether CDR reset

	} else {
		ucEQPort->ucPkt_Err = 0;
	}

	if (ucEQPort->ucPkt_Err > (MINECCFAILCOUNT - 2)) {

		if (ucEQPort->ucEQState > EQSTATE_START) {

//FIX_ID_020 xxxxx		//Turn off DEQ for HDMI port 1 with 20m DVI Cable
			IT_INFO("1111111111111111111111111111111111111111111111111111111111111111111111111 ");

			if (ucEQPort->ucPortID == F_PORT_SEL_1) {
				Code_Err = hdmirxrd(REG_RX_0B9);
				hdmirxwr(REG_RX_0B9, Code_Err);

				if (Code_Err == 0xFF) {
					if (hdmirxrd(REG_RX_038) == 0x00) {
						hdmirxwr(REG_RX_038, 0x3F);        // Dr. Liu suggestion to 0x00

					} else {
						hdmirxwr(REG_RX_038, 0x00);        // Dr. Liu suggestion to 0x3F
					}

					IT_INFO("Port 1 Reg38=%X !!!\n", (int) hdmirxrd(REG_RX_038));
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

					IT_INFO("Port 0 Reg20=%X !!!\n", (int) hdmirxrd(REG_RX_020));
				}
			}

			IT_INFO("1111111111111111111111111111111111111111111111111111111111111111111111111 ");
//FIX_ID_020 xxxxx

			if (ucEQPort->ucPortID == F_PORT_SEL_0) {

				hdmirxset(REG_RX_011, (B_P0_DCLKRST | B_P0_CDRRST), (B_P0_DCLKRST | B_P0_CDRRST/*|B_P0_SWRST*/));
				hdmirxset(REG_RX_011, (B_P0_DCLKRST | B_P0_CDRRST), 0x00);
				IT_INFO(" HDMIPollingErrorCount( ) Port 0 CDR reset !!!!!!!!!!!!!!!!!!  ");

			} else {
				hdmirxset(REG_RX_018, (B_P1_DCLKRST | B_P1_CDRRST), (B_P1_DCLKRST | B_P1_CDRRST/*|B_P1_SWRST*/));
				hdmirxset(REG_RX_018, (B_P1_DCLKRST | B_P1_CDRRST), 0x00);
				IT_INFO(" HDMIPollingErrorCount( ) Port 1 CDR reset !!!!!!!!!!!!!!!!!!  ");
			}
		}

		ucEQPort->ucPkt_Err = 0;

//xxxxx 2013-0812  ++++
		ucEQPort->ucECCfailCount |= 0x40;
		ucEQPort->ucECCfailCount &= 0xF0;
//xxxxx 2013-0812

	}

	//07-18 xxxxx
//FIX_ID_007 xxxxx
#endif


//	if(Receive_Err>32 )

//xxxxx 2013-0812  ++++
	if (Receive_Err != 0)
//xxxxx 2013-0812
	{
		IT_INFO("Video_Err = %X  ", (int) Video_Err);
		IT_INFO("Code_Err = %X  ", (int) Code_Err);
		IT_INFO("Pkt_Err = %X  ", (int) Pkt_Err);
		IT_INFO("CrtErr = %X  ", (int) CrtErr);

		ucEQPort->ucECCvalue++;
		ucEQPort->ucECCfailCount++;

	} else {
		ucEQPort->ucECCfailCount = 0;
	}

//	IT_INFO("ucEQPort->ucECCvalue = %X 666666666666666666666666 ",ucEQPort->ucECCvalue);
//xxxxx 2013-0812  ++++
#if 1

	if ((ucEQPort->ucECCfailCount & 0x7F) < (0x40)) {	// before CDR reset , dont care pkt_error and code_error

		if (Pkt_Err == 0xFF || Code_Err == 0xFF) {
			return;
		}
	}

#endif
//xxxxx 2013-0812

//	if((ucEQPort->ucECCfailCount & 0x7F) > (0x40 + MINECCFAILCOUNT-2))

//xxxxx 2013-0812  ++++
	if ((ucEQPort->ucECCfailCount & 0x0F) > (MINECCFAILCOUNT - 2)) {

		ucEQPort->ucECCvalue = MAXECCWAIT;

		ucCurrentPort = hdmirxrd(REG_RX_051)&B_PORT_SEL;

		if (ucEQPort->ucPortID == F_PORT_SEL_1) {
			ucStatus = hdmirxrd(REG_RX_P1_SYS_STATUS);

		} else {
			ucStatus = hdmirxrd(REG_RX_P0_SYS_STATUS);
		}

		if (ucCurrentPort == ucEQPort->ucPortID) {
			if (((ucStatus & B_P0_SCDT) == 0x00) || ((ucEQPort->ucECCfailCount & 0x80) != 0x00)) {
				ucEQPort->ucECCvalue = MAXECCWAIT | 0x80; 	// 0x80 for Identify SCDT off with Ecc error
			}
		}

		StoreEccCount(ucEQPort);	// abnormal judge ucECCvalue mode


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

//xxxxx 2013-0812
	UNUSED(ucHDCP);
}

static void HDMIJudgeECCvalue(struct it6602_eq_data *ucEQPort)
/*
 * This is the HDMIJudgeECCvalue
 * @param it6602_eq_data
 * @return void
 */
{

	//unsigned char uc;

	IT_INFO("!!! HDMI Judge ECCvalue( ) %X!!!  ", (int) ucEQPort->ucECCvalue);

	StoreEccCount(ucEQPort);	// normal judge ucECCvalue mode

	if ((ucEQPort->ucECCvalue) > (MAXECCWAIT / 2)) {
		//uc = CheckErrorCode(ucEQPort);

		//if(CheckErrorCode()==FALSE)
		//if(uc == FALSE)
		{

			if (ucEQPort->ucEQState == EQSTATE_START) {
				HDMISwitchEQstate(ucEQPort, EQSTATE_START);

			} else if (ucEQPort->ucEQState == EQSTATE_LOW) {
				HDMISwitchEQstate(ucEQPort, EQSTATE_LOW);

			} else if (ucEQPort->ucEQState == EQSTATE_MIDDLE) {
				HDMISwitchEQstate(ucEQPort, EQSTATE_MIDDLE);

			} else if (ucEQPort->ucEQState == EQSTATE_HIGH) {
				HDMISwitchEQstate(ucEQPort, EQSTATE_HIGH);
			}
		}

	} else {
		HDMISwitchEQstate(ucEQPort, EQSTATE_END);	// quit EQadjust( )
	}


	ucEQPort->ucPkt_Err = 0;
	ucEQPort->ucECCvalue = 0;
	ucEQPort->ucECCfailCount = 0;

}


static void HDMIAdjustEQ(struct it6602_eq_data *ucEQPort)
/*
 * This is the HDMIAdjustEQ
 * @param it6602_eq_data
 * @return void
 */
{
	unsigned char ucCurrentPort;
	ucCurrentPort = hdmirxrd(REG_RX_051)&B_PORT_SEL;

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

	case EQSTATE_HIGH+1:
	case EQSTATE_END+1:
		ucEQPort->f_manualEQadjust = FALSE;
		ucEQPort->ucEQState	= 0xFF;

		if (ucEQPort->ucPortID == ucCurrentPort) {
			IT6602VideoCountClr();
		}

		break;

	case 0xff:
	default:
		break;
	}

	if (ucEQPort->ucEQState != 0xFF) {

		if (ucEQPort->ucEQState < EQSTATE_WAIT) {		//20120410
			HDMICheckSCDTon(ucEQPort);

		} else if (ucEQPort->ucEQState < EQSTATE_HIGH) {
			HDMIPollingErrorCount(ucEQPort);
		}

		ucEQPort->ucEQState++;

	} else {
		ucEQPort->f_manualEQadjust = FALSE;
	}
}

//FIX_ID_010 xxxxx 	//Add JudgeBestEQ to avoid wrong EQ setting
static void StoreEccCount(struct it6602_eq_data *ucEQPort)
{

	IT_INFO("StoreEccCount() ucEQPort->ucECCvalue = %02X  ", (int) ucEQPort->ucECCvalue);

	if (ucEQPort->ucEQState <= EQSTATE_LOW) {
		ucEQPort->ErrorCount[0] = ucEQPort->ucECCvalue ;

	} else if (ucEQPort->ucEQState <= EQSTATE_MIDDLE) {
		ucEQPort->ErrorCount[1] = ucEQPort->ucECCvalue ;

	} else if (ucEQPort->ucEQState <= EQSTATE_HIGH) {
		ucEQPort->ErrorCount[2] = ucEQPort->ucECCvalue ;
		JudgeBestEQ(ucEQPort);
	}

}



void JudgeBestEQ(struct it6602_eq_data *ucEQPort)
{
	unsigned char i, j, Result;

	j = 0;
	Result = ucEQPort->ErrorCount[0];

	for (i = 1; i < MaxEQIndex; i++) {
//FIX_ID_033 xxxxx  //Fine-tune EQ Adjust function for HDCP receiver and repeater mode
// use Min value to be best EQ !			if(Result>ucEQPort->ErrorCount[i])
// use Max value to be best EQ !			if(Result>=ucEQPort->ErrorCount[i])
		if (Result >= ucEQPort->ErrorCount[i])
//FIX_ID_033 xxxxx
		{
			Result = ucEQPort->ErrorCount[i];
			j = i;
		}
	}

	IT_INFO(" Best IT6602EQTable ErrorCount[%X]=%X !!! IT6602EQTable Value=%X !!!\n", (int) j, (int) Result,
		(int) IT6602EQTable[j]);

	//if(j==0 && Result==0)
//2014-0102 bug xxxxx !!!!!
	if (ucEQPort->ucPortID == F_PORT_SEL_0) {
#ifdef _SUPPORT_AUTO_EQ_

		if ((hdmirxrd(REG_RX_027) & 0x80) == 0) {
			OverWriteAmpValue2EQ(ucEQPort->ucPortID);

		} else
#endif
		{
			hdmirxset(REG_RX_026, 0x20, 0x20);	//07-04 add for adjust EQ
			hdmirxwr(REG_RX_027, IT6602EQTable[j]);
			IT_INFO("Port=%X ,ucIndex = %X ,JudgeBestEQ Reg027 = %X  ", (int) ucEQPort->ucPortID, (int) j,
				(int) hdmirxrd(REG_RX_027));
		}

	}


	else {
#ifdef _SUPPORT_AUTO_EQ_

		if ((hdmirxrd(REG_RX_03F) & 0x80) == 0) {
			OverWriteAmpValue2EQ(ucEQPort->ucPortID);

		} else
#endif
		{
			hdmirxset(REG_RX_03E, 0x20, 0x20);	//07-04 add for adjust EQ
			hdmirxwr(REG_RX_03F, IT6602EQTable[j]);
			IT_INFO("Port=%X ,ucIndex = %X ,JudgeBestEQ Reg03F = %X  ", (int) ucEQPort->ucPortID, (int) j,
				(int) hdmirxrd(REG_RX_03F));

		}
	}

}


static void IT6602VideoCountClr(void)
{
	struct it6602_dev_data *it6602data = get_it6602_dev_data();
	it6602data->m_VideoCountingTimer = 1;
}
#endif

static unsigned char CheckAVMute(void)
{

	unsigned char ucAVMute;
	unsigned char ucPortSel;

	ucAVMute = hdmirxrd(REG_RX_0A8) & (B_P0_AVMUTE | B_P1_AVMUTE);
	ucPortSel = hdmirxrd(REG_RX_051)&B_PORT_SEL;

	if (((ucAVMute & B_P0_AVMUTE) && (ucPortSel == F_PORT_SEL_0)) ||
	    ((ucAVMute & B_P1_AVMUTE) && (ucPortSel == F_PORT_SEL_1))) {
		return TRUE;

	} else {
		return FALSE;
	}
}


static unsigned char CheckPlg5VPwr(unsigned char ucPortSel)
{

	if (ucPortSel != 0) {
		PX4_ERR("it66021 only support Port0 in reg51 \r\n");
		return FALSE;
	}

	unsigned char sys_state_P0 = hdmirxrd(REG_RX_P0_SYS_STATUS);

	return sys_state_P0 & B_P0_PWR5V_DET;

}


// ---------------------------------------------------------------------------
static unsigned char IsHDMIMode(void)
{

	unsigned char sys_state_P0;
	unsigned char sys_state_P1;
	unsigned char ucPortSel;

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

// ---------------------------------------------------------------------------
unsigned char IsVideoOn(void)
{

	struct it6602_dev_data *it6602data = get_it6602_dev_data();

	if (it6602data->m_VState == VSTATE_VideoOn) {
		return TRUE;

	} else {
		return FALSE;
	}

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

	if (it6602->RGBQuantizationRange == 0) {
		if (it6602->VIC >= 2) {
			// CE Mode
			it6602->RGBQuantizationRange = 1 ; // limited range

		} else {
			// IT mode
			it6602->RGBQuantizationRange = 2 ; // Full range
		}
	}

	IT_INFO("AVI ColorMode = %X  ", (int) it6602->ColorMode);
	IT_INFO("AVI Colorimetry = %X  ", (int) it6602->Colorimetry);
	IT_INFO("AVI ExtendedColorimetry = %X  ", (int) it6602->ExtendedColorimetry);
	IT_INFO("AVI RGBQuantizationRange = %X  ", (int) it6602->RGBQuantizationRange);
	IT_INFO("AVI VIC = %X  ", (int) it6602->VIC);
	IT_INFO("AVI YCCQuantizationRange = %X  ", (int) it6602->YCCQuantizationRange);
}


// ---------------------------------------------------------------------------
static void SetVideoInputFormatWithInfoFrame(struct it6602_dev_data *it6602)
{
	unsigned char i;

	chgbank(2);
	i = hdmirxrd(REG_RX_215);	//REG_RX_AVI_DB1
	chgbank(0);
	it6602->m_bInputVideoMode &= ~F_MODE_CLRMOD_MASK;


	switch ((i >> O_AVI_COLOR_MODE)&M_AVI_COLOR_MASK) {
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


	IT_INFO("SetVideoInputFormatWithInfoFrame - RegAE=%X it6602->m_bInputVideoMode=%X\n", (int) i,
		(int) it6602->m_bInputVideoMode);
	i = hdmirxrd(REG_RX_IN_CSC_CTRL);
	i &= ~B_IN_FORCE_COLOR_MODE;
	hdmirxwr(REG_RX_IN_CSC_CTRL, i);
}

// ---------------------------------------------------------------------------
static void SetColorimetryByInfoFrame(struct it6602_dev_data *it6602)
{
	unsigned char i;

	chgbank(2);
	i = hdmirxrd(REG_RX_216);	//REG_RX_AVI_DB2
	chgbank(0);
	i &= M_AVI_CLRMET_MASK << O_AVI_CLRMET;

	if (i == (B_AVI_CLRMET_ITU601 << O_AVI_CLRMET)) {
		it6602->m_bInputVideoMode &= ~F_MODE_ITU709;

	} else if (i == (B_AVI_CLRMET_ITU709 << O_AVI_CLRMET)) {
		it6602->m_bInputVideoMode |= F_MODE_ITU709;

	}


}


static void SetCSCBYPASS(struct it6602_dev_data *it6602)
{

	it6602->m_bOutputVideoMode = it6602->m_bInputVideoMode;

	switch (it6602->m_bInputVideoMode & F_MODE_CLRMOD_MASK) {
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


// ---------------------------------------------------------------------------
static void SetColorSpaceConvert(struct it6602_dev_data *it6602)
{
	unsigned char csc = 0;

	unsigned char filter = 0 ; // filter is for Video CTRL DN_FREE_GO, EN_DITHER, and ENUDFILT

	IT_INFO("\n!!! SetColorSpaceConvert( ) !!!\n");

	switch (it6602->m_bOutputVideoMode & F_MODE_CLRMOD_MASK) {
#if defined(SUPPORT_OUTPUTYUV444)

	case F_MODE_YUV444:
		IT_INFO("Output mode is YUV444\n");

		switch (it6602->m_bInputVideoMode & F_MODE_CLRMOD_MASK) {
		case F_MODE_YUV444:
			IT_INFO("Input mode is YUV444\n");
			csc = B_CSC_BYPASS ;
			break ;

		case F_MODE_YUV422:
			IT_INFO("Input mode is YUV422\n");
			csc = B_CSC_BYPASS ;

			if (it6602->m_bOutputVideoMode & F_MODE_EN_UDFILT) { // RGB24 to YUV422 need up/dn filter.
				filter |= B_RX_EN_UDFILTER ;
			}

			if (it6602->m_bOutputVideoMode & F_MODE_EN_DITHER) { // RGB24 to YUV422 need up/dn filter.
				filter |= B_RX_EN_UDFILTER | B_RX_DNFREE_GO ;
			}

			break ;

		case F_MODE_RGB24:
			IT_INFO("Input mode is RGB444\n");
			csc = B_CSC_RGB2YUV ;
			break ;
		}

		break ;


#if defined(SUPPORT_OUTPUTYUV422)

	case F_MODE_YUV422:
		switch (it6602->m_bInputVideoMode & F_MODE_CLRMOD_MASK) {
		case F_MODE_YUV444:
			IT_INFO("Input mode is YUV444\n");

			if (it6602->m_bOutputVideoMode & F_MODE_EN_UDFILT) {
				filter |= B_RX_EN_UDFILTER ;
			}

			csc = B_CSC_BYPASS ;
			break ;

		case F_MODE_YUV422:
			IT_INFO("Input mode is YUV422\n");
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
			IT_INFO("Input mode is RGB444\n");

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
		IT_INFO("Output mode is RGB24\n");

		switch (it6602->m_bInputVideoMode & F_MODE_CLRMOD_MASK) {
		case F_MODE_YUV444:
			IT_INFO("Input mode is YUV444\n");
			csc = B_CSC_YUV2RGB ;
			break ;

		case F_MODE_YUV422:
			IT_INFO("Input mode is YUV422\n");
			csc = B_CSC_YUV2RGB ;

			if (it6602->m_bOutputVideoMode & F_MODE_EN_UDFILT) { // RGB24 to YUV422 need up/dn filter.
				filter |= B_RX_EN_UDFILTER ;
			}

			if (it6602->m_bOutputVideoMode & F_MODE_EN_DITHER) { // RGB24 to YUV422 need up/dn filter.
				filter |= B_RX_EN_UDFILTER | B_RX_DNFREE_GO ;
			}

			break ;

		case F_MODE_RGB24:
			IT_INFO("Input mode is RGB444\n");
			csc = B_CSC_BYPASS ;
			break ;
		}

		break ;
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
		IT_INFO(" Clear Reg67 and Reg68 ...  ");

		//FIX_ID_039 xxxxx
		if (it6602->m_bInputVideoMode & F_MODE_ITU709) {
			IT_INFO("ITU709 ");

			if (it6602->m_bInputVideoMode & F_MODE_16_235) {
				IT_INFO(" 16-235\n");
				chgbank(1);	//for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_RGB2YUV_ITU709_16_235), &bCSCMtx_RGB2YUV_ITU709_16_235[0]);

			} else {
				IT_INFO(" 0-255\n");
				chgbank(1);	//for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_RGB2YUV_ITU709_0_255), &bCSCMtx_RGB2YUV_ITU709_0_255[0]);
			}

		} else {
			IT_INFO("ITU601 ");

			if (it6602->m_bInputVideoMode & F_MODE_16_235) {
				chgbank(1);	//for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_RGB2YUV_ITU601_16_235), &bCSCMtx_RGB2YUV_ITU601_16_235[0]);
				IT_INFO(" 16-235\n");

			} else {
				chgbank(1);	//for CSC setting Reg170 ~ Reg184 !!!!
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
		IT_INFO(" Clear Reg67 and Reg68 ...  ");

		//FIX_ID_039 xxxxx
		if (it6602->m_bInputVideoMode & F_MODE_ITU709) {
			IT_INFO("ITU709 ");

			if (it6602->m_bOutputVideoMode & F_MODE_16_235) {
				IT_INFO("16-235\n");
				chgbank(1);	//for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_YUV2RGB_ITU709_16_235), &bCSCMtx_YUV2RGB_ITU709_16_235[0]);

			} else {
				IT_INFO("0-255\n");
				chgbank(1);	//for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_YUV2RGB_ITU709_0_255), &bCSCMtx_YUV2RGB_ITU709_0_255[0]);
			}

		} else {
			IT_INFO("ITU601 ");

			if (it6602->m_bOutputVideoMode & F_MODE_16_235) {
				IT_INFO("16-235\n");
				chgbank(1);	//for CSC setting Reg170 ~ Reg184 !!!!
				hdmirxbwr(REG_RX_170, sizeof(bCSCMtx_YUV2RGB_ITU601_16_235), &bCSCMtx_YUV2RGB_ITU601_16_235[0]);

			} else {
				IT_INFO("0-255\n");
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
					IT_INFO(" bCSCMtx_RGB_16_235_RGB_0_255  ");
					// printf("pccmd w 65 02 90; ");
					// printf("pccmd w 67 78 90; ");
					// printf("pccmd w 68 ED 90; ");
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
					IT_INFO(" bCSCMtx_RGB_0_255_RGB_16_235  ");
					// printf("pccmd w 65 02 90; ");
					// printf("pccmd w 67 40 90; ");
					// printf("pccmd w 68 10 90; ");
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

// ---------------------------------------------------------------------------
static void SetNewInfoVideoOutput(struct it6602_dev_data *it6602)
{

	IT_INFO("SetNewInfoVideoOutput() \n");

	SetVideoInputFormatWithInfoFrame(it6602);
	SetColorimetryByInfoFrame(it6602);
	SetColorSpaceConvert(it6602);

	SetVideoOutputColorFormat(it6602);	//2013-0502

//	get_vid_info();
//	show_vid_info();

}

// ---------------------------------------------------------------------------
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
// ---------------------------------------------------------------------------
static void SetColorimetryByMode(struct it6602_dev_data *it6602)
{
	unsigned char  RxClkXCNT;
	RxClkXCNT = hdmirxrd(REG_RX_PIXCLK_SPEED);

	IT_INFO(" SetColorimetryByMode() REG_RX_PIXCLK_SPEED=%X \n", (int) RxClkXCNT);

	it6602->m_bInputVideoMode &= ~F_MODE_ITU709;

	if (RxClkXCNT < 0x34) {

		it6602->m_bInputVideoMode |= F_MODE_ITU709;

	} else {

		it6602->m_bInputVideoMode &= ~F_MODE_ITU709;
	}

}
// ---------------------------------------------------------------------------
static void SetDVIVideoOutput(struct it6602_dev_data *it6602)
{
	IT_INFO("SetDVIVideoOutput() \n");

	SetVideoInputFormatWithoutInfoFrame(it6602, F_MODE_RGB24);
	SetColorimetryByMode(it6602);
	SetColorSpaceConvert(it6602);

	SetVideoOutputColorFormat(it6602);	//2013-0502
}



//FIX_ID_003 xxxxx	//Add IT6602 Video Output Configure setting
static void IT6602_VideoOutputModeSet(struct it6602_dev_data *it6602)
{
	unsigned char ucReg51;
	unsigned char ucReg65;

	IT_INFO("IT6602_VideoOutputModeSet()  ");


	IT_INFO("+++ %s", VModeStateStr[(unsigned char)it6602->m_VidOutConfigMode]);


	ucReg51 = hdmirxrd(REG_RX_051) & 0x9B;	// Reg51 [6] Half PCLK DDR , [5] Half Bus DDR , [2] CCIR656 mode
	ucReg65 = hdmirxrd(REG_RX_065) &
		  0x0F;	// Reg65 [7] BTA1004Fmt , [6] SyncEmb , [5:4] output color 0x00 RGB, 0x10 YUV422, 0x20 YUV444

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


	IT_INFO("Reg51 = %X ", (int) ucReg51);
	IT_INFO("Reg65 = %X ", (int) ucReg65);

	hdmirxwr(REG_RX_051, ucReg51);
	hdmirxwr(REG_RX_065, ucReg65);


}
//FIX_ID_003 xxxxx


static void IT6602VideoOutputConfigure(struct it6602_dev_data *it6602)
{

	// Configure Output color space convert

	it6602->m_bUpHDMIMode = IsHDMIMode();

	if (it6602->m_bUpHDMIMode == FALSE) {
		SetDVIVideoOutput(it6602);

	} else {

		GetAVIInfoFrame(it6602);
		SetNewInfoVideoOutput(it6602);

	}

	it6602->m_NewAVIInfoFrameF = FALSE;

	// Configure Output Color Depth

	it6602->GCP_CD = ((hdmirxrd(0x99) & 0xF0) >> 4);

	switch (it6602->GCP_CD) {
	case 5 :
		IT_INFO("\n Output ColorDepth = 30 bits per pixel ");
		hdmirxset(0x65, 0x0C, 0x04);
		break;

	case 6 :
		IT_INFO("\n Output ColorDepth = 36 bits per pixel ");
		hdmirxset(0x65, 0x0C, 0x08);
		break;

	default :
		IT_INFO("\n Output ColorDepth = 24 bits per pixel ");
		hdmirxset(0x65, 0x0C, 0x00);
		break;
	}

	// Configure TTL Video Output mode
	IT6602_VideoOutputModeSet(it6602);

}

// ---------------------------------------------------------------------------
static void SetVideoOutputColorFormat(struct it6602_dev_data *it6602)
{
	switch (it6602->m_bOutputVideoMode & F_MODE_CLRMOD_MASK) {
	case F_MODE_RGB24 :
		hdmirxset(REG_RX_OUT_CSC_CTRL, (M_OUTPUT_COLOR_MASK), B_OUTPUT_RGB24);
		break;

	case F_MODE_YUV422 :
		hdmirxset(REG_RX_OUT_CSC_CTRL, (M_OUTPUT_COLOR_MASK), B_OUTPUT_YUV422);
		break;

	case F_MODE_YUV444 :
		hdmirxset(REG_RX_OUT_CSC_CTRL, (M_OUTPUT_COLOR_MASK), B_OUTPUT_YUV444);

		break;
	}
}

void it6602PortSelect(unsigned char ucPortSel)
{
	struct it6602_dev_data *it6602data = get_it6602_dev_data();

	hdmirxset(REG_RX_051, B_PORT_SEL, F_PORT_SEL_0); //select port 0
	it6602data->m_ucCurrentHDMIPort = F_PORT_SEL_0;


	if (it6602data->m_ucCurrentHDMIPort != ucPortSel) {

		IT6602SwitchVideoState(it6602data, VSTATE_SyncWait);
		it6602data->m_ucCurrentHDMIPort = ucPortSel;
		IT_INFO("it6602PortSelect = %X  ", (int) ucPortSel);
	}

}

void it6602HPDCtrl(unsigned char ucport, unsigned char ucEnable)
{
	if (ucport != 0) {
		IT_INFO("it66021 only support ucport in it6602HPDCtrl ");
		return;
	}

	if (ucEnable == 0) {
		// Disable HDMI DDC Bus to access ITEHDMI EDID RAM
		//hdmirxset(REG_RX_0C0, 0x01, 0x01);                            // HDMI RegC0[1:0]=11 for disable HDMI DDC bus to access EDID RAM
		IT_INFO("[it6602HPDCtrl]: Port 0 HPD HDMI");
		chgbank(1);
		hdmirxset(REG_RX_1B0, 0x03, 0x01); //clear port 0 HPD=1 for EDID update
		chgbank(0);

	} else {
		if ((hdmirxrd(REG_RX_P0_SYS_STATUS) & B_P0_PWR5V_DET)) {
			// Enable HDMI DDC bus to access ITEHDMI EDID RAM
			//hdmirxset(REG_RX_0C0, 0x01, 0x00);                        // HDMI RegC0[1:0]=00 for enable HDMI DDC bus to access EDID RAM
			IT_INFO("[it6602HPDCtrl] Port 0 HPD HDMI 11111 ");
			chgbank(1);
			hdmirxset(REG_RX_1B0, 0x03, 0x03); //set port 0 HPD=1
			chgbank(0);
		}
	}
}






static void hdmirx_ECCTimingOut(unsigned char ucport)
{
	IT_INFO("CDR reset for hdmirx_ECCTimingOut()   ");

	if (ucport == F_PORT_SEL_0) {

		it6602HPDCtrl(0, 0);	// MHL port , set HPD = 0

		hdmirxset(REG_RX_011, (B_P0_DCLKRST | B_P0_CDRRST | B_P0_HDCPRST | B_P0_SWRST),
			  (B_P0_DCLKRST | B_P0_CDRRST | B_P0_HDCPRST | B_P0_SWRST));
		usleep(1000 * 300);
		hdmirxset(REG_RX_011, (B_P0_DCLKRST | B_P0_CDRRST | B_P0_HDCPRST | B_P0_SWRST), 0x00);

		//set port 0 HPD=1
		it6602HPDCtrl(0, 1);	// MHL port , set HPD = 1

	} else {
		//set port 1 HPD=0
		it6602HPDCtrl(1, 0);	// HDMI port , set HPD = 0


		hdmirxset(REG_RX_018, (B_P1_DCLKRST | B_P1_CDRRST | B_P1_HDCPRST | B_P1_SWRST),
			  (B_P1_DCLKRST | B_P1_CDRRST | B_P1_HDCPRST | B_P1_SWRST));
		usleep(1000 * 300);
		hdmirxset(REG_RX_018, (B_P1_DCLKRST | B_P1_CDRRST | B_P1_HDCPRST | B_P1_SWRST), 0x00);

		//set port 1 HPD=1
		it6602HPDCtrl(1, 1);	// HDMI port , set HPD = 1
	}
}

#endif



#ifdef _ITEHDMI_
// ***************************************************************************
// Audio function
// ***************************************************************************
static void aud_fiforst(void)
{
	unsigned char uc ;

	hdmirxset(REG_RX_074, 0x0c, 0x0c);	// enable Mute i2s and ws	and s/pdif
	//usleep(1000*100);
	hdmirxset(REG_RX_074, 0x0c, 0x00);	// disable Mute i2s and ws	and s/pdif


	hdmirxset(REG_RX_010, 0x02, 0x02);
	hdmirxset(REG_RX_010, 0x02, 0x00);

	uc = hdmirxrd(REG_RX_07B) ;
	hdmirxwr(REG_RX_07B, uc) ;
	hdmirxwr(REG_RX_07B, uc) ;
	hdmirxwr(REG_RX_07B, uc) ;
	hdmirxwr(REG_RX_07B, uc) ; // HOPE said, after FIFO reset, four valid update will active the AUDIO FIFO.
}


static void IT6602AudioOutputEnable(unsigned char bEnable)
{
	if (bEnable == TRUE) {
		hdmirxset(REG_RX_052, (B_TriI2SIO | B_TriSPDIF), 0x00);
	} else {
		hdmirxset(REG_RX_052, (B_TriI2SIO | B_TriSPDIF), (B_TriI2SIO | B_TriSPDIF));
	}
}


static void hdmirx_ResetAudio(void)
{
	unsigned char uc ;
	hdmirxset(REG_RX_RST_CTRL, B_AUDRST, B_AUDRST);
	hdmirxset(REG_RX_RST_CTRL, B_AUDRST, 0x00);

	uc = hdmirxrd(REG_RX_07B) ;
	hdmirxwr(REG_RX_07B, uc) ;
	hdmirxwr(REG_RX_07B, uc) ;
	hdmirxwr(REG_RX_07B, uc) ;
	hdmirxwr(REG_RX_07B, uc) ; // HOPE said, after FIFO reset, four valid update will active the AUDIO FIFO.

}


static void hdmirx_SetHWMuteClrMode(void)
{
	hdmirxset(REG_RX_HWMuteCtrl, (B_HWAudMuteClrMode), (B_HWAudMuteClrMode));
}

static void hdmirx_SetHWMuteClr(void)
{
	hdmirxset(REG_RX_HWMuteCtrl, (B_HWMuteClr), (B_HWMuteClr));
}

static void hdmirx_ClearHWMuteClr(void)
{
	hdmirxset(REG_RX_HWMuteCtrl, (B_HWMuteClr), 0);
}


static void getHDMIRXInputAudio(AUDIO_CAPS *pAudioCaps)
{

	unsigned char uc;

	uc = hdmirxrd(REG_RX_0AE);	// REG_RX_AUD_CHSTAT3
	pAudioCaps->SampleFreq = uc & M_FS;

	uc = hdmirxrd(REG_RX_0AA);	//REG_RX_AUDIO_CH_STAT
	pAudioCaps->AudioFlag = uc & 0xF0;
	pAudioCaps->AudSrcEnable = uc & M_AUDIO_CH;
	pAudioCaps->AudSrcEnable |= hdmirxrd(REG_RX_0AA)&M_AUDIO_CH;

	if ((uc & (B_HBRAUDIO | B_DSDAUDIO)) == 0) {
		uc = hdmirxrd(REG_RX_0AB);	//REG_RX_AUD_CHSTAT0

		if ((uc & B_NLPCM) == 0) {
			pAudioCaps->AudioFlag |= B_CAP_LPCM;
		}
	}
}


static void IT6602SwitchAudioState(struct it6602_dev_data *it6602, Audio_State_Type state)
{
	if (it6602->m_AState == state) {
		return ;
	}

	IT_INFO("+++ %s\n", AStateStr[(unsigned char)state]);

	it6602->m_AState = state;

	switch (it6602->m_AState) {
	case ASTATE_AudioOff:
		hdmirxset(REG_RX_RST_CTRL, B_AUDRST, B_AUDRST);
		IT6602AudioOutputEnable(OFF);
		break;

	case ASTATE_RequestAudio:
		IT6602AudioOutputEnable(OFF);
		break;

	case ASTATE_WaitForReady:
		hdmirx_SetHWMuteClr();
		hdmirx_ClearHWMuteClr();
		it6602->m_AudioCountingTimer = AUDIO_READY_TIMEOUT;
		break;

	case ASTATE_AudioOn:

		IT6602AudioOutputEnable(ON);

		IT_INFO("Cat6023 Audio--> Audio flag=%02X,Ch No=%02X,Fs=%02X ... \n",
			(int)it6602->m_RxAudioCaps.AudioFlag,
			(int)it6602->m_RxAudioCaps.AudSrcEnable,
			(int)it6602->m_RxAudioCaps.SampleFreq);
		break;

	default :
		break;

	}
}


static void IT6602AudioHandler(struct it6602_dev_data *it6602)
{
//    unsigned char uc;

	if (it6602->m_AudioCountingTimer > MS_LOOP) {
		it6602->m_AudioCountingTimer -= MS_LOOP;

	} else {
		it6602->m_AudioCountingTimer = 0;
	}


	if (it6602->m_RxHDCPState == RxHDCP_ModeCheck) {
		return;
	}

	switch (it6602->m_AState) {
	case ASTATE_RequestAudio:

		getHDMIRXInputAudio(&(it6602->m_RxAudioCaps));

		if (it6602->m_RxAudioCaps.AudioFlag & B_CAP_AUDIO_ON) {

			hdmirxset(REG_RX_MCLK_CTRL, M_MCLKSel, B_256FS);

			if (it6602->m_RxAudioCaps.AudioFlag & B_CAP_HBR_AUDIO) {
				IT_INFO("+++++++++++ B_CAP_HBR_AUDIO +++++++++++++++++\n");

				hdmirxset(REG_RX_MCLK_CTRL, M_MCLKSel, B_128FS);	// MCLK = 128fs only for HBR audio

				hdmirx_SetHWMuteClrMode();
				hdmirx_ResetAudio();

			} else if (it6602->m_RxAudioCaps.AudioFlag & B_CAP_DSD_AUDIO) {

				hdmirx_SetHWMuteClrMode();
				hdmirx_ResetAudio();

			} else {

				hdmirxset(REG_RX_HWMuteCtrl, B_HWMuteClr, 0x00);
				hdmirx_SetHWMuteClrMode();
				hdmirx_ResetAudio();

			}

			IT6602SwitchAudioState(it6602, ASTATE_WaitForReady);

		}

		break;

	case ASTATE_WaitForReady:

		if (it6602->m_AudioCountingTimer == 0) {
			IT6602SwitchAudioState(it6602, ASTATE_AudioOn);
		}

		break;

	case ASTATE_AudioOn:

		//if(AudioTimeOutCheck(AUDIO_MONITOR_TIMEOUT)==TRUE)
		if (it6602->m_AudioCountingTimer == 0) {
			AUDIO_CAPS CurAudioCaps;
			//it6602->m_AudioCountingTimer = GetCurrentVirtualTime();
			//AssignAudioTimerTimeout(AUDIO_MONITOR_TIMEOUT);
			it6602->m_AudioCountingTimer = AUDIO_MONITOR_TIMEOUT;

			getHDMIRXInputAudio(&CurAudioCaps);

			if (it6602->m_RxAudioCaps.AudioFlag != CurAudioCaps.AudioFlag
			    || it6602->m_RxAudioCaps.AudSrcEnable != CurAudioCaps.AudSrcEnable
			    || it6602->m_RxAudioCaps.SampleFreq != CurAudioCaps.SampleFreq) {
				//it6602->m_ucHDMIAudioErrorCount=0;
				IT6602SwitchAudioState(it6602, ASTATE_RequestAudio);
			}
		}

		break;

	default :
		break;

	}
}

#endif
#ifdef _ITEHDMI_
// ***************************************************************************
// Video function
// ***************************************************************************


static void IT6602_AFE_Rst(void)
{
	unsigned char Reg51h;

	struct it6602_dev_data *it6602data = get_it6602_dev_data();		//2013-0814

	chgbank(0);
	Reg51h = hdmirxrd(0x51);

	if (Reg51h & 0x01) {
		IT_INFO("=== port 1 IT6602_AFE_Rst() ===  ");
		hdmirxset(REG_RX_018, 0x01, 0x01);
		usleep(1000 * 1);
		hdmirxset(REG_RX_018, 0x01, 0x00);
#ifdef _SUPPORT_AUTO_EQ_
		DisableOverWriteRS(1);	//2013-1129
#endif

	} else {
		IT_INFO("=== port 0 IT6602_AFE_Rst() ===  ");
		hdmirxset(REG_RX_011, 0x01, 0x01);
		usleep(1000 * 1);
		hdmirxset(REG_RX_011, 0x01, 0x00);
#ifdef _SUPPORT_AUTO_EQ_
		DisableOverWriteRS(0);	//2013-1129 for MHL unplug detected
#endif

	}

	it6602data->m_ucSCDTOffCount = 0;	//2013-0814
}



//FIX_ID_037 xxxxx //Allion MHL compliance issue !!!
//xxxxx 2014-0529 //HDCP Content On/Off
static void IT6602_ManualVideoTristate(unsigned char bOff)
{
	if (bOff) {
		hdmirxset(REG_RX_053, (B_VDGatting | B_VIOSel),
			  (B_VDGatting | B_VIOSel));	//Reg53[7][5] = 11    // for enable B_VDIO_GATTING and VIO_SEL
		hdmirxset(REG_RX_052, (B_DisVAutoMute), (B_DisVAutoMute));				//Reg52[5] = 1 for disable Auto video MUTE
		hdmirxset(REG_RX_053, (B_TriVDIO), (0x00));								//Reg53[2:0] = 000;         // 0 for enable video io data output
		IT_INFO("+++++++++++ Manual Video / Audio off  +++++++++++++++++\n");

	} else {
		hdmirxset(REG_RX_053, (B_TriSYNC), (0x00));								//Reg53[0] = 0;                 // for enable video sync
		hdmirxset(REG_RX_053, (B_TriVDIO), (0x00));								//Reg53[3:1] = 000;         // 0 for enable video io data output
		hdmirxset(REG_RX_053, (B_TriVDIO), (
				  B_TriVDIO));							//Reg53[2:0] = 111;         // 1 for enable tri-state of video io data
		hdmirxset(REG_RX_053, (B_TriVDIO), (0x00));								//Reg53[2:0] = 000;         // 0 for enable video io data output
		hdmirxset(REG_RX_053, (B_VDGatting | B_VIOSel),
			  (B_VDGatting | B_VIOSel));	//Reg53[7][5] = 11    // for enable B_VDIO_GATTING and VIO_SEL
		hdmirxset(REG_RX_053, (B_VDGatting | B_VIOSel), (B_VIOSel));				//Reg53[7][5] = 01    // for disable B_VDIO_GATTING
		IT_INFO("+++++++++++ Manual Video on  +++++++++++++++++\n");
	}
}
static void IT6602_HDCP_ContentOff(unsigned char ucPort, unsigned char bOff)
{
	struct it6602_dev_data *it6602data = get_it6602_dev_data();		//2013-0814

	if (IT6602_IsSelectedPort(ucPort) == FALSE) {
		return;
	}

	if (bOff != 0) {
		//******** Content Off ********//
		IT6602_ManualVideoTristate(1);
		it6602data->m_HDCP_ContentOff = 1;
		IT_INFO("+++++++++++ HDCP Content Off   +++++++++++++++++\n");

	} else {
		if (it6602data->m_VState == VSTATE_VideoOn) {
			if (it6602data->m_HDCP_ContentOff == 1) {
				//******** Content On ********//
				IT6602_ManualVideoTristate(0);
				IT_INFO("+++++++++++ HDCP Content On   +++++++++++++++++\n");
			}
		}

		it6602data->m_HDCP_ContentOff = 0;
	}

}



static void IT6602_RAPContentOff(unsigned char bOff)
{
	struct it6602_dev_data *it6602data = get_it6602_dev_data();		//2013-0814

	if (IT6602_IsSelectedPort(0) == FALSE) {
		return;
	}

	if (bOff != 0) {
		//******** RAP Content Off ********//
		IT6602_ManualVideoTristate(1);
		it6602data->m_RAP_ContentOff = 1;
		IT_INFO("+++++++++++ RAP Content Off   +++++++++++++++++\n");

		//xxxxx 2014-0603 for RAP Content off
		IT6602AudioOutputEnable(0);
		//xxxxx 2014-0603

	} else {
		if (it6602data->m_VState == VSTATE_VideoOn) {
			if (it6602data->m_RAP_ContentOff == 1) {
				//******** RAP Content On ********//
				IT6602_ManualVideoTristate(0);
				IT_INFO("+++++++++++ RAP Content On   +++++++++++++++++\n");

#ifndef _FIX_ID_028_
				//FIX_ID_028 xxxxx //For Debug Audio error with S2
				//xxxxx 2014-0603 for RAP Content On
				IT6602SwitchAudioState(it6602data, ASTATE_RequestAudio);
				//xxxxx 2014-0603
				//FIX_ID_028 xxxxx //For Debug Audio error with S2
#endif

			}
		}

		it6602data->m_RAP_ContentOff = 0;
	}
}
//xxxxx 2014-0529
//FIX_ID_037 xxxxx

static void IT6602_SetVideoMute(struct it6602_dev_data *it6602, unsigned char bMute)
{

	if (bMute) {
		//******** AV Mute -> ON ********//
		hdmirxset(REG_RX_053, (B_VDGatting | B_VIOSel),
			  (B_VDGatting | B_VIOSel));	//Reg53[7][5] = 11    // for enable B_VDIO_GATTING and VIO_SEL
		hdmirxset(REG_RX_052, (B_DisVAutoMute), (B_DisVAutoMute));				//Reg52[5] = 1 for disable Auto video MUTE
		hdmirxset(REG_RX_053, (B_TriVDIO), (0x00));								//Reg53[2:0] = 000;         // 0 for enable video io data output

		IT_INFO("+++++++++++ IT6602_SetVideoMute -> On +++++++++++++++++\n");

	} else {
		if (it6602->m_VState == VSTATE_VideoOn) {
			//******** AV Mute -> OFF ********//
			hdmirxset(REG_RX_053, (B_TriSYNC), (0x00));								//Reg53[0] = 0;                 // for enable video sync
			hdmirxset(REG_RX_053, (B_TriVDIO), (0x00));								//Reg53[3:1] = 000;         // 0 for enable video io data output

			if (CheckAVMute() == TRUE) {
				hdmirxset(REG_RX_052, (B_DisVAutoMute), (B_DisVAutoMute));				//Reg52[5] = 1 for disable Auto video MUTE

			} else {

				hdmirxset(REG_RX_053, (B_TriVDIO), (
						  B_TriVDIO));							//Reg53[2:0] = 111;         // 1 for enable tri-state of video io data
				hdmirxset(REG_RX_053, (B_TriVDIO), (0x00));								//Reg53[2:0] = 000;         // 0 for enable video io data output

				hdmirxset(REG_RX_053, (B_VDGatting | B_VIOSel),
					  (B_VDGatting | B_VIOSel));	//Reg53[7][5] = 11    // for enable B_VDIO_GATTING and VIO_SEL
				hdmirxset(REG_RX_053, (B_VDGatting | B_VIOSel), (B_VIOSel));				//Reg53[7][5] = 01    // for disable B_VDIO_GATTING

				IT_INFO("+++++++++++  IT6602_SetVideoMute -> Off +++++++++++++++++\n");
			}
		}
	}
}



static void IT6602VideoOutputEnable(unsigned char bEnableOutput)
{
//	struct it6602_dev_data *it6602data = get_it6602_dev_data();
	if (bEnableOutput) {
		// enable output
		hdmirxset(REG_RX_053, (B_TriSYNC | B_TriVDIO), (0x00));
		IT_INFO("---------------- IT6602VideoOutputEnable -> On ----------------\n");
//		IT6602VideoOutputCDSet();

		//FIX_ID_016 xxxxx Support Dual Pixel Mode for IT66023 Only
#if defined(_IT66023_)
		IT66023JudgeOutputMode();
#endif
		//FIX_ID_016 xxxxx

	} else {
		// disable output
		hdmirxset(REG_RX_053, (B_TriSYNC | B_TriVDIO), (B_TriSYNC | B_TriVDIO));
		IT_INFO("---------------- IT6602VideoOutputEnable -> Off ----------------\n");

		//FIX_ID_016 xxxxx Support Dual Pixel Mode for IT66023 Only
#if defined(_IT66023_)
		hdmirxset(REG_RX_08C, 0x08, 0x00);		// Reg8C[3] = 0    // VDIO3en�G//  for disable QA IO
#endif
		//FIX_ID_016 xxxxx


	}
}


static void IT6602SwitchVideoState(struct it6602_dev_data *it6602, Video_State_Type  eNewVState)
{

	if (it6602->m_VState == eNewVState) {
		return;
	}

	IT_INFO("+++ %s\n", VStateStr[(unsigned char)eNewVState]);


	it6602->m_VState = eNewVState;
//	it6602->m_VideoCountingTimer = GetCurrentVirtualTime(); // get current time tick, and the next tick judge in the polling handler.

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
			// DLOG_Output(1000);
			hdmirxwr(0x84, 0x8F);	//2011/06/17 xxxxx, for enable Rx Chip count

#ifdef Enable_Vendor_Specific_packet
			hdmirxwr(REG_RX_06A, 0x81);
#endif

			it6602->m_ucSCDTOffCount = 0;

		}
		break;

	default :
		break;
	}

}

// ---------------------------------------------------------------------------
static void IT6602VideoHandler(struct it6602_dev_data *it6602)
{
//	unsigned char uc;
	usleep(1000 * 500);

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
		}
		break;

	case VSTATE_SyncChecking: {
			if (it6602->m_VideoCountingTimer == 0) {
				IT6602SwitchVideoState(it6602, VSTATE_VideoOn);
			}
		}
		break;

	case VSTATE_VideoOn: {
			if (it6602->m_NewAVIInfoFrameF == TRUE) {
				if (it6602->m_RxHDCPState != RxHDCP_ModeCheck) {
					IT6602VideoOutputConfigure(it6602);
					it6602->m_NewAVIInfoFrameF = FALSE;
				}
			}

			if (hdmirxrd(REG_RX_053)&B_VDGatting) {
				{
					if ((it6602->m_RAP_ContentOff == 0) && (it6602->m_HDCP_ContentOff == 0)) {
						if (CheckAVMute() == FALSE) {
							IT6602_SetVideoMute(it6602, OFF);
						}
					}
				}
			}
		}
		break;

	default :
		break;
	}
}

#endif

#ifdef _ITEHDMI_
// ***************************************************************************
// Interrupt function
// ***************************************************************************
static void hdmirx_INT_5V_Pwr_Chg(struct it6602_dev_data *it6602, unsigned char ucport)
{

	unsigned char ucPortSel;
	ucPortSel = hdmirxrd(REG_RX_051)&B_PORT_SEL;

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


//FIX_ID_037 xxxxx
}
// ---------------------------------------------------------------------------
static void hdmirx_INT_P0_ECC(struct it6602_dev_data *it6602)
{

	if ((it6602->m_ucEccCount_P0++) > ECC_TIMEOUT) {

#ifdef _SUPPORT_EQ_ADJUST_

		if (it6602->EQPort[F_PORT_SEL_0].f_manualEQadjust == TRUE) {	// ignore ECC interrupt when manual EQ adjust !!!
			return;
		}

#endif

		it6602->m_ucEccCount_P0 = 0;

		IT_INFO("CDR reset for Port0 ECC_TIMEOUT !!! ");

		hdmirx_ECCTimingOut(F_PORT_SEL_0);

	}
}

// ---------------------------------------------------------------------------
static void hdmirx_INT_P1_ECC(struct it6602_dev_data *it6602)
{


	if ((it6602->m_ucEccCount_P1++) > ECC_TIMEOUT) {
#ifdef _SUPPORT_EQ_ADJUST_

		if (it6602->EQPort[F_PORT_SEL_1].f_manualEQadjust == TRUE) {	// ignore ECC interrupt when manual EQ adjust !!!
			return;
		}

#endif

		it6602->m_ucEccCount_P1 = 0;

		IT_INFO("CDR reset for Port1 ECC_TIMEOUT !!! ");

		hdmirx_ECCTimingOut(F_PORT_SEL_1);
	
	}
}

// ---------------------------------------------------------------------------
static void hdmirx_INT_P0_Deskew(struct it6602_dev_data *it6602)
{
	if ((it6602->m_ucDeskew_P0++) > DESKEW_TIMEOUT) {
#ifdef _SUPPORT_EQ_ADJUST_

		if (it6602->EQPort[F_PORT_SEL_0].f_manualEQadjust == TRUE) {	// ignore ECC interrupt when manual EQ adjust !!!
			return;
		}

#endif
		it6602->m_ucDeskew_P0 = 0;

		IT_INFO("CDR reset for Port0 DESKEW_TIMEOUT !!! ");

		if (hdmirxrd(REG_RX_020) == 0x00) {
			hdmirxwr(REG_RX_020, 0x3F);        // Dr. Liu suggestion to 0x00

		} else {
			hdmirxwr(REG_RX_020, 0x00);        // Dr. Liu suggestion to 0x3F
		}

	}
}

// ---------------------------------------------------------------------------
static void hdmirx_INT_P1_Deskew(struct it6602_dev_data *it6602)
{
	if ((it6602->m_ucDeskew_P1++) > DESKEW_TIMEOUT) {
#ifdef _SUPPORT_EQ_ADJUST_

		if (it6602->EQPort[F_PORT_SEL_1].f_manualEQadjust == TRUE) {	// ignore ECC interrupt when manual EQ adjust !!!
			return;
		}

#endif

		it6602->m_ucDeskew_P1 = 0;

		IT_INFO("CDR reset for Port1 DESKEW_TIMEOUT !!! ");

		if (hdmirxrd(REG_RX_038) == 0x00) {
			hdmirxwr(REG_RX_038, 0x3F);        // Dr. Liu suggestion to 0x00

		} else {
			hdmirxwr(REG_RX_038, 0x00);        // Dr. Liu suggestion to 0x3F
		}
	}
}


// ---------------------------------------------------------------------------
//FIX_ID_009 xxxxx	//verify interrupt event with reg51[0] select port
static void hdmirx_INT_HDMIMode_Chg(struct it6602_dev_data *it6602, unsigned char ucport)
{
	unsigned char ucPortSel;
	ucPortSel = hdmirxrd(REG_RX_051)&B_PORT_SEL;
	IT_INFO("hdmirx_INT_HDMIMode_Chg = %d", ucPortSel);

	if (ucPortSel != ucport) {
		return;
	}

//FIX_ID_009 xxxxx

	if (IsHDMIMode()) {
		if (it6602->m_VState == VSTATE_VideoOn) {
			IT6602SwitchAudioState(it6602, ASTATE_RequestAudio);
		}

		it6602->m_bUpHDMIMode = TRUE ;
		IT_INFO("#### HDMI/DVI Mode : HDMI #### ");

	} else {
		IT6602SwitchAudioState(it6602, ASTATE_AudioOff);
		it6602->m_NewAVIInfoFrameF = FALSE;

		if (it6602->m_VState == VSTATE_VideoOn) {
			SetDVIVideoOutput(it6602);
		}

		it6602->m_bUpHDMIMode = FALSE ;
		IT_INFO("#### HDMI/DVI Mode : DVI #### ");
	}
}

// ---------------------------------------------------------------------------
static void hdmirx_INT_SCDT_Chg(struct it6602_dev_data *it6602)
{
	if (CheckSCDT(it6602) == TRUE) {
		IT_INFO("#### SCDT ON #### ");
		IT6602SwitchVideoState(it6602, VSTATE_SyncChecking);

	} else {
		IT_INFO("#### SCDT OFF #### ");
		IT6602SwitchVideoState(it6602, VSTATE_SyncWait);
		IT6602SwitchAudioState(it6602, ASTATE_AudioOff);

	}
}


//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_AUTO_EQ_
static void hdmirx_INT_EQ_FAIL(struct it6602_dev_data *it6602, unsigned char ucPortSel)
{
	if (ucPortSel > F_PORT_SEL_1) {
		return;
	}

#ifdef _SUPPORT_EQ_ADJUST_

	if (it6602->EQPort[ucPortSel].f_manualEQadjust == FALSE)	// ignore EQ fail interrupt when manual EQ adjust !!!
#endif
	{
		if (CheckPlg5VPwr(ucPortSel)) {

			//07-08
			if (ucPortSel	== 0) {
				if ((it6602->HDMIIntEvent & (B_PORT0_TMDSEvent))) {
					IT_INFO("#### hdmirx_INT_EQ_FAIL not yet !!! #### ");

//FIX_ID_033 xxxxx  //Fine-tune EQ Adjust function for HDCP receiver and repeater mode
					if ((it6602->HDMIIntEvent & (B_PORT0_Waiting)) == 0) {
						hdmirxwr(REG_RX_022, 0x00);	// power down auto EQ
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
					IT_INFO("#### hdmirx_INT_EQ_FAIL not yet !!! #### ");

//FIX_ID_033 xxxxx  //Fine-tune EQ Adjust function for HDCP receiver and repeater mode
					if ((it6602->HDMIIntEvent & (B_PORT1_Waiting)) == 0) {
						hdmirxwr(REG_RX_03A, 0x00);	// power down auto EQ
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
				//07-08
				if (ucPortSel	== 0) {
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
//FIX_ID_001 xxxxx
#endif


#ifdef _SUPPORT_EDID_RAM_
/*****************************************************************************/
/* EDID RAM  functions    *******************************************************/
/*****************************************************************************/

//static unsigned char UpdateEDIDRAM(_CODE unsigned char *pEDID,unsigned char BlockNUM)
static unsigned char UpdateEDIDRAM(unsigned char *pEDID, unsigned char BlockNUM)
{
	unsigned char  i, offset, sum = 0;

	if (BlockNUM == 0x02) {
		offset = 0x00 + 128 * 0x01;

	} else {
		offset = 0x00 + 128 * BlockNUM;
	}

	IT_INFO("block No =%02X offset = %02X \n", (int) BlockNUM, (int) offset);

	for (i = 0; i < 0x7F; i++) {
		EDID_RAM_Write(offset, 1, (pEDID + offset));

		IT_INFO("%02X ", (int) * (pEDID + offset));
		sum += *(pEDID + offset);
		offset ++;

	}

	sum = 0x00 - sum;
	return 	sum;
}

static void EnableEDIDupdata(void)
{
	IT_INFO("EnableEDIDupdata() \n");
	it6602HPDCtrl(0, 0);	// HDMI/MHL port 0, set HPD = 0
}

static void DisableEDIDupdata(void)
{
	
}


//static void EDIDRAMInitial(_CODE unsigned char *pIT6602EDID)
static void EDIDRAMInitial(unsigned char *pIT6602EDID)
{

	unsigned char Block0_CheckSum;
	unsigned char Block1_CheckSum;
	unsigned char u8_VSDB_Addr;
	unsigned char BlockNo;

	u8_VSDB_Addr = 0;

	EnableEDIDupdata();

	for (BlockNo = 0; BlockNo < 2; BlockNo++) {

		IT_INFO("IT6602 EDIDRAMInitial = %02X\n", (int) BlockNo);

		if (BlockNo == 0) {
			Block0_CheckSum =  UpdateEDIDRAM(pIT6602EDID, 0);
			hdmirxwr(REG_RX_0C4, Block0_CheckSum);		//Port 0 Bank 0 CheckSum
			hdmirxwr(REG_RX_0C8, Block0_CheckSum);		//Port 1 Bank 0 CheckSum

			IT_INFO(" Block0_CheckSum = %02X\n", (int) Block0_CheckSum);

		} else {
			Block1_CheckSum =  UpdateEDIDRAM(pIT6602EDID, 1);
			IT_INFO(" Block1_CheckSum = %02X\n", (int) Block1_CheckSum);
			u8_VSDB_Addr = Find_Phyaddress_Location(pIT6602EDID, 1);

			IT_INFO("u8_VSDB_Addr = %02X\n", (int) u8_VSDB_Addr);
			PhyAdrSet();

			if (u8_VSDB_Addr != 0) {

				UpdateEDIDReg(u8_VSDB_Addr, pIT6602EDID[u8_VSDB_Addr], pIT6602EDID[u8_VSDB_Addr + 1], Block1_CheckSum);
				IT_INFO("EDID Parsing OK\n");
			}
		}
	}

	DisableEDIDupdata();
}


//static unsigned char Find_Phyaddress_Location(_CODE unsigned char *pEDID,unsigned char Block_Number)
static unsigned char Find_Phyaddress_Location(unsigned char *pEDID, unsigned char Block_Number)
{
	unsigned char AddStart;
	unsigned char tag, count;
	unsigned char offset, End;
	unsigned char u8_VSDB_Addr;

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

	for (offset = (AddStart + 0x04); offset < (AddStart + End);) {


		tag = (*(pEDID + offset)) >> 5;
		count = (*(pEDID + offset)) & 0x1f;

		//#ifdef printf_EDID
		IT_INFO("offset = %X , Tag = %X , count =%X \n", (int) offset, (int)  tag, (int) count);
		//#endif

		offset++;

		if (tag == 0x03) {	// HDMI VSDB Block of EDID
			//#ifdef printf_EDID
			IT_INFO("HDMI VSDB Block address = %X\n", (int)  offset);
			//#endif

			if ((*(pEDID + offset)) == 0x03 &&
			    (*(pEDID + offset + 1)) == 0x0C &&
			    (*(pEDID + offset + 2)) == 0x0) {
				u8_VSDB_Addr = offset + 3;
				txphyadr[0] = (*(pEDID + offset + 3));
				txphyadr[1] = (*(pEDID + offset + 4));
				//#ifdef printf_EDID
				IT_INFO("txphyadr[0] = %X\n", (int)  txphyadr[0]);
				IT_INFO("txphyadr[1] = %X\n", (int)  txphyadr[1]);
				//#endif

				return u8_VSDB_Addr;
			}
		}

		offset = offset + count;
	}

	return 0;
}



static void UpdateEDIDReg(unsigned char u8_VSDB_Addr, unsigned char CEC_AB, unsigned char CEC_CD,
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
static void PhyAdrSet(void)
{
	rxphyadr[0][0] = 0x10;
	rxphyadr[0][1] = 0x00;
	rxphyadr[1][0] = 0x20;
	rxphyadr[1][1] = 0x00;
}

#endif

//FIX_ID_014 xxxxx
static void IT6602HDMIEventManager(struct it6602_dev_data *it6602)
{
	if (it6602->HDMIIntEvent != 0) {
//============================================================================

		if ((it6602->HDMIIntEvent & B_PORT0_Waiting) == B_PORT0_Waiting) {
			if (it6602->HDMIWaitNo[0] == 0) {
				it6602->HDMIIntEvent &= ~(B_PORT0_Waiting);
				IT_INFO("B_PORT0_Waiting  OK ...\n");

			} else {
				it6602->HDMIWaitNo[0]--;
				IT_INFO("B_PORT0_Waiting  %X ...Event=%X ...Reg93=%X \n",
					(int) it6602->HDMIWaitNo[0], (int) it6602->HDMIIntEvent, (int) hdmirxrd(0x93));
			}

		} else {
			if ((it6602->HDMIIntEvent & B_PORT0_TMDSEvent) == B_PORT0_TMDSEvent) {
				if (CLKCheck(F_PORT_SEL_0)) {
					IT_INFO("TMDSEvent &&&&& Port 0 Rx CKOn Detect &&&&& ");

					TMDSCheck(F_PORT_SEL_0);

					it6602->HDMIIntEvent &= ~(B_PORT0_TMDSEvent);	// finish MSC
				}

			} else if ((it6602->HDMIIntEvent & B_PORT0_TimingChgEvent) == B_PORT0_TimingChgEvent) {
				if (CLKCheck(F_PORT_SEL_0)) {
					IT_INFO("TimingChgEvent &&&&& Port 0 Rx CKOn Detect &&&&& ");
					//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_EQ_ADJUST_
					HDMIStartEQDetect(&(it6602->EQPort[F_PORT_SEL_0]));
#endif
					//FIX_ID_001 xxxxx

					it6602->HDMIIntEvent &= ~(B_PORT0_TimingChgEvent);	// finish MSC
				}
			}

		}

//============================================================================

//============================================================================

		if ((it6602->HDMIIntEvent & B_PORT1_Waiting) == B_PORT1_Waiting) {
			if (it6602->HDMIWaitNo[1] == 0) {
				it6602->HDMIIntEvent &= ~(B_PORT1_Waiting);
				IT_INFO("B_PORT1_Waiting  OK ...\n");

			} else {
				it6602->HDMIWaitNo[1]--;
				IT_INFO("B_PORT1_Waiting  %X ...\n", (int) it6602->HDMIWaitNo[1]);
			}

		} else {
			if ((it6602->HDMIIntEvent & B_PORT1_TMDSEvent) == B_PORT1_TMDSEvent) {
				if (CLKCheck(F_PORT_SEL_1)) {
					IT_INFO("TMDSEvent &&&&& Port 1 Rx CKOn Detect &&&&& ");

					TMDSCheck(F_PORT_SEL_1);

					it6602->HDMIIntEvent &= ~(B_PORT1_TMDSEvent);	// finish MSC
				}

			} else if ((it6602->HDMIIntEvent & B_PORT1_TimingChgEvent) == B_PORT1_TimingChgEvent) {
				if (CLKCheck(F_PORT_SEL_1)) {
					IT_INFO("TimingChgEvent &&&&& Port 1 Rx CKOn Detect &&&&& ");
#ifdef _SUPPORT_EQ_ADJUST_
					HDMIStartEQDetect(&(it6602->EQPort[F_PORT_SEL_1]));
#endif
					it6602->HDMIIntEvent &= ~(B_PORT1_TimingChgEvent);	// finish MSC
				}
			}

		}
	}

}

//FIX_ID_009 xxxxx	//verify interrupt event with reg51[0] select port
static unsigned char  IT6602_IsSelectedPort(unsigned char ucPortSel)
{
	unsigned char ucCurrentPort ;

	ucCurrentPort = hdmirxrd(REG_RX_051) & B_PORT_SEL;

	if (ucCurrentPort == ucPortSel) {
		return TRUE;

	} else {
		return FALSE;
	}
}
//FIX_ID_009 xxxxx

/*****************************************************************************/
/* Driver State Machine Process **********************************************/
/*****************************************************************************/
#ifdef _ITEHDMI_


static void IT6602HDMIInterruptHandler(struct it6602_dev_data *it6602)
{
	volatile unsigned char Reg05h;
	volatile unsigned char Reg06h;
	volatile unsigned char Reg07h;
	volatile unsigned char Reg08h;
	volatile unsigned char Reg09h;
	volatile unsigned char Reg0Ah;
//	volatile unsigned char Reg0Bh;
	volatile unsigned char RegD0h;

	unsigned char MHL04;
	unsigned char MHL05;
	unsigned char MHL06;
	unsigned char MHLA0;
	unsigned char MHLA1;
	unsigned char MHLA2;
	unsigned char MHLA3;

	chgbank(0);

	Reg05h = hdmirxrd(REG_RX_005);
	Reg06h = hdmirxrd(REG_RX_006);
	Reg07h = hdmirxrd(REG_RX_007);
	Reg08h = hdmirxrd(REG_RX_008);
	Reg09h = hdmirxrd(REG_RX_009);

	Reg0Ah = hdmirxrd(REG_RX_P0_SYS_STATUS);
//	Reg0Bh = hdmirxrd(REG_RX_P1_SYS_STATUS);
	RegD0h = hdmirxrd(REG_RX_0D0);


	hdmirxwr(REG_RX_005, Reg05h);
	hdmirxwr(REG_RX_006, Reg06h);
	hdmirxwr(REG_RX_007, Reg07h);
	hdmirxwr(REG_RX_008, Reg08h);
	hdmirxwr(REG_RX_009, Reg09h);
//2013-0606 disable ==>
	hdmirxwr(REG_RX_0D0, RegD0h & 0x0F);
	IT_INFO("---------ite interrupt------------");
	IT_INFO("Reg05 = %X", (int) Reg05h);
	IT_INFO("Reg06 = %X", (int) Reg06h);
	IT_INFO("Reg07 = %X", (int) Reg07h);
	IT_INFO("Reg08 = %X", (int) Reg08h);
	IT_INFO("Reg09 = %X", (int) Reg09h);
	IT_INFO("Reg0A = %X", (int) Reg0Ah);
	IT_INFO("RegD0 = %X", (int) RegD0h);
	MHL04 = mhlrxrd(0x04);
	MHL05 = mhlrxrd(0x05);
	MHL06 = mhlrxrd(0x06);

	mhlrxwr(0x04, MHL04);
	mhlrxwr(0x05, MHL05);
	mhlrxwr(0x06, MHL06);

	MHLA0 = mhlrxrd(0xA0);
	MHLA1 = mhlrxrd(0xA1);
	MHLA2 = mhlrxrd(0xA2);
	MHLA3 = mhlrxrd(0xA3);

	mhlrxwr(0xA0, MHLA0);
	mhlrxwr(0xA1, MHLA1);
	mhlrxwr(0xA2, MHLA2);
	mhlrxwr(0xA3, MHLA3);


	hdmirxrd(REG_RX_005);
	hdmirxrd(REG_RX_006);
	hdmirxrd(REG_RX_007);
	hdmirxrd(REG_RX_008);

	hdmirxrd(REG_RX_009);
	hdmirxrd(REG_RX_P0_SYS_STATUS);
	hdmirxrd(REG_RX_0D0);

	if (Reg05h != 0x00) {

		IT_INFO("Reg05 = %X", (int) Reg05h);

		if (Reg05h & 0x80) {
			IT_INFO("#### Port 0 HDCP Off Detected ### ");
			it6602->m_ucEccCount_P0 = 0;
		}

		if (Reg05h & 0x40) {
			IT_INFO("#### Port 0 ECC Error %X #### ", (int)(it6602->m_ucEccCount_P0));
			hdmirx_INT_P0_ECC(it6602);
		}

		if (Reg05h & 0x20) {
			IT_INFO("#### Port 0 HDMI/DVI Mode change #### ");

			if (CLKCheck(0)) {
				hdmirx_INT_HDMIMode_Chg(it6602, 0);
			}

		}

		if (Reg05h & 0x08) {
			IT_INFO("#### Port 0 HDCP Authentication Start #### ");
			it6602->m_ucEccCount_P0 = 0;

#ifdef _SUPPORT_AUTO_EQ_

//FIX_ID_014 xxxxx
			if (ucPortAMPOverWrite[F_PORT_SEL_0] == 0) {
				if ((it6602->HDMIIntEvent & (B_PORT0_Waiting)) == 0) {
					hdmirxwr(REG_RX_022, 0x00);	// power down auto EQ

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

//FIX_ID_014 xxxxx
#endif

			if ((Reg0Ah & 0x40)) {
				it6602->CBusIntEvent |= (B_MSC_Waiting);
				it6602->CBusWaitNo = MAX_CBUS_WAITNO;
			}

		}

		if (Reg05h & 0x10) {
			IT_INFO("#### Port 0 HDCP Authentication Done #### ");

			if ((Reg0Ah & 0x40)) {
				it6602->CBusIntEvent |= (B_MSC_Waiting);
				it6602->CBusWaitNo = MAX_CBUS_WAITNO;
			}


#ifdef _SUPPORT_AUTO_EQ_

			//FIX_ID_033 xxxxx  //Fine-tune EQ Adjust function for HDCP receiver and repeater mode
			//FIX_ID_032 xxxxx	//Support HDCP Repeater function for HDMI Tx device
			//xxxxx 2014-0421
			//FIX_ID_014 xxxxx
			if (ucPortAMPOverWrite[0] == 0) {	// 2013-0801
				it6602->HDMIIntEvent &= ~(B_PORT0_Waiting);
				it6602->HDMIWaitNo[0] = 0;
				it6602->HDMIIntEvent |= B_PORT0_TMDSEvent;
				//return;
			}

			//FIX_ID_014 xxxxx
			//xxxxx 2014-0421
			//FIX_ID_032 xxxxx
			//FIX_ID_033 xxxxx
#endif

		}

		if (Reg05h & 0x04) {
			IT_INFO("#### Port 0 Input Clock Change Detect #### ");
		}

		if (Reg05h & 0x02) {

			it6602->m_ucEccCount_P0 = 0;
			it6602->m_ucDeskew_P0 = 0;
			//it6602->m_ucDeskew_P1=0;
			//it6602->m_ucEccCount_P1=0;

			IT_INFO("#### Port 0 Rx CKOn Detect #### ");

//#ifdef _SUPPORT_HDCP_REPEATER_

//FIX_ID_032 xxxxx	//Support HDCP Repeater function for HDMI Tx device
			if (CLKCheck(F_PORT_SEL_0)) {
				TMDSCheck(F_PORT_SEL_0);
			}
		}

		if (Reg05h & 0x01) {
			IT_INFO("#### Port 0 Power 5V change #### ");
			hdmirx_INT_5V_Pwr_Chg(it6602, 0);
		}
	}

	if (Reg06h != 0x00) {
		//IT_INFO("Reg06h = %X",(int) Reg06h);
		if (Reg06h & 0x80) {
			IT_INFO("#### Port 1 HDCP Off Detected ### ");
			it6602->m_ucEccCount_P1 = 0;

		}

		if (Reg06h & 0x40) {
			IT_INFO("#### Port 1 ECC Error #### ");
			hdmirx_INT_P1_ECC(it6602);
		}

		if (Reg06h & 0x20) {
			IT_INFO("#### Port 1 HDMI/DVI Mode change #### ");

			if (CLKCheck(1)) {
				hdmirx_INT_HDMIMode_Chg(it6602, 1);
			}

		}

		if (Reg06h & 0x08) {
			IT_INFO("#### Port 1 HDCP Authentication Start #### ");
			it6602->m_ucEccCount_P1 = 0;

#ifdef _SUPPORT_AUTO_EQ_

			if (ucPortAMPOverWrite[F_PORT_SEL_1] == 0) {
				if ((it6602->HDMIIntEvent & (B_PORT1_Waiting)) == 0) {
					IT_INFO(" power down auto EQ of PORT 1 ");
					hdmirxwr(REG_RX_03A, 0x00);	// power down auto EQ

					it6602->HDMIIntEvent |= (B_PORT1_Waiting);
					it6602->HDMIIntEvent |= (B_PORT1_TMDSEvent);
					it6602->HDMIWaitNo[1] = MAX_TMDS_WAITNO;

				} else if ((it6602->HDMIIntEvent & (B_PORT1_TMDSEvent))) {
					it6602->HDMIIntEvent |= (B_PORT1_Waiting);
					it6602->HDMIWaitNo[1] += MAX_HDCP_WAITNO;
				}

			} else {
				if ((it6602->HDMIIntEvent & (B_PORT1_TMDSEvent))) {
					it6602->HDMIIntEvent |= (B_PORT1_Waiting);
					it6602->HDMIWaitNo[1] += MAX_HDCP_WAITNO;
				}
			}

#endif

		}

		if (Reg06h & 0x10) {
			IT_INFO("#### Port 1 HDCP Authentication Done #### ");

			if ((it6602->HDMIIntEvent & (B_PORT1_Waiting))) {
				it6602->HDMIWaitNo[1] = 0;
			}


#ifdef _SUPPORT_AUTO_EQ_

//FIX_ID_033 xxxxx  //Fine-tune EQ Adjust function for HDCP receiver and repeater mode
// disable ->	if( ucPortAMPValid[F_PORT_SEL_1] == 0)
			if (ucPortAMPOverWrite[1] == 0) {	// 2013-0801
				it6602->HDMIIntEvent &= ~(B_PORT1_Waiting);
				it6602->HDMIWaitNo[1] = 0;
				it6602->HDMIIntEvent |= B_PORT1_TMDSEvent;
			}

//FIX_ID_033 xxxxx
#endif


		}



		if (Reg06h & 0x04) {
			IT_INFO("#### Port 1 Input Clock Change Detect #### ");
		}

		if (Reg06h & 0x02) {
			IT_INFO("#### Port 1 Rx CKOn Detect #### ");
			//it6602->m_ucEccCount_P0=0;
			//it6602->m_ucDeskew_P0=0;
			it6602->m_ucDeskew_P1 = 0;
			it6602->m_ucEccCount_P1 = 0;

//FIX_ID_033 xxxxx  //Fine-tune EQ Adjust function for HDCP receiver and repeater mode
//FIX_ID_032 xxxxx	//Support HDCP Repeater function for HDMI Tx device

			// NO --> Authentication Start 	&& 	Input Clock Change Detect 	&&	 B_PORT1_TMDSEvent
			if ((Reg06h & 0x08) == 0 && (Reg06h & 0x04) == 0  && (it6602->HDMIIntEvent & (B_PORT1_TMDSEvent)) == 0) {
				if (CLKCheck(F_PORT_SEL_1)) {
#ifdef _SUPPORT_AUTO_EQ_
					TMDSCheck(F_PORT_SEL_1);
#endif
				}

			} else {
				if ((Reg06h & 0x10) == 0) {
					if ((it6602->HDMIIntEvent & (B_PORT1_Waiting)) == 0) {
						hdmirxwr(REG_RX_03A, 0x00);	// power down auto EQ
						it6602->HDMIIntEvent |= (B_PORT1_Waiting);
						it6602->HDMIIntEvent |= (B_PORT1_TMDSEvent);
						it6602->HDMIWaitNo[1] = MAX_TMDS_WAITNO;
					}

				} else {
					if (CLKCheck(F_PORT_SEL_1)) {
						TMDSCheck(F_PORT_SEL_1);
					}
				}
			}

		}

		if (Reg06h & 0x01) {
			IT_INFO("#### Port 1 Power 5V change #### ");
			hdmirx_INT_5V_Pwr_Chg(it6602, 1);
		}

	}

	if (Reg07h != 0x00) {
		IT_INFO("Reg07h = %X", (int) Reg07h);

		if (Reg07h & 0x80) {
			IT_INFO("#### Audio FIFO Error #### ");
			aud_fiforst();

		}

		if (Reg07h & 0x40) {
			IT_INFO("#### Audio Auto Mute #### ");
		}

		if (Reg07h & 0x20) {
			IT_INFO("#### Packet Left Mute #### ");
			IT6602_SetVideoMute(it6602, OFF);
		}

		if (Reg07h & 0x10) {
			IT_INFO("#### Set Mute Packet Received #### ");

			IT6602_SetVideoMute(it6602, ON);
		}

		if (Reg07h & 0x08) {
			IT_INFO("#### Timer Counter Tntterrupt #### ");

		}

		if (Reg07h & 0x04) {
			IT_INFO("#### Video Mode Changed #### ");
		}

		if (Reg07h & 0x02) {
			hdmirx_INT_SCDT_Chg(it6602);
		}

		if (Reg07h & 0x01) {

			if ((Reg0Ah & 0x40) >> 6) {
				IT_INFO("#### Port 0 Bus Mode : MHL ####\r\n");

				chgbank(1);
				hdmirxset(REG_RX_1B6, 0x07, 0x00);
				//FIX_ID_007 xxxxx 	//for debug IT6681  HDCP issue
				hdmirxset(REG_RX_1B1, 0x20, 0x20); //Reg1b1[5] = 1 for enable over-write
				hdmirxset(REG_RX_1B2, 0x07, 0x01); // default 0x04 , change to 0x01
				IT_INFO(" Port 0 Bus Mode Reg1B1  = %X ,Reg1B2  = %X\r\n", (int)hdmirxrd(REG_RX_1B1), (int)hdmirxrd(REG_RX_1B2));
				//FIX_ID_007 xxxxx
				chgbank(0);

			} else {
				IT_INFO("#### Port 0 Bus Mode : HDMI ####\r\n");

				//FIX_ID_002 xxxxx 	Check IT6602 chip version Identify for TogglePolarity and Port 1 Deskew
				chgbank(1);
				hdmirxset(REG_RX_1B6, 0x07, 0x03);
				////FIX_ID_007 xxxxx 	//for debug IT6681  HDCP issue
				hdmirxset(REG_RX_1B1, 0x20, 0x00); //Reg1b1[5] = 0 for disable over-write
				hdmirxset(REG_RX_1B2, 0x07, 0x04); // default 0x04 , change to 0x01
				IT_INFO(" Port 0 Bus Mode Reg1B1  = %X ,Reg1B2  = %X\r\n", (int)hdmirxrd(REG_RX_1B1), (int)hdmirxrd(REG_RX_1B2));
				////FIX_ID_007 xxxxx
				chgbank(0);

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
			IT_INFO("#### No Audio InfoFrame Received #### ");
		}

		if (Reg08h & 0x10) {
			IT_INFO("#### No AVI InfoFrame Received #### ");
		}

		if (Reg08h & 0x08) {
			IT_INFO("#### CD Detect #### ");

		}

		if (Reg08h & 0x04) {
			//			 IT_INFO("#### Gen Pkt Detect ####\n");
			IT_INFO("#### 3D InfoFrame Detect #### ");

#ifdef Enable_Vendor_Specific_packet

			if (it6602->f_de3dframe_hdmi == FALSE) {
				it6602->f_de3dframe_hdmi = IT6602_DE3DFrame(TRUE);
			}

#endif

		}

		if (Reg08h & 0x02) {
			IT_INFO("#### ISRC2 Detect #### ");
		}

		if (Reg08h & 0x01) {
			IT_INFO("#### ISRC1 Detect #### ");
		}
	}

	if (Reg09h != 0x00) {
		//IT_INFO("Reg09h = %X",(int) Reg09h);
		if (Reg09h & 0x80) {
			IT_INFO("#### H2V Buffer Skew Fail #### ");
		}

		if (Reg09h & 0x40) {

			//FIX_ID_002 xxxxx 	Check IT6602 chip version Identify for TogglePolarity and Port 1 Deskew
			hdmirxwr(0x09, 0x40);
			//FIX_ID_002 xxxxx
			IT_INFO("#### Port 1 Deskew Error #### ");
			hdmirx_INT_P1_Deskew(it6602);
		}

		if (Reg09h & 0x20) {
			hdmirxwr(0x09, 0x20);
			IT_INFO("#### Port 0 Deskew Error #### ");
			hdmirx_INT_P0_Deskew(it6602);
		}

		if (Reg09h & 0x10) {
			IT_INFO("#### New Audio Packet Received #### ");
		}

		if (Reg09h & 0x08) {
			IT_INFO("#### New ACP Packet Received #### ");
		}

		if (Reg09h & 0x04) {
			IT_INFO("#### New SPD Packet Received #### ");
		}

		if (Reg09h & 0x02) {
			IT_INFO("#### New MPEG InfoFrame Received #### ");
		}

		if (Reg09h & 0x01) {
			IT_INFO("#### New AVI InfoFrame Received #### ");
			//IT6602VideoOutputConfigure();
			it6602->m_NewAVIInfoFrameF = TRUE;
		}

	}


	if (RegD0h != 0x00) {

		if (RegD0h & 0x10) {

			hdmirxwr(0xD0, 0x30);
			RegD0h &= 0x30;

			IT_INFO("#### Port 0 EQ done interrupt #### ");

#ifdef _SUPPORT_AUTO_EQ_
			//2013-0923 disable ->	ucPortAMPOverWrite[0]=1;	//2013-0801
			AmpValidCheck(0);	//2013-0801
#endif

//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_EQ_ADJUST_
			HDMIStartEQDetect(&(it6602->EQPort[F_PORT_SEL_0]));
#endif
//FIX_ID_001 xxxxx

		}

		if (RegD0h & 0x40) {

			hdmirxwr(0xD0, 0xC0);
			RegD0h &= 0xC0;

			IT_INFO("#### Port 1 EQ done interrupt #### ");


#ifdef _SUPPORT_AUTO_EQ_
			//2013-0923 disable ->	ucPortAMPOverWrite[1]=1;	//2013-0801
			AmpValidCheck(1);	//2013-0801
#endif
		}

		if (RegD0h & 0x20) {

			hdmirxwr(0xD0, 0x20);
			IT_INFO("#### Port 0 EQ Fail Interrupt #### ");
//	HDMICheckErrorCount(&(it6602->EQPort[F_PORT_SEL_0]));	//07-04 for port 0
//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_AUTO_EQ_
			hdmirx_INT_EQ_FAIL(it6602, F_PORT_SEL_0);
#endif
//FIX_ID_001 xxxxx
		}

		if (RegD0h & 0x80) {

			hdmirxwr(0xD0, 0x80);
			IT_INFO("#### Port 1 EQ Fail Interrupt #### ");
//	HDMICheckErrorCount(&(it6602->EQPort[F_PORT_SEL_1]));	//07-04 for port 0
//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_AUTO_EQ_
			hdmirx_INT_EQ_FAIL(it6602, F_PORT_SEL_1);
#endif
//FIX_ID_001 xxxxx
		}
	}

}
void IT6602_Interrupt(void)
{
	struct it6602_dev_data *it6602data = get_it6602_dev_data();
	IT6602HDMIInterruptHandler(it6602data);
}

void IT6602_fsm(void)
{
	struct it6602_dev_data *it6602data = get_it6602_dev_data();

	IT6602VideoHandler(it6602data);

	IT6602AudioHandler(it6602data);


//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_EQ_ADJUST_

	if (it6602data->EQPort[F_PORT_SEL_0].f_manualEQadjust == TRUE) {
		IT_INFO("[IT6602_fsm]: f_manualEQadjust == TRUE \n");
		HDMIAdjustEQ(&(it6602data->EQPort[F_PORT_SEL_0]));        // for port 0
	}

	if (it6602data->EQPort[F_PORT_SEL_1].f_manualEQadjust == TRUE) {
		HDMIAdjustEQ(&(it6602data->EQPort[F_PORT_SEL_1]));        // for port 1
	}

#endif


	IT6602HDMIEventManager(it6602data);

}

void get_vid_info(void)
{
#if 1
	int HSyncPol, VSyncPol, InterLaced;
	int HTotal, HActive, HFP, HSYNCW;
	int VTotal, VActive, VFP, VSYNCW;

	unsigned int ucTMDSClk = 0;	//, sumt;
	unsigned char ucPortSel;
	unsigned char rddata;
	unsigned char ucClk;
	int PCLK;	//, sump;

	rddata = hdmirxrd(0x9A);
	PCLK = (124 * 255 / rddata) / 10;


	ucPortSel = hdmirxrd(REG_RX_051) & B_PORT_SEL;
	rddata = hdmirxrd(0x90);

	if (ucPortSel == F_PORT_SEL_1) {

		IT_INFO("Reg51[0] = 1 Active Port HDMI  ");
		ucClk = hdmirxrd(REG_RX_092) ;

		if (ucClk != 0) {

			if (rddata & 0x04) {
				ucTMDSClk = 2 * RCLKVALUE * 256 / ucClk;

			} else if (rddata & 0x08) {
				ucTMDSClk = 4 * RCLKVALUE * 256 / ucClk;

			} else {
				ucTMDSClk = RCLKVALUE * 256 / ucClk;
			}

			IT_INFO(" Port 1 TMDS CLK  = %d  ", (int)ucTMDSClk);
		}


	} else {
		IT_INFO("Reg51[0] = 0 Active Port MHL  ");
		ucClk = hdmirxrd(REG_RX_091) ;

		if (ucClk != 0) {
			if (rddata & 0x01) {
				ucTMDSClk = 2 * RCLKVALUE * 256 / ucClk;

			} else if (rddata & 0x02) {
				ucTMDSClk = 4 * RCLKVALUE * 256 / ucClk;

			} else {
				ucTMDSClk = RCLKVALUE * 256 / ucClk;
			}

			IT_INFO("Port 0 TMDS CLK  = %d  ", (int)ucTMDSClk);
		}
	}


	InterLaced = (hdmirxrd(0x99) & 0x02) >> 1;

	HTotal   = ((hdmirxrd(0x9D) & 0x3F) << 8) + hdmirxrd(0x9C);
	HActive  = ((hdmirxrd(0x9F) & 0x3F) << 8) + hdmirxrd(0x9E);
	HFP      = ((hdmirxrd(0xA1) & 0xF0) << 4) + hdmirxrd(0xA2);
	HSYNCW   = ((hdmirxrd(0xA1) & 0x01) << 8) + hdmirxrd(0xA0);
	HSyncPol = hdmirxrd(0xA8) & 0x04 >> 2;

	VTotal   = ((hdmirxrd(0xA4) & 0x0F) << 8) + hdmirxrd(0xA3);
	VActive  = ((hdmirxrd(0xA4) & 0xF0) << 4) + hdmirxrd(0xA5);
	VFP      = hdmirxrd(0xA7) & 0x3F;
	VSYNCW   = hdmirxrd(0xA6) & 0x1F;
	VSyncPol = (hdmirxrd(0xA8) & 0x08) >> 3;

//	CurVTiming.TMDSCLK     = (int)TMDSCLK;
	CurTMDSCLK             = (int)ucTMDSClk;
	CurVTiming.PCLK        = (int)PCLK;
	CurVTiming.HActive     = HActive;
	CurVTiming.HTotal      = HTotal;
	CurVTiming.HFrontPorch = HFP;
	CurVTiming.HSyncWidth  = HSYNCW;
	CurVTiming.HBackPorch  = HTotal - HActive - HFP - HSYNCW;
	CurVTiming.VActive     = VActive;
	CurVTiming.VTotal      = VTotal;
	CurVTiming.VFrontPorch = VFP;
	CurVTiming.VSyncWidth  = VSYNCW;
	CurVTiming.VBackPorch  = VTotal - VActive - VFP - VSYNCW;
	CurVTiming.ScanMode    = (InterLaced) & 0x01;
	CurVTiming.VPolarity   = (VSyncPol) & 0x01;
	CurVTiming.HPolarity   = (HSyncPol) & 0x01;
#endif
}



void show_vid_info(void)
{
#if 1
	// int InBPC, InBPP;
	int MHL_Mode;
	int MHL_CLK_Mode;
	int GCP_CD       = CD8BIT; //24 bits per pixel

	unsigned long FrameRate;

	GCP_CD = ((hdmirxrd(0x99) & 0xF0) >> 4);

	switch (GCP_CD) {
	case 5 :
		IT_INFO("I/P ColorDepth = 30 bits per pixel  ");
		// InBPC = 10;
		hdmirxset(0x65, 0x0C, 0x04);
		OutCD = OUT10B;
		break;

	case 6 :
		IT_INFO("I/P ColorDepth = 36 bits per pixel  ");
		// InBPC = 12;
		hdmirxset(0x65, 0x0C, 0x08);
		OutCD = OUT12B;
		break;

	default :
		IT_INFO("I/P ColorDepth = 24 bits per pixel  ");
		// InBPC = 8;
		hdmirxset(0x65, 0x0C, 0x00);
		OutCD = OUT8B;
		break;
	}

	switch (OutCD) {
	case 1 :
		IT_INFO("O/P ColorDepth = 30 bits per pixel  ");
		break;

	case 2 :
		IT_INFO("O/P ColorDepth = 36 bits per pixel  ");
		break;

	default :
		IT_INFO("O/P ColorDepth = 24 bits per pixel  ");
		break;
	}

	chgbank(2);
	InColorMode = (hdmirxrd(0x15) & 0x60) >> 5;
	chgbank(0);

	switch (InColorMode) {
	case 0 :
		IT_INFO("Input Color Mode = RGB444 \n");
		//		 hdmirxset(0xAE, 0x01, 0x01);
		//		 defaultrgb();
		break;

	case 1 :
		IT_INFO("Input Color Mode = YCbCr422\n");
		//		 hdmirxset(0xAE, 0x01, 0x00);
		//		 yuv422torgb();
		break;

	case 2 :
		IT_INFO("Input Color Mode = YCbCr444\n");
		//		 hdmirxset(0xAE, 0x01, 0x00);
		//		 yuv444torgb();
		break;

	default :
		IT_INFO("Input Color Mode = Reserved !!!\n");
		break;
	}


	OutColorMode = (hdmirxrd(0x65) & 0x30) >> 4;

	switch (OutColorMode) {
	case 0 :
		IT_INFO("Output Color Mode = RGB444\n");
		//		 hdmirxset(0x65, 0x30, 0x00);
		break;

	case 1 :
		IT_INFO("Output Color Mode = YCbCr422\n");
		//		 hdmirxset(0x65, 0x30, 0x10);
		break;

	case 2 :
		IT_INFO("Output Color Mode = YCbCr444\n");
		//		 hdmirxset(0x65, 0x30, 0x20);
		break;

	default :
		IT_INFO("Output Color Mode = Reserved !!!\n");
		break;
	}

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
		IT_INFO("Reg40=%X  ", (int)hdmirxrd(REG_RX_040));
		IT_INFO("Reg41=%X  ", (int)hdmirxrd(REG_RX_041));
		chgbank(1);
		IT_INFO("Rec_B_CS=%X  ", (int)(hdmirxrd(REG_RX_1DD) & 0x80) >> 7);
		IT_INFO("Rec_G_CS=%X  ", (int)(hdmirxrd(REG_RX_1DE) & 0x80) >> 7);
		IT_INFO("Rec_R_CS=%X  \n", (int)(hdmirxrd(REG_RX_1DF) & 0x80) >> 7);
		IT_INFO("Rec_B_RS=%X  ", (int)(hdmirxrd(REG_RX_1DD) & 0x7F));
		IT_INFO("Rec_G_RS=%X  ", (int)(hdmirxrd(REG_RX_1DE) & 0x7F));
		IT_INFO("Rec_R_RS=%X  \n", (int)(hdmirxrd(REG_RX_1DF) & 0x7F));
		IT_INFO(" Reg1C1  = %X , Reg1C2  = %X ", (int)hdmirxrd(REG_RX_1C1), (int)hdmirxrd(REG_RX_1C2));
		chgbank(0);

	} else {

		IT_INFO("Port= 0 ,Reg11=%X ,", (int)hdmirxrd(REG_RX_011));
		IT_INFO("Reg20=%X, ", (int)hdmirxrd(REG_RX_020));
		IT_INFO("Reg26=%X, ", (int)hdmirxrd(REG_RX_026));
		IT_INFO("Reg27=%X, ", (int)hdmirxrd(REG_RX_027));
		IT_INFO("Reg28=%X, ", (int)hdmirxrd(REG_RX_028));
		IT_INFO("Reg29=%X  ", (int)hdmirxrd(REG_RX_029));
		chgbank(1);
		IT_INFO("Rec_B_CS=%X  ", (int)(hdmirxrd(REG_RX_1D5) & 0x80) >> 7);
		IT_INFO("Rec_G_CS=%X  ", (int)(hdmirxrd(REG_RX_1D6) & 0x80) >> 7);
		IT_INFO("Rec_R_CS=%X  \n", (int)(hdmirxrd(REG_RX_1D7) & 0x80) >> 7);

		IT_INFO("Rec_B_RS=%X  ", (int)(hdmirxrd(REG_RX_1D5) & 0x7F));
		IT_INFO("Rec_G_RS=%X  ", (int)(hdmirxrd(REG_RX_1D6) & 0x7F));
		IT_INFO("Rec_R_RS=%X  \n", (int)(hdmirxrd(REG_RX_1D7) & 0x7F));
		IT_INFO("REG_RX_1B1 = %X ,  REG_RX_1B2 = %X ", (int)hdmirxrd(REG_RX_1B1), (int)hdmirxrd(REG_RX_1B2));


		chgbank(0);
	}

	IT_INFO("TMDSCLK = %d MHz\n", (int)(CurTMDSCLK));
	IT_INFO("PCLK = %d MHz\n", (int)(CurVTiming.PCLK));
	IT_INFO("HActive = %d\n", CurVTiming.HActive);
	IT_INFO("VActive = %d\n", CurVTiming.VActive);
	IT_INFO("HTotal = %d\n", CurVTiming.HTotal);
	IT_INFO("VTotal = %d\n", CurVTiming.VTotal);

	MHL_Mode = 0;

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

#endif



/*********************************************************************************/
/* End of ITEHDMI.c ***************************************************************/
/*********************************************************************************/


#ifdef Enable_Vendor_Specific_packet

#define HDMI_3DFORMAT_PRESENT           0x40
#define HDMI_3DFORMAT_OFF               0x00
#define FRAME_PACKING                   0x00
#define TOP_AND_BOTTOM                  0x60
#define SIDE_BY_SIDE                    0x80


SET_DE3D_FRAME t_3d_syncgen[] = {
	//640x480      //524   //559   //514   //526
	{0x01, 0x020C, 0x022F, 0x0202, 0x020E,	480},         // 60Hz
	//480p      //524   //560   //515   //530
	{0x02, 0x020C, 0x0230, 0x0203, 0x0212,	480},         // 60Hz
	{0x03, 0x020C, 0x0230, 0x0203, 0x0212,	480},         // 60Hz
	//576p      //624   //668   //619   //629
	{0x11, 0x0270, 0x029C, 0x026B, 0x0275,	576},         // 50Hz
	{0x12, 0x0270, 0x029C, 0x026B, 0x0275,	576},         // 50Hz
	//720p      //749   //774   //744   //754
	{0x3c, 0x02ED, 0x0306, 0x02E8, 0x02F2,	720},         // 24Hz
	{0x3d, 0x02ED, 0x0306, 0x02E8, 0x02F2,	720},         // 25Hz
	{0x3e, 0x02ED, 0x0306, 0x02E8, 0x02F2,	720},         // 30Hz
	{0x13, 0x02ED, 0x0306, 0x02E8, 0x02F2,	720},         // 50Hz
	{0x04, 0x02ED, 0x0306, 0x02E8, 0x02F2,	720},         // 60Hz

	//1080p    //1124   //1165   //1120   //1129
	{0x20, 0x0464, 0x048D, 0x0460, 0x0469,	1080},         // 24Hz
	{0x21, 0x0464, 0x048D, 0x0460, 0x0469,	1080},         // 25Hz
	{0x22, 0x0464, 0x048D, 0x0460, 0x0469,	1080},         // 30Hz

	//default
	{0xFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000}
};

#define Reg_PGVTotal_19D	0x9D	//[11:4]	0x19D[7:0]
#define Reg_PGVTotal_19C	0x9C	//[3:0]		0x19C[7:4]
#define Reg_PGVActSt_192	0x92	//[7:0]		0x192[7:0]
#define Reg_PGVActSt_193	0x93	//[11:8]	0x193[3:0]
#define Reg_PGVActEd_193	0x93	//[3:0]		0x193[7:4]
#define Reg_PGVActEd_194	0x94	//[11:4]	0x194[7:0]
#define Reg_PGVSyncEd_19F	0x9F	//[3:0]		0x19F[7:4]
#define Reg_PGVSyncSt_19F	0x9F	//[11:8]	0x19F[3:0]
#define Reg_PGVSyncSt_19E	0x9E	//[7:0]		0x19E[7:0]

#define Reg_PG3DRSt_18F		0x8F	//[7:0]		0x190[11:8] 0x18F[7:0]
#define Reg_PG3DRStEd_190	0x90	//[7:0]		0x191[3:0] 0x18F[11:8]
#define Reg_PG3DREd_191	0x91	//[11:4]		0x191[11:4] 0x190[3:0]

#define REG_RX_066_4_DE3DFrame	0x66	//[4] 1: 3D frame-packet mode to sequence mode
#define REG_RX_085_5_En3DROut 		0x85	//[5] 1: Enable 3DR output


static void Dump3DReg(void)
{
	ushort	i, j ;
	BYTE ucData ;

	IT_INFO("        ");

	for (j = 0 ; j < 16 ; j++) {
		IT_INFO(" %02X", (int) j);

		if ((j == 3) || (j == 7) || (j == 11)) {
			IT_INFO(" :");
		}
	}

	IT_INFO(" ");

	chgbank(1);

	for (i = 0x80 ; i < 0xa0 ; i += 16) {
		IT_INFO("[%03X]  ", i);

		for (j = 0 ; j < 16 ; j++) {
			ucData = hdmirxrd((BYTE)((i + j) & 0xFF)) ;
			IT_INFO(" %02X", (int) ucData);

			if ((j == 3) || (j == 7) || (j == 11)) {
				IT_INFO(" :");
			}
		}

		IT_INFO(" ");

	}

	IT_INFO("\n        ===================================================== ");

	chgbank(0);

	UNUSED(ucData);
}

static unsigned char IT6602_DE3DFrame(unsigned char ena_de3d)
/*
 * This function configures the HDMI DE3DFrame
 * @param uunsigned char ena_de3d
 * @return      TRUE
 *              FALSE
 */
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
	//dbmsg_trace(DBM_DPATH,"ITEHDMI - HDMI_DE3DFrame  ");
#endif

	struct it6602_dev_data *it6602data = get_it6602_dev_data();

	if (ena_de3d  == TRUE) {

		chgbank(2);
		uc = hdmirxrd(REG_RX_224);
		chgbank(0);

		if (uc == 0x81) { // 3D InfoFrame Packet Type is valid

			chgbank(2);
			it6602data->s_Current3DFr.VIC = hdmirxrd(REG_RX_218);	//AVI INFO PB4
			it6602data->s_Current3DFr.HB0 = hdmirxrd(REG_RX_224);	// General Packet Header Byte 0
			it6602data->s_Current3DFr.HB1 = hdmirxrd(REG_RX_225);
			it6602data->s_Current3DFr.HB2 = hdmirxrd(REG_RX_226);
			it6602data->s_Current3DFr.PB0 = hdmirxrd(REG_RX_227);	// General Packet Data Byte 0
			it6602data->s_Current3DFr.PB1 = hdmirxrd(REG_RX_228);
			it6602data->s_Current3DFr.PB2 = hdmirxrd(REG_RX_229);
			it6602data->s_Current3DFr.PB3 = hdmirxrd(REG_RX_22A);
			it6602data->s_Current3DFr.PB4 = hdmirxrd(REG_RX_22B);
			it6602data->s_Current3DFr.PB5 = hdmirxrd(REG_RX_22C);
			it6602data->s_Current3DFr.PB6 = hdmirxrd(REG_RX_22D);
			it6602data->s_Current3DFr.PB7 = hdmirxrd(REG_RX_22E);
			chgbank(0);

			IT_INFO(" IT653x - HDMI_DumpDE3DFrameInfo:  ");
			IT_INFO("        HDMI VIC = 0x%X  ", it6602data->s_Current3DFr.VIC);
			IT_INFO("        Record HDMI vender specific inforframe HB0 = 0x%X  ", (int) it6602data->s_Current3DFr.HB0);
			IT_INFO("        Record HDMI vender specific inforframe HB1 = 0x%X  ", (int) it6602data->s_Current3DFr.HB1);
			IT_INFO("        Record HDMI vender specific inforframe HB2 = 0x%X  ", (int) it6602data->s_Current3DFr.HB2);
			IT_INFO("        Record HDMI vender specific inforframe PB0 = 0x%X  ", (int) it6602data->s_Current3DFr.PB0);
			IT_INFO("        Record HDMI vender specific inforframe PB1 = 0x%X  ", (int) it6602data->s_Current3DFr.PB1);
			IT_INFO("        Record HDMI vender specific inforframe PB2 = 0x%X  ", (int) it6602data->s_Current3DFr.PB2);
			IT_INFO("        Record HDMI vender specific inforframe PB3 = 0x%X  ", (int) it6602data->s_Current3DFr.PB3);
			IT_INFO("        Record HDMI vender specific inforframe PB4 = 0x%X  ", (int) it6602data->s_Current3DFr.PB4);
			IT_INFO("        Record HDMI vender specific inforframe PB5 = 0x%X  ", (int) it6602data->s_Current3DFr.PB5);
			IT_INFO("        Record HDMI vender specific inforframe PB6 = 0x%X  ", (int) it6602data->s_Current3DFr.PB6);
			IT_INFO("        Record HDMI vender specific inforframe PB7 = 0x%X  ", (int) it6602data->s_Current3DFr.PB7);




			/******************************  3D integration  *************************************/

			it6602data->de3dframe_config.LR_Reference             =  2; // Source of the 3D L/R reference.
			it6602data->de3dframe_config.FrameDominance           =  0; // Left or Right Eye is first in L/R image pair.
			it6602data->de3dframe_config.LR_Encoding              =  1; // Type of 3D L/R encoding
			it6602data->de3dframe_config.TB_Reference             =  2; // Top/Bottom reference for vertically sub-sampled sources
			it6602data->de3dframe_config.OE_Reference             =  2; // Odd/Even reference for horizontally sub-sampled sources

			it6602data->de3dframe_config.NumActiveBlankLines      =
				0; // Number of lines separating vertically packed L/R data to be removed (cropped)before being displayed
			it6602data->de3dframe_config.NumberOfEncodedLines     =
				0; // Number of encoded lines in one L/R eye frame of the display data to be blanked out with "Blanking Color".
			it6602data->de3dframe_config.LeftEncodedLineLocation  =
				-1; // Active line number of 1st encoded line in one Left eye frame of the display data (-1=unknown).
			it6602data->de3dframe_config.RightEncodedLineLocation =
				-1; // Active line number of 1st encoded line in one Right eye frame of the display data (-1=unknown).
			it6602data->de3dframe_config.BlankingColor            =
				7; // Color to use when blanking (or masking off) any embedded L/R encoding

			if (((it6602data->s_Current3DFr.PB4 & 0xE0) == HDMI_3DFORMAT_PRESENT)
			    && ((it6602data->s_Current3DFr.PB5 & 0xF0) == FRAME_PACKING)) {
				i = 0;

				while (t_3d_syncgen[i].Vic != 0xFF) {
					if (t_3d_syncgen[i].Vic == it6602data->s_Current3DFr.VIC) {
						break;
					}

					i++;
				}

				v_total     = t_3d_syncgen[i].V_total;
				v_act_start = t_3d_syncgen[i].V_act_start;
				v_act_end   = t_3d_syncgen[i].V_act_end;
				v_sync_end  = t_3d_syncgen[i].V_sync_end;
				v_2d_Vtotal = t_3d_syncgen[i].V_2D_active_total;
				chgbank(1);
				hdmirxset(Reg_PGVTotal_19D, 0xFF, (unsigned char)((v_total & 0xFF0) >> 4));			 //pccmd w 9d 2e
				hdmirxset(Reg_PGVTotal_19C, 0xF0, (unsigned char)((v_total & 0x00F) << 4));			//pccmd w 9c d0
				hdmirxset(Reg_PGVActSt_192, 0xFF, (unsigned char)((v_act_start & 0x0FF)));			//pccmd w 92 06
				hdmirxset(Reg_PGVActSt_193, 0x0F, (unsigned char)((v_act_start & 0xF00) >> 8));		//pccmd w 93 83
				hdmirxset(Reg_PGVActEd_193, 0xF0, (unsigned char)((v_act_end & 0x00F) << 4));		//pccmd w 93 83
				hdmirxset(Reg_PGVActEd_194, 0xFF, (unsigned char)((v_act_end & 0xFF0) >> 4));		//pccmd w 94 2E
				hdmirxset(Reg_PGVSyncEd_19F, 0xF0, (unsigned char)((v_sync_end & 0x00F) << 4));	//pccmd w 9f 22


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

				IT_INFO("\nv_total = %X or %d  ", (int)(v_total), (int)(v_total));
				IT_INFO("Reg_PGVTotal_19D = %X  ", (int)(hdmirxrd(Reg_PGVTotal_19D)));
				IT_INFO("Reg_PGVTotal_19C = %X  ", (int)(hdmirxrd(Reg_PGVTotal_19C)));
				IT_INFO("\nv_act_start = %X or %d  ", (int)(v_act_start), (int)(v_act_start));
				IT_INFO("Reg_PGVActSt_192 = %X  ", (int)(hdmirxrd(Reg_PGVActSt_192)));
				IT_INFO("Reg_PGVActSt_193 = %X  ", (int)(hdmirxrd(Reg_PGVActSt_193)));
				IT_INFO("\nv_act_end = %X or %d  ", (int)(v_act_end), (int)(v_act_end));
				IT_INFO("Reg_PGVActEd_193 = %X  ", (int)(hdmirxrd(Reg_PGVActEd_193)));
				IT_INFO("Reg_PGVActEd_194 = %X  ", (int)(hdmirxrd(Reg_PGVActEd_194)));
				IT_INFO("\nv_sync_end = %X or %d  ", (int)(v_sync_end), (int)(v_sync_end));
				IT_INFO("Reg_PGVSyncEd_19F = %X  ", (int)(hdmirxrd(Reg_PGVSyncEd_19F)));

				IT_INFO("LR_3D_Start = %X or %d   ", (int)(LR_3D_Start), (int)(LR_3D_Start));
				IT_INFO("Reg_PG3DRSt_18F = %X  ", (int)(hdmirxrd(Reg_PG3DRSt_18F)));
				IT_INFO("Reg_PG3DRStEd_190 = %X  ", (int)(hdmirxrd(Reg_PG3DRStEd_190)));
				IT_INFO("Reg_PG3DREd_191 = %X  ", (int)(hdmirxrd(Reg_PG3DREd_191)));
				IT_INFO("LR_3D_End = %X or %d   ", (int)(LR_3D_End), (int)(LR_3D_End));

				IT_INFO("\n\nv_total = %X or %d  ", (int)(v_total), (int)(v_total));
				IT_INFO("v_act_start = %X or %d  ", (int)(v_act_start), (int)(v_act_start));
				IT_INFO("v_act_end = %X or %d  ", (int)(v_act_end), (int)(v_act_end));
				IT_INFO("v_sync_end = %X or %d  ", (int)(v_sync_end), (int)(v_sync_end));
				IT_INFO("LR_3D_Start = %X or %d   ", (int)(LR_3D_Start), (int)(LR_3D_Start));
				IT_INFO("LR_3D_End = %X or %d   ", (int)(LR_3D_End), (int)(LR_3D_End));

				chgbank(0);
				hdmirxset(REG_RX_066_4_DE3DFrame, 0x10, 0x10);		// Reg66[4] = 1 for enable 3D FP2FS
				hdmirxset(REG_RX_085_5_En3DROut, 0x20, 0x20);			// Reg85[5] = 1 for enable 3DR output


				Dump3DReg();


				// enable output
				//HActive  = ((hdmirxrd(0x9F) & 0x3F) << 8) + hdmirxrd(0x9E);
				//ChangePicoResolution(HActive,v_2d_Vtotal);
				v_act_bspace = v_act_start - v_act_end;
			}

			if (((it6602data->s_Current3DFr.PB4 & 0xE0) == HDMI_3DFORMAT_PRESENT) && (!it6602data->DE3DFormat_HDMIFlag)) {
				it6602data->DE3DFormat_HDMIFlag = TRUE;
			}

			if (((it6602data->s_Current3DFr.PB4 & 0xE0) == HDMI_3DFORMAT_PRESENT) && (it6602data->DE3DFormat_HDMIFlag)) {
				if (((it6602data->s_Current3DFr.PB5 & 0xF0) == FRAME_PACKING) && (!it6602data->FramePacking_Flag)) {
					it6602data->FramePacking_Flag   = TRUE;
					it6602data->TopAndBottom_Flag   = FALSE;
					it6602data->SideBySide_Flag     = FALSE;
					it6602data->oldVIC              = 0;
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
							it6602data->de3dframe_config.NumActiveBlankLines  = (unsigned char)v_act_bspace;
							it6602data->de3dframe_config.Format = VERT_PACKED_FULL; // Type of 3D source format is FRAME_PACKING(VERT_PACKED_FULL)

#ifdef DEBUG_MODE_3D
							dbmsg_trace(DBM_3D, "ITEHDMI - HDMI_3DFORMAT is FRAME_PACKING  ");
#else
							IT_INFO("ITEHDMI - HDMI_3DFORMAT is FRAME_PACKING  ");

#endif

						} else {
							it6602data->de3dframe_config.Format    =  6; // Type of 3D source format is UNDEFINED_FORMAT

#ifdef DEBUG_MODE_3D
							dbmsg_trace(DBM_3D, "ITEHDMI - HDMI_3DFORMAT is UNDEFINED_FORMAT  ");
#endif
						}

#ifdef DEBUG_MODE_3D
						dbmsg_trace(DBM_3D, "ITEHDMI - HDMI_3DFORMAT is FRAME_PACKING call detect3D_Port_3D_On( )  ");
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
						it6602data->de3dframe_config.Format   =
							VERT_PACKED_HALF; // Type of 3D source format is TOP_AND_BOTTOM(VERT_PACKED_HALF)

#ifdef DEBUG_MODE_3D
						dbmsg_trace(DBM_3D, "ITEHDMI - HDMI_3DFORMAT is TOP_AND_BOTTOM  ");
#else
						IT_INFO("ITEHDMI - HDMI_3DFORMAT is TOP_AND_BOTTOM  ");
#endif

					} else {
						it6602data->de3dframe_config.Format   =  6; // Type of 3D source format is UNDEFINED_FORMAT

#ifdef DEBUG_MODE_3D
						dbmsg_trace(DBM_3D, "ITEHDMI - HDMI_3DFORMAT is UNDEFINED_FORMAT  ");
#endif
					}

					//detect3D_Port_3D_On(&it6602data->de3dframe_config);  //ralph
					//HDMI_DumpDE3DFrameInfo(&it6602data->s_Current3DFr);

					it6602data->FramePacking_Flag   = FALSE;
					it6602data->TopAndBottom_Flag   = TRUE;
					it6602data->SideBySide_Flag     = FALSE;
				}

				if (((it6602data->s_Current3DFr.PB5 & 0xF0) == SIDE_BY_SIDE) && (!it6602data->SideBySide_Flag)) {
					if ((it6602data->s_Current3DFr.VIC == 0x3c) || (it6602data->s_Current3DFr.VIC == 0x3e)
					    || (it6602data->s_Current3DFr.VIC == 0x13) || (it6602data->s_Current3DFr.VIC == 0x04)
					    || (it6602data->s_Current3DFr.VIC == 0x05) ||
					    (it6602data->s_Current3DFr.VIC == 0x14) || (it6602data->s_Current3DFr.VIC == 0x20)
					    || (it6602data->s_Current3DFr.VIC == 0x22) || (it6602data->s_Current3DFr.VIC == 0x1f)
					    || (it6602data->s_Current3DFr.VIC == 0x10)) {
						it6602data->de3dframe_config.Format   =
							HORIZ_PACKED_HALF; // Type of 3D source format is SIDE_BY_SIDE(HORIZ_PACKED_HALF)

#ifdef DEBUG_MODE_3D
						dbmsg_trace(DBM_3D, "ITEHDMI - HDMI_3DFORMAT is SIDE_BY_SIDE  ");
#else
						IT_INFO("ITEHDMI - HDMI_3DFORMAT is SIDE_BY_SIDE  ");
#endif

					} else {
						it6602data->de3dframe_config.Format   =  6; // Type of 3D source format is UNDEFINED_FORMAT

#ifdef DEBUG_MODE_3D
						dbmsg_trace(DBM_3D, "ITEHDMI - HDMI_3DFORMAT is UNDEFINED_FORMAT  ");
#endif
					}

					//detect3D_Port_3D_On(&it6602data->de3dframe_config);  //ralph
					//HDMI_DumpDE3DFrameInfo(&it6602data->s_Current3DFr);

					it6602data->FramePacking_Flag   = FALSE;
					it6602data->TopAndBottom_Flag   = FALSE;
					it6602data->SideBySide_Flag     = TRUE;
				}

#ifdef DEBUG_MODE_3D
				dbmsg_trace(DBM_3D, " ITEHDMI - HDMI_3D_SourceConfiguration:  ");
				dbmsg_ftrace(DBM_3D, "        Format                   = %X  ", (int) it6602data->de3dframe_config.Format);
				dbmsg_ftrace(DBM_3D, "        LR_Reference             = %X  ", (int) it6602data->de3dframe_config.LR_Reference);
				dbmsg_ftrace(DBM_3D, "        FrameDominance           = %X  ", (int) it6602data->de3dframe_config.FrameDominance);
				dbmsg_ftrace(DBM_3D, "        LR_Encoding              = %X  ", (int) it6602data->de3dframe_config.LR_Encoding);
				dbmsg_ftrace(DBM_3D, "        TB_Reference             = %X  ", (int) it6602data->de3dframe_config.TB_Reference);
				dbmsg_ftrace(DBM_3D, "        OE_Reference             = %X  ", (int) it6602data->de3dframe_config.OE_Reference);
				dbmsg_ftrace(DBM_3D, "        NumActiveBlankLines      = %X  ",
					     (int) it6602data->de3dframe_config.NumActiveBlankLines);
				dbmsg_ftrace(DBM_3D, "        NumberOfEncodedLines     = %X  ",
					     (int) it6602data->de3dframe_config.NumberOfEncodedLines);
				dbmsg_ftrace(DBM_3D, "        LeftEncodedLineLocation  = %X  ",
					     (int) it6602data->de3dframe_config.LeftEncodedLineLocation);
				dbmsg_ftrace(DBM_3D, "        RightEncodedLineLocation = %X  ",
					     (int) it6602data->de3dframe_config.RightEncodedLineLocation);
				dbmsg_ftrace(DBM_3D, "        BlankingColor            = %X  ", (int) it6602data->de3dframe_config.BlankingColor);

#else
				IT_INFO(" ITEHDMI - HDMI_3D_SourceConfiguration:  ");
				IT_INFO("        Format                   = %X  ", (int) it6602data->de3dframe_config.Format);
				IT_INFO("        LR_Reference             = %X  ", (int) it6602data->de3dframe_config.LR_Reference);
				IT_INFO("        FrameDominance           = %X  ", (int) it6602data->de3dframe_config.FrameDominance);
				IT_INFO("        LR_Encoding              = %X  ", (int) it6602data->de3dframe_config.LR_Encoding);
				IT_INFO("        TB_Reference             = %X  ", (int) it6602data->de3dframe_config.TB_Reference);
				IT_INFO("        OE_Reference             = %X  ", (int) it6602data->de3dframe_config.OE_Reference);
				IT_INFO("        NumActiveBlankLines      = %X  ", (int) it6602data->de3dframe_config.NumActiveBlankLines);
				IT_INFO("        NumberOfEncodedLines     = %X  ", (int) it6602data->de3dframe_config.NumberOfEncodedLines);
				IT_INFO("        LeftEncodedLineLocation  = %X  ", (int) it6602data->de3dframe_config.LeftEncodedLineLocation);
				IT_INFO("        RightEncodedLineLocation = %X  ", (int) it6602data->de3dframe_config.RightEncodedLineLocation);
				IT_INFO("        BlankingColor            = %X  ", (int) it6602data->de3dframe_config.BlankingColor);
#endif

				return TRUE;
			}
		}

		if (it6602data->DE3DFormat_HDMIFlag) { // 3D InfoFrame Packet Type is not valid
#ifdef DEBUG_MODE_3D
			dbmsg_trace(DBM_3D, "ITEHDMI - HDMI_3DFORMAT is OFF  ");
#endif

			it6602data->DE3DFormat_HDMIFlag = FALSE;
			it6602data->FramePacking_Flag   = FALSE;
			it6602data->TopAndBottom_Flag   = FALSE;
			it6602data->SideBySide_Flag     = FALSE;
		}

		/******************************  3D integration  *************************************/

	} else {

		//it6602data->f_de3dframe_hdmi = FALSE;
		hdmirxwr(REG_RX_06A, 0x82);
		hdmirxset(REG_RX_066_4_DE3DFrame, 0x10, 0x00);		// Reg66[4] = 0 for disable 3D FP2FS
		hdmirxset(REG_RX_085_5_En3DROut, 0x20, 0x00);			// Reg85[5] = 0 for disable 3DR output

	}

	return FALSE;
}
#endif



//FIX_ID_003 xxxxx	//Add IT6602 Video Output Configure Table
void IT6602ChangeTTLVideoOutputMode(void)
{
	//for test video output format  only !!!
	Video_Output_Configure i;
	struct it6602_dev_data *it6602data = get_it6602_dev_data();


	if (it6602data->m_VidOutConfigMode < eVOMreserve) {
		it6602data->m_VidOutConfigMode = (Video_Output_Configure)(it6602data->m_VidOutConfigMode + 1);

	} else {
		it6602data->m_VidOutConfigMode = eRGB444_SDR;
	}

	i = it6602data->m_VidOutConfigMode;

	IT6602_VideoOutputConfigure_Init(it6602data, i);
	IT6602_VideoOutputModeSet(it6602data);
//FIX_ID_003 xxxxx
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
		IT_INFO("REG_RX_012=%2x", hdmirxrd(REG_RX_012));
		IT_INFO("REG_RX_063=%2x", hdmirxrd(REG_RX_063));
		IT_INFO("MHL_RX_0A=%2x", mhlrxrd(MHL_RX_0A));
		IT_INFO("MHL_RX_08=%2x", mhlrxrd(MHL_RX_08));
		IT_INFO("MHL_RX_09=%2x", mhlrxrd(MHL_RX_09));
	}

	return ret;
}

void it6602_GetAVIInfoFrame(void)
{
	struct it6602_dev_data *it6602data = get_it6602_dev_data();
	GetAVIInfoFrame(it6602data);
}




uint8_t IT_66021_GetVideoFormat(uint16_t *widthPtr, uint16_t *hightPtr, uint8_t *framteratePtr, uint8_t *vic)
{
	if (TRUE == IsVideoOn()) {
		uint8_t i = 0;
		uint8_t array_size = sizeof(s_u8Array_ARCastSupportedOutputFormat) / sizeof(s_u8Array_ARCastSupportedOutputFormat[0]);
		struct it6602_dev_data *it6602data = get_it6602_dev_data();

		for (i = 0; i < array_size; i++) {
			IT_INFO("GET THE FORMAT %d", i);

			if (it6602data->VIC == s_u8Array_ARCastSupportedOutputFormat[i][3]) {
				*widthPtr = s_u8Array_ARCastSupportedOutputFormat[i][0];
				*hightPtr = s_u8Array_ARCastSupportedOutputFormat[i][1];
				*framteratePtr = s_u8Array_ARCastSupportedOutputFormat[i][2];
				*vic = it6602data->VIC;
				return TRUE;
			}
		}

		IT_INFO("NEW FORMAT %d", i);

		uint32_t u32_HTotal   = ((hdmirxrd(0x9D) & 0x3F) << 8) + hdmirxrd(0x9C);
		uint32_t u32_HActive  = ((hdmirxrd(0x9F) & 0x3F) << 8) + hdmirxrd(0x9E);
		uint32_t u32_VTotal   = ((hdmirxrd(0xA4) & 0x0F) << 8) + hdmirxrd(0xA3);
		uint32_t u32_VActive  = ((hdmirxrd(0xA4) & 0xF0) << 4) + hdmirxrd(0xA5);

		uint8_t u8_rddata = hdmirxrd(0x9A);

		uint32_t PCLK = (124 * 255 / u8_rddata) / 10;
		uint64_t u64_FrameRate = (uint64_t)(PCLK) * 1000 * 1000;
		u64_FrameRate /= u32_HTotal;
		u64_FrameRate /= u32_VTotal;

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


uint32_t HDMI_RX_CheckVideoFormatSupportOrNot(uint16_t u16_width, uint16_t u16_hight, uint8_t u8_framerate)
{
	uint8_t i = 0;
	uint8_t array_size = sizeof(s_st_hdmiRxSupportedOutputFormat) / sizeof(s_st_hdmiRxSupportedOutputFormat[0]);

	for (i = 0; i < array_size; i++) {
		if ((u16_width == s_st_hdmiRxSupportedOutputFormat[i].u16_width) &&
		    (u16_hight == s_st_hdmiRxSupportedOutputFormat[i].u16_hight) &&
		    (u8_framerate == s_st_hdmiRxSupportedOutputFormat[i].u8_framerate)) {
			break;
		}
	}

	if (i < array_size) {
		return HAL_OK;

	} else {
		return -1;
	}

}


static uint32_t HDMI_RX_CheckVideoFormatChangeOrNot(uint16_t u16_width,
		uint16_t u16_hight,
		uint8_t u8_framerate)
{
	if ((ite_a.u16_width != u16_width) ||
	    (ite_a.u16_hight != u16_hight) ||
	    (ite_a.u8_framerate != u8_framerate)) {
		return HAL_OK;	// change

	} else {
		return -1;	// not
	}
}

void HDMI_RX_CheckFormatStatus(void)
{
	if (h264_input_format_topic == NULL) {
		h264_input_format_topic = orb_advertise(ORB_ID(h264_input_format), &att);
	}

	STRU_HDMI_RX_OUTPUT_FORMAT get_formate;

	IT_66021_GetVideoFormat(&get_formate.u16_width,
				&get_formate.u16_hight,
				&get_formate.u8_framerate,
				&get_formate.u8_vic);

	if (HDMI_RX_CheckVideoFormatSupportOrNot(get_formate.u16_width, get_formate.u16_hight,
			get_formate.u8_framerate) == HAL_OK) { // support
		IT_INFO("RIGHT FORMATE");

		if (HDMI_RX_CheckVideoFormatChangeOrNot(get_formate.u16_width, get_formate.u16_hight,
							get_formate.u8_framerate) == HAL_OK) {
			att.index = 1;
			att.width = get_formate.u16_width;
			att.hight = get_formate.u16_hight;
			att.framerate = get_formate.u8_framerate;
			att.vic = get_formate.u8_vic;

			att.e_h264InputSrc = ENCODER_INPUT_SRC_HDMI_1;

			orb_publish(ORB_ID(h264_input_format), h264_input_format_topic, &att);

			ite_a.u16_width = get_formate.u16_width;
			ite_a.u16_hight = get_formate.u16_hight;
			ite_a.u8_framerate = get_formate.u8_framerate;
			ite_a.u8_vic = get_formate.u8_vic;

		}

	} else {
		IT_INFO("can't get the right FORMATE");
		att.index = 1;
		att.width = 0;
		att.width = 0;
		att.hight = 0;
		att.framerate = 0;
		att.vic = get_formate.u8_vic;
		att.e_h264InputSrc = ENCODER_INPUT_SRC_HDMI_1;

		orb_publish(ORB_ID(h264_input_format), h264_input_format_topic, &att);

		ite_a.u16_width = 0;
		ite_a.u16_hight = 0;
		ite_a.u8_framerate = 0;
		ite_a.u8_vic = get_formate.u8_vic;
	}
}
