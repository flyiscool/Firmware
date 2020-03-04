#ifndef _COOLFLY_IT66021_H_
#define _COOLFLY_IT66021_H_

#include <stddef.h>
#include <stdint.h>
#include "it66021_define.h"


#define _SUPPORT_HDCP_				FALSE
#define _SUPPORT_EDID_RAM_			TRUE

//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#define _SUPPORT_AUTO_EQ_           TRUE
#define _SUPPORT_EQ_ADJUST_         TRUE
//FIX_ID_001 xxxxx

#define _SelectExtCrystalForCbus_	TRUE

/*****************************************************************************/
/* Type defs enums  **********************************************************/
/*****************************************************************************/
//FIX_ID_033 xxxxx  //Fine-tune EQ Adjust function for HDCP receiver and repeater mode
//xxxxx 2014-0421 modify 50 to 100
#define MS_LOOP                 100
//xxxxx 2014-0421
//FIX_ID_033 xxxxx

enum {
	MHD_RAP_CMD_POLL         	= 0x00,
	MHD_RAP_CMD_CHG_ACTIVE_PWR  = 0x10,
	MHD_RAP_CMD_CHG_QUIET       = 0x11,
	MHD_RAP_CMD_END             = 0x12
};

//RAPK sub commands
enum {
	MHD_MSC_MSG_RAP_NO_ERROR        		= 0x00,     // RAP No Error
	MHD_MSC_MSG_RAP_UNRECOGNIZED_ACT_CODE  	= 0x01,
	MHD_MSC_MSG_RAP_UNSUPPORTED_ACT_CODE  	= 0x02,
	MHD_MSC_MSG_RAP_RESPONDER_BUSY   		= 0x03
};

enum {
	RCP_SELECT          = 0x00,
	RCP_UP              = 0x01,
	RCP_DOWN            = 0x02,
	RCP_LEFT            = 0x03,
	RCP_RIGHT           = 0x04,
	RCP_RIGHT_UP        = 0x05,
	RCP_RIGHT_DOWN      = 0x06,
	RCP_LEFT_UP         = 0x07,
	RCP_LEFT_DOWN       = 0x08,
	RCP_ROOT_MENU       = 0x09,
	RCP_SETUP_MENU      = 0x0A,
	RCP_CONTENTS_MENU   = 0x0B,
	RCP_FAVORITE_MENU   = 0x0C,
	RCP_EXIT            = 0x0D,

	//0x0E - 0x1F are reserved

	RCP_NUM_0           = 0x20,
	RCP_NUM_1           = 0x21,
	RCP_NUM_2           = 0x22,
	RCP_NUM_3           = 0x23,
	RCP_NUM_4           = 0x24,
	RCP_NUM_5           = 0x25,
	RCP_NUM_6           = 0x26,
	RCP_NUM_7           = 0x27,
	RCP_NUM_8           = 0x28,
	RCP_NUM_9           = 0x29,

	RCP_DOT             = 0x2A,
	RCP_ENTER           = 0x2B,
	RCP_CLEAR           = 0x2C,

	//0x2D - 0x2F are reserved

	RCP_CH_UP           = 0x30,
	RCP_CH_DOWN         = 0x31,
	RCP_PRE_CH          = 0x32,
	RCP_SOUND_SELECT    = 0x33,
	RCP_INPUT_SELECT    = 0x34,
	RCP_SHOW_INFO       = 0x35,
	RCP_HELP            = 0x36,
	RCP_PAGE_UP         = 0x37,
	RCP_PAGE_DOWN       = 0x38,

	//0x39 - 0x40 are reserved

	RCP_VOL_UP	        = 0x41,
	RCP_VOL_DOWN        = 0x42,
	RCP_MUTE            = 0x43,
	RCP_PLAY            = 0x44,
	RCP_STOP            = 0x45,
	RCP_PAUSE           = 0x46,
	RCP_RECORD          = 0x47,
	RCP_REWIND          = 0x48,
	RCP_FAST_FWD        = 0x49,
	RCP_EJECT           = 0x4A,
	RCP_FWD             = 0x4B,
	RCP_BKWD            = 0x4C,

	//0x4D - 0x4F are reserved

	RCP_ANGLE            = 0x50,
	RCP_SUBPICTURE       = 0x51,

	//0x52 - 0x5F are reserved

	RCP_PLAY_FUNC       = 0x60,
	RCP_PAUSE_PLAY_FUNC = 0x61,
	RCP_RECORD_FUNC     = 0x62,
	RCP_PAUSE_REC_FUNC  = 0x63,
	RCP_STOP_FUNC       = 0x64,
	RCP_MUTE_FUNC       = 0x65,
	RCP_UN_MUTE_FUNC    = 0x66,
	RCP_TUNE_FUNC       = 0x67,
	RCP_MEDIA_FUNC      = 0x68,

	//0x69 - 0x70 are reserved

	RCP_F1              = 0x71,
	RCP_F2              = 0x72,
	RCP_F3              = 0x73,
	RCP_F4              = 0x74,
	RCP_F5              = 0x75,

	//0x76 - 0x7D are reserved

	RCP_VS              = 0x7E,
	RCP_RSVD            = 0x7F

};

//FIX_ID_003 xxxxx	//Add IT6602 Video Output Configure setting
typedef enum _Video_Output_Configure {
	eRGB444_SDR = 0,
	eYUV444_SDR,
	eRGB444_DDR,
	eYUV444_DDR,
	eYUV422_Emb_Sync_SDR,
	eYUV422_Emb_Sync_DDR,
	eYUV422_Sep_Sync_SDR,
	eYUV422_Sep_Sync_DDR,
	eCCIR656_Emb_Sync_SDR,
	eCCIR656_Emb_Sync_DDR,
	eCCIR656_Sep_Sync_SDR,
	eCCIR656_Sep_Sync_DDR,
	eRGB444_Half_Bus,
	eYUV444_Half_Bus,
	eBTA1004_SDR,
	eBTA1004_DDR,
	eVOMreserve
} Video_Output_Configure;

typedef enum _Video_OutputDataTrigger_Mode {
	eSDR = 0,
	eHalfPCLKDDR,
	eHalfBusDDR,
	eSDR_BTA1004,
	eDDR_BTA1004
} Video_DataTrigger_Mode;

typedef enum _Video_OutputSync_Mode {
	eSepSync = 0,
	eEmbSync,
	eCCIR656SepSync,
	eCCIR656EmbSync
} Video_OutputSync_Mode;

//FIX_ID_003 xxxxx
typedef enum _Video_State_Type {
	VSTATE_Off = 0,
	VSTATE_TerminationOff,
	VSTATE_TerminationOn,
	VSTATE_5VOff,
	VSTATE_SyncWait,
	VSTATE_SWReset,
	VSTATE_SyncChecking,
	VSTATE_HDCPSet,
	VSTATE_HDCP_Reset,
	VSTATE_ModeDetecting,
	VSTATE_VideoOn,
	VSTATE_ColorDetectReset,
	VSTATE_HDMI_OFF,
	VSTATE_Reserved
} Video_State_Type;

typedef enum _RxHDCP_State_Type {
	RxHDCP_PwrOff = 0,
	RxHDCP_ModeCheck,
	RxHDCP_Receiver,
	RxHDCP_Repeater,
	RxHDCP_SetKSVFifoList,
	RxHDCP_GenVR,
	RxHDCP_WriteVR,
	RxHDCP_Auth_WaitRi,
	RxHDCP_Authenticated,
	RxHDCP_Reserved
} RxHDCP_State_Type;

typedef enum  {
	RCP_Received = 0,
	RCP_Error,
	RCP_ACK,
	RCP_Transfer,
	RCP_Empty,
	RCP_Unknown
} RCPState_Type;

typedef enum  {
	RCP_Result_OK = 0,
	RCP_Result_FAIL,
	RCP_Result_ABORT,
	RCP_Result_Transfer,
	RCP_Result_Finish,
	RCP_Result_Unknown
} RCPResult_Type;


#define F_MODE_RGB24	0
#define F_MODE_RGB444  	0
#define F_MODE_YUV422  	1
#define F_MODE_YUV444  	2
#define F_MODE_CLRMOD_MASK 	3
#define F_MODE_ITU709  	(1<<4)
#define F_MODE_ITU601  	0
#define F_MODE_0_255   	0
#define F_MODE_16_235  	(1<<5)
#define F_MODE_EN_UDFILT 	(1<<6)
#define F_MODE_EN_DITHER 	(1<<7)

#define RCVABORT        2
#define RCVNACK         3
#define ARBLOSE         4
#define FWTXFAIL        5
#define FWRXPKT         6
#define FAIL			-1
#define ABORT           -2

#define HIGH			1
#define LOW				0

#define CD8BIT			4
#define CD10BIT	 		5
#define CD12BIT			6
#define CD16BIT			7

#define OUT8B           0
#define OUT10B          1
#define OUT12B          2

#define RGB444			0
#define YCbCr422		1
#define YCbCr444		2

#define NORM            0
#define FAST            1
#define SLOW            2

#define AUD32K			0x3
#define AUD48K			0x2
#define AUD96K			0xA
#define AUD192K			0xE
#define AUD44K			0x0
#define AUD88K			0x8
#define AUD176K			0xC

#define I2S				0
#define SPDIF			1

#define MHLInt00B       0x20
#define DCAP_CHG        0x01
#define DSCR_CHG        0x02
#define REQ_WRT         0x04
#define GRT_WRT         0x08

#define MHLInt01B       0x21
#define EDID_CHG        0x01

#define MHLSts00B       0x30
#define DCAP_RDY        0x01

#define MHLSts01B       0x31
#define NORM_MODE       0x03
#define PACK_MODE       0x02
#define PATH_EN         0x08
#define MUTED           0x10

#define MSG_MSGE	0x02
#define MSG_RCP		0x10
#define MSG_RCPK	0x11
#define MSG_RCPE	0x12
#define MSG_RAP		0x20
#define MSG_RAPK	0x21
#define MSG_UCP		0x30
#define MSG_UCPK	0x31
#define MSG_UCPE	0x32


#define MHL  1
#define HDMI 0

#define BUS10B 1
#define BUS20B 0


#define MAXRCPINDEX 5

#define SUPPORT_INPUTRGB
#define SUPPORT_INPUTYUV444
#define SUPPORT_INPUTYUV422

#if defined(SUPPORT_INPUTYUV444)|| defined(SUPPORT_INPUTYUV422)
#define SUPPORT_INPUTYUV
#endif


#define SUPPORT_OUTPUTYUV
#define SUPPORT_OUTPUTYUV444
#define SUPPORT_OUTPUTYUV422
#define SUPPORT_OUTPUTRGB

#define F_PORT_SEL_0      0
#define F_PORT_SEL_1      1




#if 1
/*****************************************************************************/
/* Type defs struct **********************************************************/
/*****************************************************************************/
struct IT6602_REG_INI {
	unsigned char ucAddr;
	unsigned char andmask;
	unsigned char ucValue;
};

struct IT6602_VIDEO_CONFIGURE_REG {
	unsigned char ucReg51;
	unsigned char ucReg65;
};

typedef struct _3D_SourceConfiguration {
	unsigned char
	Format;              /**< Type of 3D source format expected or found.                                                        */
	unsigned char
	LR_Reference;        /**< Source of the 3D L/R reference.                                                                    */
	unsigned char
	FrameDominance;      /**< Left or Right Eye is first in L/R image pair.                                                      */
	unsigned char
	LR_Encoding;         /**< Type of 3D L/R encoding expected or detected.                                                      */
	unsigned char
	TB_Reference;        /**< Top/Bottom reference for vertically sub-sampled sources.                                           */
	unsigned char
	OE_Reference;        /**< Odd/Even reference for horizontally sub-sampled sources.                                           */
	unsigned char
	NumActiveBlankLines;                 /**< Number of lines separating vertically packed L/R data to be removed (cropped)
                                                  *  before being displayed. Does not include any embedded encoding.                                    */
	unsigned char NumberOfEncodedLines;                /**< Number of encoded lines in one L/R eye frame of the display data
                                                  *  to be blanked out with "Blanking Color". (assumed same number in second eye frame)                 */
	unsigned int
	LeftEncodedLineLocation;             /**< Active line number of 1st encoded line in one Left eye frame of the display data (-1=unknown).     */
	unsigned int
	RightEncodedLineLocation;            /**< Active line number of 1st encoded line in one Right eye frame of the display data (-1=unknown).
                                                  *  If format is Horizontally Packed, set RightEncodedLineLocation=LeftEncodedLineLocation             */
	unsigned char
	BlankingColor;                 /**< Color to use when blanking (or masking off) any embedded L/R encoding.                             */
} SRC_3D_SOURCE_CONFIG;

typedef struct _de3dframe {
	unsigned char VIC;
	unsigned char HB0;
	unsigned char HB1;
	unsigned char HB2;
	unsigned char PB0;
	unsigned char PB1;
	unsigned char PB2;
	unsigned char PB3;
	unsigned char PB4;
	unsigned char PB5;
	unsigned char PB6;
	unsigned char PB7;
} DE3DFRAME ;

typedef struct _set_de3d_frame {
	unsigned char Vic;
	unsigned int V_total;        // Vtotal -1
	unsigned int V_act_start;    // VTotal -1 + Vactive_start -1
	unsigned int V_act_end;      // Vactive end -1
	unsigned int V_sync_end;     // LSB(Vtotal -1 + sync With)
	unsigned int V_2D_active_total;     // V_2D_active_total
} SET_DE3D_FRAME;

typedef enum {
	VSYNC_SEPARATED_HALF,       /**< VSync separated (field sequential) format.                         */
	VSYNC_SEPARATED_FULL,       /**< VSync separated (frame sequential progressive) format.             */
	VERT_PACKED_HALF,           /**< Over Under (vertically packed) half resolution format.             */
	VERT_PACKED_FULL,           /**< Over Under (vertically packed) full resolution format.             */
	HORIZ_PACKED_HALF,          /**< Side by Side (horizontally packed) half resolution format.         */
	HORIZ_PACKED_FULL,          /**< Side by Side (horizontally packed) full resolution format.         */
	UNDEFINED_FORMAT            /**< Undefined format.                                                  */
} SRC_3D_FORMAT;

typedef enum _pixel_mode {
	SINGLE_PIXEL,
	DUAL_PIXEL,
	MODE_UNKNOWN
} PIXEL_MODE;


//FIX_ID_010 xxxxx 	//Add JudgeBestEQ to avoid wrong EQ setting
#define MaxEQIndex 3
//FIX_ID_010 xxxxx

//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_EQ_ADJUST_
struct it6602_eq_data {
	unsigned char ucEQState;
	unsigned char ucAuthR0;								//20130327 for R0 fail issue
	unsigned char ucECCvalue;								//20130328 for acc ecc error
	unsigned char ucECCfailCount;							//20130328 for acc ecc error
	unsigned char ucPkt_Err;
	unsigned char ucPortID;
	unsigned char f_manualEQadjust;
	//FIX_ID_010 xxxxx 	//Add JudgeBestEQ to avoid wrong EQ setting
	unsigned char ErrorCount[MaxEQIndex];
	//FIX_ID_010 xxxxx
};
#endif
//FIX_ID_001 xxxxx


//FIX_ID_005 xxxxx	//Add Cbus Event Handler
#define B_MSC_Waiting 		0x10
#define B_DevCapChange 		0x08
#define B_3DSupporpt 		0x04 // bit2 B_3DSupporpt
#define B_ReadDevCap 		0x02 // bit1 B_ReadDevCap
#define B_DiscoveryDone 	0x01 // bit0 B_DiscoveryDone
//FIX_ID_005 xxxxx

//FIX_ID_014 xxxxx
#define B_PORT1_TimingChgEvent	0x40
#define B_PORT1_TMDSEvent	0x20
#define B_PORT1_Waiting		0x10
#define B_PORT0_TimingChgEvent	0x04
#define B_PORT0_TMDSEvent	0x02
#define B_PORT0_Waiting		0x01
//FIX_ID_014 xxxxx

//FIX_ID_013	xxxxx	//For MSC 3D request issue
#define MSC_3D_VIC		(0x0010)
#define MSC_3D_DTD		(0x0011)

typedef enum _PARSE3D_STA {
	PARSE3D_START,
	PARSE3D_LEN,
	PARSE3D_STRUCT_H,
	PARSE3D_STRUCT_L,
	PARSE3D_MASK_H,
	PARSE3D_MASK_L,
	PARSE3D_VIC,
	PARSE3D_DONE
} PARSE3D_STA;

typedef enum _MHL3D_STATE {
	MHL3D_REQ_START,
	MHL3D_REQ_WRT,
	MHL3D_GNT_WRT,
	MHL3D_WRT_BURST,
	MHL3D_REQ_DONE
} MHL3D_STATE;

struct PARSE3D_STR {
	unsigned char	uc3DEdidStart;
	unsigned char	uc3DBlock;
	unsigned char	uc3DInfor[32];
	unsigned char	ucVicStart;
	unsigned char	ucVicCnt;
	unsigned char	uc3DTempCnt;
	unsigned char	ucDtdCnt;
	unsigned char	bVSDBspport3D;
};
//FIX_ID_013	xxxxx


struct AVI_info {
	unsigned char ColorMode;
	unsigned char Colorimetry;
	unsigned char ExtendedColorimetry;
	unsigned char RGBQuantizationRange;
	unsigned char YCCQuantizationRange;
	unsigned char VIC;
	//unsigned char PixelRepetition;
};


struct it6602_dev_data {
	Video_State_Type m_VState;
	Audio_State_Type m_AState;
	RxHDCP_State_Type m_RxHDCPState;
	AUDIO_CAPS m_RxAudioCaps;
	unsigned short m_SWResetTimeOut;
	unsigned short m_VideoCountingTimer;
	unsigned short m_AudioCountingTimer;
	unsigned char m_ucCurrentHDMIPort;
	unsigned char m_bOutputVideoMode;
	unsigned char m_bInputVideoMode;

//FIX_ID_039 xxxxx fix image flick when enable RGB limited / Full range convert
#ifdef _AVOID_REDUNDANCE_CSC_
	unsigned char m_Backup_OutputVideoMode;
	unsigned char m_Backup_InputVideoMode;
#endif
//FIX_ID_039 xxxxx

	unsigned char m_ucSCDTOffCount;
	unsigned char m_ucEccCount_P0;
	unsigned char m_ucDeskew_P0;

	SRC_3D_SOURCE_CONFIG de3dframe_config;
	DE3DFRAME s_Current3DFr;

	unsigned char oldVIC;
	unsigned char newVIC;
	unsigned char f_de3dframe_hdmi;

//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_EQ_ADJUST_
	struct it6602_eq_data EQPort[2];
#endif
//FIX_ID_001 xxxxx

	//FIX_ID_003 xxxxx	//Add IT6602 Video Output Configure setting
	Video_Output_Configure m_VidOutConfigMode;
	Video_DataTrigger_Mode m_VidOutDataTrgger;
	Video_OutputSync_Mode m_VidOutSyncMode;
	//FIX_ID_003 xxxxx

	//FIX_ID_005 xxxxx	//Add Cbus Event Handler
	unsigned char CBusIntEvent;
	unsigned char CBusSeqNo;
	unsigned char CBusWaitNo;
	//FIX_ID_005 xxxxx

	//FIX_ID_014 xxxxx	//Add Cbus Event Handler
	unsigned char HDMIIntEvent;
	unsigned char HDMIWaitNo[2];
	//FIX_ID_014 xxxxx

//FIX_ID_021 xxxxx		//To use CP_100ms for CBus_100ms and CEC_100m
// #ifndef _SelectExtCrystalForCbus_
// 	unsigned long RCLK;
// 	unsigned long PCLK;
//
// 	int  t10usint;
// 	int  t10usflt;
// #endif
//FIX_ID_021 xxxxx

	//AVI_info m_avi;
	unsigned char ColorMode;
	unsigned char Colorimetry;
	unsigned char ExtendedColorimetry;
	unsigned char RGBQuantizationRange;
	unsigned char YCCQuantizationRange;
	unsigned char VIC;


//FIX_ID_034 xxxxx //Add MHL HPD Control by it6602HPDCtrl( )
	unsigned char m_DiscoveryDone;
//FIX_ID_034 xxxxx

//FIX_ID_037 xxxxx //Allion MHL compliance issue !!!
//xxxxx 2014-0529 //Manual Content On/Off
	unsigned char m_RAP_ContentOff;
	unsigned char m_HDCP_ContentOff;
//xxxxx
//FIX_ID_037 xxxxx

	PIXEL_MODE pixelMode;	//Output TTL Pixel mode
	unsigned char GCP_CD;	//Output Color Depth
	unsigned char  DE3DFormat_HDMIFlag: 1;
	unsigned char  FramePacking_Flag: 1;
	unsigned char  TopAndBottom_Flag: 1;
	unsigned char  SideBySide_Flag: 1;




#ifdef _IT6607_GeNPacket_Usage_
	BYTE m_PollingPacket;
	BYTE m_PacketState;
	BYTE m_ACPState;
	BYTE m_GeneralRecPackType;
	BYTE m_GamutPacketRequest: 1;
#endif

//#if(_SUPPORT_HDCP_)
//    //HDCP
//    unsigned char HDCPEnable;
//    HDCPSts_Type Hdcp_state;
//    unsigned int HDCPFireCnt ;
//#endif

	//CBUS MSC
	unsigned char Mhl_devcap[16];
	unsigned char txmsgdata[2];
	unsigned char rxmsgdata[2];
	unsigned char txscrpad[16];
	unsigned char rxscrpad[16];
	unsigned char RCPTxArray[MAXRCPINDEX];
	unsigned char	RCPhead;
	unsigned char	RCPtail;
	RCPState_Type RCPState;
	RCPResult_Type RCPResult;
//FIX_ID_015	xxxxx peer device no response
	unsigned char RCPCheckResponse;
//FIX_ID_015	xxxxx

//FIX_ID_024	xxxxx Fixed for RCP compliance issue
	unsigned char m_bRCPTimeOut: 1;
	unsigned char m_bRCPError: 1;
//FIX_ID_024	xxxxx
	unsigned char m_bRxAVmute: 1;
	unsigned char m_bVideoOnCountFlag: 1;
	unsigned char m_MuteAutoOff: 1;
	unsigned char m_bUpHDMIMode: 1;
	unsigned char m_bUpHDCPMode: 1;
	unsigned char m_NewAVIInfoFrameF: 1;
	unsigned char m_NewAUDInfoFrameF: 1;
	unsigned char m_HDCPRepeater: 1;
	unsigned char m_MuteByPKG: 1;

};

#endif


/*****************************************************************************/
/* Error codes ***************************************************************/
/*****************************************************************************/
/* Error code specifying an I2C driver error */
#define ITEHDMI_I2C_DRIVER_ERROR                     -4
#define ITEHDMI_I2C_DRIVER_TERMINAL_ERROR            -5

/* Error code specifying RTA errors */
#define ITEHDMI_RTA_SEM_IN_USE                       -6
#define ITEHDMI_RTA_ERROR                            -7

/* Error code specifying other errors */
#define ITEHDMI_DATA_INVALID                         -8


/* ITEHDMI IO Functions   ***********************************************************/
/* ITEHDMI Configuration and Initialization ***********************************/
char IT6602_fsm_init(void);
/* HDMI RX functions   *********************************************************/
void it6602PortSelect(unsigned char ucPortSel);
void it6602HPDCtrl(unsigned char ucport, unsigned char ucEnable)	;
/* HDMI Audio function    *********************************************************/
/* HDMI Video function    *********************************************************/
/* HDMI Interrupt function    *********************************************************/


/* MHLRX functions    *********************************************************/
/* MHL interrupt    *******************************************************/
/* MHL 3D functions    *******************************************************/
/* EDID RAM  functions    *******************************************************/
/* RCP functions    *******************************************************/
#ifdef _SUPPORT_RCP_
void RCPKeyPush(unsigned char ucKey);
#endif


////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
#include <drivers/device/i2c.h>
#include <px4_workqueue.h>
#include "edid.h"
#include "px4_module.h"

#ifndef _CODE
#define _CODE
#endif

#define HAL_HDMI_RX_ERR_MASK	(0x70000)
#define HAL_HDMI_RX_FALSE   	(HAL_HDMI_RX_ERR_MASK | 0x9)


class IT66021: public device::I2C, public ModuleBase<IT66021>
{
public:
	IT66021(I2CARG arg);
	virtual ~IT66021();

	_EXT_ITCM static int custom_command(int argc, char *argv[]);

	_EXT_ITCM static int task_spawn(int argc, char *argv[]);

	_EXT_ITCM static int print_usage(const char *reason = nullptr);

	EDID *edid;

	work_s work;

	struct it6602_dev_data it6602DEV;

	STRU_HDMI_RX_STATUS s_st_hdmiRxStatus;

	uint8_t HDMI_RX_MapToDeviceIndex(ENUM_HAL_HDMI_RX e_hdmiIndex);

	uint32_t HDMI_RX_CheckVideoFormatChangeOrNot(ENUM_HAL_HDMI_RX e_hdmiIndex,
			uint16_t u16_width,
			uint16_t u16_hight,
			uint8_t u8_framerate);

	uint8_t IT_66021_GetVideoFormat(uint8_t index, uint16_t *widthPtr, uint16_t *hightPtr, uint8_t *framteratePtr,
					uint8_t *vic);

	void HDMI_RX_CheckFormatStatus(ENUM_HAL_HDMI_RX e_hdmiIndex, uint32_t b_noDiffCheck);

	/////////////////////////////////
	virtual int init();
	virtual int probe();

	static void cycle_trampoline(void *arg);

	void get_vid_info(void);
	void show_vid_info(void);
	void IT6602_Interrupt(void);

	unsigned char mhlrxrd(unsigned char offset);
	unsigned char mhlrxwr(unsigned char offset, unsigned char ucdata);

	int read(unsigned address, void *data, unsigned count);
	int write(unsigned address, void *data, unsigned count);

	SYS_STATUS EDID_RAM_Write(unsigned char offset, unsigned char byteno, _CODE unsigned char *p_data);
	unsigned char EDID_RAM_Read(unsigned char offset);
	unsigned char hdmirxrd(unsigned char address);
	unsigned char hdmirxwr(unsigned char address, unsigned char data);
	unsigned char hdmirxset(unsigned char offset, unsigned char mask, unsigned char ucdata);
	void hdmirxbwr(unsigned char offset, unsigned char byteno, unsigned char *rddata);

	struct it6602_dev_data *get_it6602_dev_data(void);
	void hdimrx_write_init(struct IT6602_REG_INI _CODE *tdata);

	void IT6602_VideoOutputConfigure_Init(struct it6602_dev_data *it6602, Video_Output_Configure eVidOutConfig);
	void hdmirx_Var_init(struct it6602_dev_data *it6602);
	void IT6602_Rst(struct it6602_dev_data *it6602);

	void chgbank(int bank);
	unsigned char CheckSCDT(struct it6602_dev_data *it6602);
	void WaitingForSCDT(struct it6602_dev_data *it6602);
	unsigned char CLKCheck(unsigned char ucPortSel);

	unsigned char IT6602_IsSelectedPort(unsigned char ucPortSel);

#ifdef _SUPPORT_EQ_ADJUST_
	void HDMIStartEQDetect(struct it6602_eq_data *ucEQPort);
	void HDMISetEQValue(struct it6602_eq_data *ucEQPort, unsigned char ucIndex);
	void HDMISwitchEQstate(struct it6602_eq_data *ucEQPort, unsigned char state);
	void HDMICheckSCDTon(struct it6602_eq_data *ucEQPort);
	void HDMIPollingErrorCount(struct it6602_eq_data *ucEQPort);
	void HDMIJudgeECCvalue(struct it6602_eq_data *ucEQPort);
	void HDMIAdjustEQ(struct it6602_eq_data *ucEQPort);
	void JudgeBestEQ(struct it6602_eq_data *ucEQPort);
	void StoreEccCount(struct it6602_eq_data *ucEQPort);
	void IT6602VideoCountClr(void);

#endif

#ifdef _SUPPORT_AUTO_EQ_
	void DisableOverWriteRS(unsigned char ucPortSel);
	void AmpValidCheck(unsigned char ucPortSel);
	void TogglePolarity(unsigned char ucPortSel);
	void TMDSCheck(unsigned char ucPortSel);
	void OverWriteAmpValue2EQ(unsigned char ucPortSel);
#endif

	unsigned char CheckAVMute(void);
	unsigned char CheckPlg5VPwr(unsigned char ucPortSel);
	unsigned char IsHDMIMode(void);
	void GetAVIInfoFrame(struct it6602_dev_data *it6602);

	void SetVideoInputFormatWithInfoFrame(struct it6602_dev_data *it6602);
	void SetColorimetryByInfoFrame(struct it6602_dev_data *it6602);
	void SetCSCBYPASS(struct it6602_dev_data *it6602);
	void SetColorSpaceConvert(struct it6602_dev_data *it6602);
	void SetNewInfoVideoOutput(struct it6602_dev_data *it6602);
	void SetVideoInputFormatWithoutInfoFrame(struct it6602_dev_data *it6602, unsigned char bInMode);
	void SetColorimetryByMode(struct it6602_dev_data *it6602);
	void SetDVIVideoOutput(struct it6602_dev_data *it6602);

	void IT6602_VideoOutputModeSet(struct it6602_dev_data *it6602);
	void IT6602VideoOutputConfigure(struct it6602_dev_data *it6602);
	void SetVideoOutputColorFormat(struct it6602_dev_data *it6602);
	void it6602PortSelect(unsigned char ucPortSel);
	void hdmirx_ECCTimingOut(unsigned char ucport);

	/* HDMI Audio function    *********************************************************/
	void aud_fiforst(void);
	void IT6602AudioOutputEnable(unsigned char bEnable);
	void hdmirx_ResetAudio(void);
	void hdmirx_SetHWMuteClrMode(void);
	void hdmirx_SetHWMuteClr(void);
	void hdmirx_ClearHWMuteClr(void);
	void getHDMIRXInputAudio(AUDIO_CAPS *pAudioCaps);
	void IT6602SwitchAudioState(struct it6602_dev_data *it6602, Audio_State_Type state);

	void IT6602AudioHandler(struct it6602_dev_data *it6602);

	/* HDMI Video function    *********************************************************/
	void IT6602_AFE_Rst(void);
	void IT6602_HDCP_ContentOff(unsigned char ucPort, unsigned char bOff);
	void IT6602_RAPContentOff(unsigned char bOff);

	void IT6602_SetVideoMute(struct it6602_dev_data *it6602, unsigned char bMute);
	void IT6602VideoOutputEnable(unsigned char bEnableOutput);
	void IT6602SwitchVideoState(struct it6602_dev_data *it6602, Video_State_Type eNewVState);
	void IT6602VideoHandler(struct it6602_dev_data *it6602);

	/* HDMI Interrupt function    *********************************************************/
	void hdmirx_INT_5V_Pwr_Chg(struct it6602_dev_data *it6602, unsigned char ucport);
	void hdmirx_INT_P0_ECC(struct it6602_dev_data *it6602);
	void hdmirx_INT_P0_Deskew(struct it6602_dev_data *it6602);
	//FIX_ID_009 xxxxx	//verify interrupt event with reg51[0] select port
	void hdmirx_INT_HDMIMode_Chg(struct it6602_dev_data *it6602, unsigned char ucport);
	//FIX_ID_009 xxxxx
	void hdmirx_INT_SCDT_Chg(struct it6602_dev_data *it6602);



	//FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
#ifdef _SUPPORT_AUTO_EQ_
	void hdmirx_INT_EQ_FAIL(struct it6602_dev_data *it6602, unsigned char ucPortSel);
#endif


	/* EDID RAM  functions    *******************************************************/
#ifdef _SUPPORT_EDID_RAM_
	unsigned char UpdateEDIDRAM(unsigned char *pEDID, unsigned char BlockNUM);

	void EnableEDIDupdata(void);
	void DisableEDIDupdata(void);
	// void EDIDRAMInitial(_CODE unsigned char *pIT6602EDID);
	void EDIDRAMInitial(unsigned char *pIT6602EDID);
	// unsigned char Find_Phyaddress_Location(_CODE unsigned char *pEDID,unsigned char Block_Number);
	unsigned char Find_Phyaddress_Location(unsigned char *pEDID, unsigned char Block_Number);
	void UpdateEDIDReg(unsigned char u8_VSDB_Addr, unsigned char CEC_AB, unsigned char CEC_CD,
			   unsigned char Block1_CheckSum);
	void PhyAdrSet(void);
#endif

	/* Driver State Machine Process **********************************************/
	void IT6602HDMIInterruptHandler(struct it6602_dev_data *it6602);

#ifndef Enable_IR
	void it6602AutoPortSelect(struct it6602_dev_data *it6602);
#endif

// #ifdef Enable_Vendor_Specific_packet
	void Dump3DReg(void);
	unsigned char IT6602_DE3DFrame(unsigned char ena_de3d);
// #endif

	char it66021_init(void);
	void it6602HPDCtrl(unsigned char ucport, unsigned char ucEnable);

	void IT6602HDMIEventManager(struct it6602_dev_data *it6602);

	void IT6602_fsm(void);

	char IT6602_fsm_init(void);
	void IT6602_ManualVideoTristate(unsigned char bOff);
	int mscCheckResult(void);
	void Dump_ITEHDMIReg(void);
};



typedef struct {
	I2CARG *it66021arg;

	I2CARG *edidarg;

	IT66021TYPE type;

	IT66021 *dev;

} IT66021_BUS_ARG;


#endif