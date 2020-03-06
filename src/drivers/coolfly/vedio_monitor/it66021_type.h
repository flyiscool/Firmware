#ifndef _COOLFLY_IT66021_I2C_H_
#define _COOLFLY_IT66021_I2C_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <board_config.h>

#include "px4_log.h"
#include "it66021_define.h"


//#define IT66021DEBUG

// debug opt
#ifdef IT66021DEBUG
#define  IT_INFO(...) PX4_INFO(__VA_ARGS__)
#else
#define  IT_INFO(...)
#endif


#define IT66021A_HDMI_ADDR 	0x92
#define EDID_ADDR 			0xA8
#define MHL_ADDR 			0xE0
#define CEC_ADDR 			0xC8

#define IT66021_DEFAULT_BUS_SPEED 100000	// default 100kHz


typedef enum {
	IT66021TYPE_A,
	IT66021TYPE_B,
} IT66021TYPE;


typedef void (*HAL_HDMI_RxHandle)(void *pu8_rxBuf);

typedef struct {
	uint16_t u16_width;
	uint16_t u16_hight;
	uint8_t  u8_framerate;
	uint8_t  u8_vic;
} STRU_HDMI_RX_OUTPUT_FORMAT;

typedef enum {
	HAL_HDMI_POLLING = 0,
	HAL_HDMI_INTERRUPT,
} ENUM_HAL_HDMI_GETFORMATMETHOD;

typedef enum {
	HAL_HDMI_RX_16BIT = 0,
	HAL_HDMI_RX_8BIT,
} ENUM_HAL_HDMI_COLOR_DEPTH;

typedef struct {
	ENUM_HAL_HDMI_GETFORMATMETHOD e_getFormatMethod;
	ENUM_HAL_HDMI_COLOR_DEPTH e_colorDepth;
	uint8_t u8_hdmiToEncoderCh;
} STRU_HDMI_CONFIGURE;

typedef struct {
	uint8_t u8_devEnable;
	STRU_HDMI_RX_OUTPUT_FORMAT st_videoFormat;
	STRU_HDMI_CONFIGURE st_configure;
} STRU_HDMI_RX_STATUS;

typedef enum {
	HAL_HDMI_RX_1 = 0,
	HAL_HDMI_RX_0,
	HAL_HDMI_RX_MAX
} ENUM_HAL_HDMI_RX;

#define HDMI_RX_FORMAT_NOT_SUPPORT_COUNT_MAX 50	// may need used

#define MS_TimeOut(x) (x+1)



#endif
