#ifndef _COOLFLY_IT66021_I2C_H_
#define _COOLFLY_IT66021_I2C_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include <board_config.h>


#define IT6602A0_HDMI_ADDR 0x92 //Hardware Fixed I2C address of IT6602 HDMI
#define IT6602B0_HDMI_ADDR 0x90 //Hardware Fixed I2C address of IT6602 HDMI
#define MHL_ADDR 0xE0   //Software programmable I2C address of IT6602 MHL
#define EDID_ADDR 0xA8  //Software programmable I2C address of IT6602 EDID RAM
#define CEC_ADDR 0xC8   //Software programmable I2C address of IT6602 CEC

#define IT66021A_HDMI_ADDR 		(0x92)
#define IT66021A_HDMI_ADDR_CEC 	(0xC8)
#define IT66021A_HDMI_ADDR_EDID (0xA8)
#define IT66021A_HDMI_ADDR_RING (0xC0)

#define PX4_I2C_BUS_IT66021_A_EDID PX4_I2C_BUS_IT66021_A
#define PX4_I2C_BUS_IT66021_A_RING PX4_I2C_BUS_IT66021_A

#define IT66021_DEFAULT_BUS_SPEED 100000


typedef struct 
{   
    const char *name;
    const char *devname;
    uint8_t bus;
    uint32_t address;
    uint32_t frequency;

} I2CARG; // I2C parameter

typedef enum
{
	IT66021TYPE_A,
	IT66021TYPE_B,
} IT66021TYPE;


class IT66021; 

typedef struct
{
    I2CARG *it66021arg;
	
	I2CARG *edidarg;

	IT66021TYPE type;

    IT66021 *dev;

} IT66021_BUS_ARG;

typedef void (*HAL_HDMI_RxHandle)(void *pu8_rxBuf);

typedef struct
{
    uint16_t u16_width;
    uint16_t u16_hight;
    uint8_t  u8_framerate;
    uint8_t  u8_vic;
} STRU_HDMI_RX_OUTPUT_FORMAT;


typedef enum
{
    HAL_HDMI_POLLING = 0,
    HAL_HDMI_INTERRUPT,
} ENUM_HAL_HDMI_GETFORMATMETHOD;


typedef enum
{
    HAL_HDMI_RX_16BIT = 0,
    HAL_HDMI_RX_8BIT,
} ENUM_HAL_HDMI_COLOR_DEPTH;


typedef struct
{
    ENUM_HAL_HDMI_GETFORMATMETHOD e_getFormatMethod;
    ENUM_HAL_HDMI_COLOR_DEPTH e_colorDepth;
    // STRU_HDMI_GPIOCONFIGURE st_interruptGpio;
    uint8_t u8_hdmiToEncoderCh;
} STRU_HDMI_CONFIGURE;

typedef struct
{
    uint8_t u8_devEnable;
    STRU_HDMI_RX_OUTPUT_FORMAT st_videoFormat;
    STRU_HDMI_CONFIGURE        st_configure;
} STRU_HDMI_RX_STATUS;

typedef enum
{
    HAL_HDMI_RX_1 = 0,
    HAL_HDMI_RX_0,
    HAL_HDMI_RX_MAX
} ENUM_HAL_HDMI_RX;


#define HDMI_RX_FORMAT_NOT_SUPPORT_COUNT_MAX 50



#endif