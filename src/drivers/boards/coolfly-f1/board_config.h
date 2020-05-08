/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file board_config.h
 *
 * PX4FMU-v5 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include "ar_gpio.h"

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* Configuration ************************************************************************************/

/* Un-comment to support some RC00 polarities inversions
 * on test HW as well as R and G LEDs on UI LED are swapped
 */
//#define PX4_FMUV5_RC00

#define COOLFLY_F1

#define PX4_FMUV5_RC01
#define BOARD_HAS_LTC4417

#if defined(BOARD_HAS_LTC4417)
#  define BOARD_HAS_LTC44XX_VALIDS      2 // No. LTC or N Bricks
#  define BOARD_HAS_USB_VALID           1 // LTC Has No. USB valid
#  define BOARD_HAS_NBAT_V              2 // Only one Vbat to ADC
#  define BOARD_HAS_NBAT_I              2 // No. Ibat ADC
#else
#  define BOARD_HAS_LTC44XX_VALIDS      0 // No LTC or N Bricks
#  define BOARD_HAS_USB_VALID           0 // LTC Has No USB valid
#  define BOARD_HAS_NBAT_V              1 // Only one Vbat to ADC
#  define BOARD_HAS_NBAT_I              0 // No Ibat ADC
#endif

/* PX4FMU GPIOs ***********************************************************************************/

/* LEDs are driven with push open drain to support Anode to 5V or 3.3V */

#define GPIO_nLED_RED  		(GPIO_OUTPUT|GPIO_OUTSET|GPIO_PIN11)
#define GPIO_nLED_GREEN		(GPIO_OUTPUT|GPIO_OUTSET|GPIO_PIN10)
#define GPIO_nLED_BLUE 		(GPIO_OUTPUT|GPIO_OUTSET|GPIO_PIN12) // need add next hard version

#define BOARD_HAS_CONTROL_STATUS_LEDS      1

#define PX4_SPI_BUS_SENSORS   6
#define PX4_SPI_BUS_BARO      4
#define PX4_SPI_BUS_EXT   	  PX4_SPI_BUS_BARO

#define PX4_SPI_BUS_RAMTRON   0
#define PX4_SPI_BUS_MEMORY    0


#define GPIO_HDMI_VIDEO_INDICATOR 			(GPIO_PIN13)

/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

//#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT))
#define _PIN_OFF(def) 	(((def) & GPIO_PIN_MASK) | (GPIO_INPUT))

/*  Define the Chip Selects, Data Ready and Control signals per SPI bus */

/* SPI 6 CS */

#define GPIO_SPI6_CS1_ICM20689      	(GPIO_OUTPUT|GPIO_OUTSET|GPIO_PIN46)
#define GPIO_SPI6_CS2_ICM20602      	(GPIO_OUTPUT|GPIO_OUTSET|GPIO_PIN63)
#define GPIO_SPI6_CS3_BMI055_GYRO   	(GPIO_OUTPUT|GPIO_OUTSET|GPIO_PIN39)
#define GPIO_SPI6_CS4_BMI055_ACC    	(GPIO_OUTPUT|GPIO_OUTSET|GPIO_PIN59)
#define GPIO_SPI6_CS5_AUX_MEM       	(GPIO_OUTPUT|GPIO_OUTSET|GPIO_PIN41)

/*  Define the SPI6 Data Ready interrupts */

#define GPIO_SPI6_DRDY1_ICM20689      	(GPIO_INPUT|GPIO_PIN96)
#define GPIO_SPI6_DRDY2_BMI055_GYRO   	(GPIO_INPUT|GPIO_PIN97)
#define GPIO_SPI6_DRDY3_BMI055_ACC    	(GPIO_INPUT|GPIO_PIN98)
#define GPIO_SPI6_DRDY4_ICM20602      	(GPIO_INPUT|GPIO_PIN99)
#define GPIO_SPI6_DRDY5_BMI055_GYRO   	(GPIO_INPUT|GPIO_PIN100)
#define GPIO_SPI6_DRDY6_BMI055_ACC    	(GPIO_INPUT|GPIO_PIN101)

/* SPI6 off */

#define GPIO_SPI6_SCK_OFF  	_PIN_OFF(GPIO_SPI6_SCK)
#define GPIO_SPI6_MISO_OFF  _PIN_OFF(GPIO_SPI6_MISO)
#define GPIO_SPI6_MOSI_OFF  _PIN_OFF(GPIO_SPI6_MOSI)

#define GPIO_DRDY_OFF_SPI6_DRDY1_ICM20689    	_PIN_OFF(GPIO_SPI6_DRDY1_ICM20689)
#define GPIO_DRDY_OFF_SPI6_DRDY2_BMI055_GYRO 	_PIN_OFF(GPIO_SPI6_DRDY2_BMI055_GYRO)
#define GPIO_DRDY_OFF_SPI6_DRDY3_BMI055_ACC  	_PIN_OFF(GPIO_SPI6_DRDY3_BMI055_ACC)
#define GPIO_DRDY_OFF_SPI6_DRDY4_ICM20602    	_PIN_OFF(GPIO_SPI6_DRDY4_ICM20602)
#define GPIO_DRDY_OFF_SPI6_DRDY5_BMI055_GYRO 	_PIN_OFF(GPIO_SPI6_DRDY5_BMI055_GYRO)
#define GPIO_DRDY_OFF_SPI6_DRDY6_BMI055_ACC  	_PIN_OFF(GPIO_SPI6_DRDY6_BMI055_ACC)

// #define GPIO_VDD_3V3_SENSORS_EN

/* SPI 4 CS */

#define GPIO_SPI4_CS1_MS5611            (GPIO_OUTPUT|GPIO_OUTSET|GPIO_PIN54)
#define GPIO_SPI4_CS2_MS5611_ONBOARD    (GPIO_OUTPUT|GPIO_OUTSET|GPIO_PIN40)

#define GPIO_SPI4_SCK_OFF  	_PIN_OFF(GPIO_SPI4_SCK)
#define GPIO_SPI4_MISO_OFF  _PIN_OFF(GPIO_SPI4_MISO)
#define GPIO_SPI4_MOSI_OFF  _PIN_OFF(GPIO_SPI4_MOSI)

/* v BEGIN Legacy SPI defines TODO: fix this with enumeration */
#define PX4_SPIDEV_BMA 0
#define PX4_SPIDEV_BMI 0
/* ^ END Legacy SPI defines TODO: fix this with enumeration */


#define PX4_SPIDEV_ICM_20689        PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,0)
#define PX4_SPIDEV_ICM_20602        PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,1)
#define PX4_SPIDEV_BMI055_GYR       PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,2)
#define PX4_SPIDEV_BMI055_ACC       PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,3)
// #define PX4_SPIDEV_AUX_MEM          PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,4)

#define PX4_SENSOR_BUS_CS_GPIO      {	GPIO_SPI6_CS1_ICM20689, 	\
		GPIO_SPI6_CS2_ICM20602, 	\
		GPIO_SPI6_CS3_BMI055_GYRO, 	\
		GPIO_SPI6_CS4_BMI055_ACC	\
	}

#define PX4_SENSORS_BUS_FIRST_CS    PX4_SPIDEV_ICM_20689
#define PX4_SENSORS_BUS_LAST_CS     GPIO_SPI6_CS4_BMI055_ACC


#define PX4_SPIDEV_BARO                     PX4_MK_SPI_SEL(PX4_SPI_BUS_BARO,0)
#define PX4_SPIDEV_BARO_ONBOARD             PX4_MK_SPI_SEL(PX4_SPI_BUS_BARO,1)
#define PX4_SPIDEV_EXT_BARO					PX4_SPIDEV_BARO_ONBOARD

#define PX4_BARO_BUS_CS_GPIO            {\
		GPIO_SPI4_CS1_MS5611, 		  \
		GPIO_SPI4_CS2_MS5611_ONBOARD  \
	}


#define PX4_BARO_BUS_FIRST_CS       PX4_SPIDEV_BARO
#define PX4_BARO_BUS_LAST_CS        PX4_SPIDEV_BARO_ONBOARD




/*SPI0   */

#define GPIO_SPI0_CS_FRAM           (GPIO_OUTPUT|GPIO_OUTSET|GPIO_PIN70)
#define PX4_MEMORY_BUS_CS_GPIO      {GPIO_SPI0_CS_FRAM}
#define PX4_SPI_BUS_RAMTRON         0
#define PX4_SPI_BUS_MEMORY          0
#define PX4_SPIDEV_MEMORY           PX4_MK_SPI_SEL(PX4_SPI_BUS_MEMORY,0)
#define PX4_MEMORY_BUS_FIRST_CS     PX4_SPIDEV_MEMORY
#define PX4_MEMORY_BUS_LAST_CS      PX4_SPIDEV_MEMORY


/* I2C busses */

#define AR_I2C0_BUS                 1
#define AR_I2C1_BUS                 2
#define AR_I2C2_BUS                 3
#define AR_I2C3_BUS                 4

#define PX4_I2C_BUS_EXPANSION       AR_I2C3_BUS   // 1. rgbeled 
#define PX4_I2C_BUS_EXPANSION1      AR_I2C0_BUS   // GPS
#define PX4_I2C_BUS_ONBOARD         AR_I2C2_BUS   // on board
#define PX4_I2C_BUS_EXPANSION2      AR_I2C1_BUS

// #define PX4_I2C_BUS_LED             PX4_I2C_BUS_EXPANSION
#define PX4_I2C_BUS_LED             AR_I2C2_BUS
#define PX4_I2C_BUS_ONBOARD         AR_I2C2_BUS

#define PX4_I2C_BUS_IT66021_A         AR_I2C1_BUS
#define PX4_I2C_BUS_IT66021_B         AR_I2C0_BUS

// new liuwei
#define PX4_I2C_IT66021_A         	AR_I2C1_BUS

#define BOARD_NUMBER_I2C_BUSES      4
#define BOARD_I2C_BUS_CLOCK_INIT    {100000, 100000, 100000, 100000}

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */

#define ADC1_CH(n)                  (n)

/* Define Channel numbers must match above GPIO pin IN(n)*/

#define ADC_BATTERY1_VOLTAGE_CHANNEL                ADC1_CH(3)
#define ADC_BATTERY1_CURRENT_CHANNEL                ADC1_CH(4)
#define ADC_BATTERY2_VOLTAGE_CHANNEL                ADC1_CH(5)
#define ADC_BATTERY2_CURRENT_CHANNEL                ADC1_CH(6)
#define ADC1_SPARE_2_CHANNEL                        ADC1_CH(8)

#define ADC_RSSI_IN_CHANNEL                         ADC1_CH(9)      // temp for compile

#define ADC_SCALED_5V_CHANNEL                       ADC1_CH(2)

#define ADC_SCALED_VDD_3V3_SENSORS_CHANNEL          ADC1_CH(15) 	// temp for compile

#define ADC_HW_VER_SENSE_CHANNEL                    ADC1_CH(0)
#define ADC_HW_REV_SENSE_CHANNEL                    ADC1_CH(1)
#define ADC1_SPARE_1_CHANNEL                        ADC1_CH(7)
#define ADC_TEMPERATURE_AR8020_CHANNEL              ADC1_CH(12)
#define ADC_TEMPERATURE_PA_A_CHANNEL                ADC1_CH(14)
#define ADC_TEMPERATURE_PA_B_CHANNEL                ADC1_CH(10)


#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY1_CURRENT_CHANNEL)       | \
	 (1 << ADC_BATTERY2_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY2_CURRENT_CHANNEL)       | \
	 (1 << ADC1_SPARE_2_CHANNEL)               | \
	 (1 << ADC_RSSI_IN_CHANNEL)                | \
	 (1 << ADC_SCALED_5V_CHANNEL)              | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL) | \
	 (1 << ADC_HW_VER_SENSE_CHANNEL)           | \
	 (1 << ADC_HW_REV_SENSE_CHANNEL)           | \
	 (1 << ADC1_SPARE_1_CHANNEL)               | \
	 (1 << ADC_TEMPERATURE_AR8020_CHANNEL)     | \
	 (1 << ADC_TEMPERATURE_PA_A_CHANNEL)       | \
	 (1 << ADC_TEMPERATURE_PA_B_CHANNEL))



/* Define Battery 1 Voltage Divider and A per V
 */
// todo:
#define BOARD_BATTERY1_V_DIV         (25.9f)     /* measured with the provided PM board */
#define BOARD_BATTERY1_A_PER_V       (36.367515152f)

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* HW Version and Revision drive signals Default to 1 to detect */

#define BOARD_HAS_HW_VERSIONING


#define HW_INFO_INIT         {'V','2','x', 'x',0}
#define HW_INFO_INIT_VER     2
#define HW_INFO_INIT_REV     3
/* CAN Silence
 *
 * Silent mode control \ ESC Mux select
 */


/* HEATER
 * PWM in future
 */
//#define GPIO_HEATER        /* PA7  T14CH1 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN7)

/* PWM Capture
 *
 * 3  PWM Capture inputs are configured.
 *
 * Pins:
 *
 * FMU_CAP1 : PA5  : TIM2_CH1
 * FMU_CAP2 : PB3  : TIM2_CH2
 * FMU_CAP3 : PB11 : TIM2_CH4
 */
#if 0
#define GPIO_TIM2_CH1_IN     /* PA5   T22C1  FMU_CAP1 */ GPIO_TIM2_CH1IN_3
#define GPIO_TIM2_CH2_IN     /* PB3   T22C2  FMU_CAP2 */ GPIO_TIM2_CH2IN_2
#define GPIO_TIM2_CH4_IN     /* PB11  T22C4  FMU_CAP3 */ GPIO_TIM2_CH4IN_2

#define DIRECT_PWM_CAPTURE_CHANNELS  3

/* TIM5_CH4 SPARE PIN */
#define GPIO_TIM5_CH4IN    /* PI0   T5C4  TIM5_SPARE_4 */  GPIO_TIM5_CH4IN_2
#define GPIO_TIM5_CH4OUT   /* PI0   T5C4  TIM5_SPARE_4 */   GPIO_TIM5_CH4OUT_2
#endif

/* PWM
 *
 * 8  PWM outputs are configured.
 *
 * Pins:
 *
 * FMU_CH1 : PE14 : TIM1_CH4
 * FMU_CH2 : PA10 : TIM1_CH3
 * FMU_CH3 : PE11 : TIM1_CH2
 * FMU_CH4 : PE9  : TIM1_CH1
 * FMU_CH5 : PD13 : TIM4_CH2
 * FMU_CH6 : PD14 : TIM4_CH3
 * FMU_CH7 : PH6  : TIM12_CH1
 * FMU_CH8 : PH9  : TIM12_CH2
 *
 */

#define DIRECT_PWM_OUTPUT_CHANNELS  	8

/* Power supply control and monitoring GPIOs */

#define GPIO_nPOWER_IN_A         	(GPIO_INPUT| GPIO_PIN77)
#define GPIO_nPOWER_IN_B         	(GPIO_INPUT| GPIO_PIN78)
#define GPIO_nPOWER_IN_C         	(GPIO_INPUT| GPIO_PIN79)

#define GPIO_nVDD_BRICK1_VALID          GPIO_nPOWER_IN_A /* Brick 1 Is Chosen */
#define GPIO_nVDD_BRICK2_VALID          GPIO_nPOWER_IN_B /* Brick 2 Is Chosen  */
#define BOARD_NUMBER_BRICKS             2
#define GPIO_nVDD_USB_VALID             GPIO_nPOWER_IN_C /* USB     Is Chosen */


#define GPIO_VDD_3V3_SENSORS_EN         (GPIO_OUTPUT|GPIO_OUTSET|GPIO_PIN102)

#define VDD_3V3_SENSORS_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, (on_true))


#define RC_SERIAL_PORT               "/dev/ttyS21"


/* High-resolution timer */
// ar8020 uses 2 clk  to  instead of the stm32 ccr irq

#define HRT_TIMER_FREERUN               1  /* use timer2 for the HRT */
#define HRT_TIMER_FREERUN_CHANNEL       7  /* use capture/compare channel 7*/

#define HRT_TIMER_CCR               	1  /* use timer2 for the HRT */
#define HRT_TIMER_CCR_CHANNEL       	6  /* use capture/compare channel 6 */


/* Tone alarm output */
#define TONE_ALARM_TIMER                0  /* timer 0 */
#define TONE_ALARM_CHANNEL              0  /* TIM0_CH0 */

#define GPIO_BUZZER_1                   (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN80)

#define GPIO_TONE_ALARM_IDLE            GPIO_BUZZER_1
#define GPIO_TONE_ALARM                 (GPIO_DEFAULT|GPIO_PIN80)


#define GPS_DEFAULT_UART_PORT              "/dev/ttyS2" /* UART6 on FMUv5 */



/* Safety Switch is HW version dependent on having an PX4IO
 * So we init to a benign state with the _INIT definition
 * and provide the the non _INIT one for the driver to make a run time
 * decision to use it.
 */
#define GPIO_nSAFETY_SWITCH_LED_OUT_INIT   (GPIO_INPUT|GPIO_PIN104)
#define GPIO_nSAFETY_SWITCH_LED_OUT        (GPIO_OUTPUT|GPIO_OUTSET|GPIO_PIN104)

/* Enable the FMU to control it if there is no px4io fixme:This should be BOARD_SAFETY_LED(__ontrue) */
#define GPIO_LED_SAFETY                    (GPIO_nSAFETY_SWITCH_LED_OUT)
#define GPIO_SAFETY_SWITCH_IN              (GPIO_INPUT|GPIO_PIN103)

/* Enable the FMU to use the switch it if there is no px4io fixme:This should be BOARD_SAFTY_BUTTON() */
#define GPIO_BTN_SAFETY                    (GPIO_SAFETY_SWITCH_IN) /* Enable the FMU to control it if there is no px4io */

/* Power switch controls ******************************************************/

//#define SPEKTRUM_POWER(_on_true)           VDD_3V3_SPEKTRUM_POWER_EN(_on_true)

#define SDIO_SLOTNO                    0  /* Only one slot */
#define SDIO_MINOR                     0

/* SD card bringup does not work if performed on the IDLE thread because it
 * will cause waiting.  Use either:
 *
 *  CONFIG_LIB_BOARDCTL=y, OR
 *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
 */

#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_LIB_BOARDCTL) && \
   !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

#define  BOARD_NAME "COOLFLY_F1_V2"

/* FMUv5 never powers odd the Servo rail */

#if !defined(BOARD_HAS_LTC44XX_VALIDS) || BOARD_HAS_LTC44XX_VALIDS == 0
#  define BOARD_ADC_BRICK1_VALID  (1)
#  define BOARD_ADC_BRICK2_VALID  (0)
#elif BOARD_HAS_LTC44XX_VALIDS == 1
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (0)
#elif BOARD_HAS_LTC44XX_VALIDS == 2
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#elif BOARD_HAS_LTC44XX_VALIDS == 3
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#  define BOARD_ADC_BRICK3_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK3_VALID))
#elif BOARD_HAS_LTC44XX_VALIDS == 4
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#  define BOARD_ADC_BRICK3_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK3_VALID))
#  define BOARD_ADC_BRICK4_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK4_VALID))
#else
#  error Unsupported BOARD_HAS_LTC44XX_VALIDS value
#endif


#define BOARD_HAS_PWM           DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_FMU_GPIO_TAB { \
		{GPIO_GPIO1_INPUT,       GPIO_GPIO1_OUTPUT,              0}, \
		{GPIO_GPIO2_INPUT,       GPIO_GPIO2_OUTPUT,              0}, \
		{GPIO_GPIO3_INPUT,       GPIO_GPIO3_OUTPUT,              0}, \
		{GPIO_GPIO4_INPUT,       GPIO_GPIO4_OUTPUT,              0}, \
		{GPIO_GPIO5_INPUT,       GPIO_GPIO5_OUTPUT,              0}, \
		{GPIO_GPIO6_INPUT,       GPIO_GPIO6_OUTPUT,              0}, \
		{GPIO_GPIO7_INPUT,       GPIO_GPIO7_OUTPUT,              0}, \
		{GPIO_GPIO8_INPUT,       GPIO_GPIO8_OUTPUT,              0}, \
		{GPIO_GPIO9_INPUT,       GPIO_GPIO9_OUTPUT,              0}, \
		{GPIO_nPOWER_IN_A,       0,                              0}, \
		{GPIO_nPOWER_IN_B,       0,                              0}, \
		{GPIO_nPOWER_IN_C,       0,                              0} \
	}

/*
 * GPIO numbers.
 *
 * There are no alternate functions on this board.
 */
#define GPIO_SERVO_1                (1<<0)   /**< servo 1 output */
#define GPIO_SERVO_2                (1<<1)   /**< servo 2 output */
#define GPIO_SERVO_3                (1<<2)   /**< servo 3 output */
#define GPIO_SERVO_4                (1<<3)   /**< servo 4 output */
#define GPIO_SERVO_5                (1<<4)   /**< servo 5 output */
#define GPIO_SERVO_6                (1<<5)   /**< servo 6 output */
#define GPIO_SERVO_7                (1<<6)   /**< servo 7 output */
#define GPIO_SERVO_8                (1<<7)   /**< servo 8 output */

#define GPIO_nPOWER_INPUT_A         (1<<8)   /**<PG1 GPIO_nPOWER_IN_A */
#define GPIO_nPOWER_INPUT_B         (1<<9)   /**<PG2 GPIO_nPOWER_IN_B */
#define GPIO_nPOWER_INPUT_C         (1<<10)  /**<PG3 GPIO_nPOWER_IN_C */

#define GPIO_3V3_SENSORS_EN         (1<<15)  /**< PE3  - VDD_3V3_SENSORS_EN */

/* This board provides a DMA pool and APIs */

#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 1






/* The list of GPIO that will be initialized */

/* User GPIOs
 *
 * GPIO1-9 are the PWM servo outputs.
 */

#define GPIO_GPIO9_INPUT        	(GPIO_INPUT|GPIO_PIN89)
#define GPIO_GPIO8_INPUT        	(GPIO_INPUT|GPIO_PIN88)
#define GPIO_GPIO7_INPUT        	(GPIO_INPUT|GPIO_PIN87)
#define GPIO_GPIO6_INPUT        	(GPIO_INPUT|GPIO_PIN86)
#define GPIO_GPIO5_INPUT        	(GPIO_INPUT|GPIO_PIN85)
#define GPIO_GPIO4_INPUT        	(GPIO_INPUT|GPIO_PIN84)
#define GPIO_GPIO3_INPUT       		(GPIO_INPUT|GPIO_PIN83)
#define GPIO_GPIO2_INPUT       		(GPIO_INPUT|GPIO_PIN82)
#define GPIO_GPIO1_INPUT       		(GPIO_INPUT|GPIO_PIN81)
#define GPIO_GPIO0_INPUT       		(GPIO_INPUT|GPIO_PIN80)


#define GPIO_GPIO9_OUTPUT       	(GPIO_DEFAULT|GPIO_PIN89)
#define GPIO_GPIO8_OUTPUT       	(GPIO_DEFAULT|GPIO_PIN88)
#define GPIO_GPIO7_OUTPUT       	(GPIO_DEFAULT|GPIO_PIN87)
#define GPIO_GPIO6_OUTPUT       	(GPIO_DEFAULT|GPIO_PIN86)
#define GPIO_GPIO5_OUTPUT       	(GPIO_DEFAULT|GPIO_PIN85)
#define GPIO_GPIO4_OUTPUT       	(GPIO_DEFAULT|GPIO_PIN84)
#define GPIO_GPIO3_OUTPUT       	(GPIO_DEFAULT|GPIO_PIN83)
#define GPIO_GPIO2_OUTPUT       	(GPIO_DEFAULT|GPIO_PIN82)
#define GPIO_GPIO1_OUTPUT       	(GPIO_DEFAULT|GPIO_PIN81)
#define GPIO_GPIO0_OUTPUT       	(GPIO_DEFAULT|GPIO_PIN80)


#define GP_TIM1_CH1IN             GPIO_GPIO9_INPUT
#define GP_TIM1_CH0IN             GPIO_GPIO8_INPUT
#define GP_TIM0_CH7IN             GPIO_GPIO7_INPUT
#define GP_TIM0_CH6IN             GPIO_GPIO6_INPUT
#define GP_TIM0_CH5IN             GPIO_GPIO5_INPUT
#define GP_TIM0_CH4IN             GPIO_GPIO4_INPUT
#define GP_TIM0_CH3IN             GPIO_GPIO3_INPUT
#define GP_TIM0_CH2IN             GPIO_GPIO2_INPUT
#define GP_TIM0_CH1IN             GPIO_GPIO1_INPUT
// #define GP_TIM0_CH0IN             GPIO_GPIO0_INPUT

#define GP_TIM1_CH1OUT            GPIO_GPIO9_OUTPUT
#define GP_TIM1_CH0OUT            GPIO_GPIO8_OUTPUT
#define GP_TIM0_CH7OUT            GPIO_GPIO7_OUTPUT
#define GP_TIM0_CH6OUT            GPIO_GPIO6_OUTPUT
#define GP_TIM0_CH5OUT            GPIO_GPIO5_OUTPUT
#define GP_TIM0_CH4OUT            GPIO_GPIO4_OUTPUT
#define GP_TIM0_CH3OUT            GPIO_GPIO3_OUTPUT
#define GP_TIM0_CH2OUT            GPIO_GPIO2_OUTPUT
#define GP_TIM0_CH1OUT            GPIO_GPIO1_OUTPUT
// #define GP_TIM0_CH0OUT            GPIO_GPIO0_OUTPUT



#define PX4_GPIO_PWM_INIT_LIST { \
		GPIO_GPIO9_INPUT, \
		GPIO_GPIO8_INPUT, \
		GPIO_GPIO7_INPUT, \
		GPIO_GPIO6_INPUT, \
		GPIO_GPIO5_INPUT, \
		GPIO_GPIO4_INPUT, \
		GPIO_GPIO3_INPUT, \
		GPIO_GPIO2_INPUT, \
		GPIO_GPIO1_INPUT  \
	}

#define GPIO_CAN0_TX    	(GPIO_DEFAULT|GPIO_PIN94)
#define GPIO_CAN0_RX    	(GPIO_DEFAULT|GPIO_PIN90)

#define GPIO_CAN1_TX    	(GPIO_DEFAULT|GPIO_PIN95)
#define GPIO_CAN1_RX    	(GPIO_DEFAULT|GPIO_PIN91)


#define GPIO_LNA_BYPASS     (GPIO_INPUT|GPIO_PIN105)


#define PX4_GPIO_INIT_LIST {    \
		GPIO_CAN0_TX,       \
		GPIO_CAN0_RX,       \
		GPIO_CAN1_TX,       \
		GPIO_CAN1_RX,       \
		GPIO_LNA_BYPASS,    \
		GPIO_nPOWER_IN_A,                 	\
		GPIO_nPOWER_IN_B,                 	\
		GPIO_nPOWER_IN_C,                 	\
		GPIO_TONE_ALARM_IDLE,             	\
		GPIO_nSAFETY_SWITCH_LED_OUT_INIT, 	\
		GPIO_SAFETY_SWITCH_IN,            	\
		GPIO_VDD_3V3_SENSORS_EN,          	\
		GPIO_SPI6_DRDY1_ICM20689, 			\
		GPIO_SPI6_DRDY2_BMI055_GYRO,		\
		GPIO_SPI6_DRDY3_BMI055_ACC,			\
		GPIO_SPI6_DRDY4_ICM20602,			\
		GPIO_SPI6_DRDY5_BMI055_GYRO,		\
		GPIO_SPI6_DRDY6_BMI055_ACC 			\
	}


// #define GPIO_NOUSED_G8     	(GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN8)
// #define GPIO_NOUSED_G9     	(GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN9)
// #define GPIO_NOUSED_G10     (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN10)
// #define GPIO_NOUSED_G11     (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN11)
// #define GPIO_NOUSED_G12     (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN12)
// #define GPIO_NOUSED_G13     (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN13)
// #define GPIO_NOUSED_G14     (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN14)
// #define GPIO_NOUSED_G15     (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN15)
// #define GPIO_NOUSED_G24    	(GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN24)
// #define GPIO_NOUSED_G25    	(GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN25)
// #define GPIO_NOUSED_G26     (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN26)
// #define GPIO_NOUSED_G27     (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN27)
// #define GPIO_NOUSED_G28     (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN28)
// #define GPIO_NOUSED_G29     (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN29)
// #define GPIO_NOUSED_G30     (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN30)
// #define GPIO_NOUSED_G31     (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN31)
// #define GPIO_NOUSED_G76     (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN76)
// #define GPIO_NOUSED_G77     (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN77)
// #define GPIO_NOUSED_G78     (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN78)
// #define GPIO_NOUSED_G79     (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN79)
// #define GPIO_NOUSED_G92     (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN92)
// #define GPIO_NOUSED_G93     (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN93)
// #define GPIO_NOUSED_G105    (GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN105)
// #define GPIO_NOUSED_G44     (GPIO_INPUT|GPIO_PIN44)



#define GPIO_SPI2_CS     	(GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN72)
#define GPIO_SPI2_SCK     	(GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN68)
#define GPIO_SPI2_MOSI     	(GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN64)
#define GPIO_SPI2_MISO     	(GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN60)

#define GPIO_SPI3_CS     	(GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN73)
#define GPIO_SPI3_SCK     	(GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN69)
#define GPIO_SPI3_MOSI     	(GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN65)
#define GPIO_SPI3_MISO     	(GPIO_OUTPUT|GPIO_OUTRESET|GPIO_PIN61)

#define CF_GPIO_INIT_NOUSED_LIST {    \
		GPIO_SPI2_CS,  		\
		GPIO_SPI2_SCK, 		\
		GPIO_SPI2_MOSI,		\
		GPIO_SPI2_MISO,     \
		GPIO_SPI3_CS,  		\
		GPIO_SPI3_SCK, 		\
		GPIO_SPI3_MOSI,		\
		GPIO_SPI3_MISO      \
	}


#define GPIO_DVP0_DATA0     (GPIO_DEFAULT|GPIO_PIN16)
#define GPIO_DVP0_DATA1     (GPIO_DEFAULT|GPIO_PIN17)
#define GPIO_DVP0_DATA2     (GPIO_DEFAULT|GPIO_PIN18)
#define GPIO_DVP0_DATA3     (GPIO_DEFAULT|GPIO_PIN19)
#define GPIO_DVP0_DATA4     (GPIO_DEFAULT|GPIO_PIN20)
#define GPIO_DVP0_DATA5     (GPIO_DEFAULT|GPIO_PIN21)
#define GPIO_DVP0_DATA6     (GPIO_DEFAULT|GPIO_PIN22)
#define GPIO_DVP0_DATA7     (GPIO_DEFAULT|GPIO_PIN23)
#define GPIO_DVP0_VS     	(GPIO_DEFAULT|GPIO_PIN35)
#define GPIO_DVP0_HS     	(GPIO_DEFAULT|GPIO_PIN36)
#define GPIO_DVP0_DE     	(GPIO_DEFAULT|GPIO_PIN37)

#define GPIO_DVP1_DATA0     (GPIO_DEFAULT|GPIO_PIN0)
#define GPIO_DVP1_DATA1     (GPIO_DEFAULT|GPIO_PIN1)
#define GPIO_DVP1_DATA2     (GPIO_DEFAULT|GPIO_PIN2)
#define GPIO_DVP1_DATA3     (GPIO_DEFAULT|GPIO_PIN3)
#define GPIO_DVP1_DATA4     (GPIO_DEFAULT|GPIO_PIN4)
#define GPIO_DVP1_DATA5     (GPIO_DEFAULT|GPIO_PIN5)
#define GPIO_DVP1_DATA6     (GPIO_DEFAULT|GPIO_PIN6)
#define GPIO_DVP1_DATA7     (GPIO_DEFAULT|GPIO_PIN7)
#define GPIO_DVP1_VS     	(GPIO_DEFAULT|GPIO_PIN32)
#define GPIO_DVP1_HS     	(GPIO_DEFAULT|GPIO_PIN33)
#define GPIO_DVP1_DE     	(GPIO_DEFAULT|GPIO_PIN34)

#define GPIO_ITE_RST0     	(GPIO_OUTPUT|GPIO_OUTSET|GPIO_PIN24)
#define GPIO_ITE_RST1     	(GPIO_OUTPUT|GPIO_OUTSET|GPIO_PIN25)
#define GPIO_ITE_INT0     	(GPIO_INPUT|GPIO_PIN26)
#define GPIO_ITE_INT1     	(GPIO_INPUT|GPIO_PIN27)

#define CF_GPIO_INIT_DVP_LIST {    \
		GPIO_DVP0_DATA0,    \
		GPIO_DVP0_DATA1,    \
		GPIO_DVP0_DATA2,    \
		GPIO_DVP0_DATA3,    \
		GPIO_DVP0_DATA4,    \
		GPIO_DVP0_DATA5,    \
		GPIO_DVP0_DATA6,    \
		GPIO_DVP0_DATA7,    \
		GPIO_DVP0_VS,       \
		GPIO_DVP0_HS,       \
		GPIO_DVP0_DE,       \
		GPIO_DVP1_DATA0,    \
		GPIO_DVP1_DATA1,    \
		GPIO_DVP1_DATA2,    \
		GPIO_DVP1_DATA3,    \
		GPIO_DVP1_DATA4,    \
		GPIO_DVP1_DATA5,    \
		GPIO_DVP1_DATA6,    \
		GPIO_DVP1_DATA7,    \
		GPIO_DVP1_VS , 	 	\
		GPIO_DVP1_HS ,  	\
		GPIO_DVP1_DE,		\
		GPIO_ITE_RST0,      \
		GPIO_ITE_RST1,      \
		GPIO_ITE_INT0,      \
		GPIO_ITE_INT1      	\
	}



__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************
 * Name: ar_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int ar_sdio_initialize(void);

/****************************************************************************************************
 * Name: ar_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void ar_spiinitialize(void);

/************************************************************************************
 * Name: ar_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI Buses.
 *
 ************************************************************************************/

extern int ar_spi_bus_initialize(void);

void board_spi_reset(int ms);

extern void ar_usbinitialize(void);

extern void board_peripheral_reset(int ms);


/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization for NSH.
 *
 *   CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, &&
 *   CONFIG_NSH_ARCHINIT=n :
 *     Called from board_initialize().
 *
 ****************************************************************************/

#ifdef CONFIG_NSH_LIBRARY
int nsh_archinitialize(void);
#endif

#include "../common/board_common.h"

#endif /* __ASSEMBLY__ */

__END_DECLS
