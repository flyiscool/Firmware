/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file px4fmu_spi.c
 *
 * Board-specific SPI functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_config.h>
#include <px4_log.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <unistd.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
#include <systemlib/px4_macros.h>

#include <up_arch.h>
#include <chip.h>
#include <ar_gpio.h>
#include "board_config.h"
#include <systemlib/err.h>

#include "ar_uart.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Debug ********************************************************************/

/* Define CS GPIO array */
static const uint32_t spi6selects_gpio[] = PX4_SENSOR_BUS_CS_GPIO;
static const uint32_t spi4selects_gpio[] = PX4_BARO_BUS_CS_GPIO;
static const uint32_t spi0selects_gpio[] = PX4_MEMORY_BUS_CS_GPIO;
/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: ar_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ************************************************************************************/

__EXPORT void ar_spiinitialize(void)
{
	board_gpio_init(spi0selects_gpio, arraySize(spi0selects_gpio));
	board_gpio_init(spi4selects_gpio, arraySize(spi4selects_gpio));
	board_gpio_init(spi6selects_gpio, arraySize(spi6selects_gpio));
	putreg32(0, AR_SPI0_BASE + SPI_SSIENR);
	putreg32(0, AR_SPI4_BASE + SPI_SSIENR);
	putreg32(0, AR_SPI6_BASE + SPI_SSIENR);
}

/************************************************************************************
 * Name: ar_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI buses on PX4FMU board.
 *
 ************************************************************************************/

static struct spi_dev_s *spi_memory;
static struct spi_dev_s *spi_sensors;
static struct spi_dev_s *spi_baro;

__EXPORT int ar_spi_bus_initialize(void)
{
	/* Configure SPI-based devices */

	/* Get the SPI port for the Sensors */

	spi_sensors = ar_spibus_initialize(PX4_SPI_BUS_SENSORS);

	if (!spi_sensors) {
		PX4_ERR("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_SENSORS);
		return -ENODEV;
	}

	/* Default PX4_SPI_BUS_SENSORS to 1MHz and de-assert the known chip selects. */

	SPI_SETFREQUENCY(spi_sensors, 10000000);
	SPI_SETBITS(spi_sensors, 8);
	SPI_SETMODE(spi_sensors, SPIDEV_MODE3);

	for (int cs = PX4_SENSORS_BUS_FIRST_CS; cs <= PX4_SENSORS_BUS_LAST_CS; cs++) {
		SPI_SELECT(spi_sensors, cs, false);
	}

	/* Get the SPI port for the BARO */

	spi_baro = ar_spibus_initialize(PX4_SPI_BUS_BARO);

	if (!spi_baro) {
		PX4_ERR("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_BARO);
		return -ENODEV;
	}

	/* MS5611 has max SPI clock speed of 20MHz	*/

	SPI_SETFREQUENCY(spi_baro, 20 * 1000 * 1000);
	SPI_SETBITS(spi_baro, 8);
	SPI_SETMODE(spi_baro, SPIDEV_MODE3);

	for (int cs = PX4_BARO_BUS_FIRST_CS; cs <= PX4_BARO_BUS_LAST_CS; cs++) {
		SPI_SELECT(spi_baro, cs, false);
	}


	/* Get the SPI port for the Memory */

	spi_memory = ar_spibus_initialize(PX4_SPI_BUS_MEMORY);

	if (!spi_memory) {
		PX4_ERR("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_MEMORY);
		return -ENODEV;
	}

	/* Default PX4_SPI_BUS_MEMORY to 12MHz and de-assert the known chip selects.
	 */

	SPI_SETFREQUENCY(spi_memory, 12 * 1000 * 1000);
	SPI_SETBITS(spi_memory, 8);
	SPI_SETMODE(spi_memory, SPIDEV_MODE3);

	for (int cs = PX4_MEMORY_BUS_FIRST_CS; cs <= PX4_MEMORY_BUS_LAST_CS; cs++) {
		SPI_SELECT(spi_memory, cs, false);
	}

	return OK;
}

/************************************************************************************
 * Name: ar_spi6select and ar_spi6status
 *
 * Description:
 *   Called by ar spi driver on bus 6.
 *
 ************************************************************************************/

__EXPORT void ar_spi6select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	int sel = (int) devid;
	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_SENSORS);

	/* Making sure the other peripherals are not selected */

	for (size_t cs = 0; arraySize(spi6selects_gpio) > 1 && cs < arraySize(spi6selects_gpio); cs++) {
		if (spi6selects_gpio[cs] != 0) {
			ar_gpiowrite(spi6selects_gpio[cs], 1);
		}
	}

	uint32_t gpio = spi6selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		ar_gpiowrite(gpio, !selected);
	}
}

__EXPORT uint8_t ar_spi6status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}

/************************************************************************************
 * Name: ar_spi4select and ar_spi4status
 *
 * Description:
 *   Called by ar spi driver on bus 4.
 *
 ************************************************************************************/

__EXPORT void ar_spi4select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	int sel = (int) devid;

	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_BARO);

	/* Making sure the other peripherals are not selected */
	for (size_t cs = 0; arraySize(spi4selects_gpio) > 1 && cs < arraySize(spi4selects_gpio); cs++) {
		ar_gpiowrite(spi4selects_gpio[cs], 1);
	}

	uint32_t gpio = spi4selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		ar_gpiowrite(gpio, !selected);
	}
}

__EXPORT uint8_t ar_spi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}


/************************************************************************************
 * Name: ar_spi4select and ar_spi4status
 *
 * Description:
 *   Called by ar spi driver on bus 4.
 *
 ************************************************************************************/

__EXPORT void ar_spi0select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	int sel = (int) devid;

	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_MEMORY);

	/* Making sure the other peripherals are not selected */
	for (size_t cs = 0; arraySize(spi0selects_gpio) > 1 && cs < arraySize(spi0selects_gpio); cs++) {
		ar_gpiowrite(spi0selects_gpio[cs], 1);
	}

	uint32_t gpio = spi0selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		ar_gpiowrite(gpio, !selected);
	}
}

__EXPORT uint8_t ar_spi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}

/************************************************************************************
 * Name: board_spi_reset
 *
 * Description:
 *
 *
 ************************************************************************************/

__EXPORT void board_spi_reset(int ms)
{
	/* disable SPI bus */
	for (size_t cs = 0;  arraySize(spi6selects_gpio) > 1 && cs < arraySize(spi6selects_gpio); cs++) {
		if (spi6selects_gpio[cs] != 0) {
			ar_configgpio(_PIN_OFF(spi6selects_gpio[cs]));
		}
	}

	for (size_t cs = 0;  arraySize(spi4selects_gpio) > 1 && cs < arraySize(spi4selects_gpio); cs++) {
		if (spi4selects_gpio[cs] != 0) {
			ar_configgpio(_PIN_OFF(spi4selects_gpio[cs]));
		}
	}

	ar_configgpio(GPIO_SPI6_SCK_OFF);
	ar_configgpio(GPIO_SPI6_MISO_OFF);
	ar_configgpio(GPIO_SPI6_MOSI_OFF);

	ar_configgpio(GPIO_SPI4_SCK_OFF);
	ar_configgpio(GPIO_SPI4_MISO_OFF);
	ar_configgpio(GPIO_SPI4_MOSI_OFF);

#if BOARD_USE_DRDY
	ar_configgpio(GPIO_DRDY_OFF_SPI6_DRDY1_ICM20689);
	ar_configgpio(GPIO_DRDY_OFF_SPI6_DRDY2_BMI055_GYRO);
	ar_configgpio(GPIO_DRDY_OFF_SPI6_DRDY3_BMI055_ACC);
	ar_configgpio(GPIO_DRDY_OFF_SPI6_DRDY4_ICM20602);
	ar_configgpio(GPIO_DRDY_OFF_SPI6_DRDY5_BMI055_GYRO);
	ar_configgpio(GPIO_DRDY_OFF_SPI6_DRDY6_BMI055_ACC);
#endif
	/* set the sensor rail off */
	ar_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, 0);

	/* wait for the sensor rail to reach GND */
	usleep(ms * 1000);
	warnx("reset done, %d ms", ms);

	/* re-enable power */

	/* switch the sensor rail back on */
	ar_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, 1);

	/* wait a bit before starting SPI, different times didn't influence results */
	usleep(100);

	/* reconfigure the SPI pins */
	for (size_t cs = 0; arraySize(spi6selects_gpio) > 1 && cs < arraySize(spi6selects_gpio); cs++) {
		if (spi6selects_gpio[cs] != 0) {
			ar_configgpio(spi6selects_gpio[cs]);
		}
	}

	for (size_t cs = 0; arraySize(spi4selects_gpio) > 1 && cs < arraySize(spi4selects_gpio); cs++) {
		if (spi4selects_gpio[cs] != 0) {
			ar_configgpio(spi4selects_gpio[cs]);
		}
	}

	ar_configgpio(GPIO_SPI6_SCK);
	ar_configgpio(GPIO_SPI6_MISO);
	ar_configgpio(GPIO_SPI6_MOSI);

	ar_configgpio(GPIO_SPI4_SCK);
	ar_configgpio(GPIO_SPI4_MISO);
	ar_configgpio(GPIO_SPI4_MOSI);


#if BOARD_USE_DRDY
	ar_configgpio(GPIO_SPI1_DRDY1_ICM20689);
	ar_configgpio(GPIO_SPI1_DRDY2_BMI055_GYRO);
	ar_configgpio(GPIO_SPI1_DRDY3_BMI055_ACC);
	ar_configgpio(GPIO_SPI1_DRDY4_ICM20602);
	ar_configgpio(GPIO_SPI1_DRDY5_BMI055_GYRO);
	ar_configgpio(GPIO_SPI1_DRDY6_BMI055_ACC);
#endif

}
