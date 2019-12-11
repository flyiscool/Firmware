/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
 *   Author: @author David Sidrane <david_s5@nscdg.com>
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
 * @file board_hw_rev_ver.c
 * Implementation of STM32 based Board Hardware Revision and Version ID API
 */

#include <px4_config.h>
#include <stdio.h>
#include "board_config.h"

#include "../board_internal_common.h"

#include <systemlib/px4_macros.h>

#ifdef CONFIG_DEBUG_HWREV
#  define hwinfo _info
#else
#  define hwinfo(x...)
#endif



#if defined(BOARD_HAS_HW_VERSIONING)
/****************************************************************************
 * Private Data
 ****************************************************************************/
static int hw_version = 0;
static int hw_revision = 0;
static char hw_info[] = HW_INFO_INIT;

/****************************************************************************
 * Protected Functions
 ****************************************************************************/
/****************************************************************************
  * Name: determin_hw_version
 *
 * Description:
 *
 * This function fist determines if revision  and version resistors are in place.
 * if they it will read the ADC channels and decode the DN to ordinal numbers
 * that will be returned by board_get_hw_version and board_get_hw_revision API
 *
 *  This will return OK on success and -1 on not supported
 *
 *
 ****************************************************************************/

_EXT_ITCM static int dn_to_ordinal(uint16_t dn)
{

	const struct {
		uint16_t low;
		uint16_t high;
	} dn2o[] = {
		//    R1(up) R2(down) V tpy   V min   V Max   DN Min  DN Max
		{0,  205 },     //0   NC     0R       0.000   0.000   0.125   0       205
		{206,  615 },   //1   180K   20K      0.025   0.125   0.375   206     615
		{616,  1024},   //2   82K    20K      0.490   0.375   0.625   616     1024
		{1025,  1434},  //3   120K   51K      0.756   0.625   0.875   1025    1434
		{1435,  1843},  //4   150K   100K     1.000   0.875   1.125   1435    1843
		{1844,  2253},  //5   100K   100K     1.250   1.125   1.375   1844    2253
		{2254,  2663},  //6   100K   150K     1.500   1.375   1.625   2254    2663
		{2664,  3072},  //7   51K    120K     1.754   1.625   1.875   2664    3072
		{3073,  3482},  //8   20K    82K      2.001   1.875   2.125   3073    3482
		{3483,  3891},  //9   20K    180K     2.250   2.125   2.375   3483    3891
		{3892,  4095},  //10  0R     NC       2.500   2.375   2.500   3892    4095
	};

	for (unsigned int i = 0; i < arraySize(dn2o); i++) {
		if (dn >= dn2o[i].low && dn <= dn2o[i].high) {
			return i;
		}
	}

	return -1;
}

/************************************************************************************
 * Name: read_id_dn
 *
 * Description:
 *   Read the HW sense set to get a DN of the value formed by
 *                0 VDD
 *                |
 *                /
 *                \   R1
 *                /
 *                |
 *                +--------------- GPIO_HW_xxx_SENCE  ADC channel N
 *                |
 *                /
 *                \ R2
 *                /
 *                |
 *                |
 *                +--------------- gnd
 *
 * Input Parameters:
 *   id          - pointer to receive the dn for the id set
 *   gpio_drive  - gpio that is the drive
 *   gpio_sense  - gpio that is the sence
 *   adc_channel - the Channel number associated with gpio_sense
 *
 * Returned Value:
 *    0    - Success and id is set
 *   -EIO  - FAiled to init or read the ADC
 *
 ************************************************************************************/

_EXT_ITCM static int read_id_dn(int *id, int adc_channel)
{
	int rv = -EIO;
	const unsigned int samples  = 16;

	uint32_t dn_sum = 0;
	uint16_t dn = 0;

	if (board_adc_init() == OK) {
		/* Read the value */
		for (unsigned av = 0; av < samples; av++) {
			dn = board_adc_sample(adc_channel);
			hwinfo("dn = %d\r\n", dn);

			if (dn == 0xffff) { break; }

			dn_sum  += dn;
		}

		if (dn != 0xffff) {
			*id = dn_sum / samples;
			rv = OK;
		}
	}

	return rv;
}


_EXT_ITCM static int determine_hw_info(int *revision, int *version)
{
	int dn;
	int rv = read_id_dn(&dn, ADC_HW_REV_SENSE_CHANNEL);

	if (rv == OK) {
		*revision =  dn_to_ordinal(dn);
	}

	rv = read_id_dn(&dn, ADC_HW_VER_SENSE_CHANNEL);

	if (rv == OK) {
		*version =  dn_to_ordinal(dn);
	}

	return rv;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/************************************************************************************
 * Name: board_get_hw_type
 *
 * Description:
 *   Optional returns a 0 terminated string defining the HW type.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   a 0 terminated string defining the HW type. This my be a 0 length string ""
 *
 ************************************************************************************/

__EXPORT const char *board_get_hw_type_name()
{
	return (const char *) hw_info;
}

/************************************************************************************
 * Name: board_get_hw_version
 *
 * Description:
 *   Optional returns a integer HW version
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer value of this boards hardware version.
 *   A value of -1 is the default for boards not supporting the BOARD_HAS_VERSIONING API.
 *   A value of 0 is the default for boards supporting the API but not having version.
 *
 ************************************************************************************/

__EXPORT int board_get_hw_version()
{
	return  hw_version;
}

/************************************************************************************
 * Name: board_get_hw_revision
 *
 * Description:
 *   Optional returns a integer HW revision
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer value of this boards hardware revision.
 *   A value of -1 is the default for boards not supporting the BOARD_HAS_VERSIONING API.
 *   A value of 0 is the default for boards supporting the API but not having revision.
 *
 ************************************************************************************/

__EXPORT int board_get_hw_revision()
{
	return  hw_revision;
}

/************************************************************************************
  * Name: board_determine_hw_info
 *
 * Description:
 *	Uses the HW revision and version detection added in FMUv5.
 *	See https://docs.google.com/spreadsheets/d/1-n0__BYDedQrc_2NHqBenG1DNepAgnHpSGglke-QQwY
 *	HW REV and VER ID tab.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0  - on success or negated errono
 *   1) The values for integer value of this boards hardware revision is set
 *   2) The integer value of this boards hardware version is set.
 *   3) hw_info is populated
 *
 *   A value of 0 is the default for boards supporting the BOARD_HAS_HW_VERSIONING API.
 *   but not having R1 and R2.
 *
 ************************************************************************************/

int board_determine_hw_info()
{
	int rv = determine_hw_info(&hw_revision, &hw_version);

	if (rv == OK) {
		hw_info[HW_INFO_INIT_REV] = board_get_hw_revision() + '0';
		hw_info[HW_INFO_INIT_VER] = board_get_hw_version() + '0';
	}

	return rv;
}
#endif
