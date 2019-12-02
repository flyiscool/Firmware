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
 * @file px4fmu_can.c
 *
 * Board-specific CAN functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_config.h>

#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <ar_can.h>

#include "up_arch.h"

#include "chip.h"
#include "board_config.h"

#ifdef CONFIG_CAN

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#ifdef CONFIG_AR_CAN0
	#define CAN_PORT0 0
#endif

#ifdef CONFIG_AR_CAN1
	#define CAN_PORT1 1
#endif

#ifdef CONFIG_AR_CAN2
	#define CAN_PORT2 2
#endif

#ifdef CONFIG_AR_CAN3
	#define CAN_PORT3 3
#endif


/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: can_devinit
 *
 * Description:
 *   All STM32 architectures must provide the following interface to work with
 *   examples/can.
 *
 ************************************************************************************/

int can_devinit(void)
{
	static bool initialized = false;

	/* Check if we have already initialized */
	if (initialized) 
	{
		return OK;
	}

	struct can_dev_s *can;
	int ret;

	#if defined(CONFIG_CAN) && defined(CONFIG_AR_CAN0) && defined(CAN_PORT0) 
		can = ar_caninitialize(CAN_PORT0);
		
		if (can != NULL) 
		{
			ret = can_register("/dev/can0", can);
			initialized = true;

			if (ret < 0)
			{
				canerr("ERROR: can0_register failed: %d\n", ret);
			}
		} 
		else 
		{
			canerr("ERROR:  Failed to get CAN0 interface\n");
		}	
	#endif

	#if defined(CONFIG_CAN) && defined(CONFIG_AR_CAN1) && defined(CAN_PORT1) 
		can = ar_caninitialize(CAN_PORT1);
		
		if (can != NULL) 
		{
			ret = can_register("/dev/can1", can);
			initialized = true;

			if (ret < 0)
			{
				canerr("ERROR: can1_register failed: %d\n", ret);
			}
		} 
		else 
		{
			canerr("ERROR:  Failed to get CAN1 interface\n");
		}	
	#endif
	
	#if defined(CONFIG_CAN) && defined(CONFIG_AR_CAN2) && defined(CAN_PORT2) 
		can = ar_caninitialize(CAN_PORT2);
		
		if (can != NULL) 
		{
			ret = can_register("/dev/can2", can);
			initialized = true;

			if (ret < 0)
			{
				canerr("ERROR: can2_register failed: %d\n", ret);
			}
		} 
		else 
		{
			canerr("ERROR:  Failed to get CAN2 interface\n");
		}	
	#endif

	#if defined(CONFIG_CAN) && defined(CONFIG_AR_CAN3) && defined(CAN_PORT3) 
		can = ar_caninitialize(CAN_PORT3);
		
		if (can != NULL) 
		{
			ret = can_register("/dev/can3", can);
			initialized = true;

			if (ret < 0)
			{
				canerr("ERROR: can3_register failed: %d\n", ret);
			}
		} 
		else 
		{
			canerr("ERROR:  Failed to get CAN3 interface\n");
		}	
	#endif

	if (initialized == false)
	{
		return -ENODEV;
	}

	return OK;
}

#endif
